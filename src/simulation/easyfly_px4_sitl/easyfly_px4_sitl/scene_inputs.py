# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Resolve managed scene inputs before any simulator or child node starts."""

from dataclasses import dataclass
import math
from pathlib import Path

from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration

from .scene_catalog import CatalogError
from .scene_environment import installed_catalog


PREFIX = '_easyfly_scene_'


@dataclass(frozen=True)
class SceneInputs:
    world: str
    model_path: str
    model_sdf: str
    position: tuple
    yaw: float
    ground_model_frame: str = 'odom'


def resolve_scene(catalog, scene_id, profile):
    """Require a complete registered scene/profile; never update evidence fields."""
    if scene_id not in catalog.scenes:
        raise CatalogError(f'unknown scene: {scene_id}')
    scene = catalog.scenes[scene_id]
    report = catalog.validate(scene_id).as_dict()
    problems = [f"{item['scope']}: {item['message']}" for item in report['issues']]
    if profile not in scene['supported_profiles']:
        problems.append(f'incompatible scene/profile: {scene_id}/{profile}')
    if problems:
        raise CatalogError(f'scene {scene_id} refused before startup: ' + '; '.join(problems))
    sets = dict.fromkeys(scene['asset_sets'] + scene['runtime']['asset_sets'])
    roots = list(dict.fromkeys(catalog.resolve(catalog.assets[key]['model_root']) for key in sets))
    # Check all directories exposed to Gazebo, not only model names in manifests.
    # Two distinct directories must never silently shadow one another.
    seen = {}
    for root in roots:
        for config in root.glob('*/model.config'):
            name, directory = config.parent.name, config.parent.resolve()
            if name in seen and seen[name] != directory:
                raise CatalogError(f'duplicate model name in search roots: {name}')
            seen[name] = directory
    return SceneInputs(str(catalog.resolve(scene['world'])), ':'.join(str(p) for p in roots),
                       str(catalog.resolve(scene['runtime']['entrypoint'])),
                       tuple(scene['spawn']['position']), scene['spawn']['yaw'],
                       scene['navigation'].get('ground_model_frame','odom'))


def selection_arguments(default_profile):
    return [DeclareLaunchArgument('scene', default_value='', description='Registered scene ID; empty keeps legacy defaults'),
            DeclareLaunchArgument('scene_profile', default_value=default_profile,
                                  description='Compatibility profile for an explicitly selected scene'),
            DeclareLaunchArgument('world', default_value='',
                                  description='Legacy relative world filename; mutually exclusive with scene'),
        DeclareLaunchArgument('ground_model_frame',default_value='',
                              description='Low-level ground basis override; managed scenes use their manifest'),
        DeclareLaunchArgument('debug_model_path',default_value='',
                                  description='Explicit model roots for low-level raw-world debugging only'),
        DeclareLaunchArgument('debug_spawn_pose', default_value='',
                              description='Explicit x y z yaw for raw-world geometry/sensor inspection')]


def _value(context, name):
    return LaunchConfiguration(name, default='').perform(context).strip()


def simulation_preflight(context, *, px4_dir, default_world, default_roots, default_model,
                         allowed_profiles, catalog_factory=installed_catalog):
    """Legacy defaults remain local; managed scenes are fully resolved first."""
    scene, world = _value(context, 'scene'), _value(context, 'world')
    if scene and world:
        raise CatalogError('scene and world are mutually exclusive')
    if scene:
        profile = _value(context, 'scene_profile')
        if profile not in allowed_profiles:
            raise CatalogError(f'incompatible scene profile for this entrypoint: {profile}')
        if _value(context, 'model_sdf') and _value(context, 'model_sdf') != default_model:
            raise CatalogError('managed scene cannot override model_sdf')
        if _value(context,'debug_model_path'):
            raise CatalogError('managed scene cannot override debug_model_path')
        if _value(context, 'debug_spawn_pose'):
            raise CatalogError('managed scene cannot override debug_spawn_pose')
        if _value(context,'ground_model_frame'):
            raise CatalogError('managed scene cannot override ground_model_frame')
        catalog, _ = catalog_factory(px4_dir=px4_dir)
        result = resolve_scene(catalog, scene, profile)
    else:
        # Low-level legacy world remains a debugging interface, including D5.
        from ament_index_python.packages import get_package_share_directory
        root = Path(get_package_share_directory('easyfly_gazebo_worlds'))
        debug_roots=_value(context,'debug_model_path')
        roots=default_roots
        if debug_roots:
            roots=debug_roots.split(':')
            if any(not Path(p).is_absolute() or not Path(p).is_dir() for p in roots):
                raise CatalogError('debug_model_path requires existing absolute directories')
        ground_frame=_value(context,'ground_model_frame') or 'odom'
        if ground_frame not in ('map','odom'):
            raise CatalogError('ground_model_frame must be map or odom')
        pose = (0.0, 0.0, 0.83, 0.0)
        debug_pose = _value(context, 'debug_spawn_pose')
        if debug_pose:
            if not world or not debug_roots:
                raise CatalogError('debug_spawn_pose requires explicit world and debug_model_path')
            try:
                pose = tuple(float(value) for value in debug_pose.split())
            except ValueError as error:
                raise CatalogError('debug_spawn_pose requires four finite values: x y z yaw') from error
            if len(pose) != 4 or not all(math.isfinite(value) for value in pose):
                raise CatalogError('debug_spawn_pose requires four finite values: x y z yaw')
        result = SceneInputs(str(root / (world or default_world)), ':'.join(roots),
                             _value(context, 'model_sdf') or default_model, pose[:3], pose[3],ground_frame)
    values = dict(world=result.world, model_path=result.model_path, model_sdf=result.model_sdf,
                  x=str(result.position[0]), y=str(result.position[1]), z=str(result.position[2]),
                  yaw=str(result.yaw),ground_model_frame=result.ground_model_frame)
    return [SetLaunchConfiguration(PREFIX + key, value) for key, value in values.items()]


def preflight_action(**kwargs):
    return OpaqueFunction(function=_managed_preflight, kwargs=kwargs)


def _managed_preflight(context, **kwargs):
    # Validate selection first. Loading the private native middleware must
    # precede launch_ros's first component service/client DDS initialization.
    actions = simulation_preflight(context, **kwargs)
    if _value(context, 'scene'):
        import os
        from easyfly_fastdds_compat.runtime import activate
        from launch.actions import SetEnvironmentVariable
        middleware = activate()
        actions.insert(0, SetEnvironmentVariable('LD_LIBRARY_PATH', os.environ['LD_LIBRARY_PATH']))
        actions.insert(1, SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'))
        actions[2:2] = [SetEnvironmentVariable(key, value)
                       for key, value in middleware['profile_environment'].items()]
    return actions


def scene_value(name):
    return LaunchConfiguration(PREFIX + name)
