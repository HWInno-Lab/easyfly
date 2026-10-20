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
    grid_footprint_mode: str = 'cluster_boxes'
    heartbeat_rate_hz: int = 1
    clock_rate_hz: int = 10
    battery_rate_hz: int = 0
    grid_height_limit_m: float = 0.0


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
    roots = list(dict.fromkeys(catalog.asset_root(key) for key in sets))
    # Check all directories exposed to Gazebo, not only model names in manifests.
    # Two distinct directories must never silently shadow one another.
    seen = {}
    for root in roots:
        for config in root.glob('*/model.config'):
            name, directory = config.parent.name, config.parent.resolve()
            if name in seen and seen[name] != directory:
                raise CatalogError(f'duplicate model name in search roots: {name}')
            seen[name] = directory
    battery = scene['runtime'].get('battery_rate_hz',0)
    if battery and scene['runtime'].get('heartbeat_rate_hz',1)!=5:
        raise CatalogError('native battery rate requires reviewed 5Hz heartbeat configuration')
    return SceneInputs(str(catalog.resolve(scene['world'])), ':'.join(str(p) for p in roots),
                       str(catalog.resolve(scene['runtime']['entrypoint'])),
                       tuple(scene['spawn']['position']), scene['spawn']['yaw'],
                       scene['navigation'].get('ground_model_frame','odom'),
                       scene['navigation'].get('grid_footprint_mode','cluster_boxes'),
                       scene['runtime'].get('heartbeat_rate_hz',1),
                       scene['runtime'].get('clock_rate_hz',10),battery,
                       scene['navigation'].get('grid_height_limit_m',0.0))


def selection_arguments(default_profile):
    return [DeclareLaunchArgument('scene', default_value='', description='Registered scene ID; empty keeps legacy defaults'),
            DeclareLaunchArgument('scene_profile', default_value=default_profile,
                                  description='Compatibility profile for an explicitly selected scene'),
            DeclareLaunchArgument('world', default_value='',
                                  description='Legacy relative world filename; mutually exclusive with scene'),
        DeclareLaunchArgument('grid_footprint_mode',default_value='',
                              description='Low-level occupancy footprint mode; managed scenes use their manifest'),
        DeclareLaunchArgument('ground_model_frame',default_value='',
                              description='Low-level ground basis override; managed scenes use their manifest'),
        DeclareLaunchArgument('grid_height_limit_m',default_value='',
                              description='Explicit bounded 2D projection; managed scenes use their manifest'),
        DeclareLaunchArgument('debug_model_path',default_value='',
                                  description='Explicit model roots for low-level raw-world debugging only'),
        DeclareLaunchArgument('debug_spawn_pose', default_value='',
                              description='Explicit x y z yaw for raw-world geometry/sensor inspection'),
        DeclareLaunchArgument('debug_heartbeat_rate_hz', default_value='',
                              description='Reviewed native heartbeat rate (1 or 5) for explicit raw-world inspection'),
        DeclareLaunchArgument('debug_clock_rate_hz', default_value='',
                              description='Native Gazebo ROS clock rate (10 or 100) for explicit raw-world inspection'),
        DeclareLaunchArgument('debug_battery_rate_hz', default_value='',
                              description='Optional native 5Hz BATTERY_STATUS for explicit raw-world inspection')]


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
        if _value(context, 'debug_heartbeat_rate_hz'):
            raise CatalogError('managed scene cannot override debug_heartbeat_rate_hz')
        if _value(context, 'debug_clock_rate_hz'):
            raise CatalogError('managed scene cannot override debug_clock_rate_hz')
        if _value(context, 'debug_battery_rate_hz'):
            raise CatalogError('managed scene cannot override debug_battery_rate_hz')
        if _value(context,'grid_footprint_mode'):
            raise CatalogError('managed scene cannot override grid_footprint_mode')
        if _value(context,'ground_model_frame'):
            raise CatalogError('managed scene cannot override ground_model_frame')
        if _value(context,'grid_height_limit_m'):
            raise CatalogError('managed scene cannot override grid_height_limit_m')
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
        debug_rate = _value(context, 'debug_heartbeat_rate_hz')
        if debug_rate:
            if not world or not debug_roots:
                raise CatalogError('debug_heartbeat_rate_hz requires explicit world and debug_model_path')
            if debug_rate not in ('1', '5'):
                raise CatalogError('debug_heartbeat_rate_hz must be 1 or 5')
        debug_clock = _value(context, 'debug_clock_rate_hz')
        if debug_clock:
            if not world or not debug_roots:
                raise CatalogError('debug_clock_rate_hz requires explicit world and debug_model_path')
            if debug_clock not in ('10', '100'):
                raise CatalogError('debug_clock_rate_hz must be 10 or 100')
        debug_battery = _value(context, 'debug_battery_rate_hz')
        if debug_battery:
            if not world or not debug_roots or debug_battery!='5' or debug_rate!='5':
                raise CatalogError('debug_battery_rate_hz requires rate 5, explicit world/model roots and 5Hz heartbeat')
        ground_frame=_value(context,'ground_model_frame') or 'odom'
        if ground_frame not in ('map','odom'):
            raise CatalogError('ground_model_frame must be map or odom')
        grid_mode=_value(context,'grid_footprint_mode') or 'cluster_boxes'
        if grid_mode not in ('cluster_boxes','observed_cells'):
            raise CatalogError('grid_footprint_mode must be cluster_boxes or observed_cells')
        height_value=_value(context,'grid_height_limit_m')
        try:
            height_limit=float(height_value or '0.0')
        except ValueError as error:
            raise CatalogError('grid height limit requires a finite nonnegative number') from error
        if not math.isfinite(height_limit) or height_limit<0:
            raise CatalogError('grid height limit requires a finite nonnegative number')
        if height_limit and (not world or not debug_roots or grid_mode!='observed_cells'):
            raise CatalogError('bounded grid requires explicit world/model roots and observed_cells')
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
                             _value(context, 'model_sdf') or default_model, pose[:3], pose[3],ground_frame,grid_mode,
                             int(debug_rate or '1'), int(debug_clock or '10'),int(debug_battery or '0'),height_limit)
    values = dict(world=result.world, model_path=result.model_path, model_sdf=result.model_sdf,
                  x=str(result.position[0]), y=str(result.position[1]), z=str(result.position[2]),
                  yaw=str(result.yaw),ground_model_frame=result.ground_model_frame,
                  grid_footprint_mode=result.grid_footprint_mode,
                  grid_height_limit_m=str(result.grid_height_limit_m),
                  heartbeat_rate_hz=str(result.heartbeat_rate_hz),
                  clock_rate_hz=str(result.clock_rate_hz),battery_rate_hz=str(result.battery_rate_hz))
    return [SetLaunchConfiguration(PREFIX + key, value) for key, value in values.items()]


def preflight_action(**kwargs):
    return OpaqueFunction(function=_managed_preflight, kwargs=kwargs)


def _managed_preflight(context, **kwargs):
    # Validate selection first. Loading the private native middleware must
    # precede launch_ros's first component service/client DDS initialization.
    actions = simulation_preflight(context, **kwargs)
    selected_vio = (_value(context, 'localization_backend') == 'openvins' and
                    _value(context, 'slam_enabled') != 'false')
    if _value(context, 'scene') or selected_vio:
        import os
        from easyfly_fastdds_compat.runtime import activate
        from launch.actions import SetEnvironmentVariable
        middleware = activate(openvins=selected_vio)
        actions.insert(0, SetEnvironmentVariable('LD_LIBRARY_PATH', os.environ['LD_LIBRARY_PATH']))
        actions.insert(1, SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'))
        actions[2:2] = [SetEnvironmentVariable(key, value)
                       for key, value in middleware['profile_environment'].items()]
    return actions


def scene_value(name):
    return LaunchConfiguration(PREFIX + name)
