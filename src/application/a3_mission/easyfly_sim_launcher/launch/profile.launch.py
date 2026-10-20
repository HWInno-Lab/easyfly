"""D0 development entrypoint; no automatic flight goal and no hardware launch."""

from pathlib import Path
import json
import re

from ament_index_python.packages import get_package_share_directory
from easyfly_params.configuration import PROFILES, ensure_launchable, load_profile
from easyfly_params.algorithms import validate, detector_parameters, localization_backend
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from easyfly_px4_sitl.scene_environment import installed_catalog
from easyfly_px4_sitl.scene_inputs import resolve_scene


def _launch_profile(context):
    profile = load_profile(LaunchConfiguration("profile").perform(context))
    ensure_launchable(profile)
    scene = LaunchConfiguration('scene', default='').perform(context).strip()
    world = LaunchConfiguration('world', default='').perform(context).strip()
    if scene and world:
        raise ValueError('scene and world are mutually exclusive')
    if world:
        raise ValueError(
            'raw world is only supported by the low-level px4_sitl debugging entrypoint')
    arguments = dict(profile['launch_arguments'])
    headless = LaunchConfiguration('headless', default='').perform(context).strip().lower()
    if headless:
        if headless not in ('true', 'false'):
            raise ValueError('headless must be true or false')
        arguments['headless'] = headless
    for key, default in [('fcu_url', 'udp://127.0.0.1:14540@127.0.0.1:14555'),
                         ('gcs_url', ''), ('target_system', '1'),
                         ('detection_use_gpu', 'auto')]:
        arguments[key] = LaunchConfiguration(key, default=default).perform(context)
    if profile['entrypoint'] != 'base':
        arguments['rviz'] = LaunchConfiguration('rviz', default='true').perform(context)
    raw_options = LaunchConfiguration('algorithms_json', default='').perform(context)
    options = validate(json.loads(raw_options) if raw_options else {})
    backend = localization_backend(options, profile['name'])
    use_slam = backend != 'disabled'
    if profile['entrypoint'] != 'base':
        arguments['slam_enabled'] = 'true' if use_slam else 'false'
        arguments['localization_backend'] = backend
        if raw_options and backend == 'fast_lio':
            arguments['fastlio_options_json'] = json.dumps(options, separators=(',', ':'))
    if raw_options:
        arguments['detection_use_gpu'] = {'cpu': 'false', 'cuda': 'true', 'auto': 'auto'}[
            options['provider']]
        if profile['entrypoint'] != 'base' and options['strategy'] != 'profile':
            arguments['navigation_strategy'] = options['strategy']
    detectors = []
    for kind in ('ellipse', 'color_line'):
        if not options[kind + '_enabled']:
            continue
        topic = LaunchConfiguration('algorithm_image_topic').perform(context)
        valid_topic = re.fullmatch(r'/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*', topic)
        if len(topic) > 256 or not valid_topic:
            raise ValueError('几何检测输入必须是合法绝对图像话题')
        frame = options['optical_frame'] or (
            'camera_down_optical_frame' if options['camera'] == 'down'
            else 'camera_color_optical_frame')
        detectors.append(Node(
            package='easyfly_' + kind + '_detection', executable=kind + '_detection_node',
            name=kind + '_detection', output='screen', parameters=[dict(
                detector_parameters(options, kind), use_sim_time=True,
                optical_frame=frame, input_topic=topic,
                output_topic='/' + kind + '_observations')]))
    if scene:
        catalog, _ = installed_catalog()
        resolve_scene(catalog, scene, profile['name'])
        arguments.update(scene=scene, scene_profile=profile['name'])
    if profile["entrypoint"] == "base":
        package, filename = "easyfly_sim_launcher", "easyfly_sim.launch.py"
    else:
        package, filename = "easyfly_px4_sitl", "perception_sitl.launch.py"
    source = Path(get_package_share_directory(package)) / "launch" / filename
    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(source)),
        launch_arguments=arguments.items(),
    )
    platform = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory('easyfly_mission_srvs')) /
            'launch/platform.launch.py')),
        launch_arguments={'profile': profile['name'], 'use_sim_time': 'true',
                          'start_mission_executor': 'false'}.items())
    message = f"D0-D2 profile={profile['name']} readiness={profile['readiness']}"
    if profile.get("note"):
        message += ": " + profile["note"]
    # Parent generic config_file/use_sim_time overrides must not pollute child defaults.
    return [LogInfo(msg=message), GroupAction(
        scoped=True, forwarding=False, actions=[include, platform, *detectors])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("profile", default_value="base_sim", choices=PROFILES),
        DeclareLaunchArgument('scene', default_value='',
                              description='Registered scene ID; empty preserves profile defaults'),
        DeclareLaunchArgument('world', default_value='',
                              description='Reserved; use the low-level debug entrypoint'),
        DeclareLaunchArgument('headless', default_value='', choices=['', 'true', 'false'],
                              description='Display override; empty uses the profile default'),
        DeclareLaunchArgument('fcu_url', default_value='udp://127.0.0.1:14540@127.0.0.1:14555'),
        DeclareLaunchArgument('gcs_url', default_value=''),
        DeclareLaunchArgument('target_system', default_value='1'),
        DeclareLaunchArgument('detection_use_gpu', default_value='auto'),
        DeclareLaunchArgument('algorithms_json', default_value='',
                              description='Validated algorithm configuration from the UI'),
        DeclareLaunchArgument('algorithm_image_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=_launch_profile),
    ])
