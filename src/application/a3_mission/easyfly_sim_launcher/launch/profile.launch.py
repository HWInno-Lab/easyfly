"""D0 development entrypoint; no automatic flight goal and no hardware launch."""

from pathlib import Path
import json
import math
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
        arguments['local_planner_max_acceleration'] = LaunchConfiguration('local_planner_max_acceleration', default='6.0').perform(context)
        if arguments['local_planner_max_acceleration'] not in ('2.0', '6.0'):
            raise ValueError('unsupported local planner acceleration bound')
        arguments['rviz'] = LaunchConfiguration('rviz', default='true').perform(context)
    raw_options = LaunchConfiguration('algorithms_json', default='').perform(context)
    options = validate(json.loads(raw_options) if raw_options else {})
    backend = localization_backend(options, profile['name'])
    if options['strategy'] in ('gcopter', 'super') and profile['name'] != 'navigation_dev':
        raise ValueError(options['strategy'].upper() + ' simulation navigation requires navigation_dev')
    if options['strategy'] in ('gcopter', 'super'):
        key = options['strategy'] + '_evidence_directory'
        arguments[key] = LaunchConfiguration(key, default='').perform(context)
    dynamic = LaunchConfiguration('gcopter_dynamic_navigation', default='false').perform(context)
    if dynamic not in ('true', 'false'):
        raise ValueError('gcopter_dynamic_navigation must be true or false')
    if dynamic == 'true' and (options['strategy'] != 'gcopter' or
                              profile['name'] != 'navigation_dev' or options['exploration'] == 'fuel'):
        raise ValueError('Dynamic GCOPTER requires independent navigation_dev with GCOPTER')
    arguments['gcopter_dynamic_navigation'] = dynamic
    committed = LaunchConfiguration('super_execute_committed', default='false').perform(context)
    if committed not in ('true', 'false'):
        raise ValueError('super_execute_committed must be true or false')
    if committed == 'true' and (options['strategy'] != 'super' or profile['name'] != 'navigation_dev'):
        raise ValueError('committed SUPER execution requires navigation_dev and SUPER')
    arguments['super_execute_committed'] = committed
    extended = LaunchConfiguration('fuel_extended_profile', default='false').perform(context)
    if extended not in ('true', 'false'):
        raise ValueError('fuel_extended_profile must be true or false')
    if extended == 'true' and (options['exploration'] != 'fuel' or dynamic == 'true'):
        raise ValueError('Extended FUEL requires the fixed FUEL policy map')
    arguments['fuel_extended_profile'] = extended
    arguments['fuel_native_max_speed'] = LaunchConfiguration('fuel_native_max_speed', default='1.2').perform(context)
    if not math.isfinite(float(arguments['fuel_native_max_speed'])) or not .1 <= float(arguments['fuel_native_max_speed']) <= 2.:
        raise ValueError('fuel_native_max_speed must be .1 to 2 m/s')
    if options['exploration'] == 'fuel':
        if profile['name'] != 'navigation_dev' or options['strategy'] != 'gcopter' or backend != 'fast_lio':
            raise ValueError('FUEL requires navigation_dev, FAST-LIO and GCOPTER')
        evidence = LaunchConfiguration('fuel_evidence_directory', default='').perform(context)
        if not evidence or not Path(evidence).is_absolute():
            raise ValueError('FUEL requires an absolute evidence directory')
        arguments.update(exploration_backend='fuel', fuel_evidence_directory=evidence)
    vio_navigation = LaunchConfiguration('openvins_navigation', default='false').perform(context)
    if vio_navigation not in ('true', 'false'):
        raise ValueError('openvins_navigation must be true or false')
    if vio_navigation == 'true' and (backend != 'openvins' or profile['name'] != 'navigation_dev'):
        raise ValueError('OpenVINS simulation navigation requires navigation_dev and OpenVINS')
    livo_navigation = LaunchConfiguration('fastlivo_navigation', default='false').perform(context)
    if livo_navigation not in ('true', 'false'):
        raise ValueError('fastlivo_navigation must be true or false')
    if livo_navigation == 'true' and (backend != 'fast_livo2' or profile['name'] != 'navigation_dev'):
        raise ValueError('FAST-LIVO2 simulation navigation requires navigation_dev and FAST-LIVO2')
    orb_navigation = LaunchConfiguration('orbslam3_navigation', default='false').perform(context)
    if orb_navigation not in ('true', 'false'):
        raise ValueError('orbslam3_navigation must be true or false')
    if orb_navigation == 'true' and (backend != 'orbslam3' or profile['name'] != 'navigation_dev'):
        raise ValueError('ORB-SLAM3 navigation requires navigation_dev and ORB-SLAM3')
    orb_sensor = LaunchConfiguration('orbslam3_sensor_mode', default='rgbd').perform(context)
    if orb_sensor not in ('rgbd', 'rgbd_imu') or (orb_sensor == 'rgbd_imu' and backend != 'orbslam3'):
        raise ValueError('ORB sensor mode requires the matching selected backend')
    if backend == 'orbslam3':
        runtime = LaunchConfiguration('orbslam3_runtime_directory', default='').perform(context)
        if not runtime or not Path(runtime).is_absolute() or not Path(runtime).is_dir() or (Path(runtime)/'runtime-settings.yaml').exists():
            raise ValueError('ORB-SLAM3 requires a fresh absolute runtime directory')
        arguments['orbslam3_runtime_directory'] = runtime
        arguments['orbslam3_navigation'] = orb_navigation
        arguments['orbslam3_sensor_mode'] = orb_sensor
        for key in ['load_atlas', 'save_atlas']:
            arguments['orbslam3_'+key] = LaunchConfiguration('orbslam3_'+key, default='').perform(context)
    use_slam = backend != 'disabled'
    if profile['entrypoint'] != 'base':
        arguments['slam_enabled'] = 'true' if use_slam else 'false'
        arguments['localization_backend'] = backend
        if backend == 'fast_livo2':
            arguments['fastlivo_navigation'] = livo_navigation
        if backend == 'openvins':
            arguments['openvins_navigation'] = vio_navigation
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
        DeclareLaunchArgument('orbslam3_navigation', default_value='false', choices=['true','false']),
        DeclareLaunchArgument('orbslam3_sensor_mode', default_value='rgbd', choices=['rgbd','rgbd_imu']),
        *[DeclareLaunchArgument('orbslam3_'+k, default_value='') for k in ['runtime_directory','load_atlas','save_atlas']],
        DeclareLaunchArgument('fastlivo_navigation', default_value='false', choices=['true','false']),
        DeclareLaunchArgument('openvins_navigation', default_value='false',
                              choices=['true', 'false'],
                              description='Simulation navigation; requires navigation_dev'),
        DeclareLaunchArgument('algorithm_image_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('gcopter_evidence_directory', default_value=''),
        DeclareLaunchArgument('gcopter_dynamic_navigation', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('super_evidence_directory', default_value=''),
        DeclareLaunchArgument('super_execute_committed', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('fuel_evidence_directory', default_value=''),
        DeclareLaunchArgument('fuel_extended_profile', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('fuel_native_max_speed', default_value='1.2'),
        DeclareLaunchArgument('local_planner_max_acceleration', default_value='6.0'),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=_launch_profile),
    ])
