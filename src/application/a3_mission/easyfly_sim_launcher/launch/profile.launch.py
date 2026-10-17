"""D0 development entrypoint; no automatic flight goal and no hardware launch."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from easyfly_params.configuration import PROFILES, ensure_launchable, load_profile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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
        raise ValueError('raw world is only supported by the low-level px4_sitl debugging entrypoint')
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
        PythonLaunchDescriptionSource(str(Path(get_package_share_directory('easyfly_mission_srvs')) /
                                          'launch/platform.launch.py')),
        launch_arguments={'profile': profile['name'], 'use_sim_time': 'true',
                          'start_mission_executor': 'false'}.items())
    message = f"D0-D2 profile={profile['name']} readiness={profile['readiness']}"
    if profile.get("note"):
        message += ": " + profile["note"]
    # Parent generic config_file/use_sim_time overrides must not pollute child defaults.
    return [LogInfo(msg=message), GroupAction(
        scoped=True, forwarding=False, actions=[include, platform])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("profile", default_value="base_sim", choices=PROFILES),
        DeclareLaunchArgument('scene', default_value='', description='Registered scene ID; empty preserves profile defaults'),
        DeclareLaunchArgument('world', default_value='', description='Reserved; raw worlds require the low-level debug entrypoint'),
        DeclareLaunchArgument('headless', default_value='', choices=['', 'true', 'false'],
                              description='Display override; empty uses the profile default'),
        DeclareLaunchArgument('fcu_url', default_value='udp://127.0.0.1:14540@127.0.0.1:14555'),
        DeclareLaunchArgument('gcs_url', default_value=''),
        DeclareLaunchArgument('target_system', default_value='1'),
        DeclareLaunchArgument('detection_use_gpu', default_value='auto'),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=_launch_profile),
    ])
