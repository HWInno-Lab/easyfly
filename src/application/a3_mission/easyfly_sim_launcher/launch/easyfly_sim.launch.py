"""
easyfly_sim.launch.py — 一键启动 EasyFly 完整仿真环境

用法:
  ros2 launch easyfly_sim_launcher easyfly_sim.launch.py

启动:
  Gazebo + PX4 SITL → mavros2 + mavlink_bridge + rc_receiver
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
from easyfly_px4_sitl.scene_environment import installed_catalog
from easyfly_px4_sitl.scene_inputs import resolve_scene


def _validate_scene(context):
    scene = LaunchConfiguration('scene', default='').perform(context).strip()
    world = LaunchConfiguration('world', default='').perform(context).strip()
    if scene and world:
        raise ValueError('scene and world are mutually exclusive')
    if world:
        raise ValueError('raw world is only supported by the low-level px4_sitl debugging entrypoint')
    if scene:
        catalog, _ = installed_catalog()
        resolve_scene(catalog, scene, 'base_sim')
    return []


def generate_launch_description():
    headless = DeclareLaunchArgument('headless', default_value='false')
    # ---- px4_sitl (Gazebo + PX4 SITL) ----
    px4_sitl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory("easyfly_px4_sitl"),
            "/launch/px4_sitl.launch.py",
        ]),
        launch_arguments={
            'scene': LaunchConfiguration('scene'),
            'world': LaunchConfiguration('world'),
            'scene_profile': 'base_sim',
            "gzclient": PythonExpression([
                "'false' if '", LaunchConfiguration('headless'),
                "'.lower() == 'true' else 'true'"
            ]),
        }.items(),
    )

    # ---- mavlink_bridge (mavros2 + bridge + rc_receiver) ----
    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory("easyfly_mavlink_bridge"),
            "/launch/mavlink_bridge.launch.py",
        ]),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    return LaunchDescription([
        headless,
        DeclareLaunchArgument('scene', default_value=''),
        DeclareLaunchArgument('world', default_value=''),
        OpaqueFunction(function=_validate_scene),
        px4_sitl_launch,
        bridge_launch,
    ])
