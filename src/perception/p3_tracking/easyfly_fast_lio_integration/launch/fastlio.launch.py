"""M3: fast_lio + map→odom TF 集成启动

用法:
  ros2 launch easyfly_fast_lio_integration fastlio.launch.py
  # 参数: fastlio_config_file (默认 mid360.yaml), rviz (默认 false, 由 perception_sitl 统一开)

内容:
  - fast_lio::fastlio_mapping (上游 ROS2 版, config/mid360.yaml)
  - map_odom_tf (EasyFly 定制: SLAM odom + /odom → TF map→odom, 平滑)

前置 (由 perception_sitl 或手动提供):
  - /livox/lidar, /livox/imu (M0 仿真插件)
  - /odom (M1 easyfly_coord_transform, 需 /uav_state 即 mavros)
"""
import json

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def _select_fastlio_config(context):
    """Resolve the legacy alias without allowing an inherited empty value."""
    legacy = LaunchConfiguration('config_file').perform(context).strip()
    dedicated = LaunchConfiguration('fastlio_config_file').perform(context).strip()
    return legacy or dedicated or 'mid360.yaml'


def _launch_fast_lio(context):
    """Create the node after the included launch context is fully scoped."""
    selected_config = _select_fastlio_config(context)
    raw_options = LaunchConfiguration('fastlio_options_json', default='').perform(context)
    overrides = {}
    if raw_options:
        from easyfly_params.algorithms import validate, slam_parameters
        overrides = slam_parameters(validate(json.loads(raw_options)))
    profile = LaunchConfiguration('sensor_profile', default='native').perform(context)
    if profile not in ('native', 'gazebo_mid360'):
        raise ValueError('unsupported FAST-LIO sensor_profile: ' + profile)
    files = [PathJoinSubstitution([
        get_package_share_directory('fast_lio'), 'config', selected_config])]
    if profile == 'gazebo_mid360':
        files.append(PathJoinSubstitution([
            get_package_share_directory('easyfly_fast_lio_integration'),
            'config', 'gazebo_mid360_extrinsics.yaml']))
    return [Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[*files,
            dict(overrides, use_sim_time=LaunchConfiguration('use_sim_time'))],
        output='screen',
    )]


def _launch_map_odom_tf(context):
    """Resolve the clock before this include's scope is popped by a parent."""
    simulation = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool).evaluate(context)
    parameters = [{"use_sim_time": simulation}]
    if LaunchConfiguration('sensor_profile', default='native').perform(context) == 'gazebo_mid360':
        parameters.append({"body_to_base_translation": [0.011, 0.02329, -0.12412]})
    container = ComposableNodeContainer(
        name="fastlio_integration_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="easyfly_fast_lio_integration",
                plugin="easyfly_fast_lio_integration::MapOdomTf",
                name="map_odom_tf",
                parameters=parameters,
            ),
        ],
        output="screen",
    )
    return [TimerAction(period=5.0, actions=[container])]


def generate_launch_description():
    # This argument intentionally has a package-specific name.  Included launch
    # descriptions share the parent launch context, so the generic
    # ``config_file`` argument from the detection stack could otherwise replace
    # FAST-LIO's default with an empty string.
    # Keep the old standalone ``config_file:=...`` spelling as a compatibility
    # alias.  The parent perception launch scopes this alias away from the
    # detection launch, so a detection config can never select FAST-LIO's file.

    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true')
    declare_config = DeclareLaunchArgument(
        'fastlio_config_file', default_value='mid360.yaml',
        description='fast_lio config (位于 fast_lio/config/)')
    declare_legacy_config = DeclareLaunchArgument(
        'config_file', default_value='',
        description='兼容旧版参数名；非空时覆盖 fastlio_config_file')

    fast_lio_node = OpaqueFunction(function=_launch_fast_lio)

    # map→odom TF (延后启动: 等 fast_lio 出 Odometry)
    map_odom_delayed = OpaqueFunction(function=_launch_map_odom_tf)

    return LaunchDescription([
        declare_sim_time,
        DeclareLaunchArgument('sensor_profile', default_value='native',
                              choices=['native', 'gazebo_mid360'],
                              description='Select physical sensor extrinsics explicitly'),
        DeclareLaunchArgument('fastlio_options_json', default_value='',
                              description='Validated FAST-LIO input and mapping overrides'),
        declare_config,
        declare_legacy_config,
        fast_lio_node,
        map_odom_delayed,
    ])
