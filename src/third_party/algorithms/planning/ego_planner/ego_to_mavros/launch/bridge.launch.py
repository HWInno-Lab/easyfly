"""
bridge.launch.py — 启动 ego_to_mavros 桥接节点

可独立启动，也可被 single_run_real.launch.py include。
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    drone_id_arg = DeclareLaunchArgument(
        'drone_id', default_value='0',
        description='Drone ID for topic namespacing')

    setpoint_rate_arg = DeclareLaunchArgument(
        'setpoint_rate', default_value='50.0',
        description='Setpoint publishing rate (Hz)')

    cmd_timeout_arg = DeclareLaunchArgument(
        'cmd_timeout', default_value='0.5',
        description='PositionCommand timeout before fallback (seconds)')

    enable_yaw_arg = DeclareLaunchArgument(
        'enable_yaw_cmd', default_value='false',
        description='Enable yaw commands from ego_planner')

    bridge_node = Node(
        package='ego_to_mavros',
        executable='ego_to_mavros_node',
        name='ego_to_mavros_bridge',
        output='screen',
        parameters=[{
            'drone_id': ParameterValue(LaunchConfiguration('drone_id'), value_type=str),
            'setpoint_rate': LaunchConfiguration('setpoint_rate'),
            'cmd_timeout': LaunchConfiguration('cmd_timeout'),
            'enable_yaw_cmd': LaunchConfiguration('enable_yaw_cmd'),
            'auto_trigger': False,
        }],
    )

    return LaunchDescription([
        drone_id_arg,
        setpoint_rate_arg,
        cmd_timeout_arg,
        enable_yaw_arg,
        bridge_node,
    ])
