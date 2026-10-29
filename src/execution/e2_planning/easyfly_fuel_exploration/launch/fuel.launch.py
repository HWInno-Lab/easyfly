"""Starts an idle task server; clients must explicitly request exploration."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('uav_state_topic', default_value='/uav_state'),
        DeclareLaunchArgument('evidence_directory'),
        DeclareLaunchArgument('extended_profile', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('native_max_speed', default_value='1.2'),
        Node(package='easyfly_fuel_exploration', executable='fuel_exploration', output='screen',
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                          'evidence_directory': LaunchConfiguration('evidence_directory'),
                          'extended_profile': ParameterValue(LaunchConfiguration('extended_profile'), value_type=bool),
                          'native_max_speed': ParameterValue(LaunchConfiguration('native_max_speed'), value_type=float)}],
             remappings=[('/uav_state', LaunchConfiguration('uav_state_topic'))])])
