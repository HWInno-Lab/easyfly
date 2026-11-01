"""Starts an idle task server; clients must explicitly request exploration."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def fuel_node(context):
    parameter_file = LaunchConfiguration("parameter_file").perform(context)
    parameters = [parameter_file] if parameter_file else []
    parameters.append(
        {
            "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
            "evidence_directory": LaunchConfiguration("evidence_directory"),
            "extended_profile": ParameterValue(
                LaunchConfiguration("extended_profile"), value_type=bool
            ),
        }
    )
    speed = LaunchConfiguration("native_max_speed").perform(context)
    if speed:
        parameters[-1]["native_max_speed"] = float(speed)
    elif not parameter_file:
        parameters[-1]["native_max_speed"] = 1.2
    state_topic = LaunchConfiguration("uav_state_topic").perform(context)
    remappings = [("/uav_state", state_topic)] if state_topic else []
    return [
        Node(
            package="easyfly_fuel_exploration",
            executable="fuel_exploration",
            output="screen",
            parameters=parameters,
            remappings=remappings,
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("parameter_file", default_value=""),
            DeclareLaunchArgument("uav_state_topic", default_value=""),
            DeclareLaunchArgument("evidence_directory"),
            DeclareLaunchArgument(
                "extended_profile", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument("native_max_speed", default_value=""),
            OpaqueFunction(function=fuel_node),
        ]
    )
