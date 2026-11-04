# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_trajectory_optimizer/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project
#

"""Idle managed GCOPTER planner and source-time ray map; never sends flight goals."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    clock = {"use_sim_time": LaunchConfiguration("use_sim_time")}
    dynamic = ParameterValue(LaunchConfiguration("dynamic_navigation"), value_type=bool)
    extended = ParameterValue(LaunchConfiguration("fuel_native_trajectory"), value_type=bool)
    changing = ParameterValue(PythonExpression(["'", LaunchConfiguration("dynamic_navigation"),
        "' == 'true' or '", LaunchConfiguration("fuel_native_trajectory"), "' == 'true'"]), value_type=bool)
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("uav_state_topic", default_value="/uav_state"),
            DeclareLaunchArgument("evidence_directory", default_value=""),
            DeclareLaunchArgument("dynamic_navigation", default_value="false", choices=["true", "false"]),
            DeclareLaunchArgument("fuel_native_trajectory", default_value="false", choices=["true", "false"]),
            DeclareLaunchArgument("voxel_output_topic", default_value="/planning/voxel_grid"),
            DeclareLaunchArgument("uninflated_topic", default_value=""),
            DeclareLaunchArgument("allow_idle_policy_map_epoch", default_value="false"),
            Node(
                package="easyfly_trajectory_optimizer",
                executable="gcopter_voxel_map",
                output="screen",
                parameters=[clock, {"uninflated_topic": LaunchConfiguration("uninflated_topic"),
                                    "dynamic_clearing": changing, "rolling_xy": dynamic}],
                remappings=[("/planning/voxel_grid", LaunchConfiguration("voxel_output_topic"))],
            ),
            Node(
                package="easyfly_trajectory_optimizer",
                executable="gcopter_navigation",
                output="screen",
                parameters=[
                    clock,
                    {"evidence_directory": LaunchConfiguration("evidence_directory"),
                     "dynamic_map": changing, "fuel_native_trajectory": extended},
                    {"allow_idle_policy_map_epoch": ParameterValue(
                        LaunchConfiguration("allow_idle_policy_map_epoch"), value_type=bool)},
                ],
                remappings=[("/uav_state", LaunchConfiguration("uav_state_topic"))],
            ),
        ]
    )
