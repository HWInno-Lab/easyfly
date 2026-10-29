# Copyright 2026 EasyFly Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
