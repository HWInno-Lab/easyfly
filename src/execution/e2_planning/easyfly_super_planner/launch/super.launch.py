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

"""Idle managed SUPER planner and source-time ray map; never sends flight goals."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    clock = {"use_sim_time": LaunchConfiguration("use_sim_time")}
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("uav_state_topic", default_value="/uav_state"),
            DeclareLaunchArgument("evidence_directory", default_value=""),
            DeclareLaunchArgument("execute_committed", default_value="false"),
            Node(
                package="easyfly_trajectory_optimizer",
                executable="gcopter_voxel_map",
                name="super_voxel_map",
                output="screen",
                parameters=[clock],
            ),
            Node(
                package="easyfly_super_planner",
                executable="super_navigation",
                output="screen",
                parameters=[
                    clock,
                    {"evidence_directory": LaunchConfiguration("evidence_directory"),
                     "execute_committed": ParameterValue(LaunchConfiguration("execute_committed"), value_type=bool)},
                ],
                remappings=[("/uav_state", LaunchConfiguration("uav_state_topic"))],
            ),
        ]
    )
