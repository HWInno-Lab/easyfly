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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('tracker_backend', default_value='csrt'),
            DeclareLaunchArgument('backend_parameters', default_value='{}'),
            DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
            DeclareLaunchArgument('perception_version', default_value='1'),
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            Node(
                package='easyfly_single_object_tracking',
                executable='single_object_tracking_node',
                output='screen',
                remappings=[('/camera/image_raw', LaunchConfiguration('image_topic'))],
                parameters=[
                    {
                        'tracker_backend': ParameterValue(LaunchConfiguration('tracker_backend'), value_type=str),
                        'backend_parameters': ParameterValue(LaunchConfiguration('backend_parameters'), value_type=str),
                        'perception_version': ParameterValue(LaunchConfiguration('perception_version'), value_type=int),
                        'use_sim_time': ParameterValue(
                            LaunchConfiguration('use_sim_time'),
                            value_type=bool,
                        )
                    }
                ],
            ),
        ]
    )
