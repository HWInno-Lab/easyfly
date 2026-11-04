# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_single_object_tracking/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project
#

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
