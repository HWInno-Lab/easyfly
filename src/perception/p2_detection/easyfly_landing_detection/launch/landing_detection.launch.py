# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_landing_detection/licenses/.
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
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument('selected_tag_id', default_value='-1'),
            Node(
                package='easyfly_landing_detection',
                executable='landing_detection_node',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': ParameterValue(
                            LaunchConfiguration('use_sim_time'),
                            value_type=bool,
                        ),
                        'selected_tag_id': ParameterValue(
                            LaunchConfiguration('selected_tag_id'),
                            value_type=int,
                        ),
                    }
                ],
            ),
        ]
    )
