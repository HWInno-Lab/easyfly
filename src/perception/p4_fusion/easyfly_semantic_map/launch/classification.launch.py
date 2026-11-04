# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_semantic_map/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project
#

"""Independent classification copies only, not a new occupancy producer."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('use_sim_time', default_value='true'),
            Node(
                package='easyfly_semantic_map',
                executable='obstacle_classifier',
                output='screen',
                parameters=[
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}
                ],
            ),
        ]
    )
