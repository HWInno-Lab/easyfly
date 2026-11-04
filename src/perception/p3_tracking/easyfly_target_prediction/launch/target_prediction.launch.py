# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_target_prediction/licenses/.
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
            Node(
                package='easyfly_target_prediction',
                executable='target_prediction_node',
                parameters=[{"perception_version": ParameterValue(LaunchConfiguration("perception_version", default="1"), value_type=int)},
                    {
                        'use_sim_time': ParameterValue(
                            LaunchConfiguration('use_sim_time'),
                            value_type=bool,
                        )
                    }
                ],
                output='screen',
            ),
        ]
    )
