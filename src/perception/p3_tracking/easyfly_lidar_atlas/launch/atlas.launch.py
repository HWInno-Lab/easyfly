# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_lidar_atlas/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='fast_lio', choices=['fast_lio', 'fast_livo2']),
        DeclareLaunchArgument('load_atlas', default_value=''),
        DeclareLaunchArgument('save_atlas', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        Node(package='easyfly_lidar_atlas', executable='lidar_atlas', output='screen',
             parameters=[{name: ParameterValue(LaunchConfiguration(name), value_type=str)
                          for name in ('backend', 'load_atlas', 'save_atlas')},
                         {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}]),
    ])
