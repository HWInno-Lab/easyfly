# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_pointcloud_filter/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project

"""
pcf.launch.py — easyfly_pointcloud_filter 启动.

用法:
  ros2 launch easyfly_pointcloud_filter pcf.launch.py            # 仿真 (use_sim_time=true)
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="仿真时钟 (实机设 false)")

    node = Node(
        package="easyfly_pointcloud_filter",
        executable="pointcloud_filter_node",
        name="easyfly_pointcloud_filter",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "config_file": os.path.join(
                get_package_share_directory("easyfly_pointcloud_filter"),
                "config", "pointcloud_filter.yaml"),
        }],
    )

    return LaunchDescription([use_sim_time_arg, node])
