# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_camera_calib/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project
#

"""P1 标定节点启动: 静态 TF 发布 + 内参校验."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from easyfly_launch.composition import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        name="calib_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="easyfly_camera_calib",
                plugin="easyfly_camera_calib::CalibTfPublisher",
                name="calib_tf_publisher",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "calib_file": LaunchConfiguration("calib_file"),
                    }
                ],
            ),
            ComposableNode(
                package="easyfly_camera_calib",
                plugin="easyfly_camera_calib::CameraInfoChecker",
                name="camera_info_checker",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "calib_file": LaunchConfiguration("calib_file"),
                    }
                ],
            ),
        ],
        output="screen",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("calib_file", default_value=""),
            container,
        ]
    )
