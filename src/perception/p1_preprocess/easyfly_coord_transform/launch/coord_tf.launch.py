# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_coord_transform/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project

"""P1 坐标变换节点启动: odom→base_link 动态 TF + /odom."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from easyfly_launch.composition import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        name="coord_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="easyfly_coord_transform",
                plugin="easyfly_coord_transform::OdomTfPublisher",
                name="odom_tf_publisher",
                parameters=[{"use_sim_time": ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool)}],
            ),
        ],
        output="screen",
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'), container])
