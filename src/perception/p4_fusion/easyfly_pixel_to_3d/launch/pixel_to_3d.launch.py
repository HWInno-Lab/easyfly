# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_pixel_to_3d/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project

"""M5 像素转 3D 节点启动."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from easyfly_launch.composition import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    process_nice = DeclareLaunchArgument(
        'process_nice', default_value='0', choices=['0', '10'],
        description='CPU scheduling budget; simulation VIO uses 10 for perception work')
    container = ComposableNodeContainer(
        name="pixel_to_3d_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        prefix=PythonExpression(["'nice -n 10' if '", LaunchConfiguration('process_nice'),
                                 "' == '10' else ''"]),
        composable_node_descriptions=[
            ComposableNode(
                package="easyfly_pixel_to_3d",
                plugin="easyfly_pixel_to_3d::PixelTo3dNode",
                name="easyfly_pixel_to_3d",
                parameters=[{"perception_version": ParameterValue(LaunchConfiguration("perception_version", default="1"), value_type=int), "use_sim_time": ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool)}],
            ),
        ],
        output="screen",
    )
    return LaunchDescription([
        process_nice, DeclareLaunchArgument('use_sim_time', default_value='true'), container])
