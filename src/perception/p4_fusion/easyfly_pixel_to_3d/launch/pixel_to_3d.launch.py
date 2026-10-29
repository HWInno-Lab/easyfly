# Copyright 2026 EasyFly Project
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

"""M5 像素转 3D 节点启动."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from easyfly_launch.composition import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


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
                parameters=[{"use_sim_time": True}],
            ),
        ],
        output="screen",
    )
    return LaunchDescription([process_nice, container])
