# Copyright 2026 EasyFly Project
#
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
