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

"""
follow.launch.py — easyfly_target_tracking 启动 (M4.1).

用法:
  ros2 launch easyfly_target_tracking follow.launch.py    # 仿真 (use_sim_time=true)

说明: 节点常驻 (idle 时无副作用), 跟随通过 /track_target action 发起。
前置: perception 链 + flight_control 已启动, 无人机已起飞悬停。
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
        package="easyfly_target_tracking",
        executable="target_tracking_node",
        name="easyfly_target_tracking",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "config_file": os.path.join(
                get_package_share_directory("easyfly_target_tracking"),
                "config", "follow.yaml"),
        }],
    )

    return LaunchDescription([use_sim_time_arg, node])
