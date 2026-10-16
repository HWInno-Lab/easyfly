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
semantic_map.launch.py — easyfly_semantic_map 启动.

用法:
  ros2 launch easyfly_semantic_map semantic_map.launch.py     # 仿真 (use_sim_time=true)
"""

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="仿真时钟 (实机设 false)"
    )

    capture_arg = DeclareLaunchArgument(
        'capture_shutdown', default_value='false',
        description='Opt-in semantic-only stack capture; changes timing, not a normal acceptance')
    ground_arg = DeclareLaunchArgument(
        'ground_model_frame', default_value='',
        description='Ground model frame; empty preserves configured behavior')
    grid_arg = DeclareLaunchArgument(
        'grid_footprint_mode', default_value='cluster_boxes',
        description='Occupancy geometry: cluster_boxes or observed_cells')
    return LaunchDescription([
        use_sim_time_arg, capture_arg, ground_arg, grid_arg, OpaqueFunction(function=_node)])


def _node(context):
    """Keep normal startup untouched and require an explicit diagnostic selection."""
    capture = LaunchConfiguration('capture_shutdown').perform(context).strip().lower()
    if capture not in ('true', 'false'):
        raise ValueError('capture_shutdown requires true or false')
    prefix = None
    if capture == 'true':
        prefix = os.path.join(get_package_prefix('easyfly_semantic_map'),
                              'lib/easyfly_semantic_map/semantic_shutdown_capture')
    return [Node(
        package="easyfly_semantic_map",
        executable="semantic_map_node",
        name="easyfly_semantic_map",
        output="screen",
        prefix=prefix,
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "grid_footprint_mode": ParameterValue(
                    LaunchConfiguration("grid_footprint_mode"), value_type=str),
                "ground_model_frame": ParameterValue(
                    LaunchConfiguration("ground_model_frame", default=''), value_type=str),
                "config_file": os.path.join(
                    get_package_share_directory("easyfly_semantic_map"),
                    "config",
                    "semantic_map.yaml",
                ),
            }
        ],
    )]
