# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_semantic_map/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project
#

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
    height_arg = DeclareLaunchArgument(
        'grid_height_limit_m', default_value='0.0',
        description='Opt-in 2D projection height above trusted ground; zero keeps all heights')
    state_arg = DeclareLaunchArgument('uav_state_topic', default_value='/uav_state')
    return LaunchDescription([
        use_sim_time_arg, capture_arg, ground_arg, grid_arg, height_arg, state_arg,
        OpaqueFunction(function=_node)])


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
        remappings=[('/uav_state', LaunchConfiguration('uav_state_topic'))],
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "grid_footprint_mode": ParameterValue(
                    LaunchConfiguration("grid_footprint_mode"), value_type=str),
                "grid_height_limit_m": ParameterValue(
                    LaunchConfiguration("grid_height_limit_m", default='0.0'), value_type=float),
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
