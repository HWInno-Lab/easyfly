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

"""D2 service facade only; no automatic motion and no duplicate executor."""
from ament_index_python.packages import get_package_share_directory
from easyfly_params.configuration import (
    PROFILES,
    load_profile,
    ensure_launchable,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from pathlib import Path


def service_nodes(context):
    profile = LaunchConfiguration('profile').perform(context)
    ensure_launchable(load_profile(profile))
    clock = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    return [
        Node(
            package='easyfly_params',
            executable='parameter_manager',
            output='screen',
            parameters=[clock, {'profile': profile}],
        ),
        Node(
            package='easyfly_perception_srvs',
            executable='perception_services',
            output='screen',
            parameters=[clock, {'perception_version': ParameterValue(LaunchConfiguration('perception_version', default='1'), value_type=int)}],
        ),
        Node(
            package='easyfly_mission_srvs',
            executable='mission_services',
            output='screen',
            parameters=[
                clock,
                {
                    'mission_directory': LaunchConfiguration(
                        'mission_directory'
                    )
                },
            ],
        ),
    ]


def generate_launch_description():
    share = get_package_share_directory('easyfly_mission_fsm')
    executor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(share + '/launch/mission_fsm.launch.py'),
        condition=IfCondition(LaunchConfiguration('start_mission_executor')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument('perception_version', default_value='1'),
            DeclareLaunchArgument('use_sim_time', default_value='true'),
            DeclareLaunchArgument(
                'profile', default_value='navigation_dev', choices=PROFILES
            ),
            DeclareLaunchArgument(
                'mission_directory',
                default_value=str(
                    Path(share + '/missions/waypoint_demo.mission')
                    .resolve(strict=True)
                    .parent
                ),
            ),
            DeclareLaunchArgument(
                'start_mission_executor',
                default_value='false',
                description='Enable only if /execute_mission is not already running',
            ),
            OpaqueFunction(function=service_nodes),
            executor,
        ]
    )
