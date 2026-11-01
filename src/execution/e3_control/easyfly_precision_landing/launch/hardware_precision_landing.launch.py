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
"""Measured fixed downward camera pipeline; never starts flight control or a goal."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from easyfly_precision_landing.hardware_configuration import validate_hardware_configuration


def nodes(context):
    def value(key):
        return LaunchConfiguration(key).perform(context)

    if value('use_sim_time') != 'false':
        raise ValueError('hardware precision requires physical time')
    config = value('config_file')
    validate_hardware_configuration(config)
    image, info = value('image_topic'), value('camera_info_topic')
    if not image.startswith('/') or not info.startswith('/'):
        raise ValueError('explicit absolute downward camera topics required')
    return [
        Node(
            package=package,
            executable=executable,
            name=name,
            parameters=[config, {'use_sim_time': False}],
            output='screen',
            remappings=[('/camera/image_raw', image), ('/camera/camera_info', info)]
            if name == 'apriltag_detection'
            else [],
        )
        for package, executable, name in (
            ('easyfly_apriltag_detection', 'apriltag_detection_node', 'apriltag_detection'),
            ('easyfly_landing_detection', 'landing_detection_node', 'landing_detection'),
            ('easyfly_precision_landing', 'landing_transform_node', 'landing_transform'),
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('config_file'),
            DeclareLaunchArgument('use_sim_time', default_value='false', choices=['false']),
            DeclareLaunchArgument('image_topic', default_value='/camera_down/color/image_raw'),
            DeclareLaunchArgument(
                'camera_info_topic', default_value='/camera_down/color/camera_info'
            ),
            OpaqueFunction(function=nodes),
        ]
    )
