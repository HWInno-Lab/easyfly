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

"""Explicit D5 simulation chain; no default flight-node restart or motion."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from easyfly_precision_landing.configuration import validate_configuration


def nodes(context):
    """Materialize only an explicitly supplied, simulation-only configuration."""
    config = LaunchConfiguration("config_file").perform(context)
    if (
        not config
        or LaunchConfiguration("use_sim_time").perform(context) != "true"
    ):
        raise ValueError(
            "explicit config_file and use_sim_time=true required; hardware unverified"
        )
    document = validate_configuration(config)
    clock = {"use_sim_time": True}
    entities = [
        Node(
            package="easyfly_apriltag_detection",
            executable="apriltag_detection_node",
            name="apriltag_detection",
            parameters=[config, clock],
            output="screen",
            remappings=[
                ("/camera/image_raw", LaunchConfiguration("image_topic")),
                (
                    "/camera/camera_info",
                    LaunchConfiguration("camera_info_topic"),
                ),
            ],
        ),
        Node(
            package="easyfly_landing_detection",
            executable="landing_detection_node",
            name="landing_detection",
            parameters=[config, clock],
            output="screen",
        ),
        Node(
            package="easyfly_precision_landing",
            executable="landing_transform_node",
            name="landing_transform",
            parameters=[config, clock],
            output="screen",
        ),
        Node(
            package="easyfly_precision_landing",
            executable="landing_gateway_node",
            name="landing_gateway",
            parameters=[config, clock],
            output="screen",
        ),
    ]
    if document['landing_transform']['ros__parameters'].get('dynamic_camera', False):
        if 'landing_camera_mount' not in document:
            raise ValueError('dynamic camera requires explicit measured mount configuration')
        entities.append(Node(package='easyfly_precision_landing',
                             executable='landing_camera_mount_node',
                             name='landing_camera_mount', parameters=[config, clock],
                             output='screen'))
    if (
        LaunchConfiguration("start_flight_controller").perform(context)
        == "true"
    ):
        entities.append(
            Node(
                package="easyfly_flight_control",
                executable="flight_control_node",
                name="flight_controller",
                parameters=[config, clock],
                output="screen",
            )
        )
    return entities


def generate_launch_description():
    """Keep all existing profiles untouched; this opt-in chain sends no goal."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=""),
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", choices=["true"]
            ),
            DeclareLaunchArgument(
                "image_topic", default_value="/camera_down/color/image_raw"
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/camera_down/color/camera_info",
            ),
            DeclareLaunchArgument(
                "start_flight_controller",
                default_value="false",
                choices=["true", "false"],
                description="Only true when no flight_controller is already running",
            ),
            OpaqueFunction(function=nodes),
        ]
    )
