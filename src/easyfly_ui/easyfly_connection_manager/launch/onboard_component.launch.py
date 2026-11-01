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
"""Launch one frozen onboard adapter with physical time and no simulator defaults."""

import json
import os
import stat
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from easyfly_launch.composition import ComposableNodeContainer

from easyfly_connection_manager.onboard_profiles import CATALOG, NODE_NAMES, validate


# Keep shutdown cooperative. The owner reports stop_unknown after 15 seconds;
# launch must not subsequently escalate to TERM/KILL behind that receipt.
COOPERATIVE_SHUTDOWN = dict(sigterm_timeout="1000000000000", sigkill_timeout="1000000000000")


def create_component(context):
    """Resolve catalog identities only; the caller cannot supply a command or package."""
    path = Path(LaunchConfiguration("component_file").perform(context))
    document = json.loads(path.read_text())
    name, entry = document["kind"], document["component"]
    validate(
        dict(schema_version=1, name="frozen component", autostart=False, components={name: entry}),
        require_files=True,
        standalone=True,
    )
    spec = CATALOG[name]
    if not entry["enabled"]:
        raise ValueError("disabled onboard component cannot be launched")
    parameters = dict(entry["parameters"], use_sim_time=False)
    if name == "mission_services":
        parameters["mission_directory"] = document["mission_directory"]
        parameters["enable_mission_import"] = False
    if name == 'target_tracking':
        parameters = dict(config_file=parameters['config_file'], use_sim_time=False)
    remappings = list(entry["remappings"].items())
    files = [entry["parameter_file"]] if entry["parameter_file"] else []
    if name == "mission_fsm":
        return [
            Node(
                **COOPERATIVE_SHUTDOWN,
                package="easyfly_mission_fsm",
                executable=executable,
                name=node_name,
                parameters=[*files, dict(parameters, battery_invalid_grace_s=5.0)],
                output="screen",
            )
            for executable, node_name in (
                ("mission_fsm_node", "mission_fsm"),
                ("search_area_node", "search_area"),
            )
        ]
    if name == 'navigation' and parameters.get('strategy') == 'gcopter':
        parameters['session_service'] = '/planning/gcopter_session'
    if name == "navigation":
        nodes = [
            Node(
                **COOPERATIVE_SHUTDOWN,
                package="easyfly_global_planner",
                executable="global_planner_node",
                name="easyfly_global_planner",
                remappings=remappings,
                output="screen",
                parameters=[*files, dict(parameters, action_name="/navigate_astar")],
            ),
            Node(
                **COOPERATIVE_SHUTDOWN,
                package="easyfly_global_planner",
                executable="navigation_coordinator_node",
                name="navigation_coordinator",
                remappings=remappings,
                output="screen",
                parameters=[
                    str(
                        Path(get_package_share_directory("easyfly_global_planner"))
                        / "config/coordinator.yaml"
                    ),
                    *files,
                    parameters,
                ],
            ),
            Node(
                **COOPERATIVE_SHUTDOWN,
                package="easyfly_obstacle_avoidance",
                executable="obstacle_supervisor_node",
                name="obstacle_supervisor",
                remappings=remappings,
                output="screen",
                parameters=[*files, {"use_sim_time": False}],
            ),
        ]
        return [nodes[1]] if parameters.get('strategy') == 'gcopter' else nodes
    if spec["executable"].endswith(".launch.py"):
        # Native launch adapters receive an explicit physical-clock switch in an
        # isolated context. They never inherit a Gazebo sensor profile.
        arguments = {
            key: str(value).lower() if type(value) is bool else str(value)
            for key, value in parameters.items()
        }
        if name in ('gcopter', 'fuel'):
            arguments['parameter_file'] = entry['parameter_file']
            evidence = path.parent / 'exploration_evidence'
            try:
                evidence.mkdir(mode=0o700)
            except FileExistsError:
                pass
            info = os.lstat(evidence)
            if (
                not stat.S_ISDIR(info.st_mode)
                or info.st_uid != os.geteuid()
                or info.st_mode & 0o077
            ):
                raise ValueError('exploration evidence directory must be owner-private')
            arguments['evidence_directory'] = str(evidence)
            if name == 'fuel':
                arguments['extended_profile'] = 'false'
        if name in ("realsense", "down_camera") and entry["parameter_file"]:
            arguments["config_file"] = entry["parameter_file"]
        include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    Path(get_package_share_directory(spec["package"]))
                    / "launch"
                    / spec["executable"]
                )
            ),
            launch_arguments=arguments.items(),
        )
        return [
            GroupAction(
                scoped=True,
                forwarding=False,
                actions=[
                    SetLaunchConfiguration(key, value)
                    for key, value in COOPERATIVE_SHUTDOWN.items()
                ]
                + [include],
            )
        ]
    if spec["plugin"]:
        description = ComposableNode(
            package=spec["package"],
            plugin=spec["plugin"],
            name=NODE_NAMES[name],
            parameters=[*files, parameters],
            remappings=remappings,
        )
        return [
            ComposableNodeContainer(
                **COOPERATIVE_SHUTDOWN,
                package="rclcpp_components",
                executable="component_container",
                name="onboard_" + name + "_container",
                namespace="",
                composable_node_descriptions=[description],
                output="screen",
            )
        ]
    node_name = NODE_NAMES.get(name)
    return [
        Node(
            **COOPERATIVE_SHUTDOWN,
            package=spec["package"],
            executable=spec["executable"],
            name=node_name,
            parameters=[*files, parameters],
            remappings=remappings,
            output="screen",
        )
    ]


def generate_launch_description():
    """Require an owner-generated component document."""
    return LaunchDescription(
        [DeclareLaunchArgument("component_file"), OpaqueFunction(function=create_component)]
    )
