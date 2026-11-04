# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_trajectory_optimizer/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project
"""Idle hardware mapper/planner. Calibration is mandatory, never a simulated preset."""
import os
from pathlib import Path
import stat

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml
from easyfly_trajectory_optimizer.hardware import validate_hardware_parameters


def hardware_nodes(context):
    parameter_file = LaunchConfiguration("parameter_file").perform(context)
    document = yaml.safe_load(Path(parameter_file).read_text(encoding="utf-8"))
    validate_hardware_parameters(document)
    directory = LaunchConfiguration("evidence_directory").perform(context)
    info = os.lstat(directory)
    if (
        not stat.S_ISDIR(info.st_mode)
        or info.st_uid != os.getuid()
        or stat.S_IMODE(info.st_mode) & 0o077
    ):
        raise ValueError("hardware evidence_directory must be an owner-private directory")
    state = LaunchConfiguration("uav_state_topic")
    return [
        Node(
            package="easyfly_trajectory_optimizer",
            executable="gcopter_voxel_map",
            name="gcopter_voxel_map",
            output="screen",
            parameters=[
                parameter_file,
                {
                    "use_sim_time": False,
                    "physical_livox": True,
                    "dynamic_clearing": False,
                    "rolling_xy": False,
                    "rays_topic": "/livox/lidar/rays",
                    "uninflated_topic": "/planning/fuel_raw_grid",
                },
            ],
            remappings=[("/planning/voxel_grid", "/planning/fuel_observed_grid")],
        ),
        Node(
            package="easyfly_trajectory_optimizer",
            executable="gcopter_navigation",
            name="gcopter_navigation",
            output="screen",
            parameters=[
                parameter_file,
                {
                    "use_sim_time": False,
                    "allow_idle_policy_map_epoch": True,
                    "dynamic_map": False,
                    "fuel_native_trajectory": False,
                    "evidence_directory": directory,
                },
            ],
            remappings=[("/uav_state", state)],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("parameter_file"),
            DeclareLaunchArgument("evidence_directory"),
            DeclareLaunchArgument("uav_state_topic", default_value="/uav_state"),
            OpaqueFunction(function=hardware_nodes),
        ]
    )
