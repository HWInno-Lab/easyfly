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

#
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#

"""
navigate.launch.py — easyfly_global_planner 启动 (M4.2).

用法:
  ros2 launch easyfly_global_planner navigate.launch.py   # 仿真 (use_sim_time=true)

节点常驻 (idle 无副作用), 导航经 /navigate action 发起。
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="仿真时钟 (实机设 false)"
    )

    backend_arg = DeclareLaunchArgument(
        "localization_backend",
        default_value="fast_lio",
        choices=["fast_lio", "openvins", "fast_livo2", "orbslam3", "disabled"],
    )
    vio_arg = DeclareLaunchArgument(
        "openvins_navigation", default_value="false", choices=["true", "false"]
    )
    vio = PythonExpression(
        [
            "'",
            LaunchConfiguration("localization_backend"),
            "' == 'openvins' and '",
            LaunchConfiguration("openvins_navigation"),
            "' == 'true'",
        ]
    )
    livo = PythonExpression(
        [
            "'",
            LaunchConfiguration("localization_backend"),
            "' == 'fast_livo2' and '",
            LaunchConfiguration("fastlivo_navigation"),
            "' == 'true'",
        ]
    )
    orb = PythonExpression(
        [
            "'",
            LaunchConfiguration("localization_backend"),
            "' == 'orbslam3' and '",
            LaunchConfiguration("orbslam3_navigation"),
            "' == 'true'",
        ]
    )
    selected = PythonExpression(["(", vio, ") or (", livo, ") or (", orb, ")"])
    enabled = IfCondition(
        PythonExpression(
            [
                "'",
                LaunchConfiguration("localization_backend"),
                "' == 'fast_lio' or (",
                selected,
                ")",
            ]
        )
    )
    state_topic = PythonExpression(
        [
            "'/localization/' + '",
            LaunchConfiguration("localization_backend"),
            "' + '/navigation_state' if ",
            selected,
            " else '/uav_state'",
        ]
    )
    node = Node(
        condition=enabled,
        package="easyfly_global_planner",
        executable="global_planner_node",
        name="easyfly_global_planner",
        output="screen",
        remappings=[("/uav_state", state_topic)],
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "action_name": "/navigate_astar",
                "config_file": os.path.join(
                    get_package_share_directory("easyfly_global_planner"),
                    "config",
                    "navigate.yaml",
                ),
            }
        ],
    )

    coordinator = Node(
        package="easyfly_global_planner",
        executable="navigation_coordinator_node",
        name="navigation_coordinator",
        output="screen",
        remappings=[("/uav_state", state_topic)],
        parameters=[
            os.path.join(
                get_package_share_directory("easyfly_global_planner"), "config", "coordinator.yaml"
            ),
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "strategy": LaunchConfiguration("strategy"),
                "gcopter_segmented_navigation": ParameterValue(
                    LaunchConfiguration("gcopter_dynamic_navigation"), value_type=bool),
                "local_goal_distance": ParameterValue(PythonExpression([
                    "4.0 if '", LaunchConfiguration("gcopter_dynamic_navigation"),
                    "' == 'true' else 3.0"]), value_type=float),
                "goal_timeout": ParameterValue(PythonExpression([
                    "300.0 if '", LaunchConfiguration("gcopter_dynamic_navigation"),
                    "' == 'true' else 120.0"]), value_type=float),
                "max_replans": ParameterValue(PythonExpression([
                    "8 if '", LaunchConfiguration("gcopter_dynamic_navigation"),
                    "' == 'true' else 3"]), value_type=int),
                "default_speed": ParameterValue(
                    PythonExpression(
                        [
                            "0.6 if '",
                            LaunchConfiguration("strategy"),
                            "' in ('gcopter','super') else 2.0",
                        ]
                    ),
                    value_type=float,
                ),
                "session_service": PythonExpression(
                    [
                        "'/planning/' + '",
                        LaunchConfiguration("strategy"),
                        "' + '_session' if '",
                        LaunchConfiguration("strategy"),
                        "' in ('gcopter','super') else '/planning/navigation_session'",
                    ]
                ),
                "localization_backend": LaunchConfiguration("localization_backend"),
                "openvins_navigation": ParameterValue(vio, value_type=bool),
                "fastlivo_navigation": ParameterValue(livo, value_type=bool),
                "orbslam3_navigation": ParameterValue(orb, value_type=bool),
            },
        ],
    )
    supervisor = Node(
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("strategy"),
                    "' not in ('gcopter','super') and ('",
                    LaunchConfiguration("localization_backend"),
                    "' == 'fast_lio' or (",
                    selected,
                    "))",
                ]
            )
        ),
        package="easyfly_obstacle_avoidance",
        executable="obstacle_supervisor_node",
        name="obstacle_supervisor",
        output="screen",
        remappings=[("/uav_state", state_topic)],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )
    strategy_arg = DeclareLaunchArgument(
        "strategy", default_value="astar", choices=["astar", "ego", "combined", "gcopter", "super"]
    )
    return LaunchDescription(
        [
            use_sim_time_arg,
            strategy_arg,
            DeclareLaunchArgument(
                "gcopter_dynamic_navigation", default_value="false", choices=["true", "false"]),
            backend_arg,
            vio_arg,
            DeclareLaunchArgument(
                "orbslam3_navigation", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "fastlivo_navigation", default_value="false", choices=["true", "false"]
            ),
            node,
            coordinator,
            supervisor,
        ]
    )
