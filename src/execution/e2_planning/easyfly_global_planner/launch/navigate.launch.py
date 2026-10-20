"""
navigate.launch.py — easyfly_global_planner 启动 (M4.2)

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
import os


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="仿真时钟 (实机设 false)")

    backend_arg = DeclareLaunchArgument(
        'localization_backend', default_value='fast_lio',
        choices=['fast_lio', 'openvins', 'disabled'])
    enabled = IfCondition(PythonExpression([
        "'", LaunchConfiguration('localization_backend'), "' == 'fast_lio'"]))
    node = Node(
        condition=enabled,
        package="easyfly_global_planner",
        executable="global_planner_node",
        name="easyfly_global_planner",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "action_name": "/navigate_astar",
            "config_file": os.path.join(
                get_package_share_directory("easyfly_global_planner"),
                "config", "navigate.yaml"),
        }],
    )

    coordinator = Node(
        package="easyfly_global_planner",
        executable="navigation_coordinator_node",
        name="navigation_coordinator",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("easyfly_global_planner"),
                                 "config", "coordinator.yaml"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time"),
                     "strategy": LaunchConfiguration("strategy"),
                     "localization_backend": LaunchConfiguration("localization_backend")}],
    )
    supervisor = Node(
        condition=enabled,
        package="easyfly_obstacle_avoidance", executable="obstacle_supervisor_node",
        name="obstacle_supervisor", output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}])
    strategy_arg = DeclareLaunchArgument("strategy", default_value="astar",
                                         choices=["astar", "ego", "combined"])
    return LaunchDescription([
        use_sim_time_arg, strategy_arg, backend_arg, node, coordinator, supervisor])
