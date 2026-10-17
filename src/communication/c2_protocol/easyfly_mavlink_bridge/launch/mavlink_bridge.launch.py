"""
mavlink_bridge.launch.py — 启动 mavros2 + easyfly_mavlink_bridge

用法:
  # 仿真 (UDP 连接 PX4 SITL)
  ros2 launch easyfly_mavlink_bridge mavlink_bridge.launch.py fcu_url:=udp://:14540@127.0.0.1:14555 use_sim_time:=true

  # 真机 (串口连接飞控)
  ros2 launch easyfly_mavlink_bridge mavlink_bridge.launch.py fcu_url:=/dev/ttyUSB0:921600 use_sim_time:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    fcu_url_arg = DeclareLaunchArgument(
        "fcu_url",
        default_value="udp://:14540@127.0.0.1:14555",
        description="MAVLink connection: udp://... for SITL, serial:///dev/... for real")

    gcs_url_arg = DeclareLaunchArgument(
        "gcs_url", default_value="",
        description="Optional bidirectional MAVROS GCS forwarding endpoint")
    target_system_arg = DeclareLaunchArgument(
        "target_system", default_value="1", description="Expected FCU MAVLink system ID")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use /clock (true for simulation, false for real hardware)")
    start_bridge_arg = DeclareLaunchArgument(
        "start_bridge", default_value="true", choices=["true", "false"],
        description="Disable only when an explicit owner starts its own bridge instance")

    fcu_url = LaunchConfiguration("fcu_url")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # MAVROS' PX4 config enables MAVLink TIMESYNC by default.  That exchange
    # is useful for real hardware, but its FCU/ROS epoch arithmetic is not
    # meaningful for PX4 lockstep simulation.  Derive the policy from the
    # clock-domain switch so a simulation launch cannot accidentally retain
    # the hardware defaults.  The arguments remain overridable for deliberate
    # diagnostics, while the safe defaults are automatic.
    timesync_mode_arg = DeclareLaunchArgument(
        "timesync_mode",
        default_value=PythonExpression([
            "'NONE' if '", use_sim_time, "'.lower() == 'true' else 'MAVLINK'"
        ]),
        description="MAVROS time plugin mode (simulation default: NONE)")
    timesync_rate_arg = DeclareLaunchArgument(
        "timesync_rate",
        default_value=PythonExpression([
            "'0.0' if '", use_sim_time, "'.lower() == 'true' else '10.0'"
        ]),
        description="MAVROS TIMESYNC request rate in Hz")
    system_time_rate_arg = DeclareLaunchArgument(
        "system_time_rate",
        default_value=PythonExpression([
            "'0.0' if '", use_sim_time, "'.lower() == 'true' else '1.0'"
        ]),
        description="MAVROS SYSTEM_TIME transmit rate in Hz")

    use_sim_time_value = ParameterValue(use_sim_time, value_type=bool)
    mavros_use_sim_time_file = ParameterFile(
        os.path.join(get_package_share_directory("easyfly_mavlink_bridge"),
                     "config", "mavros_use_sim_time.yaml"),
        allow_substs=True,
    )

    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        output="screen",
        parameters=[
            {
                "fcu_url": fcu_url,
                "gcs_url": LaunchConfiguration("gcs_url"),
                "tgt_system": ParameterValue(LaunchConfiguration("target_system"), value_type=int),
            },
            # MAVROS's wildcard config is required for /mavros/time and peers.
            mavros_use_sim_time_file,
        ],
        ros_arguments=["--params-file",
            os.path.join(get_package_share_directory("mavros"),
                         "launch", "px4_config.yaml")],
    )

    # ---- easyfly_mavlink_bridge ----
    # use_sim_time: /uav_state.header.stamp 是下游 TF (odom→base_link) 的时间戳来源,
    # 仿真中必须用 sim 时钟, 否则 TF 时间戳超前于 /clock → RViz/查询 extrapolation 失败
    bridge_node = Node(
        package="easyfly_mavlink_bridge",
        executable="mavlink_bridge_node",
        output="screen",
        name="mavlink_bridge",
        condition=IfCondition(LaunchConfiguration("start_bridge")),
        parameters=[
            os.path.join(get_package_share_directory("easyfly_mavlink_bridge"),
                         "config", "mavlink_bridge.yaml"),
            {"use_sim_time": use_sim_time_value},
        ],
    )

    # ---- easyfly_rc_receiver ----
    rc_node = Node(
        package="easyfly_rc_receiver",
        executable="rc_receiver_node",
        output="screen",
        name="rc_receiver",
        parameters=[{"use_sim_time": use_sim_time_value}],
    )

    return LaunchDescription([
        fcu_url_arg,
        gcs_url_arg,
        target_system_arg,
        use_sim_time_arg,
        start_bridge_arg,
        timesync_mode_arg,
        timesync_rate_arg,
        system_time_rate_arg,
        mavros_node,
        bridge_node,
        rc_node,
    ])
