"""
single_run_real.launch.py — 真机部署顶层入口.

启动顺序:
  1. MAVROS (PX4 通信)
  2. D435i 深度相机
  3. ego_to_mavros 桥接节点
  4. ego_planner_node + traj_server
  5. RViz2 (可选)

使用方式:
  ros2 launch ego_planner single_run_real.launch.py
  ros2 launch ego_planner single_run_real.launch.py rviz:=false
  ros2 launch ego_planner single_run_real.launch.py fcu_url:=/dev/ttyUSB0:921600
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ──────────────────────────────────────────
    #  声明参数
    # ──────────────────────────────────────────
    drone_id_ = "0"

    # --- 通用 ---
    rviz_arg = DeclareLaunchArgument("rviz", default_value="true", description="Launch RViz2")
    fcu_url_arg = DeclareLaunchArgument(
        "fcu_url",
        default_value="/dev/ttyUSB0:921600",
        description="PX4 FCU serial port (e.g. /dev/ttyUSB0:921600 or udp://:14540@... )",
    )
    gcs_url_arg = DeclareLaunchArgument(
        "gcs_url", default_value="udp://@localhost", description="MAVROS GCS bridge URL"
    )

    # --- 地图 ---
    map_size_x_ = LaunchConfiguration("map_size_x_", default="30.0")
    map_size_y_ = LaunchConfiguration("map_size_y_", default="20.0")
    map_size_z_ = LaunchConfiguration("map_size_z_", default="3.0")

    # --- 飞行参数 ---
    max_vel_arg = DeclareLaunchArgument(
        "max_vel", default_value="1.0", description="Max velocity (m/s)"
    )
    max_acc_arg = DeclareLaunchArgument(
        "max_acc", default_value="3.0", description="Max acceleration (m/s^2)"
    )

    # --- 相机 ---
    cx_arg = DeclareLaunchArgument(
        "cx",
        default_value="321.0",
        description="Camera principal point cx (get from /camera/depth/camera_info)",
    )
    cy_arg = DeclareLaunchArgument(
        "cy", default_value="243.4", description="Camera principal point cy"
    )
    fx_arg = DeclareLaunchArgument(
        "fx", default_value="387.2", description="Camera focal length fx"
    )
    fy_arg = DeclareLaunchArgument(
        "fy", default_value="387.2", description="Camera focal length fy"
    )

    # ──────────────────────────────────────────
    #  1. MAVROS
    # ──────────────────────────────────────────
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory("mavros"), "/launch/px4.launch.py"]
        ),
        launch_arguments={
            "fcu_url": LaunchConfiguration("fcu_url"),
            "gcs_url": LaunchConfiguration("gcs_url"),
        }.items(),
    )

    # ──────────────────────────────────────────
    #  2. D435i 传感器
    # ──────────────────────────────────────────
    sensors_launch_path = os.path.join(
        get_package_share_directory("ego_planner"), "launch", "sensors.launch.py"
    )
    sensors_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(sensors_launch_path))

    # ──────────────────────────────────────────
    #  3. ego_to_mavros 桥接
    # ──────────────────────────────────────────
    bridge_launch_path = os.path.join(
        get_package_share_directory("ego_to_mavros"), "launch", "bridge.launch.py"
    )
    # 桥接需要等 MAVROS 先启动好（MAVROS 的话题 ready），延迟 3 秒
    bridge_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bridge_launch_path),
                launch_arguments={
                    "drone_id": drone_id_,
                    "setpoint_rate": "50.0",
                    "cmd_timeout": "0.5",
                    "enable_yaw_cmd": "false",
                }.items(),
            )
        ],
    )

    # ──────────────────────────────────────────
    #  4. ego_planner_node
    # ──────────────────────────────────────────
    ego_planner_node = Node(
        package="ego_planner",
        executable="ego_planner_node",
        name="drone_" + drone_id_ + "_ego_planner_node",
        output="screen",
        remappings=[
            # 里程计 — 来自 MAVROS 经 bridge 转发
            ("odom_world", "/drone_0_visual_slam/odom"),
            # 传感器 — D435i
            ("grid_map/cloud", "/camera/depth/color/points"),
            ("grid_map/depth", "/camera/depth/image_rect_raw"),
            ("grid_map/odom", "/drone_0_visual_slam/odom"),
            # planner 输出
            ("planning/bspline", "/drone_0_planning/bspline"),
            ("planning/data_display", "/drone_0_planning/data_display"),
            ("trajectory_ack", "/drone_0_planning/trajectory_ack"),
            ("planning/broadcast_bspline_from_planner", "/broadcast_bspline"),
            ("planning/broadcast_bspline_to_planner", "/broadcast_bspline"),
            ("optimal_list", "/drone_0_ego_planner_node/optimal_list"),
            ("grid_map/occupancy_inflate", "/drone_0_ego_planner_node/grid_map/occupancy_inflate"),
        ],
        parameters=[
            {
                # --- FSM ---
                "fsm/flight_type": 1,  # 手动 RViz 点击目标
                "fsm/thresh_replan_time": 1.0,
                "fsm/thresh_no_replan_meter": 1.0,
                "fsm/planning_horizon": 7.5,
                "fsm/planning_horizen_time": 3.0,
                "fsm/emergency_time": 1.0,
                "fsm/emergency_recovery_hold_time": 0.5,
                "fsm/emergency_recovery_stable_cycles": 5,
                "fsm/emergency_recovery_max_speed": 0.1,
                "fsm/emergency_recovery_clock_jump_limit": 1.0,
                "fsm/emergency_recovery_ack_timeout": 0.5,
                "fsm/emergency_recovery_ack_retry_period": 0.5,
                "fsm/emergency_recovery_ack_retry_max_period": 5.0,
                "fsm/traj_server_ack_topic": "trajectory_ack",
                "fsm/traj_server_node_name": "drone_0_traj_server",
                "fsm/traj_server_node_fqn": "/drone_0_traj_server",
                "fsm/realworld_experiment": True,  # 真机：需外部触发
                "fsm/fail_safe": True,
                "fsm/waypoint_num": 4,
                "fsm/waypoint0_x": 5.0,
                "fsm/waypoint0_y": 0.0,
                "fsm/waypoint0_z": 1.0,
                "fsm/waypoint1_x": -5.0,
                "fsm/waypoint1_y": 0.0,
                "fsm/waypoint1_z": 1.0,
                "fsm/waypoint2_x": 5.0,
                "fsm/waypoint2_y": 0.0,
                "fsm/waypoint2_z": 1.0,
                "fsm/waypoint3_x": -5.0,
                "fsm/waypoint3_y": 0.0,
                "fsm/waypoint3_z": 1.0,
                "fsm/waypoint4_x": 0.0,
                "fsm/waypoint4_y": 0.0,
                "fsm/waypoint4_z": 1.0,
                # --- Grid Map ---
                "grid_map/resolution": 0.1,
                "grid_map/map_size_x": map_size_x_,
                "grid_map/map_size_y": map_size_y_,
                "grid_map/map_size_z": map_size_z_,
                "grid_map/local_update_range_x": 5.5,
                "grid_map/local_update_range_y": 5.5,
                "grid_map/local_update_range_z": 4.5,
                "grid_map/obstacles_inflation": 0.099,
                "grid_map/local_map_margin": 10,
                "grid_map/ground_height": -0.01,
                # 相机内参 — 需用 D435i 实际标定值覆盖
                "grid_map/cx": LaunchConfiguration("cx"),
                "grid_map/cy": LaunchConfiguration("cy"),
                "grid_map/fx": LaunchConfiguration("fx"),
                "grid_map/fy": LaunchConfiguration("fy"),
                # 深度滤波
                "grid_map/use_depth_filter": True,
                "grid_map/depth_filter_tolerance": 0.15,
                "grid_map/depth_filter_maxdist": 4.0,  # D435i 可靠范围
                "grid_map/depth_filter_mindist": 0.3,  # D435i 最小可靠距离
                "grid_map/depth_filter_margin": 2,
                "grid_map/k_depth_scaling_factor": 1000.0,
                "grid_map/skip_pixel": 2,
                # 占据栅格概率
                "grid_map/p_hit": 0.65,
                "grid_map/p_miss": 0.35,
                "grid_map/p_min": 0.12,
                "grid_map/p_max": 0.90,
                "grid_map/p_occ": 0.80,
                "grid_map/min_ray_length": 0.3,  # D435i min
                "grid_map/max_ray_length": 4.0,  # D435i max
                "grid_map/virtual_ceil_height": 2.9,
                "grid_map/visualization_truncate_height": 1.8,
                "grid_map/show_occ_time": False,
                "grid_map/pose_type": 1,  # ODOMETRY 模式
                "grid_map/frame_id": "world",
                # --- Manager ---
                "manager/max_vel": LaunchConfiguration("max_vel"),
                "manager/max_acc": LaunchConfiguration("max_acc"),
                "manager/max_jerk": 4.0,
                "manager/control_points_distance": 0.4,
                "manager/feasibility_tolerance": 0.05,
                "manager/planning_horizon": 7.5,
                "manager/use_distinctive_trajs": True,
                "manager/drone_id": 0,
                # --- Optimization ---
                "optimization/lambda_smooth": 1.0,
                "optimization/lambda_collision": 0.5,
                "optimization/lambda_feasibility": 0.1,
                "optimization/lambda_fitness": 1.0,
                "optimization/dist0": 0.5,
                "optimization/swarm_clearance": 0.5,
                "optimization/max_vel": LaunchConfiguration("max_vel"),
                "optimization/max_acc": LaunchConfiguration("max_acc"),
                "bspline/limit_vel": LaunchConfiguration("max_vel"),
                "bspline/limit_acc": LaunchConfiguration("max_acc"),
                "bspline/limit_ratio": 1.1,
                # --- Prediction ---
                "prediction/obj_num": 10,
                "prediction/lambda": 1.0,
                "prediction/predict_rate": 1.0,
            }
        ],
    )

    # ──────────────────────────────────────────
    #  5. traj_server
    # ──────────────────────────────────────────
    traj_server_node = Node(
        package="ego_planner",
        executable="traj_server",
        name="drone_0_traj_server",
        output="screen",
        remappings=[
            ("position_cmd", "/drone_0_planning/pos_cmd"),
            ("planning/bspline", "/drone_0_planning/bspline"),
            ("trajectory_ack", "/drone_0_planning/trajectory_ack"),
        ],
        parameters=[
            {
                "traj_server/position_cmd_topic": "position_cmd",
                "traj_server/ack_topic": "trajectory_ack",
                "traj_server/source_node_name": "drone_0_traj_server",
                "traj_server/time_forward": 1.0,
                "traj_server/pos_gain_x": 5.0,
                "traj_server/pos_gain_y": 5.0,
                "traj_server/pos_gain_z": 5.0,
                "traj_server/vel_gain_x": 3.0,
                "traj_server/vel_gain_y": 3.0,
                "traj_server/vel_gain_z": 3.0,
            }
        ],
    )

    # ──────────────────────────────────────────
    #  6. RViz2 (可选)
    # ──────────────────────────────────────────
    rviz_para_dir = os.path.join(
        get_package_share_directory("ego_planner"), "config", "ros2_ego_rviz.rviz"
    )
    _ = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_para_dir],
        condition=None,  # 由 IfCondition 在下方覆盖 — 这里先声明
    )

    # ──────────────────────────────────────────
    #  组装 LaunchDescription
    # ──────────────────────────────────────────
    ld = LaunchDescription(
        [
            # 参数声明
            rviz_arg,
            fcu_url_arg,
            gcs_url_arg,
            max_vel_arg,
            max_acc_arg,
            cx_arg,
            cy_arg,
            fx_arg,
            fy_arg,
        ]
    )

    ld.add_action(mavros_launch)
    ld.add_action(sensors_launch)
    ld.add_action(bridge_launch)
    ld.add_action(ego_planner_node)
    ld.add_action(traj_server_node)

    # RViz: IfCondition 需要 import
    from launch.conditions import IfCondition

    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_para_dir],
            condition=IfCondition(LaunchConfiguration("rviz")),
        )
    )

    return ld
