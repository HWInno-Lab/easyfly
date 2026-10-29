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

"""
local_planner.launch.py — easyfly_local_planner: EGO-Planner 适配启动 (M4.3).

用途: 把 third_party EGO-Planner 接入 easyfly 数据链, 只起规划所需的两个节点,
      绕开上游 single_run_in_sim.launch.py 的 map_generator / pcl_render_node / rviz。

数据接线 (无 drone_id 前缀, 显式 remap):
  grid_map/cloud  <- /pointcloud_nonground (semantic_map 可信地面跟踪后的点云, odom 系)
  grid_map/odom   <- /odom                  (P1 coord_transform 由 /uav_state 生成)
  odom_world      <- /odom                  (EGO FSM 自身状态反馈)
  planning/bspline: 内部话题 (ego_planner_node -> traj_server), 保持默认名
  /position_cmd   : traj_server 输出 (quadrotor_msgs/PositionCommand), 仅给飞控执行
  /trajectory_ack : traj_server 专用校验/激活 ACK，FSM 只接受此 topic，不把 position_cmd 当 ACK

坐标系: EGO grid_map 对点云做零变换 (plan_env/grid_map.cpp cloudCallback 直接用原始
        xyz), 故点云必须已在 odom 系。/pointcloud_nonground 仅由 semantic_map 在可信
        GroundPlaneTracker 就绪后发布；模型未就绪时发布空云，不能回退到原始地面点云。

参数来源: 上游 plan_manage/launch/advanced_param.launch.py (内联), 按 MID-360 重调:
  obstacles_inflation 0.099->0.80 / max_ray_length 4.5->20 / local_update_range 8
  use_depth_filter True->False (否则点云被 depth_filter_maxdist=5.0 裁到 5m)

注意 (任务书侦察勘误): ego_replan_fsm.h 中 MANUAL_TARGET=1, PRESET_TARGET=2
  (任务书写的 "2=MANUAL_TARGET" 有误)。正常平台 navigation_managed=True，
  flight_type=1 的旧 /move_base_simple/goal 入口被禁用；RViz 由统一导航协调器转发。

用法:
  ros2 launch easyfly_local_planner local_planner.launch.py
  ros2 launch easyfly_global_planner navigate.launch.py strategy:=ego
  # RViz "2D Goal Pose" 经协调器 /navigate 准入；EGO 不直接接受 RViz 目标。

The ``virtual_ceil_height`` launch argument is an absolute ``odom`` height.
The reusable production default remains 3.2m; the G8
``test/ego_acceptance.launch.py`` wrapper explicitly overrides it to 2.0m
after admitting the perception-demo geometry and hover height.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    # flight_type: 1=MANUAL_TARGET(/move_base_simple/goal) 2=PRESET_TARGET(waypoint 列表)
    flight_type = LaunchConfiguration("flight_type")
    virtual_ceil_height = LaunchConfiguration("virtual_ceil_height")
    managed = ParameterValue(LaunchConfiguration("navigation_managed"), value_type=bool)
    declare_managed = DeclareLaunchArgument(
        "navigation_managed",
        default_value="true",
        description="Normal platform: true. False is explicitly isolated legacy debugging only.",
    )

    declare_sim = DeclareLaunchArgument("use_sim_time", default_value="true")
    declare_acceleration = DeclareLaunchArgument(
        "max_acceleration", default_value="6.0", choices=["2.0", "6.0"]
    )
    declare_yaw = DeclareLaunchArgument(
        "hold_navigation_yaw", default_value="false", choices=["true", "false"]
    )
    declare_ft = DeclareLaunchArgument(
        "flight_type",
        default_value="1",
        description="Legacy unmanaged only: 1=MANUAL_TARGET 2=PRESET_TARGET",
    )
    declare_virtual_ceil = DeclareLaunchArgument(
        "virtual_ceil_height",
        default_value="3.2",
        description=(
            "EGO absolute odom hard ceiling in metres. The G8 "
            "perception_demo wrapper overrides this to 2.0m for its "
            "fixed-height horizontal-bypass contract."
        ),
    )

    ego_params = {
        "use_sim_time": use_sim_time,
        # ---- FSM ----
        "fsm/flight_type": flight_type,
        "navigation_managed": managed,
        "fsm/thresh_replan_time": 1.0,
        "fsm/thresh_no_replan_meter": 1.0,
        "fsm/planning_horizon": 7.5,
        "fsm/planning_horizen_time": 3.0,
        "fsm/emergency_time": 1.0,
        # Leave EMERGENCY_STOP only after a published stop trajectory, fresh
        # odom/cloud, bounded low velocity for several cycles and a 0.5s hold.
        "fsm/emergency_recovery_hold_time": 0.5,
        "fsm/emergency_recovery_stable_cycles": 5,
        "fsm/emergency_recovery_max_speed": 0.1,
        "fsm/emergency_recovery_clock_jump_limit": 1.0,
        # A local publish is not an execution ACK.  EGO waits for a newer
        # matching /position_cmd trajectory_id, retrying within these bounds.
        "fsm/emergency_recovery_ack_timeout": 0.5,
        "fsm/emergency_recovery_ack_retry_period": 0.5,
        "fsm/emergency_recovery_ack_retry_max_period": 5.0,
        "fsm/traj_server_ack_topic": "trajectory_ack",
        "fsm/traj_server_node_name": "traj_server",
        "fsm/traj_server_node_fqn": "/traj_server",
        "fsm/odom_freshness_timeout": 1.0,
        "fsm/waypoint_future_tolerance": 0.2,
        # False => have_trigger_ 初始 true, 无需 /traj_start_trigger
        "fsm/realworld_experiment": False,
        "fsm/fail_safe": True,
        "fsm/waypoint_num": 0,  # MANUAL_TARGET 模式不用 waypoint 列表
        # ---- grid_map (按 MID-360 重调) ----
        "grid_map/resolution": 0.1,
        # E2 can legitimately reach x≈25.2 and E3/E4 retain a conservative
        # 4m local candidate cap.  The former 50m map (x=±25) clipped that
        # valid boundary pose and invalidated odom/cloud fail-closed.
        "grid_map/map_size_x": 80.0,
        "grid_map/map_size_y": 25.0,
        # /pointcloud_nonground 只提供障碍表面，不是飞行高度约束。地图下界
        # 同时是 EGO 的规划最低飞行边界；保留冻结基线 0.50m，防止优化器
        # 从柱体下方钻入地面/机体净空区。低于该边界的采样落在图外，
        # GridMap::getInflateOccupancy() 按 fail-closed 语义返回占据。
        "grid_map/map_size_z": 5.0,
        "grid_map/local_update_range_x": 8.0,
        "grid_map/local_update_range_y": 8.0,
        "grid_map/local_update_range_z": 3.0,
        # Point-cloud surface + vehicle footprint/safety margin; keeps the
        # acceptance clearance meaningful instead of weakening the test.
        "grid_map/obstacles_inflation": 0.80,
        "grid_map/local_map_margin": 10,
        # 这是规划最低飞行边界，不是 semantic_map 的地面剔除阈值；各会话
        # odom 地面虽可为负值，但 EGO 不得规划到贴地/地下。map_size_z=5.0
        # 生产默认 virtual_ceil_height=3.2m 保持历史通用规划走廊；G8
        # perception_demo 的 wrapper 显式传 2.0m：pillar_1 world top=3.0m、
        # 日志中 odom top≈2.36m、hover≈1.5m，因此 2.0m 在柱顶下方留约
        # 0.36m、在 hover 上方留约 0.5m，保持 E1 水平绕柱语义。
        # GridMap::getInflateOccupancy() 对传入值做连续硬拒绝，不能越过
        # 单 voxel ceiling。
        "grid_map/ground_height": 0.5,
        "grid_map/cx": 321.04638671875,
        "grid_map/cy": 243.44969177246094,
        "grid_map/fx": 387.229248046875,
        "grid_map/fy": 387.229248046875,
        "grid_map/use_depth_filter": False,  # 关键: True 会把点云裁到 depth_filter_maxdist=5m
        "grid_map/depth_filter_tolerance": 0.15,
        "grid_map/depth_filter_maxdist": 5.0,
        "grid_map/depth_filter_mindist": 0.2,
        "grid_map/depth_filter_margin": 2,
        "grid_map/k_depth_scaling_factor": 1000.0,
        "grid_map/skip_pixel": 2,
        "grid_map/p_hit": 0.65,
        "grid_map/p_miss": 0.35,
        "grid_map/p_min": 0.12,
        "grid_map/p_max": 0.90,
        "grid_map/p_occ": 0.80,
        "grid_map/min_ray_length": 0.1,
        "grid_map/max_ray_length": 10.0,
        "grid_map/virtual_ceil_height": virtual_ceil_height,
        "grid_map/visualization_truncate_height": 3.0,
        "grid_map/show_occ_time": False,
        "grid_map/pose_type": 1,
        "grid_map/frame_id": "odom",
        # EGO must stop/replan-safe when semantic_map stops producing a
        # non-empty trusted cloud; this bounds stale obstacle data to 1s.
        "grid_map/trusted_cloud_timeout": 1.0,
        "grid_map/trusted_cloud_future_tolerance": 0.2,
        # Odometry is a hard prerequisite for trusted occupancy.  A stopped
        # or reset ROS clock clears the layer; keep the freshness bound at or
        # below one second so a continuing cloud cannot revive stale state.
        "grid_map/odom_freshness_timeout": 1.0,
        "grid_map/odom_clock_jump_limit": 10.0,
        # Checked before pcl::fromROSMsg to bound direct PointCloud2 inputs.
        "grid_map/max_trusted_cloud_points": 200000,
        # /pointcloud_nonground is obstacle-only: a sparse/occluded frame is
        # not free space.  Retain trusted occupancy for a bounded window;
        # invalid/empty semantic frames still clear it fail-closed.
        "grid_map/trusted_cloud_persistence_frames": 20,
        # MID-360 returns obstacle surfaces at sparse vertical slices.  Keep
        # a bounded vertical safety margin so EGO cannot pass through a
        # surface between two trusted returns; horizontal clearance remains
        # governed by obstacles_inflation and the G8 >0.8m contract.
        "grid_map/trusted_cloud_vertical_inflation": 0.40,
        # ---- manager / optimization ----
        "manager/max_vel": 2.0,
        "manager/max_acc": ParameterValue(
            LaunchConfiguration("max_acceleration"), value_type=float
        ),
        "manager/max_jerk": 4.0,
        "manager/control_points_distance": 0.4,
        "manager/feasibility_tolerance": 0.05,
        "manager/planning_horizon": 7.5,
        "manager/use_distinctive_trajs": True,
        "manager/drone_id": 0,
        "optimization/lambda_smooth": 1.0,
        "optimization/lambda_collision": 0.5,
        "optimization/lambda_feasibility": 0.1,
        "optimization/lambda_fitness": 1.0,
        # Match the 0.8m center-clearance contract used by G8.
        "optimization/dist0": 0.80,
        "optimization/swarm_clearance": 0.5,
        "optimization/max_vel": 2.0,
        "optimization/max_acc": ParameterValue(
            LaunchConfiguration("max_acceleration"), value_type=float
        ),
        "bspline/limit_vel": 2.0,
        "bspline/limit_acc": 6.0,
        "bspline/limit_ratio": 1.1,
        "prediction/obj_num": 10,
        "prediction/lambda": 1.0,
        "prediction/predict_rate": 1.0,
    }

    ego_node = Node(
        package="ego_planner",
        executable="ego_planner_node",
        name="ego_planner_node",
        output="screen",
        parameters=[ego_params],
        remappings=[
            ("grid_map/cloud", "/pointcloud_nonground"),
            ("grid_map/odom", "/odom"),
            ("odom_world", "/odom"),
            ("planning/bspline", "/planning/bspline"),
            ("planning/data_display", "/planning/data_display"),
            ("grid_map/occupancy_inflate", "/grid_map/occupancy_inflate"),
        ],
    )

    traj_node = Node(
        package="ego_planner",
        executable="traj_server",
        name="traj_server",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "traj_server/output_frame": "odom",
                "navigation_managed": managed,
                "hold_navigation_yaw": ParameterValue(
                    LaunchConfiguration("hold_navigation_yaw"), value_type=bool
                ),
                "traj_server/position_cmd_topic": "position_cmd",
                "traj_server/ack_topic": "trajectory_ack",
                "traj_server/source_node_name": "traj_server",
            }
        ],
        remappings=[
            ("planning/bspline", "/planning/bspline"),
            # 两个输出均使用可 remap 的相对名；FSM 恢复确认来自 trajectory_ack，
            # 周期性 position_cmd 只作为飞控指令流。
        ],
    )

    return LaunchDescription(
        [
            declare_sim,
            declare_acceleration,
            declare_ft,
            declare_virtual_ceil,
            declare_managed,
            declare_yaw,
            ego_node,
            traj_node,
        ]
    )
