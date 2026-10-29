// Copyright 2026 EasyFly Project
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef EASYFLY_FLIGHT_CONTROL__FLIGHT_CONTROLLER_HPP_
#define EASYFLY_FLIGHT_CONTROL__FLIGHT_CONTROLLER_HPP_

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <deque>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <easyfly_msgs/msg/uav_state.hpp>
#include <easyfly_msgs/msg/flight_status.hpp>
#include <easyfly_msgs/msg/uav_command.hpp>
#include <easyfly_msgs/action/takeoff.hpp>
#include <easyfly_msgs/action/land.hpp>
#include <easyfly_msgs/action/go_to_waypoint.hpp>
#include <easyfly_msgs/action/follow_trajectory.hpp>
#include <easyfly_msgs/action/precision_land.hpp>
#include <easyfly_msgs/msg/landing_guidance_target.hpp>
#include <easyfly_msgs/srv/get_action_ownership.hpp>
#include <easyfly_msgs/srv/get_capabilities.hpp>
#include <easyfly_msgs/srv/get_trajectory_state.hpp>
#include <easyfly_msgs/srv/arm_motors.hpp>
#include <easyfly_msgs/srv/set_flight_mode.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>

#include "easyfly_flight_control/trajectory_startup_timeout.hpp"
#include "easyfly_precision_landing/policy.hpp"

namespace easyfly_flight_control
{

// 注: 原 PID 结构已移除 (Q1) — 本节点的实际控制策略为
// "setpoint 生成 + PX4 位置/速度外环及姿态/角速度内环闭环"，原 PID 从未被调用。
// 若阶段四需要速度/姿态外环, 请重新设计并参数化。

class FlightController : public rclcpp::Node
{
  friend class NavigationSessionTestPeer;
  friend class FlightSafetyTestPeer;
  friend class PrecisionLandingTestPeer;

public:
  using Takeoff = easyfly_msgs::action::Takeoff;
  using Land = easyfly_msgs::action::Land;
  using GoToWaypoint = easyfly_msgs::action::GoToWaypoint;
  using FollowTrajectory = easyfly_msgs::action::FollowTrajectory;
  using PrecisionLand = easyfly_msgs::action::PrecisionLand;

  explicit FlightController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void initialize_precision_landing();
  void create_precision_receiver();
  bool precision_receiver_progress();  // Caller holds flight_mtx_.
  void record_precision_timing(
    const char * outcome,
    const easyfly_msgs::msg::LandingGuidanceTarget & target,
    const rclcpp::MessageInfo & info, double entered, double acquired,
    int64_t entry_clock, int64_t entry_wall, uint64_t callback_generation);
  void flush_precision_timing();
  bool precision_control_loop();
  rclcpp_action::GoalResponse handle_precision_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const PrecisionLand::Goal>);
  rclcpp_action::CancelResponse handle_precision_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<PrecisionLand>>);
  void handle_precision_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<PrecisionLand>>);
  void finish_precision(bool success, const std::string & message, bool release = true);
  void precision_handoff();
  void precision_cancel_child();
  bool precision_land_authorized(const std::string & session) const;
  std::string precision_session() const;
  bool precision_target_ready() const;
  bool precision_handoff_ready() const;
  bool precision_handoff_mode_ready() const;
  bool precision_mode_control_lost() const;

  bool precision_enabled_{false};
  int precision_tag_id_{-1};
  double precision_tag_size_m_{0.0};
  easyfly_precision_landing::Pose precision_handoff_board_;
  bool precision_handoff_aligned_{false};
  bool precision_handoff_pending_{false};
  int64_t precision_handoff_barrier_ns_{0};
  int64_t precision_start_source_ns_{0};
  easyfly_precision_landing::Pose precision_handoff_hold_;
  easyfly_msgs::msg::FlightStatus flight_status_;
  std::chrono::steady_clock::time_point flight_status_receipt_{};
  rclcpp::Subscription<easyfly_msgs::msg::FlightStatus>::SharedPtr sub_flight_status_;
  std::string precision_optical_frame_;
  easyfly_precision_landing::Config precision_config_;
  std::unique_ptr<easyfly_precision_landing::Policy> precision_policy_;
  std::chrono::steady_clock::time_point precision_last_control_{};
  bool precision_reserved_{false};
  bool precision_handoff_{false};
  int64_t precision_handoff_source_ns_{0};
  std::chrono::steady_clock::time_point precision_handoff_started_;
  bool precision_child_admission_pending_{false};
  bool precision_child_cancel_pending_{false};
  bool precision_child_cancel_accepted_{false};
  bool precision_terminal_{false};
  bool precision_cancel_requested_{false};
  bool precision_completion_deferred_{false};
  bool precision_deferred_success_{false};
  bool precision_deferred_release_{true};
  std::string precision_deferred_message_;
  std::string precision_stop_reason_;
  rclcpp_action::GoalUUID precision_uuid_{};
  std::deque<std::string> precision_released_;
  std::chrono::steady_clock::time_point precision_start_{};
  std::chrono::steady_clock::time_point precision_target_receipt_{};
  easyfly_msgs::msg::LandingGuidanceTarget precision_target_;
  int64_t precision_last_stamp_{0};
  uint64_t precision_target_sequence_{0};
  uint64_t precision_counted_sequence_{0};
  uint64_t precision_receiver_generation_{0};
  std::chrono::steady_clock::time_point precision_receiver_progress_time_{};
  rclcpp::TimerBase::SharedPtr timer_precision_receiver_;
  bool precision_timing_diagnostics_{false};
  std::deque<std::string> precision_timing_events_;
  uint64_t precision_timing_dropped_{0};
  rclcpp::TimerBase::SharedPtr timer_precision_timing_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<PrecisionLand>> gh_precision_;
  rclcpp_action::Client<Land>::SharedPtr cli_precision_land_;
  rclcpp_action::ClientGoalHandle<Land>::SharedPtr precision_child_;
  rclcpp_action::Server<PrecisionLand>::SharedPtr as_precision_;
  rclcpp::Subscription<easyfly_msgs::msg::LandingGuidanceTarget>::SharedPtr sub_precision_;
  rclcpp::Service<easyfly_msgs::srv::GetActionOwnership>::SharedPtr srv_precision_ownership_;
  rclcpp::Service<easyfly_msgs::srv::GetCapabilities>::SharedPtr srv_precision_capabilities_;
  // ---- state callback ----
  void cb_uav_state(const easyfly_msgs::msg::UAVState::SharedPtr msg);
  // M4.3: EGO-Planner 轨迹指令回调
  void cb_position_cmd(const quadrotor_msgs::msg::PositionCommand::SharedPtr msg);

  // ---- control loop (20Hz) ----
  void control_loop();

  // ---- action servers ----
  rclcpp_action::GoalResponse handle_takeoff_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const Takeoff::Goal>);
  rclcpp_action::CancelResponse handle_takeoff_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<Takeoff>>);
  void handle_takeoff_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<Takeoff>>);

  rclcpp_action::GoalResponse handle_land_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const Land::Goal>);
  rclcpp_action::CancelResponse handle_land_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<Land>>);
  void handle_land_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<Land>>);

  rclcpp_action::GoalResponse handle_waypoint_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const GoToWaypoint::Goal>);
  rclcpp_action::CancelResponse handle_waypoint_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<GoToWaypoint>>);
  void handle_waypoint_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<GoToWaypoint>>);

  // ---- M4.3: /follow_trajectory action ----
  rclcpp_action::GoalResponse handle_traj_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowTrajectory::Goal>);
  rclcpp_action::CancelResponse handle_traj_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowTrajectory>>);
  void handle_traj_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowTrajectory>>);

  // ---- service clients ----
  rclcpp::Client<easyfly_msgs::srv::ArmMotors>::SharedPtr cli_arm_;
  rclcpp::Client<easyfly_msgs::srv::SetFlightMode>::SharedPtr cli_mode_;
  rclcpp::Service<easyfly_msgs::srv::GetCapabilities>::SharedPtr srv_flight_capabilities_;
  rclcpp::Service<easyfly_msgs::srv::GetTrajectoryState>::SharedPtr srv_trajectory_state_;
  std::string controller_session_id_;

  // ---- 独立 callback group (client 与 action 回调分离，避免死锁) ----
  rclcpp::CallbackGroup::SharedPtr cli_cb_group_;
  rclcpp::CallbackGroup::SharedPtr ctl_cb_group_;   // control loop 专用
  rclcpp::CallbackGroup::SharedPtr state_cb_group_;  // /uav_state 订阅专用 (Q9)
  rclcpp::CallbackGroup::SharedPtr action_cb_group_;

  // ---- helpers ----
  bool call_arm(bool arm, const std::function<bool()> & guard = {});
  bool call_set_mode(
    uint8_t mode, const std::function<bool()> & guard = {},
    const std::function<bool()> & before_send = {});
  double clamp(double v, double lo, double hi);
  // Q9: 等待 PX4 实际进入指定模式 (轮询 /uav_state.flight_mode)
  bool wait_mode_engaged(uint8_t mode, double timeout_s);
  // Q9: uav_ 快照 (线程安全读取)
  easyfly_msgs::msg::UAVState get_uav(
    std::chrono::steady_clock::time_point * receipt = nullptr,
    uint64_t * sequence = nullptr) const;
  bool land_disarm_confirmed(
    const easyfly_msgs::msg::UAVState & uav,
    std::chrono::steady_clock::time_point receipt) const;
  // Q4: 用渐近状态刷新悬停目标并立即发布 setpoint (恢复 OFFBOARD 数据流)
  void enter_hover_at(double x, double y, double z, double yaw);
  // Q4: 反馈值毛刺守卫 (偏差>3m 回退 ramp 值)
  static double safe_axis(double fb, double ramp);
  void stamp_command(easyfly_msgs::msg::UAVCommand & cmd);
  bool flight_state_ready(
    const easyfly_msgs::msg::UAVState & uav,
    std::chrono::steady_clock::time_point receipt) const;
  bool state_evidence_fresh(
    const easyfly_msgs::msg::UAVState & uav,
    std::chrono::steady_clock::time_point receipt) const;
  // Caller holds flight_mtx_; query observes admission without reserving it.
  easyfly_msgs::srv::GetCapabilities::Response takeoff_readiness() const;
  void stop_owned_actions(const std::string & reason);
  // Accepted goal 的所有失败/竞态退出统一经这些 helper 收尾，确保 reservation
  // 与 goal handle 不会在 action 已接受后悬挂。
  void finish_takeoff_goal(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<Takeoff>> gh,
    const std::string & message);
  void finish_land_goal(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<Land>> gh,
    const std::string & message, bool keep_hover);
  void finish_waypoint_goal(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<GoToWaypoint>> gh,
    const std::string & message);
  void finish_traj_goal(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowTrajectory>> gh,
    const std::string & message);

  // ---- subscriptions ----
  rclcpp::Subscription<easyfly_msgs::msg::UAVState>::SharedPtr sub_uav_state_;
  // M4.3: EGO-Planner 输出流 (quadrotor_msgs/PositionCommand, ~100Hz)
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr sub_position_cmd_;

  // ---- publisher ----
  rclcpp::Publisher<easyfly_msgs::msg::UAVCommand>::SharedPtr pub_uav_cmd_;

  // ---- timer ----
  rclcpp::TimerBase::SharedPtr timer_ctl_;

  // ---- action servers ----
  rclcpp_action::Server<Takeoff>::SharedPtr as_takeoff_;
  rclcpp_action::Server<Land>::SharedPtr as_land_;
  rclcpp_action::Server<GoToWaypoint>::SharedPtr as_waypoint_;
  rclcpp_action::Server<FollowTrajectory>::SharedPtr as_traj_;

  // ---- latest state (Q9: 多线程读写保护) ----
  easyfly_msgs::msg::UAVState uav_;
  std::chrono::steady_clock::time_point state_receipt_{};
  uint64_t state_sequence_{0};
  mutable std::mutex state_mtx_;
  std::atomic<bool> state_received_{false};

  // ---- waypoint 目标渐近状态 (抗反馈毛刺, 自维护无毛刺插值) ----
  double x_ramp_{0.0}, y_ramp_{0.0}, z_ramp_{0.0};
  double wp_start_x_{0.0}, wp_start_y_{0.0};         // waypoint 起点
  int wp_in_radius_frames_{0};                       // Q16: 连续在验收半径内的帧数 (去抖)
  uint64_t wp_counted_sequence_{0};
  rclcpp::Time last_ctl_time_;                       // Q12: 上一控制周期时刻 (实测 dt)

  // ---- active action ----
  enum Action
  {
    NONE, TAKEOFF_PREPARING, TAKEOFF, LAND_PREPARING, LANDING, LAND_CANCELING,
    WAYPOINT, HOVER,
    TRAJECTORY_WAIT, TRAJECTORY, PRECISION_GUIDANCE
  };
  // MultiThreadedExecutor 下，全部飞行状态、GoalHandle 和目标参数由此锁保护。
  mutable std::recursive_mutex flight_mtx_;
  // Land 的 PX4 mode service/确认事务必须串行执行；持有此锁时不得持有
  // flight_mtx_，避免阻塞模式调用阻塞 control_loop 或状态迁移。
  mutable std::mutex land_transaction_mtx_;
  Action active_{NONE};
  bool action_reserved_{false};       // goal callback 原子预留，关闭跨 Action 双接受窗口
  rclcpp_action::GoalUUID reserved_goal_id_{};
  std::atomic<bool> command_uncertain_{false};

  // hover 目标位置（takeoff/waypoint 完成后保持的位置）
  double hover_x_{0.0}, hover_y_{0.0}, hover_z_{0.0}, hover_yaw_{0.0};

  // takeoff state
  double takeoff_target_z_{0.0};
  uint8_t takeoff_baseline_mode_{0};
  bool takeoff_offboard_confirmed_{false};
  bool takeoff_preparation_running_{false};
  std::string takeoff_stop_reason_;
  double takeoff_speed_{1.0};
  double takeoff_start_x_{0.0}, takeoff_start_y_{0.0}, takeoff_start_z_{0.0};
  double takeoff_start_yaw_{0.0};    // Q7: 起飞时锁存 yaw, 全程不取反馈 (防姿态毛刺)
  double takeoff_ramp_z_{0.0};       // Q4/Q5: 起飞渐近高度 (取消/超时时悬停用)
  rclcpp::Time takeoff_start_time_;
  rclcpp::Time takeoff_deadline_;    // Q5: 起飞超时保护
  std::chrono::steady_clock::time_point land_start_steady_{};
  bool land_cancel_offboard_requested_{false};
  bool land_transaction_running_{false};
  std::string land_stop_reason_;

  // waypoint state
  double wp_target_x_{0.0}, wp_target_y_{0.0}, wp_target_z_{0.0}, wp_target_yaw_{0.0};
  double wp_max_speed_{2.0};
  double wp_accept_radius_{0.5};
  double wp_start_dist_{0.0};
  rclcpp::Time wp_deadline_;         // Q5: 航点超时保护

  // action goal handles
  std::shared_ptr<rclcpp_action::ServerGoalHandle<Takeoff>> gh_takeoff_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<Land>> gh_land_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<GoToWaypoint>> gh_waypoint_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowTrajectory>> gh_traj_;

  // ---- M4.3: /follow_trajectory 状态 ----
  // 最近一条 EGO /position_cmd 快照 (跨线程读: 回调组独立)
  quadrotor_msgs::msg::PositionCommand latest_cmd_;
  rclcpp::Time traj_cmd_time_;                       // 最近收到轨迹指令时刻 (0=未收到)
  mutable std::mutex traj_mtx_;                      // latest_cmd_/traj_cmd_time_ 保护
  std::atomic<bool> traj_cmd_received_{false};
  double traj_target_x_{0.0}, traj_target_y_{0.0}, traj_target_z_{0.0};
  double traj_accept_radius_{0.8};
  double traj_stream_timeout_{kTrajectoryStreamTimeout};  // 首帧后流断判定 [s]
  double traj_startup_grace_{kDefaultTrajectoryStartupGrace};
  // 首帧前等待 EGO 有界重试；独立于 traj_stream_timeout_ 和 action deadline。
  int traj_in_radius_frames_{0};                     // 到达判定去抖
  uint64_t traj_counted_sequence_{0};
  rclcpp::Time traj_accept_time_;
  rclcpp::Time traj_deadline_;
  // PositionCommand 的单调 ID 水位。它由 flight_mtx_ 保护，并且即使当前
  // 没有 FollowTrajectory 也会从合格消息中更新，用于绑定下一次 goal。
  uint32_t traj_seen_max_id_{0};
  // 新 goal 在 goal callback 中与 action_reserved_ 一起原子锁存的 ID 水位。
  uint32_t traj_baseline_id_{0};
  // goal response 已返回但 accepted callback 尚未完成时，阻止消息把
  // response/accepted 窗口内的 ID 提前提交到 seen 水位。
  bool traj_goal_pending_{false};
  std::string traj_session_id_;
  uint32_t traj_active_id_{0};

  // params
  double ctl_rate_{20.0};
  double land_timeout_{30.0};
  double max_altitude_{50.0};
  double max_horizontal_speed_{20.0};
  double max_goal_distance_{100.0};
};

}  // namespace easyfly_flight_control

#endif  // EASYFLY_FLIGHT_CONTROL__FLIGHT_CONTROLLER_HPP_
