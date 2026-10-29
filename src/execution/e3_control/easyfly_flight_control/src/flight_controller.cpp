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

#include "easyfly_flight_control/flight_controller.hpp"
#include <cmath>
#include <chrono>
#include <sstream>
#include <random>
#include <thread>

namespace easyfly_flight_control
{
using namespace std::placeholders;
using namespace std::chrono_literals;

namespace
{
struct ScopeExit
{
  std::function<void()> cleanup;
  ~ScopeExit() {cleanup();}
};
template<typename ServiceT>
bool call_service_bounded(
  const typename rclcpp::Client<ServiceT>::SharedPtr & client,
  const typename ServiceT::Request::SharedPtr & request,
  const rclcpp::Context::SharedPtr & context,
  const std::function<bool()> & guard,
  const std::function<bool()> & before_send,
  const std::function<void()> & uncertain,
  std::recursive_mutex & flight_mutex)
{
  if (!rclcpp::ok(context)) {return false;}
  bool request_attempted = false;
  try {
    const auto service_deadline = std::chrono::steady_clock::now() + 3s;
    while (rclcpp::ok(context) && !client->service_is_ready()) {
      if (guard && !guard()) {return false;}
      if (std::chrono::steady_clock::now() >= service_deadline) {return false;}
      std::this_thread::sleep_for(50ms);
    }
    std::unique_lock<std::recursive_mutex> send_lock(flight_mutex);
    if (!rclcpp::ok(context)) {return false;}
    if (guard && !guard()) {return false;}
    if (before_send && !before_send()) {return false;}
    if (!rclcpp::ok(context)) {return false;}
    request_attempted = true;
    auto future = client->async_send_request(request);
    send_lock.unlock();
    const auto response_deadline = std::chrono::steady_clock::now() + 5s;
    while (rclcpp::ok(context) && std::chrono::steady_clock::now() < response_deadline) {
      if (future.wait_for(50ms) == std::future_status::ready) {
        const auto response = future.get();
        if (response->message.rfind("command_result_unknown:", 0) == 0) {
          uncertain();
          return false;
        }
        return rclcpp::ok(context) && response->success;
      }
    }
    client->remove_pending_request(future);
    uncertain();
    return false;
  } catch (const std::exception &) {
    if (request_attempted) {
      uncertain();
      return false;
    }
    // Shutdown may race middleware calls; do not hide errors in a live context.
    if (rclcpp::ok(context)) {throw;}
    return false;
  }
}
}  // namespace

// 注: PID 实现已移除 (Q1) — 生成 setpoint，位置/速度及姿态/角速度闭环均由 PX4 执行。
// 原 PID::update 从未被调用, 属死代码。

FlightController::FlightController(const rclcpp::NodeOptions & options)
: Node("flight_controller", options)
{
  ctl_rate_ = declare_parameter("control_rate", 20.0);
  land_timeout_ = declare_parameter("land_timeout", 60.0);
  max_altitude_ = declare_parameter("max_altitude", 50.0);
  max_horizontal_speed_ = declare_parameter("max_horizontal_speed", 20.0);
  max_goal_distance_ = declare_parameter("max_goal_distance", 100.0);
  const double configured_traj_startup_grace = declare_parameter(
    "trajectory_startup_grace", kDefaultTrajectoryStartupGrace);
  traj_startup_grace_ = sanitizeTrajectoryStartupGrace(configured_traj_startup_grace);
  if (traj_startup_grace_ != configured_traj_startup_grace) {
    RCLCPP_WARN(
      get_logger(),
      "trajectory_startup_grace=%.3fs 无效或超出 [%.1f, %.1f]，回退安全默认 %.1fs",
      configured_traj_startup_grace, kMinTrajectoryStartupGrace,
      kMaxTrajectoryStartupGrace, kDefaultTrajectoryStartupGrace);
  }
  RCLCPP_INFO(
    get_logger(),
    "FollowTrajectory watchdog: startup grace=%.2fs, post-start stream timeout=%.2fs",
    traj_startup_grace_, traj_stream_timeout_);
  auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / ctl_rate_));
  auto qos = rclcpp::QoS(10);

  // Q9 修复: 状态订阅独立回调组 — handle_*_accepted 中的阻塞调用
  // (call_arm/wait_mode_engaged 等) 若与订阅同组, 会饿死 /uav_state 更新,
  // 导致模式校验读到永久 stale 的旧模式 (实测: OFFBOARD 已切入仍判定失败)
  state_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = state_cb_group_;
  sub_uav_state_ = create_subscription<easyfly_msgs::msg::UAVState>(
    "/uav_state", rclcpp::SensorDataQoS(),
    std::bind(&FlightController::cb_uav_state, this, _1),
    sub_opts);
  pub_uav_cmd_ = create_publisher<easyfly_msgs::msg::UAVCommand>("/uav_command", qos);

  // M4.3: EGO-Planner 轨迹流 (traj_server 100Hz); 独立回调组, 避免被 action 阻塞饿死
  sub_position_cmd_ = create_subscription<quadrotor_msgs::msg::PositionCommand>(
    "/position_cmd", rclcpp::SensorDataQoS(),
    std::bind(&FlightController::cb_position_cmd, this, _1),
    sub_opts);

  cli_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cli_arm_ = create_client<easyfly_msgs::srv::ArmMotors>(
    "/arm_motors",
    rmw_qos_profile_services_default,
    cli_cb_group_);
  cli_mode_ = create_client<easyfly_msgs::srv::SetFlightMode>(
    "/set_flight_mode",
    rmw_qos_profile_services_default,
    cli_cb_group_);

  action_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  as_takeoff_ = rclcpp_action::create_server<Takeoff>(
    this, "/takeoff",
    std::bind(&FlightController::handle_takeoff_goal, this, _1, _2),
    std::bind(&FlightController::handle_takeoff_cancel, this, _1),
    std::bind(&FlightController::handle_takeoff_accepted, this, _1),
    rcl_action_server_get_default_options(), action_cb_group_);
  as_land_ = rclcpp_action::create_server<Land>(
    this, "/land",
    std::bind(&FlightController::handle_land_goal, this, _1, _2),
    std::bind(&FlightController::handle_land_cancel, this, _1),
    std::bind(&FlightController::handle_land_accepted, this, _1),
    rcl_action_server_get_default_options(), action_cb_group_);
  as_waypoint_ = rclcpp_action::create_server<GoToWaypoint>(
    this, "/goto_waypoint",
    std::bind(&FlightController::handle_waypoint_goal, this, _1, _2),
    std::bind(&FlightController::handle_waypoint_cancel, this, _1),
    std::bind(&FlightController::handle_waypoint_accepted, this, _1),
    rcl_action_server_get_default_options(), action_cb_group_);
  as_traj_ = rclcpp_action::create_server<FollowTrajectory>(
    this, "/follow_trajectory",
    std::bind(&FlightController::handle_traj_goal, this, _1, _2),
    std::bind(&FlightController::handle_traj_cancel, this, _1),
    std::bind(&FlightController::handle_traj_accepted, this, _1),
    rcl_action_server_get_default_options(), action_cb_group_);

  ctl_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  initialize_precision_landing();
  rclcpp_action::GoalUUID controller_uuid;
  std::random_device entropy;
  constexpr char hex[] = "0123456789abcdef";
  for (auto & byte : controller_uuid) {
    byte = static_cast<uint8_t>(entropy());
    controller_session_id_ += hex[byte >> 4];
    controller_session_id_ += hex[byte & 15];
  }
  srv_trajectory_state_ = create_service<easyfly_msgs::srv::GetTrajectoryState>(
    "/flight_control/trajectory_state", [this](
      easyfly_msgs::srv::GetTrajectoryState::Request::SharedPtr,
      easyfly_msgs::srv::GetTrajectoryState::Response::SharedPtr response) {
      std::lock_guard<std::recursive_mutex> lock(flight_mtx_);
      response->header.stamp = now();
      response->controller_session_id = controller_session_id_;
      response->highest_trajectory_id = traj_seen_max_id_;
      response->idle = active_ == HOVER && !action_reserved_ && !precision_reserved_ &&
      !command_uncertain_;
      response->active_navigation_session_id = traj_session_id_;
    });
  srv_flight_capabilities_ = create_service<easyfly_msgs::srv::GetCapabilities>(
    "/platform/flight_control/capabilities", [this](
      easyfly_msgs::srv::GetCapabilities::Request::SharedPtr,
      easyfly_msgs::srv::GetCapabilities::Response::SharedPtr response) {
      std::lock_guard<std::recursive_mutex> lock(flight_mtx_);
      *response = takeoff_readiness();
    });
  timer_ctl_ = create_wall_timer(
    period, std::bind(
      &FlightController::control_loop,
      this), ctl_cb_group_);
  RCLCPP_INFO(get_logger(), "FlightController Start");
}

void FlightController::cb_uav_state(const easyfly_msgs::msg::UAVState::SharedPtr msg)
{
  if (!msg) {return;}
  {  // Q9: 与 control_loop / 模式校验轮询的跨线程读写保护
    std::lock_guard<std::mutex> lk(state_mtx_);
    uav_ = *msg;
    state_receipt_ = std::chrono::steady_clock::now();
    ++state_sequence_;
  }
  state_received_ = true;
}

// M4.3: EGO /position_cmd 快照 (TRAJECTORY 模式下由 control_loop 转发)
void FlightController::cb_position_cmd(
  const quadrotor_msgs::msg::PositionCommand::SharedPtr msg)
{
  // 仅接受 READY 轨迹 (ego traj_server 在无轨迹时不发布; 该判定是双保险)
  if (msg->trajectory_flag !=
    quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY)
  {
    return;
  }
  if (msg->header.frame_id != "odom" ||
    !std::isfinite(msg->position.x) || !std::isfinite(msg->position.y) ||
    !std::isfinite(msg->position.z) || !std::isfinite(msg->velocity.x) ||
    !std::isfinite(msg->velocity.y) || !std::isfinite(msg->velocity.z) ||
    !std::isfinite(msg->yaw))
  {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "拒绝 /position_cmd: frame 必须为 odom 且控制值必须有限");
    return;
  }
  const rclcpp::Time msg_time(msg->header.stamp, get_clock()->get_clock_type());
  const auto receipt_time = now();
  const double age = (receipt_time - msg_time).seconds();
  if (msg_time.nanoseconds() == 0 || age < -0.2 || age > 1.0) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "拒绝 /position_cmd: 时间戳无效或过期 (age=%.3fs)", age);
    return;
  }

  std::lock_guard<std::recursive_mutex> state_lk(flight_mtx_);
  // goal response 到 accepted callback 之间仍处于 HOVER；这段窗口内的
  // 消息不能改变基线，否则同一条 N+1 会在真正进入 WAIT 后被当作旧帧。
  if (traj_goal_pending_) {return;}
  // Isolate sessions before any ID watermark can be advanced.
  if (msg->navigation_session_id.size() > 128 ||
    msg->navigation_session_id != traj_session_id_) {return;}

  // 只有通过 active/time/ID 检查的候选才允许推进 seen 水位。没有 active
  // trajectory 时则是有意观测（用于下一次 goal 的 baseline），但不能把
  // WAIT 的旧帧、时间过期帧或 ACTIVE 的回退帧计入最大 ID。
  const bool waiting_for_first_frame = active_ == TRAJECTORY_WAIT;
  const uint32_t wait_id_watermark =
    std::max(traj_baseline_id_, traj_seen_max_id_);
  const bool advances_wait_watermark =
    !waiting_for_first_frame || msg->trajectory_id > wait_id_watermark;

  if (active_ != TRAJECTORY_WAIT && active_ != TRAJECTORY) {
    traj_seen_max_id_ = std::max(traj_seen_max_id_, msg->trajectory_id);
    return;
  }
  if (msg_time + rclcpp::Duration::from_seconds(0.02) < traj_accept_time_) {return;}
  if (waiting_for_first_frame) {
    // Re-check with the ROS clock while holding the state lock.  A frame may
    // have arrived before the deadline but wait behind control_loop; it must
    // not activate a trajectory after the bounded startup/deadline window.
    const auto wait_check_time = now();
    const double startup_gap = (wait_check_time - traj_accept_time_).seconds();
    const double action_deadline_seconds =
      (traj_deadline_ - traj_accept_time_).seconds();
    if (trajectoryStartupTimedOut(
        startup_gap, traj_startup_grace_, action_deadline_seconds))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "拒绝过期首帧 /position_cmd: gap=%.2fs, action deadline=%.2fs",
        startup_gap, action_deadline_seconds);
      return;
    }
  }
  if (waiting_for_first_frame && !advances_wait_watermark) {return;}
  if (active_ == TRAJECTORY && msg->trajectory_id < traj_active_id_) {return;}
  traj_seen_max_id_ = std::max(traj_seen_max_id_, msg->trajectory_id);
  std::lock_guard<std::mutex> lk(traj_mtx_);
  latest_cmd_ = *msg;
  traj_cmd_time_ = receipt_time;
  traj_active_id_ = msg->trajectory_id;
  traj_cmd_received_.store(true);
  active_ = TRAJECTORY;
}

easyfly_msgs::msg::UAVState FlightController::get_uav(
  std::chrono::steady_clock::time_point * receipt, uint64_t * sequence) const
{
  std::lock_guard<std::mutex> lk(state_mtx_);
  if (receipt) {*receipt = state_receipt_;}
  if (sequence) {*sequence = state_sequence_;}
  return uav_;
}

void FlightController::control_loop()
{
  if (!state_received_) {return;}
  std::lock_guard<std::recursive_mutex> state_lk(flight_mtx_);
  if (precision_control_loop()) {return;}
  std::chrono::steady_clock::time_point receipt;
  uint64_t sequence;
  const auto uav = get_uav(&receipt, &sequence);
  // Disarm confirmation completes only an existing Land transaction; it does
  // not grant flight admission when pose validity has been lost.
  if (active_ == LANDING && gh_land_ && land_disarm_confirmed(uav, receipt)) {
    auto result = std::make_shared<Land::Result>();
    result->success = true;
    result->message = "land complete (fresh disarm confirmed)";
    gh_land_->succeed(result);
    gh_land_.reset();
    active_ = NONE;
    action_reserved_ = false;
    return;
  }
  if (command_uncertain_ || !rclcpp::ok(get_node_base_interface()->get_context()) ||
    !flight_state_ready(uav, receipt))
  {
    stop_owned_actions("flight state unavailable or stale; setpoint stream stopped");
    return;
  }
  if (!takeoff_stop_reason_.empty() || !land_stop_reason_.empty()) {return;}
  const bool land_taken_over =
    (active_ == LAND_PREPARING && uav.flight_mode != 3 && uav.flight_mode != 7) ||
    (active_ == LANDING && uav.flight_mode != 7) ||
    (active_ == LAND_CANCELING && uav.flight_mode != 7 &&
    !(land_cancel_offboard_requested_ && uav.flight_mode == 3));
  if (land_taken_over) {
    stop_owned_actions("external mode takeover during Land; setpoint stream stopped");
    return;
  }
  const bool offboard_owned = active_ == TAKEOFF || active_ == WAYPOINT ||
    active_ == HOVER || active_ == TRAJECTORY_WAIT || active_ == TRAJECTORY;
  const bool preparing_takeover = active_ == TAKEOFF_PREPARING &&
    uav.flight_mode != 3 &&
    (takeoff_offboard_confirmed_ || uav.flight_mode != takeoff_baseline_mode_);
  if ((offboard_owned && uav.flight_mode != 3) || preparing_takeover) {
    stop_owned_actions("external mode takeover; setpoint stream stopped");
    return;
  }
  if (active_ == TAKEOFF_PREPARING && uav.flight_mode == 3) {
    takeoff_offboard_confirmed_ = true;
  }

  // Q12: 实测周期 (替代固定 1/rate), 限幅防极端值 (调度抖动/时钟跳变)
  const auto t_now = now();
  double dt = 1.0 / ctl_rate_;
  if (last_ctl_time_.nanoseconds() > 0) {
    dt = std::clamp(
      (t_now - last_ctl_time_).seconds(),
      0.5 / ctl_rate_, 2.0 / ctl_rate_);
  }
  last_ctl_time_ = t_now;

  easyfly_msgs::msg::UAVCommand cmd;
  cmd.control_mode = easyfly_msgs::msg::UAVCommand::POSITION;
  stamp_command(cmd);

  // cancel callback 返回 ACCEPT 后，rcl_action 才会把 goal 从 EXECUTING
  // 转入 CANCELING。终态必须在这里观察到 is_canceling() 后设置，不能在
  // cancel callback 内同步调用 canceled()，否则会触发非法状态迁移并杀死节点。
  if ((active_ == TAKEOFF_PREPARING || active_ == TAKEOFF) &&
    gh_takeoff_ && gh_takeoff_->is_canceling())
  {
    // Keep this reservation until the accepted callback drains its issued
    // service request. Do not hover, emit setpoints, or overlap another goal.
    if (takeoff_preparation_running_) {return;}
    auto gh = gh_takeoff_;
    if (uav.armed) {
      enter_hover_at(
        safe_axis(uav.position_x, takeoff_start_x_),
        safe_axis(uav.position_y, takeoff_start_y_),
        safe_axis(uav.position_z, takeoff_ramp_z_), takeoff_start_yaw_);
    } else {
      active_ = NONE;
      action_reserved_ = false;
    }
    auto r = std::make_shared<Takeoff::Result>();
    r->success = false; r->message = "canceled";
    gh->canceled(r);
    gh_takeoff_.reset();
    return;
  }
  if (active_ == WAYPOINT && gh_waypoint_ && gh_waypoint_->is_canceling()) {
    auto gh = gh_waypoint_;
    enter_hover_at(
      safe_axis(uav.position_x, x_ramp_),
      safe_axis(uav.position_y, y_ramp_),
      safe_axis(uav.position_z, z_ramp_), wp_target_yaw_);
    auto r = std::make_shared<GoToWaypoint::Result>();
    r->success = false; r->message = "canceled";
    gh->canceled(r);
    gh_waypoint_.reset();
    return;
  }
  if ((active_ == TRAJECTORY_WAIT || active_ == TRAJECTORY) &&
    gh_traj_ && gh_traj_->is_canceling())
  {
    auto gh = gh_traj_;
    bool have_cmd;
    quadrotor_msgs::msg::PositionCommand c;
    {
      std::lock_guard<std::mutex> lk(traj_mtx_);
      have_cmd = traj_cmd_received_.load();
      c = latest_cmd_;
    }
    if (have_cmd) {
      enter_hover_at(
        safe_axis(uav.position_x, c.position.x),
        safe_axis(uav.position_y, c.position.y),
        safe_axis(uav.position_z, c.position.z), c.yaw);
    } else {
      enter_hover_at(uav.position_x, uav.position_y, uav.position_z, uav.yaw);
    }
    auto r = std::make_shared<FollowTrajectory::Result>();
    r->success = false; r->message = "canceled";
    gh->canceled(r);
    gh_traj_.reset();
    traj_session_id_.clear();
    return;
  }
  if (active_ == LAND_CANCELING) {
    // cancel callback 正在恢复 OFFBOARD；整个事务期间持续供给悬停 setpoint。
    cmd.position_x = hover_x_; cmd.position_y = hover_y_;
    cmd.position_z = hover_z_; cmd.yaw = hover_yaw_;
    pub_uav_cmd_->publish(cmd);
    if (gh_land_ && gh_land_->is_canceling() && uav.flight_mode == 3) {
      auto gh = gh_land_;
      active_ = HOVER;
      action_reserved_ = false;
      auto r = std::make_shared<Land::Result>();
      r->success = false; r->message = "canceled";
      gh->canceled(r);
      gh_land_.reset();
    }
    return;
  }

  if (active_ == LAND_PREPARING) {
    // An accepted parent cancel must stop output while its exact child drains
    // the already issued AUTO.LAND transaction. Keep both reservations and
    // the transaction intact; known LANDING cancellation recovery is unchanged.
    if (precision_reserved_ && precision_handoff_ && precision_cancel_requested_ && gh_land_ &&
      gh_land_->get_goal()->precision_landing_session_id == precision_session())
    {
      return;
    }
    // AUTO.LAND 请求/确认期间仍保持锁存的 hover setpoint，直到 accepted
    // callback 确认 PX4 已进入 LANDING。这样模式服务阻塞或确认延迟时，
    // OFFBOARD 数据流不会中断，也不会用实时反馈覆盖发起降落前的悬停点。
    cmd.position_x = hover_x_; cmd.position_y = hover_y_;
    cmd.position_z = hover_z_; cmd.yaw = hover_yaw_;
    pub_uav_cmd_->publish(cmd);
    return;
  }
  if (active_ == NONE || active_ == LANDING) {
    // 接管/降落中: 不再发布 OFFBOARD setpoint
    if (active_ == LANDING && gh_land_) {
      auto fb = std::make_shared<Land::Feedback>();
      fb->current_altitude = uav.position_z; gh_land_->publish_feedback(fb);
      const double t = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - land_start_steady_).count();
      if (land_disarm_confirmed(uav, receipt)) {
        auto r = std::make_shared<Land::Result>(); r->success = true;
        r->message = "landed"; gh_land_->succeed(r); gh_land_.reset();
        // 注: 不做模式清理 — PX4 降落后自行停留/回退模式 (OFFBOARD 等),
        // 任何残留模式的再解锁由 takeoff 的 "先建流后解锁" 流程统一处理
        active_ = NONE;
        action_reserved_ = false;
      } else if (t > land_timeout_) {
        // Local odometry Z is not ground clearance. Never infer landed or
        // stop motors from it; leave PX4 AUTO.LAND unchanged on timeout.
        RCLCPP_ERROR(get_logger(), "Land timeout: disarm not confirmed; AUTO.LAND continues");
        auto r = std::make_shared<Land::Result>();
        r->success = false;
        r->message = "land timeout (disarm unconfirmed, AUTO.LAND continues)";
        gh_land_->abort(r); gh_land_.reset();
        active_ = NONE;
        action_reserved_ = false;
      }
    }
    return;
  }

  if (active_ == TAKEOFF_PREPARING) {
    // 切 OFFBOARD / arm 确认前只维持地面当前位置，不开始爬升计时。
    cmd.position_x = takeoff_start_x_; cmd.position_y = takeoff_start_y_;
    cmd.position_z = takeoff_start_z_; cmd.yaw = takeoff_start_yaw_;
    pub_uav_cmd_->publish(cmd);
  } else if (active_ == TAKEOFF) {
    const double t = (now() - takeoff_start_time_).seconds();
    const double tz = std::min(takeoff_start_z_ + t * takeoff_speed_, takeoff_target_z_);
    takeoff_ramp_z_ = tz;              // Q4/Q5: 记录渐近高度供取消/超时悬停用
    cmd.position_x = takeoff_start_x_; cmd.position_y = takeoff_start_y_;
    cmd.position_z = tz;
    cmd.yaw = takeoff_start_yaw_;      // Q7: 锁存值, 不取反馈 (防姿态毛刺)
    pub_uav_cmd_->publish(cmd);
    if (gh_takeoff_ && std::abs(takeoff_target_z_ - uav.position_z) < 0.5) {
      auto r = std::make_shared<Takeoff::Result>(); r->success = true;
      gh_takeoff_->succeed(r); gh_takeoff_.reset();
      hover_x_ = takeoff_start_x_; hover_y_ = takeoff_start_y_;
      hover_z_ = takeoff_target_z_; hover_yaw_ = takeoff_start_yaw_;
      x_ramp_ = takeoff_start_x_; y_ramp_ = takeoff_start_y_; z_ramp_ = takeoff_target_z_;
      active_ = HOVER;
      action_reserved_ = false;
    } else if (gh_takeoff_ && now() > takeoff_deadline_) {
      // Q5: 超时保护 — 之前会永远 pending, 卡死后续所有 goal
      RCLCPP_ERROR(
        get_logger(), "Takeoff timeout (z=%.2f / target %.2f), abort + 就地悬停",
        uav.position_z, takeoff_target_z_);
      auto r = std::make_shared<Takeoff::Result>();
      r->success = false; r->message = "takeoff timeout";
      gh_takeoff_->abort(r); gh_takeoff_.reset();
      enter_hover_at(takeoff_start_x_, takeoff_start_y_, takeoff_ramp_z_, takeoff_start_yaw_);
    }
  } else if (active_ == WAYPOINT) {
    const double ex = wp_target_x_ - uav.position_x, ey = wp_target_y_ - uav.position_y;
    const double ez = wp_target_z_ - uav.position_z;
    double dist = std::sqrt(ex * ex + ey * ey + ez * ez);
    // 全轴目标渐近式: setpoint 从起点直线渐近目标, 不依赖 local_position 反馈
    // (M6 踩坑: mavros local_position 偶发大幅毛刺如 z=-10, 增量跟随式控制
    //  cmd=feedback+Δ 会把毛刺写进 setpoint 造成正反馈发散)
    double dirx = wp_target_x_ - wp_start_x_, diry = wp_target_y_ - wp_start_y_;
    double L = std::sqrt(dirx * dirx + diry * diry);
    if (L > 1e-6) {dirx /= L; diry /= L;} else {dirx = 1; diry = 0;}
    double step = wp_max_speed_ * dt;
    double prog = std::hypot(x_ramp_ - wp_start_x_, y_ramp_ - wp_start_y_);
    double new_prog = std::min(prog + step, L);
    x_ramp_ = wp_start_x_ + dirx * new_prog;
    y_ramp_ = wp_start_y_ + diry * new_prog;
    z_ramp_ += std::clamp(wp_target_z_ - z_ramp_, -step, step);
    cmd.position_x = x_ramp_;
    cmd.position_y = y_ramp_;
    cmd.position_z = z_ramp_;
    cmd.yaw = wp_target_yaw_;
    // 距离仅用于完成判定 (基于真实反馈, 但毛刺只影响判定不影响 setpoint)
    pub_uav_cmd_->publish(cmd);

    // Q16: 完成判定去抖 — 双条件同时满足:
    //   ① ramp 已走完 (setpoint 已停在目标点: 水平到位 && z 到位)
    //   ② 连续 5 帧 (0.25s) 反馈在验收半径内 (单帧毛刺尖峰被滤除)
    const bool ramp_done =
      (new_prog >= L - 1e-6) && std::abs(z_ramp_ - wp_target_z_) < 1e-6;
    if (sequence != wp_counted_sequence_) {
      wp_counted_sequence_ = sequence;
      if (dist < wp_accept_radius_) {++wp_in_radius_frames_;} else {wp_in_radius_frames_ = 0;}
    }

    if (gh_waypoint_ && ramp_done && wp_in_radius_frames_ >= 5) {
      auto r = std::make_shared<GoToWaypoint::Result>(); r->success = true;
      gh_waypoint_->succeed(r); gh_waypoint_.reset();
      hover_x_ = wp_target_x_; hover_y_ = wp_target_y_;
      hover_z_ = wp_target_z_; hover_yaw_ = wp_target_yaw_;
      active_ = HOVER;
      action_reserved_ = false;
    } else if (gh_waypoint_ && now() > wp_deadline_) {
      // Q5: 超时保护 — abort + 就地悬停 (当前位置优先, 毛刺守卫同 Q4)
      RCLCPP_ERROR(get_logger(), "Waypoint timeout (dist=%.2f), abort + 就地悬停", dist);
      auto r = std::make_shared<GoToWaypoint::Result>();
      r->success = false; r->message = "waypoint timeout";
      gh_waypoint_->abort(r); gh_waypoint_.reset();
      enter_hover_at(
        safe_axis(uav.position_x, x_ramp_),
        safe_axis(uav.position_y, y_ramp_),
        safe_axis(uav.position_z, z_ramp_),
        wp_target_yaw_);
    }
  } else if (active_ == HOVER) {
    // 接管检测已上移至函数开头 (TAKEOFF/WAYPOINT/HOVER 通用)
    if (uav.armed) {
      cmd.position_x = hover_x_; cmd.position_y = hover_y_;
      cmd.position_z = hover_z_; cmd.yaw = hover_yaw_;
      pub_uav_cmd_->publish(cmd);
    }
  } else if (active_ == TRAJECTORY_WAIT) {
    // 等首帧期间保持进入轨迹前的悬停点，绝不读取默认 PositionCommand。
    cmd.position_x = hover_x_; cmd.position_y = hover_y_;
    cmd.position_z = hover_z_; cmd.yaw = hover_yaw_;
    pub_uav_cmd_->publish(cmd);
    const auto wait_now = now();
    const double gap = (wait_now - traj_accept_time_).seconds();
    const double action_deadline_seconds =
      (traj_deadline_ - traj_accept_time_).seconds();
    const double action_remaining = (traj_deadline_ - wait_now).seconds();
    if (trajectoryStartupTimedOut(
        gap, traj_startup_grace_, action_deadline_seconds))
    {
      const bool action_deadline_elapsed =
        !std::isfinite(action_remaining) || action_remaining <= 0.0;
      RCLCPP_ERROR(
        get_logger(),
        "FollowTrajectory 首帧超时: gap=%.2fs, startup grace=%.2fs, "
        "action remaining=%.2fs (%s; 首帧后流断门仍为 %.2fs)",
        gap, traj_startup_grace_, action_remaining,
        action_deadline_elapsed ? "action deadline" : "startup grace",
        traj_stream_timeout_);
      if (gh_traj_) {
        auto r = std::make_shared<FollowTrajectory::Result>();
        r->success = false;
        r->message = action_deadline_elapsed ?
          "trajectory_timeout_before_start" : "trajectory_start_timeout";
        gh_traj_->abort(r); gh_traj_.reset(); traj_session_id_.clear();
      }
      action_reserved_ = false;
      active_ = HOVER;
    }
  } else if (active_ == TRAJECTORY) {
    // ---- M4.3: 转发 EGO /position_cmd (位置+速度前馈+yaw) ----
    bool have_cmd;
    quadrotor_msgs::msg::PositionCommand c;
    rclcpp::Time t_cmd;
    {
      std::lock_guard<std::mutex> lk(traj_mtx_);
      have_cmd = traj_cmd_received_.load();
      c = latest_cmd_;
      t_cmd = traj_cmd_time_;
    }
    const auto trajectory_now = now();
    const double action_gap = (trajectory_now - traj_accept_time_).seconds();
    const double action_deadline_seconds =
      (traj_deadline_ - traj_accept_time_).seconds();
    // The command must be checked before any TRAJECTORY setpoint is
    // published.  This closes the boundary race where a late callback
    // activates the state just after the action deadline.
    if (trajectoryDeadlineExpired(action_gap, action_deadline_seconds)) {
      RCLCPP_ERROR(
        get_logger(),
        "FollowTrajectory action deadline reached before publish "
        "(gap=%.2fs, deadline=%.2fs) → 就地悬停 + abort",
        action_gap, action_deadline_seconds);
      if (gh_traj_) {
        auto r = std::make_shared<FollowTrajectory::Result>();
        r->success = false; r->message = "timeout";
        gh_traj_->abort(r); gh_traj_.reset(); traj_session_id_.clear();
      }
      if (have_cmd) {
        enter_hover_at(
          safe_axis(uav.position_x, c.position.x),
          safe_axis(uav.position_y, c.position.y),
          safe_axis(uav.position_z, c.position.z),
          c.yaw);
      } else {
        enter_hover_at(uav.position_x, uav.position_y, uav.position_z, uav.yaw);
      }
      return;
    }

    // 流断判定: 已流过 → >=traj_stream_timeout_ 无新帧; 从未流过 → >=起始宽限
    const double gap = have_cmd ? (trajectory_now - t_cmd).seconds() :
      action_gap;
    const bool stream_lost = trajectoryStreamLost(
      have_cmd, gap, traj_startup_grace_, traj_stream_timeout_);
    if (stream_lost) {
      RCLCPP_ERROR(
        get_logger(),
        "FollowTrajectory 流断 (%.2fs 无 /position_cmd) → 就地悬停 + abort", gap);
      if (gh_traj_) {
        auto r = std::make_shared<FollowTrajectory::Result>();
        r->success = false; r->message = "stream_lost";
        gh_traj_->abort(r); gh_traj_.reset(); traj_session_id_.clear();
      }
      if (have_cmd) {
        // 悬停于最后指令点 (反馈毛刺守卫)
        enter_hover_at(
          safe_axis(uav.position_x, c.position.x),
          safe_axis(uav.position_y, c.position.y),
          safe_axis(uav.position_z, c.position.z),
          c.yaw);
      } else {
        enter_hover_at(uav.position_x, uav.position_y, uav.position_z, uav.yaw);
      }
      return;
    }

    if (gh_traj_) {
      // 转发为 TRAJECTORY 指令 (ENU 直传; 换轴由 mavros 在 setpoint_raw 内完成)
      cmd.control_mode = easyfly_msgs::msg::UAVCommand::TRAJECTORY;
      cmd.position_x = c.position.x;
      cmd.position_y = c.position.y;
      cmd.position_z = c.position.z;
      cmd.velocity_x = c.velocity.x;
      cmd.velocity_y = c.velocity.y;
      cmd.velocity_z = c.velocity.z;
      cmd.yaw = c.yaw;
      pub_uav_cmd_->publish(cmd);

      const double dx = traj_target_x_ - uav.position_x;
      const double dy = traj_target_y_ - uav.position_y;
      const double dz = traj_target_z_ - uav.position_z;
      const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

      auto fb = std::make_shared<FollowTrajectory::Feedback>();
      fb->distance_remaining = dist;
      fb->stream_active = true;
      gh_traj_->publish_feedback(fb);

      // Keep the action boundary fail-safe even if the control tick crossed
      // it while publishing the last valid command.
      if (trajectoryDeadlineExpired(
          (now() - traj_accept_time_).seconds(), action_deadline_seconds))
      {
        RCLCPP_ERROR(
          get_logger(),
          "FollowTrajectory 超时 (dist=%.2f), abort + 就地悬停", dist);
        auto r = std::make_shared<FollowTrajectory::Result>();
        r->success = false; r->message = "timeout";
        gh_traj_->abort(r); gh_traj_.reset(); traj_session_id_.clear();
        enter_hover_at(
          safe_axis(uav.position_x, c.position.x),
          safe_axis(uav.position_y, c.position.y),
          safe_axis(uav.position_z, c.position.z),
          c.yaw);
        return;
      }

      if (sequence != traj_counted_sequence_) {
        traj_counted_sequence_ = sequence;
        if (dist < traj_accept_radius_) {++traj_in_radius_frames_;} else {
          traj_in_radius_frames_ = 0;
        }
      }

      if (traj_in_radius_frames_ >= 5) {
        RCLCPP_INFO(get_logger(), "FollowTrajectory 到达终点 (dist=%.2f)", dist);
        auto r = std::make_shared<FollowTrajectory::Result>();
        r->success = true; r->message = "arrived";
        gh_traj_->succeed(r); gh_traj_.reset(); traj_session_id_.clear();
        hover_x_ = traj_target_x_; hover_y_ = traj_target_y_;
        hover_z_ = traj_target_z_; hover_yaw_ = c.yaw;
        x_ramp_ = traj_target_x_; y_ramp_ = traj_target_y_; z_ramp_ = traj_target_z_;
        active_ = HOVER;
        action_reserved_ = false;
      }
    }
  }
}

rclcpp_action::GoalResponse FlightController::handle_takeoff_goal(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Takeoff::Goal> g)
{
  std::lock_guard<std::recursive_mutex> lk(flight_mtx_);
  const auto readiness = takeoff_readiness();
  if (!std::isfinite(g->target_altitude) || !std::isfinite(g->climb_speed) ||
    g->target_altitude <= 0.0 || g->target_altitude > max_altitude_ ||
    g->climb_speed < 0.0 || g->climb_speed > max_horizontal_speed_)
  {
    RCLCPP_WARN(
      get_logger(),
      "Takeoff admission rejected goal_uuid=%s reason=invalid_goal altitude=%.17g "
      "climb_speed=%.17g snapshot=%s",
      rclcpp_action::to_string(uuid).c_str(), g->target_altitude, g->climb_speed,
      readiness.details.front().c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!readiness.ready.front()) {
    RCLCPP_WARN(
      get_logger(), "Takeoff admission rejected goal_uuid=%s snapshot=%s",
      rclcpp_action::to_string(uuid).c_str(), readiness.details.front().c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  action_reserved_ = true;
  reserved_goal_id_ = uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse FlightController::handle_takeoff_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<Takeoff>> gh)
{
  std::lock_guard<std::recursive_mutex> lk(flight_mtx_);
  if (!gh_takeoff_ || gh_takeoff_ != gh) {return rclcpp_action::CancelResponse::REJECT;}
  return rclcpp_action::CancelResponse::ACCEPT;
}

void FlightController::handle_takeoff_accepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<Takeoff>> gh)
{
  auto g = gh->get_goal();
  std::chrono::steady_clock::time_point receipt;
  const auto uav = get_uav(&receipt);
  bool started = false;
  {
    std::lock_guard<std::recursive_mutex> lk(flight_mtx_);
    if (!command_uncertain_ && action_reserved_ && reserved_goal_id_ == gh->get_goal_id() &&
      active_ == NONE && (!gh_takeoff_ || gh_takeoff_ == gh) &&
      flight_state_ready(uav, receipt) && !uav.armed && uav.flight_mode <= 7)
    {
      takeoff_target_z_ = g->target_altitude;
      takeoff_speed_ = (g->climb_speed > 0) ? g->climb_speed : 1.0;
      takeoff_start_x_ = uav.position_x; takeoff_start_y_ = uav.position_y;
      takeoff_start_z_ = uav.position_z; takeoff_start_yaw_ = uav.yaw;
      takeoff_ramp_z_ = uav.position_z;
      takeoff_baseline_mode_ = uav.flight_mode;
      takeoff_offboard_confirmed_ = uav.flight_mode == 3;
      takeoff_preparation_running_ = true;
      takeoff_stop_reason_.clear();
      gh_takeoff_ = gh;
      active_ = TAKEOFF_PREPARING;
      started = true;
    }
  }
  if (!started) {
    finish_takeoff_goal(gh, "takeoff transaction was superseded");
    return;
  }
  ScopeExit preparation_cleanup{[this, gh]() {
      std::lock_guard<std::recursive_mutex> lk(flight_mtx_);
      if (gh_takeoff_ == gh && takeoff_preparation_running_) {
        finish_takeoff_goal(gh, "takeoff preparation ended without confirmed control");
      }
    }};

  const auto still_current = [this, &gh]() {
      std::lock_guard<std::recursive_mutex> lk(flight_mtx_);
      std::chrono::steady_clock::time_point receipt;
      const auto current = get_uav(&receipt);
      return rclcpp::ok(get_node_base_interface()->get_context()) &&
             action_reserved_ && gh_takeoff_ == gh && active_ == TAKEOFF_PREPARING &&
             takeoff_stop_reason_.empty() &&
             !gh->is_canceling() && flight_state_ready(current, receipt) &&
             (current.flight_mode == 3 ||
             (!takeoff_offboard_confirmed_ && !current.armed &&
             current.flight_mode == takeoff_baseline_mode_));
    };
  const auto arm_allowed = [this, &still_current]() {
      std::lock_guard<std::recursive_mutex> lk(flight_mtx_);
      return still_current() && get_uav().flight_mode == 3;
    };

  // ---- "先建流, 后解锁" 流程 (修复降落后无法再次起飞) ----
  // 背景: PX4 降落后停留在 OFFBOARD 模式 (或 AUTO.LAND 残留), 而
  // ① OFFBOARD 模式 arm 要求 offboard_control_mode 数据新鲜 (offboardCheck);
  // ② POSCTL 等手动模式 arm 要求遥控器 (SITL 无 RC);
  // ③ AUTO.LOITER (SITL 开机默认) 可直接 arm。
  // 统一解法: 先发初始 setpoint 建流 → 切 OFFBOARD → 等流被 PX4 处理注册
  // (offboard_control_mode.position 置位需要至少一个 setpoint 被处理) →
  // arm (带重试) → 校验模式。
  easyfly_msgs::msg::UAVCommand ic; ic.control_mode = easyfly_msgs::msg::UAVCommand::POSITION;
  stamp_command(ic);
  ic.position_x = uav.position_x; ic.position_y = uav.position_y;
  ic.position_z = uav.position_z; ic.yaw = takeoff_start_yaw_;
  {
    std::lock_guard<std::recursive_mutex> lk(flight_mtx_);
    if (!still_current()) {
      finish_takeoff_goal(gh, "takeoff transaction interrupted before initial stream");
      return;
    }
    pub_uav_cmd_->publish(ic);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));  // 建流

  const auto abort_preparing = [this, &gh](const std::string & message) {
      finish_takeoff_goal(gh, message);
    };
  if (!still_current()) {
    finish_takeoff_goal(gh, "takeoff transaction interrupted");
    return;
  }

  if (get_uav().flight_mode != 3) {
    if (!call_set_mode(3, still_current)) {
      abort_preparing("cannot switch to OFFBOARD (setpoint stream active?)");
      return;
    }
    // 切入后 PX4 需处理至少一个 setpoint 才置位 offboard_control_mode,
    // 立即 arm 会被拒 (实测 16ms 间隔必拒) — 等待注册
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  if (!still_current()) {
    finish_takeoff_goal(gh, "takeoff transaction interrupted");
    return;
  }
  // Service success only acknowledges a command; establish actual fresh
  // OFFBOARD before allowing any arm request.
  if (!wait_mode_engaged(3, 3.0) || !arm_allowed()) {
    abort_preparing("OFFBOARD not engaged before arming");
    return;
  }
  {
    std::lock_guard<std::recursive_mutex> lk(flight_mtx_);
    takeoff_offboard_confirmed_ = true;
  }
  // arm 重试 ×5 间隔 2s (总窗口 10s) — 兜底两类瞬态拒绝:
  // ① 切 OFFBOARD 后 setpoint 未被处理注册 (offboardCheck data_is_recent)
  // ② EKF/GPS 收敛边缘 ("ekf2 missing data" → Ready for takeoff 之间的窗口)
  bool armed = false;
  for (int i = 0; i < 5 && !armed; ++i) {
    if (!arm_allowed()) {
      finish_takeoff_goal(gh, "takeoff transaction interrupted");
      return;
    }
    armed = call_arm(true, arm_allowed);
    if (!armed) {
      const auto retry_deadline = std::chrono::steady_clock::now() + 2s;
      while (arm_allowed() && std::chrono::steady_clock::now() < retry_deadline) {
        std::this_thread::sleep_for(50ms);
      }
    }
  }
  if (!armed) {
    RCLCPP_ERROR(
      get_logger(), "arm 失败 (已重试 5 次/10s, OFFBOARD 流活跃; 检查 EKF/GPS 是否收敛)");
    abort_preparing("arm failed (retried 10s, OFFBOARD stream active)");
    return;
  }
  if (!still_current()) {
    finish_takeoff_goal(gh, "takeoff transaction interrupted");
    return;
  }
  // Q9: 校验 PX4 真正切入 OFFBOARD (mode_sent 只代表命令已发出)
  if (!wait_mode_engaged(3, 3.0)) {
    RCLCPP_ERROR(
      get_logger(), "OFFBOARD 未确认切入 (当前模式=%d), 起飞中止且不做飞行补偿",
      static_cast<int>(get_uav().flight_mode));
    abort_preparing("OFFBOARD not engaged");
    return;
  }
  const auto arm_confirmation_deadline = std::chrono::steady_clock::now() + 3s;
  while (arm_allowed() && !get_uav().armed &&
    std::chrono::steady_clock::now() < arm_confirmation_deadline)
  {
    std::this_thread::sleep_for(50ms);
  }
  if (!arm_allowed() || !get_uav().armed) {
    abort_preparing("fresh armed feedback not confirmed; no automatic compensation");
    return;
  }
  std::lock_guard<std::recursive_mutex> lk(flight_mtx_);
  if (!still_current()) {
    finish_takeoff_goal(gh, "takeoff transaction interrupted");
    return;
  }
  takeoff_start_time_ = now();
  takeoff_deadline_ = now() + rclcpp::Duration::from_seconds(
    std::max(1.0, takeoff_target_z_ - takeoff_start_z_) / takeoff_speed_ * 3.0 + 10.0);
  active_ = TAKEOFF;
  takeoff_preparation_running_ = false;
}

rclcpp_action::GoalResponse FlightController::handle_land_goal(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Land::Goal> g)
{
  if (!std::isfinite(g->landing_speed) || g->landing_speed != 0.0) {
    RCLCPP_WARN(get_logger(), "拒绝 Land: PX4 AUTO.LAND 暂不支持 landing_speed，必须为 0");
    return rclcpp_action::GoalResponse::REJECT;
  }
  std::chrono::steady_clock::time_point receipt;
  const auto uav = get_uav(&receipt);
  std::lock_guard<std::recursive_mutex> lk(flight_mtx_);
  if (!precision_land_authorized(g->precision_landing_session_id) ||
    (precision_reserved_ && (!precision_child_admission_pending_ || precision_child_)) ||
    (precision_reserved_ && !precision_handoff_ready()) ||
    command_uncertain_ || active_ != HOVER || action_reserved_ ||
    !flight_state_ready(uav, receipt) || !uav.armed || uav.flight_mode != 3)
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  action_reserved_ = true;
  reserved_goal_id_ = uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse FlightController::handle_land_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<Land>> gh)
{
  // Q3: 恢复悬停 — PX4 正处于 AUTO.LAND:
  //   1. 先发布悬停 setpoint 恢复数据流 (OFFBOARD 要求 >2Hz setpoint 才允许切入,
  //      降落期间 control_loop 不发指令, bridge 已超时停流)
  //   2. 再切 OFFBOARD, 之后 control_loop 的 HOVER 分支接管
  // 悬停点用 land 发起前的 hover_* (即当时我们自己的 setpoint, 无毛刺风险)
  // Serialize this whole recovery transaction with the AUTO.LAND request and
  // confirmation.  The lock is intentionally acquired before flight_mtx_ and
  // is held across blocking service/mode confirmation calls, while
  // flight_mtx_ is only held for short state snapshots/transitions below.
  std::unique_lock<std::mutex> transaction_lk(land_transaction_mtx_);
  {
    std::lock_guard<std::recursive_mutex> lk(flight_mtx_);
    // A cancel request is only valid after AUTO.LAND has been confirmed.  In
    // LAND_PREPARING the accepted callback owns the mode transaction; waiting
    // for transaction_lk above lets it finish, after which this check sees
    // either LANDING (continue recovery) or HOVER (reject safely).
    std::chrono::steady_clock::time_point receipt;
    const auto uav = get_uav(&receipt);
    if (command_uncertain_ || !gh_land_ || gh_land_ != gh || active_ != LANDING ||
      !flight_state_ready(uav, receipt) || !uav.armed || uav.flight_mode != 7)
    {
      return rclcpp_action::CancelResponse::REJECT;
    }
    active_ = LAND_CANCELING;
    land_transaction_running_ = true;
    land_stop_reason_.clear();
    land_cancel_offboard_requested_ = false;
  }
  ScopeExit cancellation_cleanup{[this, gh]() {
      std::lock_guard<std::recursive_mutex> lk(flight_mtx_);
      if (gh_land_ == gh) {
        land_transaction_running_ = false;
        if (std::uncaught_exceptions() > 0) {
          finish_land_goal(gh, "Land cancellation interrupted", false);
        }
      }
    }};
  const auto recovery_current = [this, gh]() {
      std::lock_guard<std::recursive_mutex> lk(flight_mtx_);
      std::chrono::steady_clock::time_point receipt;
      const auto uav = get_uav(&receipt);
      return gh_land_ == gh && active_ == LAND_CANCELING && land_stop_reason_.empty() &&
             flight_state_ready(uav, receipt) && uav.armed &&
             (uav.flight_mode == 7 || (land_cancel_offboard_requested_ && uav.flight_mode == 3));
    };
  // 先让 control_loop 连续建立一小段 OFFBOARD setpoint 流，再请求退出 AUTO.LAND。
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  if (!recovery_current()) {
    finish_land_goal(gh, "Land cancellation recovery denied by current state", false);
    return rclcpp_action::CancelResponse::REJECT;
  }
  const auto mark_request = [this, gh, &recovery_current]() {
      std::lock_guard<std::recursive_mutex> lk(flight_mtx_);
      if (!recovery_current() || get_uav().flight_mode != 7) {return false;}
      land_cancel_offboard_requested_ = true;
      return true;
    };
  if (!call_set_mode(3, recovery_current, mark_request) || !wait_mode_engaged(3, 3.0) ||
    !recovery_current())
  {
    RCLCPP_ERROR(
      get_logger(),
      "取消降落: 切回 OFFBOARD 失败, PX4 将继续 AUTO.LAND 降落!");
    std::lock_guard<std::recursive_mutex> lk(flight_mtx_);
    if (gh_land_ == gh && active_ == LAND_CANCELING) {
      std::chrono::steady_clock::time_point receipt;
      const auto uav = get_uav(&receipt);
      if (flight_state_ready(uav, receipt) && uav.armed && uav.flight_mode == 7 &&
        land_stop_reason_.empty())
      {
        active_ = LANDING;
        land_cancel_offboard_requested_ = false;
      } else {
        finish_land_goal(gh, "Land cancellation not confirmed; control stream stopped", false);
      }
    }
    return rclcpp_action::CancelResponse::REJECT;
  }
  std::lock_guard<std::recursive_mutex> lk(flight_mtx_);
  if (!recovery_current()) {
    return rclcpp_action::CancelResponse::REJECT;
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void FlightController::handle_land_accepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<Land>> gh)
{
  // Serialize the complete AUTO.LAND mode transaction with cancellation.
  // Do not retain flight_mtx_ while call_set_mode()/wait_mode_engaged() block;
  // control_loop must continue publishing the LAND_PREPARING hover setpoint.
  std::unique_lock<std::mutex> transaction_lk(land_transaction_mtx_);
  bool started = false;
  {
    std::lock_guard<std::recursive_mutex> lk(flight_mtx_);
    // 先登记 handle 并进入准备态，再执行可能阻塞的 mode service；这样
    // cancel/takeover 在 AUTO.LAND 确认窗口内也能找到并收尾该 goal。
    std::chrono::steady_clock::time_point receipt;
    const auto uav = get_uav(&receipt);
    if (precision_land_authorized(gh->get_goal()->precision_landing_session_id) &&
      (!precision_reserved_ || precision_handoff_ready()) &&
      !command_uncertain_ && action_reserved_ && reserved_goal_id_ == gh->get_goal_id() &&
      active_ == HOVER && (!gh_land_ || gh_land_ == gh) &&
      flight_state_ready(uav, receipt) && uav.armed && uav.flight_mode == 3)
    {
      gh_land_ = gh;
      active_ = LAND_PREPARING;
      land_transaction_running_ = true;
      land_stop_reason_.clear();
      land_cancel_offboard_requested_ = false;
      started = true;
    }
  }
  if (!started) {
    finish_land_goal(gh, "land transaction was superseded", true);
    return;
  }
  ScopeExit land_cleanup{[this, gh]() {
      std::lock_guard<std::recursive_mutex> lk(flight_mtx_);
      if (gh_land_ == gh && land_transaction_running_) {
        land_transaction_running_ = false;
        if (active_ == LAND_PREPARING) {
          finish_land_goal(gh, "Land preparation ended without confirmed mode", false);
        }
      }
    }};
  const auto still_preparing_owned = [this, gh]() {
      std::lock_guard<std::recursive_mutex> lk(flight_mtx_);
      std::chrono::steady_clock::time_point receipt;
      const auto uav = get_uav(&receipt);
      return gh_land_ == gh && active_ == LAND_PREPARING && land_stop_reason_.empty() &&
             flight_state_ready(uav, receipt) && uav.armed &&
             (uav.flight_mode == 3 || uav.flight_mode == 7);
    };

  const auto handoff_before_send = [this, gh]() {
      std::lock_guard<std::recursive_mutex> lk(flight_mtx_);
      return gh->get_goal()->precision_landing_session_id.empty() ||
             precision_handoff_ready();
    };
  if (!call_set_mode(7, still_preparing_owned, handoff_before_send) ||
    !wait_mode_engaged(7, 3.0))
  {
    if (command_uncertain_) {
      RCLCPP_ERROR(
        get_logger(), "AUTO.LAND command outcome uncertain: setpoint stream stopped; "
        "flight fault latch set; manual audit required");
    } else {
      RCLCPP_ERROR(
        get_logger(), "AUTO.LAND transaction not confirmed; abort Land; "
        "hover retained only with fresh owned OFFBOARD evidence");
    }
    finish_land_goal(gh, "AUTO.LAND not engaged", true);
    return;
  }
  bool still_preparing = false;
  {
    std::lock_guard<std::recursive_mutex> lk(flight_mtx_);
    std::chrono::steady_clock::time_point receipt;
    const auto uav = get_uav(&receipt);
    if (gh_land_ == gh && action_reserved_ && active_ == LAND_PREPARING &&
      land_stop_reason_.empty() && flight_state_ready(uav, receipt) && uav.flight_mode == 7)
    {
      active_ = LANDING;
      land_transaction_running_ = false;
      land_start_steady_ = std::chrono::steady_clock::now();
      still_preparing = true;
    }
  }
  if (!still_preparing) {
    // 例如 cancel 已把事务切入 LAND_CANCELING，或 takeover 已清理它。
    // cancel 路径仍需持有该 goal，等待 cancel callback 返回 ACCEPT 后由
    // control_loop 调用 canceled()；此处若提前 finish 会在 goal 仍处于
    // EXECUTING 时走 abort()，把一次取消报告成中止。
    {
      std::lock_guard<std::recursive_mutex> lk(flight_mtx_);
      if (gh_land_ == gh && active_ == LAND_CANCELING) {
        return;
      }
    }
    // 其他中断路径不能让 accepted callback 裸返回留下一个永远 pending 的 goal。
    finish_land_goal(gh, "land transaction interrupted", true);
  }
}

rclcpp_action::GoalResponse FlightController::handle_waypoint_goal(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const GoToWaypoint::Goal> g)
{
  const bool yaw_ok = std::isfinite(g->target_yaw) || std::isnan(g->target_yaw);
  if (!std::isfinite(g->target_x) || !std::isfinite(g->target_y) ||
    !std::isfinite(g->target_z) || !yaw_ok || !std::isfinite(g->max_speed) ||
    !std::isfinite(g->acceptance_radius) || g->target_z < 0.0 ||
    g->target_z > max_altitude_ || g->max_speed < 0.0 ||
    g->max_speed > max_horizontal_speed_ || g->acceptance_radius < 0.0)
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  std::chrono::steady_clock::time_point receipt;
  const auto uav = get_uav(&receipt);
  const double distance = std::hypot(
    g->target_x - uav.position_x, g->target_y - uav.position_y);
  if (!std::isfinite(max_goal_distance_) || max_goal_distance_ <= 0.0 ||
    !std::isfinite(distance) || distance > max_goal_distance_)
  {
    RCLCPP_WARN(
      get_logger(),
      "拒绝 GoToWaypoint: 目标水平距离 %.2fm 超出上限 %.2fm",
      distance, max_goal_distance_);
    return rclcpp_action::GoalResponse::REJECT;
  }
  std::lock_guard<std::recursive_mutex> lk(flight_mtx_);
  if (command_uncertain_ || precision_reserved_ || active_ != HOVER || action_reserved_ ||
    !flight_state_ready(uav, receipt) || !uav.armed || uav.flight_mode != 3)
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  action_reserved_ = true;
  reserved_goal_id_ = uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse FlightController::handle_waypoint_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<GoToWaypoint>> gh)
{
  std::lock_guard<std::recursive_mutex> lk(flight_mtx_);
  if (!gh_waypoint_ || gh_waypoint_ != gh) {return rclcpp_action::CancelResponse::REJECT;}
  return rclcpp_action::CancelResponse::ACCEPT;
}

void FlightController::handle_waypoint_accepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<GoToWaypoint>> gh)
{
  auto g = gh->get_goal();
  std::chrono::steady_clock::time_point receipt;
  uint64_t sequence;
  const auto uav = get_uav(&receipt, &sequence);
  bool started = false;
  {
    std::lock_guard<std::recursive_mutex> lk(flight_mtx_);
    if (!command_uncertain_ && action_reserved_ && reserved_goal_id_ == gh->get_goal_id() &&
      active_ == HOVER &&
      (!gh_waypoint_ || gh_waypoint_ == gh) && flight_state_ready(uav, receipt) &&
      uav.armed && uav.flight_mode == 3)
    {
      wp_target_x_ = g->target_x; wp_target_y_ = g->target_y; wp_target_z_ = g->target_z;
      wp_target_yaw_ = std::isfinite(g->target_yaw) ? g->target_yaw : uav.yaw;
      wp_max_speed_ = (g->max_speed > 0) ? g->max_speed : 2.0;
      wp_accept_radius_ = (g->acceptance_radius > 0) ? g->acceptance_radius : 0.5;
      double dx = wp_target_x_ - uav.position_x, dy = wp_target_y_ - uav.position_y,
        dz = wp_target_z_ - uav.position_z;
      wp_start_dist_ = std::sqrt(dx * dx + dy * dy + dz * dz);
      wp_start_x_ = uav.position_x; wp_start_y_ = uav.position_y;
      x_ramp_ = uav.position_x; y_ramp_ = uav.position_y; z_ramp_ = uav.position_z;
      wp_in_radius_frames_ = 0;            // Q16: 完成判定去抖计数清零
      wp_counted_sequence_ = sequence;
      // Q5: 超时 = 名义飞行时间 × 3 + 15s 余量
      wp_deadline_ = now() + rclcpp::Duration::from_seconds(
        wp_start_dist_ / std::max(wp_max_speed_, 0.1) * 3.0 + 15.0);
      gh_waypoint_ = gh; active_ = WAYPOINT;
      started = true;
    }
  }
  if (!started) {
    finish_waypoint_goal(gh, "waypoint transaction was superseded");
  }
}

// ================================================================
// M4.3: /follow_trajectory action (EGO-Planner 轨迹跟随)
// ================================================================

rclcpp_action::GoalResponse FlightController::handle_traj_goal(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const FollowTrajectory::Goal> g)
{
  if (g->navigation_session_id.size() > 128 ||
    !std::isfinite(g->target_x) || !std::isfinite(g->target_y) ||
    !std::isfinite(g->target_z) || !std::isfinite(g->acceptance_radius) ||
    !std::isfinite(g->timeout) || g->target_z < 0.0 ||
    g->target_z > max_altitude_ || g->acceptance_radius < 0.0 || g->timeout < 0.0)
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  // 仅 HOVER 可进入 (与既有单动作槽互斥); 需已 arm
  std::chrono::steady_clock::time_point receipt;
  const auto uav = get_uav(&receipt);
  const double distance = std::hypot(
    g->target_x - uav.position_x, g->target_y - uav.position_y);
  if (!std::isfinite(max_goal_distance_) || max_goal_distance_ <= 0.0 ||
    !std::isfinite(distance) || distance > max_goal_distance_)
  {
    RCLCPP_WARN(
      get_logger(),
      "拒绝 FollowTrajectory: 目标水平距离 %.2fm 超出上限 %.2fm",
      distance, max_goal_distance_);
    return rclcpp_action::GoalResponse::REJECT;
  }
  std::lock_guard<std::recursive_mutex> lk(flight_mtx_);
  if (command_uncertain_ || precision_reserved_ || active_ != HOVER || action_reserved_ ||
    !uav.armed ||
    !flight_state_ready(uav, receipt) || uav.flight_mode != 3)
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  // 与单动作槽 reservation 在同一把锁下锁存，确保 goal 之后到达的
  // PositionCommand 必须使用更高的 trajectory_id 才能启动新轨迹。
  traj_baseline_id_ = traj_seen_max_id_;
  traj_session_id_ = g->navigation_session_id;
  traj_goal_pending_ = true;
  action_reserved_ = true;
  reserved_goal_id_ = uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse FlightController::handle_traj_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowTrajectory>> gh)
{
  std::lock_guard<std::recursive_mutex> state_lk(flight_mtx_);
  if (!gh_traj_ || gh_traj_ != gh) {return rclcpp_action::CancelResponse::REJECT;}
  return rclcpp_action::CancelResponse::ACCEPT;
}

void FlightController::handle_traj_accepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowTrajectory>> gh)
{
  auto g = gh->get_goal();
  bool started = false;
  double timeout = 120.0;
  {
    std::lock_guard<std::recursive_mutex> state_lk(flight_mtx_);
    std::chrono::steady_clock::time_point receipt;
    uint64_t sequence;
    const auto uav = get_uav(&receipt, &sequence);
    if (!command_uncertain_ && action_reserved_ && reserved_goal_id_ == gh->get_goal_id() &&
      active_ == HOVER && (!gh_traj_ || gh_traj_ == gh) &&
      flight_state_ready(uav, receipt) && uav.armed && uav.flight_mode == 3)
    {
      traj_target_x_ = g->target_x;
      traj_target_y_ = g->target_y;
      traj_target_z_ = g->target_z;
      traj_accept_radius_ = (g->acceptance_radius > 0) ? g->acceptance_radius : 0.8;
      timeout = (g->timeout > 0) ? g->timeout : 120.0;

      {
        std::lock_guard<std::mutex> lk(traj_mtx_);
        traj_cmd_received_.store(false);           // 起始等首帧 (startup grace)
        traj_cmd_time_ = rclcpp::Time(0, 0, now().get_clock_type());
      }
      traj_accept_time_ = now();
      traj_deadline_ = now() + rclcpp::Duration::from_seconds(timeout);
      traj_in_radius_frames_ = 0;
      traj_counted_sequence_ = sequence;
      traj_active_id_ = 0;
      gh_traj_ = gh;
      traj_goal_pending_ = false;
      active_ = TRAJECTORY_WAIT;
      started = true;
    }
  }
  if (!started) {
    finish_traj_goal(gh, "trajectory transaction was superseded");
    return;
  }
  RCLCPP_INFO(
    get_logger(),
    "FollowTrajectory 开始: 终点 (%.2f, %.2f, %.2f) r=%.2f timeout=%.0fs",
    traj_target_x_, traj_target_y_, traj_target_z_, traj_accept_radius_, timeout);
}

void FlightController::stop_owned_actions(const std::string & reason)
{
  std::lock_guard<std::recursive_mutex> lk(flight_mtx_);
  if (gh_takeoff_ && takeoff_preparation_running_) {
    takeoff_stop_reason_ = reason;
    return;
  }
  if (gh_land_ && land_transaction_running_) {
    land_stop_reason_ = reason;
    return;
  }
  if (gh_takeoff_) {finish_takeoff_goal(gh_takeoff_, reason);}
  if (gh_land_) {finish_land_goal(gh_land_, reason, false);}
  if (gh_waypoint_) {finish_waypoint_goal(gh_waypoint_, reason);}
  if (gh_traj_) {finish_traj_goal(gh_traj_, reason);}
  active_ = NONE;
  action_reserved_ = false;
  traj_goal_pending_ = false;
  traj_session_id_.clear();
  land_cancel_offboard_requested_ = false;
  wp_in_radius_frames_ = traj_in_radius_frames_ = 0;
}

void FlightController::enter_hover_at(double x, double y, double z, double yaw)
{
  std::lock_guard<std::recursive_mutex> lk(flight_mtx_);
  std::chrono::steady_clock::time_point receipt;
  const auto uav = get_uav(&receipt);
  if (!rclcpp::ok(get_node_base_interface()->get_context()) ||
    !flight_state_ready(uav, receipt) || !uav.armed || uav.flight_mode != 3)
  {
    active_ = NONE;
    action_reserved_ = false;
    return;
  }
  hover_x_ = x; hover_y_ = y; hover_z_ = z; hover_yaw_ = yaw;
  // 立即发布一条 setpoint: 恢复/保持 OFFBOARD 数据流 (bridge 1s 无指令即停流)
  easyfly_msgs::msg::UAVCommand cmd;
  cmd.control_mode = easyfly_msgs::msg::UAVCommand::POSITION;
  stamp_command(cmd);
  cmd.position_x = x; cmd.position_y = y; cmd.position_z = z; cmd.yaw = yaw;
  pub_uav_cmd_->publish(cmd);
  active_ = HOVER;
  action_reserved_ = false;
}

void FlightController::finish_takeoff_goal(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<Takeoff>> gh,
  const std::string & message)
{
  if (!gh) {return;}
  {
    std::lock_guard<std::recursive_mutex> lk(flight_mtx_);
    if (gh_takeoff_ == gh ||
      (action_reserved_ && reserved_goal_id_ == gh->get_goal_id()))
    {
      gh_takeoff_.reset();
      takeoff_preparation_running_ = false;
      takeoff_stop_reason_.clear();
      active_ = NONE;
      action_reserved_ = false;
    }
  }
  auto result = std::make_shared<Takeoff::Result>();
  result->success = false;
  result->message = command_uncertain_ ?
    "command_result_unknown: flight fault latch set; manual audit required" : message;
  try {
    if (gh->is_canceling()) {gh->canceled(result);} else {gh->abort(result);}
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "结束 Takeoff goal 异常: %s", e.what());
  }
}

void FlightController::finish_land_goal(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<Land>> gh,
  const std::string & message, bool keep_hover)
{
  if (!gh) {return;}
  {
    std::lock_guard<std::recursive_mutex> lk(flight_mtx_);
    if (gh_land_ == gh ||
      (action_reserved_ && reserved_goal_id_ == gh->get_goal_id()))
    {
      gh_land_.reset();
      land_transaction_running_ = false;
      land_stop_reason_.clear();
      land_cancel_offboard_requested_ = false;
      std::chrono::steady_clock::time_point receipt;
      const auto uav = get_uav(&receipt);
      active_ = keep_hover && !command_uncertain_ && flight_state_ready(uav, receipt) &&
        uav.armed && uav.flight_mode == 3 ? HOVER : NONE;
      action_reserved_ = false;
    }
  }
  auto result = std::make_shared<Land::Result>();
  result->success = false;
  result->message = command_uncertain_ ?
    "command_result_unknown: flight fault latch set; manual audit required" : message;
  try {
    if (gh->is_canceling()) {gh->canceled(result);} else {gh->abort(result);}
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "结束 Land goal 异常: %s", e.what());
  }
}

void FlightController::finish_waypoint_goal(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<GoToWaypoint>> gh,
  const std::string & message)
{
  if (!gh) {return;}
  {
    std::lock_guard<std::recursive_mutex> lk(flight_mtx_);
    if (gh_waypoint_ == gh ||
      (action_reserved_ && reserved_goal_id_ == gh->get_goal_id()))
    {
      gh_waypoint_.reset();
      active_ = NONE;
      action_reserved_ = false;
    }
  }
  auto result = std::make_shared<GoToWaypoint::Result>();
  result->success = false;
  result->message = message;
  try {
    if (gh->is_canceling()) {gh->canceled(result);} else {gh->abort(result);}
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "结束 GoToWaypoint goal 异常: %s", e.what());
  }
}

void FlightController::finish_traj_goal(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowTrajectory>> gh,
  const std::string & message)
{
  if (!gh) {return;}
  {
    std::lock_guard<std::recursive_mutex> lk(flight_mtx_);
    if (gh_traj_ == gh ||
      (action_reserved_ && reserved_goal_id_ == gh->get_goal_id()))
    {
      gh_traj_.reset();
      traj_goal_pending_ = false;
      traj_session_id_.clear();
      active_ = NONE;
      action_reserved_ = false;
    }
  }
  auto result = std::make_shared<FollowTrajectory::Result>();
  result->success = false;
  result->message = message;
  try {
    if (gh->is_canceling()) {gh->canceled(result);} else {gh->abort(result);}
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "结束 FollowTrajectory goal 异常: %s", e.what());
  }
}

void FlightController::stamp_command(easyfly_msgs::msg::UAVCommand & cmd)
{
  cmd.header.stamp = now();
  cmd.header.frame_id = "odom";
}

bool FlightController::flight_state_ready(
  const easyfly_msgs::msg::UAVState & uav,
  std::chrono::steady_clock::time_point receipt) const
{
  return state_evidence_fresh(uav, receipt) && uav.pose_valid && uav.attitude_valid &&
         std::isfinite(uav.position_x) && std::isfinite(uav.position_y) &&
         std::isfinite(uav.position_z) && std::isfinite(uav.yaw) &&
         std::isfinite(uav.velocity_x) && std::isfinite(uav.velocity_y) &&
         std::isfinite(uav.velocity_z);
}

bool FlightController::state_evidence_fresh(
  const easyfly_msgs::msg::UAVState & uav,
  std::chrono::steady_clock::time_point receipt) const
{
  const auto age = std::chrono::steady_clock::now() - receipt;
  if (!state_received_ || !uav.connected || !rclcpp::ok(get_node_options().context()) ||
    command_uncertain_ || uav.header.stamp.sec < 0 ||
    uav.header.stamp.nanosec >= 1000000000U ||
    (uav.header.stamp.sec == 0 && uav.header.stamp.nanosec == 0) ||
    receipt == std::chrono::steady_clock::time_point{} ||
    age < std::chrono::steady_clock::duration::zero() || age > 1s) {return false;}
  const auto current = now();
  if (current.nanoseconds() <= 0) {return false;}
  const double source_age =
    (current - rclcpp::Time(uav.header.stamp, current.get_clock_type())).seconds();
  return source_age >= -0.2 && source_age <= 1.0;
}

easyfly_msgs::srv::GetCapabilities::Response FlightController::takeoff_readiness() const
{
  std::chrono::steady_clock::time_point receipt;
  uint64_t sequence;
  const auto uav = get_uav(&receipt, &sequence);
  const bool flight_ready = flight_state_ready(uav, receipt);
  const auto clock = now();
  const auto steady = std::chrono::steady_clock::now();
  const bool received = state_received_.load();
  const bool fault = command_uncertain_.load();
  const bool context_ok = rclcpp::ok(get_node_options().context());
  const bool stamp_valid = uav.header.stamp.sec >= 0 &&
    uav.header.stamp.nanosec < 1000000000U &&
    (uav.header.stamp.sec != 0 || uav.header.stamp.nanosec != 0);
  const int64_t source = static_cast<int64_t>(uav.header.stamp.sec) * 1000000000LL +
    uav.header.stamp.nanosec;
  const double source_age = static_cast<double>(clock.nanoseconds() - source) * 1e-9;
  const bool receipt_present = receipt != std::chrono::steady_clock::time_point{};
  const double receipt_age = receipt_present ?
    std::chrono::duration<double>(steady - receipt).count() : -1.0;
  const bool numeric_valid = std::isfinite(uav.position_x) &&
    std::isfinite(uav.position_y) && std::isfinite(uav.position_z) &&
    std::isfinite(uav.yaw) && std::isfinite(uav.velocity_x) &&
    std::isfinite(uav.velocity_y) && std::isfinite(uav.velocity_z);
  const bool ready = !fault && !precision_reserved_ && active_ == NONE &&
    !action_reserved_ && flight_ready && !uav.armed && uav.flight_mode <= 7;
  const char * reason = "ready";
  if (fault) {reason = "command_uncertain";} else if (precision_reserved_) {
    reason = "precision_reserved";
  } else if (active_ != NONE || action_reserved_) {reason = "busy";} else if (!flight_ready) {
    if (!received) {reason = "state_not_received";} else if (!context_ok) {
      reason = "context_unavailable";
    } else if (!uav.connected) {reason = "disconnected";} else if (!stamp_valid) {
      reason = "source_stamp_invalid";
    } else if (!receipt_present || receipt_age < 0.0 || receipt_age > 1.0) {
      reason = "receipt_stale_or_missing";
    } else if (clock.nanoseconds() <= 0) {reason = "clock_not_initialized";} else if (
      source_age < -0.2 || source_age > 1.0)
    {
      reason = "source_stale_or_future";
    } else if (!uav.pose_valid || !uav.attitude_valid) {
      reason = "pose_or_attitude_invalid";
    } else if (!numeric_valid) {
      reason = "nonfinite_state";
    } else {reason = "flight_state_not_ready";}
  } else if (uav.armed) {reason = "already_armed";} else if (uav.flight_mode > 7) {
    reason = "unsupported_mode";
  }
  std::ostringstream details;
  details << std::boolalpha << "{\"reason\":\"" << reason << "\",\"ready\":" << ready
          << ",\"flight_ready\":" << flight_ready << ",\"state_received\":" << received
          << ",\"state_sequence\":" << sequence << ",\"state_source_ns\":" << source
          << ",\"clock_ns\":" << clock.nanoseconds() << ",\"source_age_s\":" << source_age
          << ",\"receipt_age_s\":" << receipt_age << ",\"connected\":" << uav.connected
          << ",\"pose_valid\":" << uav.pose_valid << ",\"attitude_valid\":" << uav.attitude_valid
          << ",\"numeric_valid\":" << numeric_valid << ",\"armed\":" << uav.armed
          << ",\"mode\":" << static_cast<int>(uav.flight_mode) << ",\"active\":" << active_
          << ",\"action_reserved\":" << action_reserved_
          << ",\"precision_reserved\":" << precision_reserved_
          << ",\"command_uncertain\":" << fault << ",\"context_ok\":" << context_ok << "}";
  easyfly_msgs::srv::GetCapabilities::Response response;
  response.header.stamp = clock;
  response.header.frame_id = "odom";
  response.names = {"takeoff"};
  response.implemented = {true};
  response.ready = {ready};
  response.details = {details.str()};
  return response;
}

// Q4: 反馈值毛刺守卫 — 与渐近值偏差 <3m (正常飞行中飞机紧跟 ramp) 才采信,
// 否则视为反馈尖峰 (M6 实测 local_position z=-10 毛刺), 回退无毛刺的 ramp 值
double FlightController::safe_axis(double fb, double ramp)
{
  return (std::isfinite(fb) && std::abs(fb - ramp) < 3.0) ? fb : ramp;
}

bool FlightController::wait_mode_engaged(uint8_t mode, double timeout_s)
{
  if (!std::isfinite(timeout_s) || timeout_s < 0.0) {return false;}
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(timeout_s);
  const auto context = get_node_base_interface()->get_context();
  while (rclcpp::ok(context) && std::chrono::steady_clock::now() < deadline) {
    std::chrono::steady_clock::time_point receipt;
    const auto uav = get_uav(&receipt);
    if (!flight_state_ready(uav, receipt)) {return false;}
    if (uav.flight_mode == mode) {return rclcpp::ok(context);}
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  std::chrono::steady_clock::time_point receipt;
  const auto uav = get_uav(&receipt);
  return rclcpp::ok(context) && flight_state_ready(uav, receipt) && uav.flight_mode == mode;
}

bool FlightController::land_disarm_confirmed(
  const easyfly_msgs::msg::UAVState & uav,
  std::chrono::steady_clock::time_point receipt) const
{
  return state_evidence_fresh(uav, receipt) && !uav.armed;
}

bool FlightController::call_arm(bool arm, const std::function<bool()> & guard)
{
  if (command_uncertain_) {return false;}
  auto r = std::make_shared<easyfly_msgs::srv::ArmMotors::Request>(); r->arm = arm;
  return call_service_bounded<easyfly_msgs::srv::ArmMotors>(
    cli_arm_, r, get_node_base_interface()->get_context(), guard, {}, [this]() {
      command_uncertain_ = true;
      RCLCPP_ERROR(
        get_logger(),
        "Arm command outcome uncertain: flight fault latch set; manual audit required");
    }, flight_mtx_);
}

bool FlightController::call_set_mode(
  uint8_t m, const std::function<bool()> & guard,
  const std::function<bool()> & before_send)
{
  if (command_uncertain_) {return false;}
  auto r = std::make_shared<easyfly_msgs::srv::SetFlightMode::Request>(); r->flight_mode = m;
  return call_service_bounded<easyfly_msgs::srv::SetFlightMode>(
    cli_mode_, r, get_node_base_interface()->get_context(), guard, before_send, [this]() {
      command_uncertain_ = true;
      RCLCPP_ERROR(
        get_logger(),
        "Mode command outcome uncertain: flight fault latch set; manual audit required");
    }, flight_mtx_);
}

}  // namespace easyfly_flight_control
