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

#include "easyfly_mavlink_bridge/mavlink_bridge.hpp"

#include <cmath>
#include <initializer_list>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace easyfly_mavlink_bridge
{

using namespace std::placeholders;

namespace
{
// A timeout does not cancel a remote ROS service. Distinguish unavailable
// (never sent) from an ambiguous request, and bound shutdown with steady time.
template<typename ServiceT>
typename ServiceT::Response::SharedPtr request_bounded(
  const typename rclcpp::Client<ServiceT>::SharedPtr & client,
  const typename ServiceT::Request::SharedPtr & request,
  const rclcpp::Context::SharedPtr & context, bool & uncertain)
{
  using namespace std::chrono_literals;
  uncertain = false;
  if (!rclcpp::ok(context)) {return nullptr;}
  bool sent = false;
  try {
    const auto discovery_deadline = std::chrono::steady_clock::now() + 3s;
    while (rclcpp::ok(context) && !client->service_is_ready()) {
      if (std::chrono::steady_clock::now() >= discovery_deadline) {return nullptr;}
      std::this_thread::sleep_for(50ms);
    }
    if (!rclcpp::ok(context)) {return nullptr;}
    sent = true;  // A send exception cannot prove no remote request exists.
    auto future = client->async_send_request(request);
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (rclcpp::ok(context) && std::chrono::steady_clock::now() < deadline) {
      if (future.wait_for(50ms) == std::future_status::ready) {return future.get();}
    }
    client->remove_pending_request(future);
  } catch (const std::exception &) {
    // No automatic compensating command: it could race the original request.
  }
  uncertain = sent;
  return nullptr;
}
}  // namespace

// ---- 四元数 → yaw (绕 Z 轴旋转) ----
double MavlinkBridge::quat_to_yaw(double x, double y, double z, double w)
{
  double siny_cosp = 2.0 * (w * z + x * y);
  double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

// ================================================================
// 构造
// ================================================================

MavlinkBridge::MavlinkBridge(const rclcpp::NodeOptions & options)
: Node("mavlink_bridge", options)
{
  auto sensor_qos = rclcpp::SensorDataQoS();
  auto default_qos = rclcpp::QoS(10);

  // ---- 参数 ----
  double publish_rate = declare_parameter("publish_rate", 20.0);
  setpoint_timeout_ = declare_parameter("setpoint_timeout", 1.0);
  motion_freshness_s_ = declare_parameter("motion_freshness_s", 1.0);
  slow_sensor_freshness_s_ = declare_parameter("slow_sensor_freshness_s", 5.0);
  for (const double timeout : {setpoint_timeout_, motion_freshness_s_, slow_sensor_freshness_s_}) {
    if (!std::isfinite(timeout) || timeout <= 0.0) {
      throw std::invalid_argument("bridge freshness/watchdog timeouts must be finite and positive");
    }
  }

  auto period_ms = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));

  // ---- decoder 订阅 ----
  sub_state_ = create_subscription<mavros_msgs::msg::State>(
    "/mavros/state", default_qos,
    std::bind(&MavlinkBridge::cb_state, this, _1));

  sub_local_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", sensor_qos,
    std::bind(&MavlinkBridge::cb_local_pose, this, _1));

  sub_local_vel_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "/mavros/local_position/velocity_local", sensor_qos,
    std::bind(&MavlinkBridge::cb_local_velocity, this, _1));

  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
    "/mavros/imu/data", sensor_qos,
    std::bind(&MavlinkBridge::cb_imu, this, _1));

  sub_altitude_ = create_subscription<mavros_msgs::msg::Altitude>(
    "/mavros/altitude", sensor_qos,
    std::bind(&MavlinkBridge::cb_altitude, this, _1));

  sub_global_pos_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    "/mavros/global_position/global", sensor_qos,
    std::bind(&MavlinkBridge::cb_global_position, this, _1));

  sub_battery_ = create_subscription<sensor_msgs::msg::BatteryState>(
    "/mavros/battery", sensor_qos,
    std::bind(&MavlinkBridge::cb_battery, this, _1));

  sub_rc_in_ = create_subscription<mavros_msgs::msg::RCIn>(
    "/mavros/rc/in", default_qos,
    std::bind(&MavlinkBridge::cb_rc_in, this, _1));

  // ---- encoder 订阅 ----
  sub_uav_command_ = create_subscription<easyfly_msgs::msg::UAVCommand>(
    "/uav_command", default_qos,
    std::bind(&MavlinkBridge::cb_uav_command, this, _1));

  // ---- 发布者 ----
  pub_uav_state_ = create_publisher<easyfly_msgs::msg::UAVState>(
    "/uav_state", default_qos);
  pub_flight_status_ = create_publisher<easyfly_msgs::msg::FlightStatus>(
    "/flight_status", default_qos);

  // ---- encoder 发布者 (到 mavros2 setpoint topics) ----
  pub_setpoint_pos_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "/mavros/setpoint_position/local", default_qos);

  pub_setpoint_vel_ = create_publisher<geometry_msgs::msg::Twist>(
    "/mavros/setpoint_velocity/cmd_vel_unstamped", default_qos);

  pub_setpoint_att_ = create_publisher<mavros_msgs::msg::AttitudeTarget>(
    "/mavros/setpoint_raw/attitude", default_qos);

  pub_setpoint_raw_ = create_publisher<mavros_msgs::msg::PositionTarget>(
    "/mavros/setpoint_raw/local", default_qos);

  // ---- service 专用 callback group (避免与 client response 死锁) ----
  srv_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // ---- 服务端 (封装 mavros2 服务) ----
  srv_arm_ = create_service<easyfly_msgs::srv::ArmMotors>(
    "/arm_motors",
    std::bind(&MavlinkBridge::cb_arm_motors, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_cb_group_);

  srv_mode_ = create_service<easyfly_msgs::srv::SetFlightMode>(
    "/set_flight_mode",
    std::bind(&MavlinkBridge::cb_set_flight_mode, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_cb_group_);

  // ---- mavros 客户端（预创建，避免每次回调内新建） ----
  client_arming_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
  client_set_mode_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
  srv_capabilities_ = create_service<easyfly_msgs::srv::GetCapabilities>(
    "/platform/mavlink_bridge/capabilities", [this](
      easyfly_msgs::srv::GetCapabilities::Request::SharedPtr,
      easyfly_msgs::srv::GetCapabilities::Response::SharedPtr response) {
      *response = command_capabilities();
    });

  // ---- 定时器 ----
  timer_state_ = create_wall_timer(
    period_ms,
    std::bind(&MavlinkBridge::publish_state, this));

  timer_setpoint_ = create_wall_timer(
    period_ms,
    std::bind(&MavlinkBridge::publish_setpoint, this));

  // ---- 初始化缓存 ----
  uav_.header.frame_id = "odom";
  uav_.armed = false;
  uav_.flight_mode = 0;
  uav_.flight_mode_name = "";

  last_cmd_time_ = now();

  RCLCPP_INFO(
    get_logger(),
    "MavlinkBridge started — %gHz state timer (one /uav_state per ROS stamp), "
    "encoder, /arm_motors, /set_flight_mode",
    publish_rate);
}

// ================================================================
// decoder callbacks
// ================================================================

void MavlinkBridge::cb_state(const mavros_msgs::msg::State::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(state_mtx_);
  uav_.connected = msg->connected;
  uav_.armed = msg->armed;
  uav_.flight_mode_name = msg->mode;

  // 字符串模式名 → 枚举值。
  // Q18 注意: 这是 easyfly 的语义抽象而非 PX4 模式直译 — AUTO.LOITER 映射为 2(POSCTL 等位)
  // 是历史设计 (自动悬停 ≈ 位置控制语义)。下游仅依赖 3(OFFBOARD)/6(RTL)/7(LAND) 三个值;
  // 需要精确区分 PX4 模式时请使用 flight_mode_name 字符串字段, 勿依赖此整数。
  // PX4 SITL 启动后默认进入 AUTO.LOITER。
  if (msg->mode == "MANUAL") {uav_.flight_mode = 0;} else if (msg->mode == "ALTCTL") {
    uav_.flight_mode = 1;
  } else if (msg->mode == "POSCTL") {uav_.flight_mode = 2;} else if (msg->mode == "OFFBOARD") {
    uav_.flight_mode = 3;
  } else if (msg->mode == "STABILIZED") {uav_.flight_mode = 4;} else if (msg->mode == "ACRO") {
    uav_.flight_mode = 5;
  } else if (msg->mode == "RTL") {uav_.flight_mode = 6;} else if (msg->mode == "LAND") {
    uav_.flight_mode = 7;
  } else if (msg->mode == "AUTO.LOITER") {
    uav_.flight_mode = 2;                                       // 等同于 POSCTL
  } else if (msg->mode == "AUTO.TAKEOFF") {
    uav_.flight_mode = 1;                                        // 等同于 ALTCTL
  } else if (msg->mode == "AUTO.LAND") {
    uav_.flight_mode = 7;                                      // 等同于 LAND
  } else if (msg->mode == "AUTO.RTL") {
    uav_.flight_mode = 6;                                      // 等同于 RTL
  } else {uav_.flight_mode = 99;}

  const auto & stamp = msg->header.stamp;
  const int64_t source = stamp.sec >= 0 && stamp.nanosec < 1000000000U ?
    static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec : 0;
  // Repeated/older observations cannot renew original receipt evidence.
  if (source > flight_status_watermark_) {
    flight_status_watermark_ = source;
    flight_status_receipt_ = std::chrono::steady_clock::now();
    flight_status_source_valid_ = true;
  } else {
    flight_status_source_valid_ = false;
  }
  flight_status_.header = msg->header;
  flight_status_.connected = msg->connected;
  flight_status_.armed = msg->armed;
  flight_status_.flight_mode = uav_.flight_mode;
  flight_status_.flight_mode_name = msg->mode;

  rcvd_armed_ = true;
  last_state_time_ = now();
  last_state_receipt_ = std::chrono::steady_clock::now();
}

void MavlinkBridge::cb_local_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(state_mtx_);
  uav_.position_x = msg->pose.position.x;
  uav_.position_y = msg->pose.position.y;
  uav_.position_z = msg->pose.position.z;
  rcvd_pose_ = true;
  last_pose_time_ = now();
  last_pose_receipt_ = std::chrono::steady_clock::now();
}

void MavlinkBridge::cb_local_velocity(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(state_mtx_);
  uav_.velocity_x = msg->twist.linear.x;
  uav_.velocity_y = msg->twist.linear.y;
  uav_.velocity_z = msg->twist.linear.z;
  rcvd_vel_ = true;
  last_velocity_time_ = now();
  last_velocity_receipt_ = std::chrono::steady_clock::now();
}

void MavlinkBridge::cb_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  double qx = msg->orientation.x;
  double qy = msg->orientation.y;
  double qz = msg->orientation.z;
  double qw = msg->orientation.w;

  std::lock_guard<std::mutex> lk(state_mtx_);

  // 四元数 → 欧拉角 (roll, pitch, yaw 简化版)
  double sinr_cosp = 2.0 * (qw * qx + qy * qz);
  double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
  uav_.roll = std::atan2(sinr_cosp, cosr_cosp);

  double sinp = 2.0 * (qw * qy - qz * qx);
  if (std::abs(sinp) >= 1.0) {
    uav_.pitch = std::copysign(M_PI / 2.0, sinp);
  } else {
    uav_.pitch = std::asin(sinp);
  }

  uav_.yaw = quat_to_yaw(qx, qy, qz, qw);

  // 角速度
  uav_.rollspeed = msg->angular_velocity.x;
  uav_.pitchspeed = msg->angular_velocity.y;
  uav_.yawspeed = msg->angular_velocity.z;

  rcvd_imu_ = true;
  last_imu_time_ = now();
  last_imu_receipt_ = std::chrono::steady_clock::now();
}

void MavlinkBridge::cb_altitude(const mavros_msgs::msg::Altitude::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(state_mtx_);
  uav_.altitude = msg->amsl;
  uav_.relative_altitude = msg->relative;
  rcvd_alt_ = true;
}

void MavlinkBridge::cb_global_position(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(state_mtx_);
  uav_.latitude = msg->latitude;
  uav_.longitude = msg->longitude;
  uav_.eph = static_cast<double>(msg->position_covariance[0]);
  uav_.epv = static_cast<double>(msg->position_covariance[8]);
  rcvd_gps_ = true;
  last_gps_time_ = now();
  last_gps_receipt_ = std::chrono::steady_clock::now();
}

void MavlinkBridge::cb_battery(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(state_mtx_);
  uav_.battery_voltage = msg->voltage;
  uav_.battery_remaining = msg->percentage;
  rcvd_bat_ = true;
  last_battery_time_ = now();
  last_battery_receipt_ = std::chrono::steady_clock::now();
}

void MavlinkBridge::cb_rc_in(const mavros_msgs::msg::RCIn::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(state_mtx_);
  // 只要收到 RC 数据就认为遥控器连接 (Q17: 记录时刻, 掉线由 publish_state 超时判定)
  uav_.rc_connected = !msg->channels.empty();
  if (uav_.rc_connected) {
    last_rc_time_ = now();
    last_rc_receipt_ = std::chrono::steady_clock::now();
  }
  rcvd_rc_ = true;
}

// ================================================================
// 定时发布
// ================================================================

void MavlinkBridge::publish_state()
{
  const auto stamp = now();
  easyfly_msgs::msg::UAVState state_snapshot;
  easyfly_msgs::msg::FlightStatus status_snapshot;
  bool publish_aggregate = false;
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    uav_.header.stamp = stamp;

    const auto wall_now = std::chrono::steady_clock::now();
    const auto fresh = [&stamp, wall_now](bool received, const rclcpp::Time & last,
        std::chrono::steady_clock::time_point receipt, double limit) {
        if (!received || last.nanoseconds() == 0) {return false;}
        const double age = (stamp - last).seconds();
        const double wall_age = std::chrono::duration<double>(wall_now - receipt).count();
        return receipt != std::chrono::steady_clock::time_point{} &&
               age >= 0.0 && age <= limit && wall_age >= 0.0 && wall_age <= limit;
      };
    uav_.connected = uav_.connected &&
      fresh(rcvd_armed_, last_state_time_, last_state_receipt_, slow_sensor_freshness_s_);
    uav_.pose_valid = fresh(rcvd_pose_, last_pose_time_, last_pose_receipt_, motion_freshness_s_) &&
      std::isfinite(uav_.position_x) && std::isfinite(uav_.position_y) &&
      std::isfinite(uav_.position_z);
    uav_.velocity_valid = fresh(
      rcvd_vel_, last_velocity_time_, last_velocity_receipt_,
      motion_freshness_s_) &&
      std::isfinite(uav_.velocity_x) && std::isfinite(uav_.velocity_y) &&
      std::isfinite(uav_.velocity_z);
    uav_.attitude_valid =
      fresh(rcvd_imu_, last_imu_time_, last_imu_receipt_, motion_freshness_s_) &&
      std::isfinite(uav_.roll) && std::isfinite(uav_.pitch) && std::isfinite(uav_.yaw) &&
      std::isfinite(uav_.rollspeed) && std::isfinite(uav_.pitchspeed) &&
      std::isfinite(uav_.yawspeed);
    uav_.battery_valid = fresh(
      rcvd_bat_, last_battery_time_, last_battery_receipt_,
      slow_sensor_freshness_s_) &&
      std::isfinite(uav_.battery_voltage) && std::isfinite(uav_.battery_remaining) &&
      uav_.battery_remaining >= 0.0 && uav_.battery_remaining <= 1.0;
    uav_.gps_valid =
      fresh(rcvd_gps_, last_gps_time_, last_gps_receipt_, slow_sensor_freshness_s_) &&
      std::isfinite(uav_.latitude) && std::isfinite(uav_.longitude) &&
      uav_.latitude >= -90.0 && uav_.latitude <= 90.0 &&
      uav_.longitude >= -180.0 && uav_.longitude <= 180.0;

    // Q17: RC 掉线检测 — 超过 1s 未收到 RCIn 则视为遥控器断连
    // (仿真无遥控器: 从未收到 → last_rc_time_=0 → 恒为 false, 语义正确)
    if (uav_.rc_connected &&
      !fresh(rcvd_rc_, last_rc_time_, last_rc_receipt_, RC_TIMEOUT_S))
    {
      uav_.rc_connected = false;
    }

    state_snapshot = uav_;
    // A stepped simulation clock can repeat across wall-timer ticks while MAVROS
    // updates the payload. Never publish conflicting aggregates with one identity.
    // Frozen clocks must not refresh downstream receipt watchdogs. Backward jumps
    // still publish a new epoch sample so observers can invalidate the old epoch.
    publish_aggregate = stamp.nanoseconds() > 0 &&
      stamp.nanoseconds() != last_published_state_stamp_;
    if (publish_aggregate) {last_published_state_stamp_ = stamp.nanoseconds();}
    status_snapshot = flight_status_;
    status_snapshot.receipt_age_s = flight_status_receipt_ ==
      std::chrono::steady_clock::time_point{} ? -1.0 :
    std::chrono::duration<double>(wall_now - flight_status_receipt_).count();
    const auto & source = status_snapshot.header.stamp;
    const double source_age = source.sec >= 0 && source.nanosec < 1000000000U ?
      (stamp - rclcpp::Time(source, stamp.get_clock_type())).seconds() : -1.0;
    status_snapshot.valid = flight_status_source_valid_ && status_snapshot.connected &&
      flight_status_watermark_ > 0 && source_age >= -0.2 &&
      source_age <= slow_sensor_freshness_s_ && status_snapshot.receipt_age_s >= 0 &&
      status_snapshot.receipt_age_s <= slow_sensor_freshness_s_;
  }

  if (publish_aggregate) {pub_uav_state_->publish(state_snapshot);}
  pub_flight_status_->publish(status_snapshot);
}

// ================================================================
// encoder: UAVCommand → mavros2 setpoint
// ================================================================

void MavlinkBridge::cb_uav_command(const easyfly_msgs::msg::UAVCommand::SharedPtr msg)
{
  if (command_fault_) {
    std::lock_guard<std::mutex> lk(command_mtx_);
    ++fault_rejected_command_count_;
    last_fault_rejected_source_ns_ = static_cast<int64_t>(msg->header.stamp.sec) * 1000000000 +
      msg->header.stamp.nanosec;
    return;
  }
  if (msg->header.frame_id != "odom") {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "拒绝 /uav_command: frame_id='%s'，期望 'odom'", msg->header.frame_id.c_str());
    return;
  }

  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  const double age = (now() - stamp).seconds();
  if (stamp.nanoseconds() == 0 || age < -0.2 || age > setpoint_timeout_) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "拒绝 /uav_command: 时间戳无效或过期 (age=%.3fs)", age);
    return;
  }

  const auto finite = [](std::initializer_list<double> values) {
      for (const double v : values) {
        if (!std::isfinite(v)) {
          return false;
        }
      }
      return true;
    };
  bool values_valid = false;
  switch (msg->control_mode) {
    case easyfly_msgs::msg::UAVCommand::POSITION:
      values_valid = finite({msg->position_x, msg->position_y, msg->position_z, msg->yaw});
      break;
    case easyfly_msgs::msg::UAVCommand::VELOCITY:
      values_valid = finite({msg->velocity_x, msg->velocity_y, msg->velocity_z, msg->yaw_rate});
      break;
    case easyfly_msgs::msg::UAVCommand::ATTITUDE:
    case easyfly_msgs::msg::UAVCommand::ATTITUDE_THRUST:
      values_valid = finite({msg->roll, msg->pitch, msg->yaw_att, msg->thrust}) &&
        msg->thrust >= 0.0 && msg->thrust <= 1.0;
      break;
    case easyfly_msgs::msg::UAVCommand::ANGULAR_RATE:
      values_valid = finite({msg->roll_rate, msg->pitch_rate, msg->yaw_rate_cmd, msg->thrust}) &&
        msg->thrust >= 0.0 && msg->thrust <= 1.0;
      break;
    case easyfly_msgs::msg::UAVCommand::TRAJECTORY:
      values_valid = finite(
        {msg->position_x, msg->position_y, msg->position_z,
          msg->velocity_x, msg->velocity_y, msg->velocity_z, msg->yaw});
      break;
    default:
      values_valid = false;
  }
  if (!values_valid) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "拒绝 /uav_command: control_mode=%u 含非法数值", msg->control_mode);
    return;
  }

  {
    std::lock_guard<std::mutex> lk(command_mtx_);
    last_cmd_ = *msg;
    last_cmd_time_ = now();
    last_cmd_receipt_ = std::chrono::steady_clock::now();
    cmd_received_ = true;
  }

  // 立即发布一次，不等 timer（PX4 OFFBOARD 要求 0.5s 内有 setpoint）
  publish_setpoint();

  RCLCPP_DEBUG_STREAM(
    get_logger(),
    "/uav_command mode=" << static_cast<int>(msg->control_mode)
                         << " pos=(" << msg->position_x << "," << msg->position_y << ","
                         << msg->position_z << ")"
                         << " yaw=" << msg->yaw);
}

void MavlinkBridge::publish_setpoint()
{
  if (command_fault_ || !rclcpp::ok(get_node_base_interface()->get_context())) {return;}
  // 只在这里做一个短快照；ROS publish 必须在释放 command_mtx_ 后执行，
  // 否则 command callback/timer/service 会被慢 DDS 发布阻塞。
  easyfly_msgs::msg::UAVCommand cmd_snapshot;
  rclcpp::Time cmd_time;
  std::chrono::steady_clock::time_point receipt;
  bool command_received = false;
  {
    std::lock_guard<std::mutex> lk(command_mtx_);
    cmd_snapshot = last_cmd_;
    cmd_time = last_cmd_time_;
    receipt = last_cmd_receipt_;
    command_received = cmd_received_;
  }

  // 超时保护：超过 setpoint_timeout_ 秒无新指令则停止发布。set_mode 服务
  // 不会触碰 cmd_time；旧命令必须自然过期，不能被模式切换续命。
  const auto elapsed = (now() - cmd_time).seconds();
  const double wall_elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - receipt).count();
  if (!command_received || receipt == std::chrono::steady_clock::time_point{} ||
    elapsed<0.0 || elapsed> setpoint_timeout_ ||
    wall_elapsed<0.0 || wall_elapsed> setpoint_timeout_)
  {
    return;  // 停流，由 PX4 已配置的 OFFBOARD-loss 行为处理，不假定某个模式。
  }

  switch (cmd_snapshot.control_mode) {
    case easyfly_msgs::msg::UAVCommand::POSITION: {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = now();
        pose.header.frame_id = "odom";
        pose.pose.position.x = cmd_snapshot.position_x;
        pose.pose.position.y = cmd_snapshot.position_y;
        pose.pose.position.z = cmd_snapshot.position_z;

        // yaw → 四元数
        double half_yaw = cmd_snapshot.yaw * 0.5;
        pose.pose.orientation.z = std::sin(half_yaw);
        pose.pose.orientation.w = std::cos(half_yaw);

        pub_setpoint_pos_->publish(pose);
        break;
      }
    case easyfly_msgs::msg::UAVCommand::VELOCITY: {
        geometry_msgs::msg::Twist vel;
        vel.linear.x = cmd_snapshot.velocity_x;
        vel.linear.y = cmd_snapshot.velocity_y;
        vel.linear.z = cmd_snapshot.velocity_z;
        vel.angular.z = cmd_snapshot.yaw_rate;

        pub_setpoint_vel_->publish(vel);
        break;
      }
    case easyfly_msgs::msg::UAVCommand::ATTITUDE:
    // Q15: ATTITUDE_THRUST 是 ATTITUDE 的历史别名 — 语义同为
    // "姿态四元数 + 归一化推力 [0-1]" (thrust 字段注释见 UAVCommand.msg), 两者处理一致
    case easyfly_msgs::msg::UAVCommand::ATTITUDE_THRUST: {
        mavros_msgs::msg::AttitudeTarget att;
        att.header.stamp = now();

        // roll/pitch/yaw → 四元数
        double half_r = cmd_snapshot.roll * 0.5;
        double half_p = cmd_snapshot.pitch * 0.5;
        double half_y = cmd_snapshot.yaw_att * 0.5;

        double cr = std::cos(half_r), sr = std::sin(half_r);
        double cp = std::cos(half_p), sp = std::sin(half_p);
        double cy = std::cos(half_y), sy = std::sin(half_y);

        att.orientation.w = cr * cp * cy + sr * sp * sy;
        att.orientation.x = sr * cp * cy - cr * sp * sy;
        att.orientation.y = cr * sp * cy + sr * cp * sy;
        att.orientation.z = cr * cp * sy - sr * sp * cy;

        att.thrust = static_cast<float>(cmd_snapshot.thrust);
        // MAVLink type_mask 位定义: bit0=忽略roll速率 bit1=忽略pitch速率
        // bit2=忽略yaw速率 bit3=忽略推力 bit7=忽略姿态四元数
        // 本模式使用姿态+推力 → 忽略三个角速率位 (0b111)
        att.type_mask = 0b111;
        att.body_rate.x = 0.0f;
        att.body_rate.y = 0.0f;
        att.body_rate.z = 0.0f;

        pub_setpoint_att_->publish(att);
        break;
      }
    case easyfly_msgs::msg::UAVCommand::ANGULAR_RATE: {
        mavros_msgs::msg::AttitudeTarget att;
        att.header.stamp = now();

        att.body_rate.x = static_cast<float>(cmd_snapshot.roll_rate);
        att.body_rate.y = static_cast<float>(cmd_snapshot.pitch_rate);
        att.body_rate.z = static_cast<float>(cmd_snapshot.yaw_rate_cmd);
        att.thrust = static_cast<float>(cmd_snapshot.thrust);
        // Q14 修复: 本模式使用角速率+推力 → 仅忽略姿态四元数 (bit7)。
        // 原值 0b10000111 把 bit0-2 (三个角速率) 也置了忽略, 角速率指令被静默丢弃!
        att.type_mask = 0b10000000;

        pub_setpoint_att_->publish(att);
        break;
      }
    case easyfly_msgs::msg::UAVCommand::TRAJECTORY: {
        // M4.3: EGO-Planner 轨迹 (位置 + 速度前馈 + yaw)。
        // 坐标系 (mavros 2.14.0 setpoint_raw.cpp local_cb 源码实证):
        //   除 FRAME_BODY_* 外一律按 ENU 处理并内部 transform_frame_enu_ned,
        //   故此处必须直传 ENU 数值 + FRAME_LOCAL_NED, type_mask 原样透传。
        //   mask = IGNORE_AFX|IGNORE_AFY|IGNORE_AFZ|IGNORE_YAW_RATE = 64+128+256+2048 = 2496
        //   (mavros_msgs: IGNORE_YAW=1024, IGNORE_YAW_RATE=2048; 用 position+velocity+yaw,
        //    忽略加速度/角速率)。注: 任务书里写的 1472 是把 YAW/YAW_RATE 位搞反了,
        //    此处以命名常量求值为准。
        mavros_msgs::msg::PositionTarget pt;
        pt.header.stamp = now();
        pt.header.frame_id = "odom";
        pt.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
        pt.position.x = cmd_snapshot.position_x;
        pt.position.y = cmd_snapshot.position_y;
        pt.position.z = cmd_snapshot.position_z;
        pt.velocity.x = cmd_snapshot.velocity_x;
        pt.velocity.y = cmd_snapshot.velocity_y;
        pt.velocity.z = cmd_snapshot.velocity_z;
        pt.yaw = static_cast<float>(cmd_snapshot.yaw);
        pt.yaw_rate = 0.0f;
        pt.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_AFX |
          mavros_msgs::msg::PositionTarget::IGNORE_AFY |
          mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
          mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;

        pub_setpoint_raw_->publish(pt);
        break;
      }
    default:
      RCLCPP_WARN_STREAM(
        get_logger(),
        "Unknown control_mode=" << static_cast<int>(cmd_snapshot.control_mode));
      break;
  }
}

// ================================================================
// service handlers: arm / set_mode (封装 mavros2 服务调用)
// ================================================================

easyfly_msgs::srv::GetCapabilities::Response MavlinkBridge::command_capabilities() const
{
  const bool fault = command_fault_.load();
  const bool context = rclcpp::ok(get_node_options().context());
  const bool mode_available = client_set_mode_->service_is_ready();
  const bool arm_available = client_arming_->service_is_ready();
  const bool ready = !fault && context && mode_available && arm_available;
  const auto clock = now();
  uint64_t rejected_count;
  int64_t rejected_source;
  {
    std::lock_guard<std::mutex> lk(command_mtx_);
    rejected_count = fault_rejected_command_count_;
    rejected_source = last_fault_rejected_source_ns_;
  }
  std::ostringstream details;
  details << std::boolalpha << "{\"reason\":\""
          << (fault ? "command_fault_locked" :
  ready ? "command_transport_available" : "command_transport_unavailable")
          << "\",\"command_fault\":" << fault << ",\"ready\":" << ready
          << ",\"context_ok\":" << context << ",\"clock_ns\":" << clock.nanoseconds()
          << ",\"mode_service_ready\":" << mode_available
          << ",\"arming_service_ready\":" << arm_available
          << ",\"fault_rejected_command_count\":" << rejected_count
          << ",\"last_fault_rejected_source_ns\":" << rejected_source << "}";
  easyfly_msgs::srv::GetCapabilities::Response response;
  response.header.stamp = clock;
  response.header.frame_id = "odom";
  response.names = {"flight_command_transport"};
  response.implemented = {true};
  response.ready = {ready};
  response.details = {details.str()};
  return response;
}

void MavlinkBridge::cb_arm_motors(
  const std::shared_ptr<easyfly_msgs::srv::ArmMotors::Request> req,
  std::shared_ptr<easyfly_msgs::srv::ArmMotors::Response> res)
{
  RCLCPP_INFO(get_logger(), "cb_arm_motors CALLED arm=%d", req->arm);
  if (command_fault_) {
    res->success = false;
    res->message =
      "command_result_unknown: bridge fault locked; inspect remote state before restart";
    return;
  }

  auto mav_req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  mav_req->value = req->arm;

  bool uncertain = false;
  const auto mav_res = request_bounded<mavros_msgs::srv::CommandBool>(
    client_arming_, mav_req, get_node_base_interface()->get_context(), uncertain);
  if (!mav_res) {
    if (uncertain) {
      command_fault_.store(true);
      RCLCPP_ERROR(
        get_logger(), "MAVROS arming command outcome uncertain: bridge fault latch set; "
        "manual audit required");
    }
    res->success = false;
    res->message = uncertain ? "command_result_unknown: MAVROS arming response unconfirmed" :
      "mavros /cmd/arming unavailable or context stopped before send";
    return;
  }
  res->success = mav_res->success;
  res->message = req->arm ? "armed" : "disarmed";
  RCLCPP_INFO(get_logger(), "Arm=%s → success=%d", req->arm ? "true" : "false", mav_res->success);
}

void MavlinkBridge::cb_set_flight_mode(
  const std::shared_ptr<easyfly_msgs::srv::SetFlightMode::Request> req,
  std::shared_ptr<easyfly_msgs::srv::SetFlightMode::Response> res)
{
  if (command_fault_) {
    res->success = false;
    res->message =
      "command_result_unknown: bridge fault locked; inspect remote state before restart";
    return;
  }
  auto mav_req = std::make_shared<mavros_msgs::srv::SetMode::Request>();

  switch (req->flight_mode) {
    case 0:  mav_req->custom_mode = "MANUAL";     break;
    case 1:  mav_req->custom_mode = "ALTCTL";     break;
    case 2:  mav_req->custom_mode = "POSCTL";     break;
    case 3:  mav_req->custom_mode = "OFFBOARD";   break;
    case 4:  mav_req->custom_mode = "STABILIZED"; break;
    case 5:  mav_req->custom_mode = "ACRO";       break;
    // PX4 模式名是 AUTO.RTL；"RTL" 是 ArduPilot 名，PX4 会拒绝。
    case 6:  mav_req->custom_mode = "AUTO.RTL";   break;
    case 7:  mav_req->custom_mode = "AUTO.LAND";       break;
    default:
      res->success = false;
      res->message = "unknown flight_mode=" + std::to_string(req->flight_mode);
      return;
  }

  bool uncertain = false;
  const auto mav_res = request_bounded<mavros_msgs::srv::SetMode>(
    client_set_mode_, mav_req, get_node_base_interface()->get_context(), uncertain);
  if (!mav_res) {
    if (uncertain) {
      command_fault_.store(true);
      RCLCPP_ERROR(
        get_logger(), "MAVROS mode command outcome uncertain: bridge fault latch set; "
        "manual audit required");
    }
    res->success = false;
    res->message = uncertain ? "command_result_unknown: MAVROS mode response unconfirmed" :
      "mavros /set_mode unavailable or context stopped before send";
    return;
  }
  res->success = mav_res->mode_sent;
  res->message = "mode=" + mav_req->custom_mode;
  RCLCPP_INFO(
    get_logger(), "SetMode → %s (success=%d)",
    mav_req->custom_mode.c_str(), mav_res->mode_sent);
}

}  // namespace easyfly_mavlink_bridge
