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

#ifndef EASYFLY_MAVLINK_BRIDGE__MAVLINK_BRIDGE_HPP_
#define EASYFLY_MAVLINK_BRIDGE__MAVLINK_BRIDGE_HPP_

#include <chrono>
#include <atomic>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <easyfly_msgs/msg/uav_state.hpp>
#include <easyfly_msgs/msg/flight_status.hpp>
#include <easyfly_msgs/msg/uav_command.hpp>
#include <easyfly_msgs/msg/gimbal_state.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/altitude.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <easyfly_msgs/srv/arm_motors.hpp>
#include <easyfly_msgs/srv/set_flight_mode.hpp>
#include <easyfly_msgs/srv/get_capabilities.hpp>

namespace easyfly_mavlink_bridge
{

/**
 * @brief MAVLink ↔ easyfly_msgs 桥接节点
 *
 * decoder: 订阅 mavros2 topic → 聚合发布 /uav_state
 * encoder: 订阅 /uav_command → 翻译为 mavros2 setpoint + 20Hz 持续发布
 * services: /arm_motors、/set_flight_mode → 封装 mavros2 服务调用
 */
class MavlinkBridge : public rclcpp::Node
{
  friend class BridgeFreshnessTestPeer;

public:
  explicit MavlinkBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ---- decoder callbacks ----
  void cb_state(const mavros_msgs::msg::State::SharedPtr msg);
  void cb_local_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void cb_local_velocity(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void cb_imu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void cb_altitude(const mavros_msgs::msg::Altitude::SharedPtr msg);
  void cb_global_position(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void cb_battery(const sensor_msgs::msg::BatteryState::SharedPtr msg);
  void cb_rc_in(const mavros_msgs::msg::RCIn::SharedPtr msg);

  void publish_state();

  // ---- encoder ----
  void cb_uav_command(const easyfly_msgs::msg::UAVCommand::SharedPtr msg);
  void publish_setpoint();

  // ---- service handlers ----
  void cb_arm_motors(
    const std::shared_ptr<easyfly_msgs::srv::ArmMotors::Request> req,
    std::shared_ptr<easyfly_msgs::srv::ArmMotors::Response> res);
  void cb_set_flight_mode(
    const std::shared_ptr<easyfly_msgs::srv::SetFlightMode::Request> req,
    std::shared_ptr<easyfly_msgs::srv::SetFlightMode::Response> res);
  easyfly_msgs::srv::GetCapabilities::Response command_capabilities() const;

  // ---- 静态转换工具 ----
  static double quat_to_yaw(double x, double y, double z, double w);

  // ---- 订阅者 ----
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr sub_state_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_local_pose_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_local_vel_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<mavros_msgs::msg::Altitude>::SharedPtr sub_altitude_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_global_pos_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr sub_battery_;
  rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr sub_rc_in_;
  rclcpp::Subscription<easyfly_msgs::msg::UAVCommand>::SharedPtr sub_uav_command_;

  // ---- 发布者 ----
  rclcpp::Publisher<easyfly_msgs::msg::UAVState>::SharedPtr pub_uav_state_;
  rclcpp::Publisher<easyfly_msgs::msg::FlightStatus>::SharedPtr pub_flight_status_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_setpoint_pos_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_setpoint_vel_;
  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr pub_setpoint_att_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr pub_setpoint_raw_;

  // ---- 服务端 ----
  rclcpp::Service<easyfly_msgs::srv::ArmMotors>::SharedPtr srv_arm_;
  rclcpp::Service<easyfly_msgs::srv::SetFlightMode>::SharedPtr srv_mode_;
  rclcpp::Service<easyfly_msgs::srv::GetCapabilities>::SharedPtr srv_capabilities_;

  // ---- mavros 客户端 (预创建，避免每次回调重建) ----
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr client_arming_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr client_set_mode_;
  std::atomic<bool> command_fault_{false};  // 已发送请求结果未知，人工核查后重启复位。

  // ---- 独立 callback group (service 与 client 分离，避免死锁) ----
  rclcpp::CallbackGroup::SharedPtr srv_cb_group_;

  // ---- 定时器 ----
  rclcpp::TimerBase::SharedPtr timer_state_;
  rclcpp::TimerBase::SharedPtr timer_setpoint_;

  // ---- 聚合缓存 ----
  mutable std::mutex state_mtx_;
  easyfly_msgs::msg::UAVState uav_;
  int64_t last_published_state_stamp_{-1};  // One aggregate identity per ROS timestamp.
  easyfly_msgs::msg::FlightStatus flight_status_;
  int64_t flight_status_watermark_{0};
  bool flight_status_source_valid_{false};
  std::chrono::steady_clock::time_point flight_status_receipt_{};

  // ---- Q17: RC 掉线检测 ----
  rclcpp::Time last_rc_time_;          // 最近一次收到 RCIn 的时刻 (0=从未收到)
  std::chrono::steady_clock::time_point last_rc_receipt_{};
  static constexpr double RC_TIMEOUT_S = 1.0;  // 超过此时长无 RC 数据视为断连

  // ---- encoder 缓存 ----
  // cb_uav_command、timer 和立即 publish 路径可并行执行；快照后再发布，
  // 禁止持锁调用 ROS publish/service。
  mutable std::mutex command_mtx_;
  uint64_t fault_rejected_command_count_{0};
  int64_t last_fault_rejected_source_ns_{0};
  easyfly_msgs::msg::UAVCommand last_cmd_;
  rclcpp::Time last_cmd_time_;
  std::chrono::steady_clock::time_point last_cmd_receipt_{};
  bool cmd_received_    {false};
  double setpoint_timeout_{1.0};     // 超时秒数，超时后停止发 setpoint (via param)

  // 标志位
  bool rcvd_armed_        {false};
  bool rcvd_pose_         {false};
  bool rcvd_vel_          {false};
  bool rcvd_imu_          {false};
  bool rcvd_alt_          {false};
  bool rcvd_gps_          {false};
  bool rcvd_bat_          {false};
  bool rcvd_rc_           {false};

  // 同锁记录两种接收时刻：ROS检测时间回退，steady防止暂停时旧数据续命。
  rclcpp::Time last_state_time_;
  rclcpp::Time last_pose_time_;
  rclcpp::Time last_velocity_time_;
  rclcpp::Time last_imu_time_;
  rclcpp::Time last_battery_time_;
  rclcpp::Time last_gps_time_;
  std::chrono::steady_clock::time_point last_state_receipt_{};
  std::chrono::steady_clock::time_point last_pose_receipt_{};
  std::chrono::steady_clock::time_point last_velocity_receipt_{};
  std::chrono::steady_clock::time_point last_imu_receipt_{};
  std::chrono::steady_clock::time_point last_battery_receipt_{};
  std::chrono::steady_clock::time_point last_gps_receipt_{};
  double motion_freshness_s_{1.0};
  double slow_sensor_freshness_s_{5.0};
};

}  // namespace easyfly_mavlink_bridge

#endif  // EASYFLY_MAVLINK_BRIDGE__MAVLINK_BRIDGE_HPP_
