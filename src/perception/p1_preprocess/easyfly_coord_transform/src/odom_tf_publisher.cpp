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
// EasyFly P1: 里程计 TF 发布器
//
// 职责:
//   订阅 /uav_state (easyfly_mavlink_bridge 聚合, 来自 mavros local_position/pose,
//   姿态与位置均为 ENU 系, 机体 FLU), 发布:
//     TF  odom → base_link   (50 Hz, TF 树的动态主干)
//     话题 /odom             (nav_msgs/Odometry, 供 FAST-LIO 对照/规划层使用)
//
//   TF 树全景 (M1 完成后):
//     odom ── base_link ── camera_link ── camera_color_optical_frame
//           │                        └── camera_depth_optical_frame
//           └── livox_frame          (静态部分由 easyfly_camera_calib 发布)
//   map 坐标系将在 M3 (FAST-LIO) 接入后提供 map→odom。
#include <tf2_ros/transform_broadcaster.h>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <easyfly_msgs/msg/uav_state.hpp>
#include "easyfly_coord_transform/coord_utils.hpp"
namespace easyfly_coord_transform
{
class OdomTfPublisher : public rclcpp::Node
{
public:
  explicit OdomTfPublisher(const rclcpp::NodeOptions & options)
  : Node("odom_tf_publisher", options)
  {
    declare_parameter<std::string>("uav_state_topic", "/uav_state");
    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<double>("publish_rate", 50.0);
    state_timeout_ = declare_parameter<double>("state_timeout", 0.5);
    const std::string odom_frame = get_parameter("odom_frame").as_string();
    const std::string base_frame = get_parameter("base_frame").as_string();
    const double rate = get_parameter("publish_rate").as_double();
    if (!std::isfinite(rate) || rate <= 0.0 ||
      !std::isfinite(state_timeout_) || state_timeout_ <= 0.0)
    {
      throw std::invalid_argument("publish_rate and state_timeout must be finite and positive");
    }
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    state_sub_ = create_subscription<easyfly_msgs::msg::UAVState>(
      get_parameter("uav_state_topic").as_string(), 10,
      [this](easyfly_msgs::msg::UAVState::ConstSharedPtr msg) {
        latest_ = msg;
        last_receipt_ = std::chrono::steady_clock::now();
      });
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate),
      [this, odom_frame, base_frame]() {publish(odom_frame, base_frame);});
    RCLCPP_INFO(
      get_logger(), "odom_tf_publisher 启动: %s → %s @ %.0f Hz",
      odom_frame.c_str(), base_frame.c_str(), rate);
  }

private:
  void publish(const std::string & odom_frame, const std::string & base_frame)
  {
    if (!latest_) {return;}
    const auto & s = *latest_;
    // Never give invalid, stale or differently framed state a valid-looking TF.
    if (!s.connected || !s.pose_valid || !s.velocity_valid || !s.attitude_valid ||
      s.header.frame_id != odom_frame ||
      s.header.stamp.sec < 0 || s.header.stamp.nanosec >= 1000000000u ||
      (s.header.stamp.sec == 0 && s.header.stamp.nanosec == 0)) {return;}
    for (double value : {s.position_x, s.position_y, s.position_z,
        s.roll, s.pitch, s.yaw, s.velocity_x, s.velocity_y, s.velocity_z,
        s.rollspeed, s.pitchspeed, s.yawspeed})
    {
      if (!std::isfinite(value)) {return;}
    }
    const double age =
      (now() - rclcpp::Time(s.header.stamp, get_clock()->get_clock_type())).seconds();
    if (age < -0.1 || age > state_timeout_ ||
      std::chrono::duration<double>(std::chrono::steady_clock::now() - last_receipt_).count() >
      state_timeout_) {return;}
    // rpy (ZYX) → 四元数; /uav_state 姿态由 mavros ENU 四元数分解而来
    const auto rotation = rpy_to_quat(s.roll, s.pitch, s.yaw);
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = s.header.stamp;
    tf.header.frame_id = odom_frame;
    tf.child_frame_id = base_frame;
    tf.transform.translation.x = s.position_x;
    tf.transform.translation.y = s.position_y;
    tf.transform.translation.z = s.position_z;
    tf.transform.rotation.x = rotation[0];
    tf.transform.rotation.y = rotation[1];
    tf.transform.rotation.z = rotation[2];
    tf.transform.rotation.w = rotation[3];
    tf_broadcaster_->sendTransform(tf);
    nav_msgs::msg::Odometry odom;
    odom.header = tf.header;
    odom.child_frame_id = base_frame;
    odom.pose.pose.position.x = s.position_x;
    odom.pose.pose.position.y = s.position_y;
    odom.pose.pose.position.z = s.position_z;
    odom.pose.pose.orientation = tf.transform.rotation;
    // Odometry 惯例: twist 在 child_frame (base_link/FLU) 系
    // /uav_state 速度为 ENU 系 → v_body = R(world→body) · v_world
    // R (body→world) = Rz(yaw)·Ry(pitch)·Rx(roll); v_body = Rᵀ·v_world
    const auto velocity = enu_to_flu(
      std::array<double, 3>{s.velocity_x, s.velocity_y, s.velocity_z}, s.roll, s.pitch, s.yaw);
    odom.twist.twist.linear.x = velocity[0];
    odom.twist.twist.linear.y = velocity[1];
    odom.twist.twist.linear.z = velocity[2];
    odom.twist.twist.angular.x = s.rollspeed;
    odom.twist.twist.angular.y = s.pitchspeed;
    odom.twist.twist.angular.z = s.yawspeed;
    odom_pub_->publish(odom);
  }
  easyfly_msgs::msg::UAVState::ConstSharedPtr latest_;
  double state_timeout_{0.5};
  std::chrono::steady_clock::time_point last_receipt_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<easyfly_msgs::msg::UAVState>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace easyfly_coord_transform
RCLCPP_COMPONENTS_REGISTER_NODE(easyfly_coord_transform::OdomTfPublisher)
