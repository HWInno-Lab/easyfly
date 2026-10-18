//
//  ego_to_mavros — 头文件
//  Bridge: ego_planner PositionCommand → PX4 OFFBOARD via MAVROS
//
//  I/O 拓扑:
//    订阅 /drone_{id}_planning/pos_cmd   ← traj_server (100Hz, ENU)
//    订阅 /mavros/state                   ← PX4 飞控状态
//    订阅 /mavros/local_position/odom     ← PX4 EKF2 里程计
//    发布 /mavros/setpoint_raw/local      → PX4 OFFBOARD 指令 (NED)
//    发布 /drone_{id}_visual_slam/odom    → ego_planner 里程计 (转发)
//    发布 /traj_start_trigger             → FSM 触发信号
//

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"

#include "quadrotor_msgs/msg/position_command.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <string>

// ─── Bridge 状态机 ─────────────────────────────────────
enum class BridgeState
{
  WAITING_FCU,        // 等待飞控连接 + 上锁
  HOLD_POSITION,      // OFFBOARD 就绪，悬停等待触发
  TRAJECTORY_ACTIVE,  // 执行 ego_planner 轨迹
  FALLBACK_HOLD       // 轨迹断流，回退到悬停
};

// ─── EgoToMavrosBridge ─────────────────────────────────
class EgoToMavrosBridge : public rclcpp::Node
{
public:
  EgoToMavrosBridge();

private:
  // ── 回调 ──────────────────────────
  void stateCallback(const mavros_msgs::msg::State::SharedPtr msg);
  void posCmdCallback(const quadrotor_msgs::msg::PositionCommand::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // ── 主循环 ─────────────────────────
  void setpointLoop();

  // ── 辅助 ──────────────────────────
  void fillTrajectorySetpoint(mavros_msgs::msg::PositionTarget &sp);
  void fillHoldSetpoint(mavros_msgs::msg::PositionTarget &sp);

  // ── 参数 ──────────────────────────
  std::string drone_id_;
  double setpoint_rate_;
  rclcpp::Duration cmd_timeout_{0, 0};
  bool auto_trigger_;
  bool enable_yaw_cmd_;

  // ── 状态 ──────────────────────────
  BridgeState state_{BridgeState::WAITING_FCU};
  bool fcu_connected_{false};
  bool fcu_armed_{false};
  std::string fcu_mode_;
  bool have_odom_{false};
  bool trigger_sent_{false};
  bool had_traj_{false};

  // ── 数据缓存 ──────────────────────
  nav_msgs::msg::Odometry latest_odom_;
  geometry_msgs::msg::Pose last_hold_pose_;
  double latest_yaw_{0.0};
  double last_hold_yaw_{0.0};

  quadrotor_msgs::msg::PositionCommand::SharedPtr latest_cmd_;
  rclcpp::Time last_cmd_time_;

  // ── ROS 接口 ──────────────────────
  // Publishers
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr trigger_pub_;

  // Subscriptions
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Timer
  rclcpp::TimerBase::SharedPtr setpoint_timer_;
};
