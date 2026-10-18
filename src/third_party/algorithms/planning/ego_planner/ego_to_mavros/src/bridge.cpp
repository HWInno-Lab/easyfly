#include "ego_to_mavros/bridge.hpp"

using namespace std::chrono_literals;

// ─── 构造函数 ─────────────────────────────────────────
EgoToMavrosBridge::EgoToMavrosBridge()
    : Node("ego_to_mavros_bridge")
{
  // ── 声明参数 ──────────────────────────
  this->declare_parameter<std::string>("drone_id", "0");
  this->declare_parameter<double>("setpoint_rate", 50.0);
  this->declare_parameter<double>("cmd_timeout", 0.5);
  this->declare_parameter<bool>("auto_trigger", false);
  this->declare_parameter<bool>("enable_yaw_cmd", false);

  drone_id_       = this->get_parameter("drone_id").as_string();
  setpoint_rate_  = this->get_parameter("setpoint_rate").as_double();
  cmd_timeout_    = rclcpp::Duration::from_seconds(
                        this->get_parameter("cmd_timeout").as_double());
  auto_trigger_   = this->get_parameter("auto_trigger").as_bool();
  enable_yaw_cmd_ = this->get_parameter("enable_yaw_cmd").as_bool();

  // ── 话题名组装 ──────────────────────────
  std::string pos_cmd_topic  = "/drone_" + drone_id_ + "_planning/pos_cmd";
  std::string odom_out_topic = "/drone_" + drone_id_ + "_visual_slam/odom";

  // ── 发布者 ──────────────────────────
  setpoint_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
      "/mavros/setpoint_raw/local", 10);
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      odom_out_topic, 10);
  trigger_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/traj_start_trigger", 10);

  // ── 订阅者 ──────────────────────────
  pos_cmd_sub_ = this->create_subscription<quadrotor_msgs::msg::PositionCommand>(
      pos_cmd_topic, rclcpp::SensorDataQoS(),
      std::bind(&EgoToMavrosBridge::posCmdCallback, this, std::placeholders::_1));

  state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", 10,
      std::bind(&EgoToMavrosBridge::stateCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/mavros/local_position/odom", 10,
      std::bind(&EgoToMavrosBridge::odomCallback, this, std::placeholders::_1));

  // ── 定时器 ──────────────────────────
  double period = 1.0 / setpoint_rate_;
  setpoint_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(period),
      std::bind(&EgoToMavrosBridge::setpointLoop, this));

  RCLCPP_INFO(this->get_logger(),
      "[ego_to_mavros] bridge started. drone_id=%s, cmd topic=%s",
      drone_id_.c_str(), pos_cmd_topic.c_str());
}

// ─── 飞控状态回调 ──────────────────────────────────────
void EgoToMavrosBridge::stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
{
  fcu_connected_ = msg->connected;
  fcu_armed_     = msg->armed;
  fcu_mode_      = msg->mode;

  bool offboard_now = (fcu_mode_ == "OFFBOARD");

  if (!fcu_connected_)
  {
    state_ = BridgeState::WAITING_FCU;
    return;
  }

  if (state_ == BridgeState::WAITING_FCU && fcu_armed_ && offboard_now && have_odom_)
  {
    last_hold_pose_ = latest_odom_.pose.pose;
    last_hold_yaw_  = latest_yaw_;
    state_          = BridgeState::HOLD_POSITION;
    RCLCPP_WARN(this->get_logger(),
        "[ego_to_mavros] OFFBOARD armed — holding position, waiting for trigger.");
  }

  if (!offboard_now || !fcu_armed_)
  {
    if (state_ != BridgeState::WAITING_FCU)
    {
      state_    = BridgeState::WAITING_FCU;
      had_traj_ = false;
      RCLCPP_WARN(this->get_logger(),
          "[ego_to_mavros] OFFBOARD exited — back to WAITING_FCU.");
    }
  }
}

// ─── 位置指令回调 ─────────────────────────────────────
void EgoToMavrosBridge::posCmdCallback(
    const quadrotor_msgs::msg::PositionCommand::SharedPtr msg)
{
  if (msg->trajectory_flag !=
      quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY)
    return;

  latest_cmd_    = msg;
  last_cmd_time_ = this->now();
  had_traj_      = true;

  // 首次收到轨迹 → 触发 FSM
  if (state_ == BridgeState::HOLD_POSITION && !trigger_sent_)
  {
    geometry_msgs::msg::PoseStamped trigger;
    trigger.header.stamp    = this->now();
    trigger.header.frame_id = "world";
    trigger.pose.position.x = msg->position.x;
    trigger.pose.position.y = msg->position.y;
    trigger.pose.position.z = msg->position.z;
    trigger.pose.orientation.w = 1.0;
    trigger_pub_->publish(trigger);
    trigger_sent_ = true;

    RCLCPP_WARN(this->get_logger(),
        "[ego_to_mavros] TRAJECTORY TRIGGERED at (%.2f, %.2f, %.2f)",
        msg->position.x, msg->position.y, msg->position.z);
  }

  if (state_ == BridgeState::HOLD_POSITION || state_ == BridgeState::FALLBACK_HOLD)
  {
    state_ = BridgeState::TRAJECTORY_ACTIVE;
  }
}

// ─── MAVROS 里程计回调 ────────────────────────────────
void EgoToMavrosBridge::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  //
  //  MAVROS 的 /mavros/local_position/odom 已转换为 ENU（ROS 标准），
  //  此处 pass-through 转发即可。
  //  若将来发现坐标系不匹配，在此处做 NED↔ENU 转换。
  //

  latest_odom_             = *msg;
  latest_odom_.header.frame_id = "world";
  have_odom_               = true;

  // 提取 yaw
  tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  latest_yaw_ = yaw;

  // 转发给 ego_planner
  odom_pub_->publish(latest_odom_);
}

// ─── 轨迹模式 setpoint 填充 ───────────────────────────
void EgoToMavrosBridge::fillTrajectorySetpoint(
    mavros_msgs::msg::PositionTarget &sp)
{
  // 坐标系 (mavros 2.14.0 setpoint_raw.cpp local_cb 源码实证):
  //   除 FRAME_BODY_* 外一律按 ENU 处理并内部 transform_frame_enu_ned。
  //   因此这里必须直传 ego_planner 的 ENU 数值 (不可再手动换轴, 否则双重变换
  //   会造成轴序错乱 + 高度反号)。
  sp.position.x = latest_cmd_->position.x;
  sp.position.y = latest_cmd_->position.y;
  sp.position.z = latest_cmd_->position.z;

  sp.velocity.x = latest_cmd_->velocity.x;
  sp.velocity.y = latest_cmd_->velocity.y;
  sp.velocity.z = latest_cmd_->velocity.z;

  sp.acceleration_or_force.x = latest_cmd_->acceleration.x;
  sp.acceleration_or_force.y = latest_cmd_->acceleration.y;
  sp.acceleration_or_force.z = latest_cmd_->acceleration.z;

  if (enable_yaw_cmd_)
  {
    sp.yaw      = static_cast<float>(latest_cmd_->yaw);
    sp.yaw_rate = static_cast<float>(latest_cmd_->yaw_dot);
  }

  sp.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
  if (!enable_yaw_cmd_)
  {
    sp.type_mask |= mavros_msgs::msg::PositionTarget::IGNORE_YAW |
                    mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
  }
}

// ─── 悬停模式 setpoint 填充 ───────────────────────────
void EgoToMavrosBridge::fillHoldSetpoint(
    mavros_msgs::msg::PositionTarget &sp)
{
  // 同 fillTrajectorySetpoint: 直传 ENU (mavros 内部 ENU→NED)
  sp.position.x = last_hold_pose_.position.x;
  sp.position.y = last_hold_pose_.position.y;
  sp.position.z = last_hold_pose_.position.z;

  if (enable_yaw_cmd_)
    sp.yaw = static_cast<float>(last_hold_yaw_);

  // 悬停：只发位置 + yaw，速度和加速度由 PX4 内部计算
  sp.type_mask =
      mavros_msgs::msg::PositionTarget::IGNORE_VX |
      mavros_msgs::msg::PositionTarget::IGNORE_VY |
      mavros_msgs::msg::PositionTarget::IGNORE_VZ |
      mavros_msgs::msg::PositionTarget::IGNORE_AFX |
      mavros_msgs::msg::PositionTarget::IGNORE_AFY |
      mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
      mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;

  if (!enable_yaw_cmd_)
  {
    sp.type_mask |= mavros_msgs::msg::PositionTarget::IGNORE_YAW |
                    mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
  }
}

// ─── 主循环：以固定频率发送 setpoint ──────────────────
void EgoToMavrosBridge::setpointLoop()
{
  if (state_ == BridgeState::WAITING_FCU)
    return;

  mavros_msgs::msg::PositionTarget setpoint;
  setpoint.header.stamp    = this->now();
  setpoint.header.frame_id = "map";
  setpoint.coordinate_frame =
      mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;

  // 断流检测
  bool cmd_timed_out = ((this->now() - last_cmd_time_) > cmd_timeout_);
  bool use_traj = (state_ == BridgeState::TRAJECTORY_ACTIVE)
                  && latest_cmd_ && !cmd_timed_out;

  if (cmd_timed_out && state_ == BridgeState::TRAJECTORY_ACTIVE)
  {
    state_ = BridgeState::FALLBACK_HOLD;
    RCLCPP_WARN(this->get_logger(),
        "[ego_to_mavros] PositionCommand timeout (%.2fs) → FALLBACK_HOLD",
        cmd_timeout_.seconds());
  }

  if (use_traj)
  {
    fillTrajectorySetpoint(setpoint);
  }
  else
  {
    if (!have_odom_)
      return;
    fillHoldSetpoint(setpoint);
  }

  setpoint_pub_->publish(setpoint);
}

// ─── main ──────────────────────────────────────────────
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EgoToMavrosBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
