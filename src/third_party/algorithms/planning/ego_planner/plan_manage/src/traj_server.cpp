#include "bspline_opt/uniform_bspline.h"
#include "plan_manage/bspline_runtime_validator.h"
#include "plan_manage/swarm_bspline_validator.h"

#include "traj_utils/msg/bspline.hpp"
#include "traj_utils/msg/trajectory_ack.hpp"
#include "easyfly_msgs/msg/navigation_session.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>
rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub;
rclcpp::Publisher<traj_utils::msg::TrajectoryAck>::SharedPtr trajectory_ack_pub;

quadrotor_msgs::msg::PositionCommand cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

using ego_planner::UniformBspline;

bool receive_traj_ = false;
bool navigation_managed = false;
std::string navigation_session_id;
std::string trajectory_session_id;
vector<UniformBspline> traj_;
double traj_duration_;
rclcpp::Time start_time_;
uint32_t traj_id_;

// yaw control
double last_yaw_, last_yaw_dot_;
double time_forward_;
std::shared_ptr<rclcpp::Clock> node_clock;
std::string output_frame = "odom";
std::string position_cmd_topic = "position_cmd";
std::string trajectory_ack_topic = "trajectory_ack";
std::string source_node_name = "traj_server";
std::uint64_t activation_sequence = 0;

void bsplineCallback(const std::shared_ptr<traj_utils::msg::Bspline> msg)
{
  // Foreign/late session traffic must not invalidate the current session.
  if (msg && (msg->navigation_session_id.size() > 128 ||
      (navigation_managed && (navigation_session_id.empty() ||
       msg->navigation_session_id != navigation_session_id)))) return;
  if (!msg)
  {
    if (node_clock)
    {
      RCLCPP_ERROR_THROTTLE(
          rclcpp::get_logger("traj_server"), *node_clock, 2000,
          "reject null B-spline message");
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("traj_server"), "reject null B-spline message");
    }
    receive_traj_ = false;
    return;
  }

  // This must be the first message-dependent operation.  In particular, do
  // not convert untrusted order/id values, allocate Eigen matrices, resize
  // trajectory storage, or index the knot vector before this shared ingress
  // contract has bounded and checked every field.
  const auto validation = ego_planner::swarm_validation::validateCubicBspline(*msg);
  if (!validation.valid())
  {
    if (node_clock)
    {
      RCLCPP_ERROR_THROTTLE(
          rclcpp::get_logger("traj_server"), *node_clock, 2000,
          "reject malformed B-spline: %s",
          ego_planner::swarm_validation::validationErrorName(validation.error));
    }
    else
    {
      RCLCPP_ERROR(
          rclcpp::get_logger("traj_server"), "reject malformed B-spline: %s",
          ego_planner::swarm_validation::validationErrorName(validation.error));
    }
    receive_traj_ = false;
    return;
  }

  if (!node_clock)
  {
    RCLCPP_ERROR(rclcpp::get_logger("traj_server"), "reject B-spline before node clock is ready");
    receive_traj_ = false;
    return;
  }

  // parse pos traj; sizes are bounded by validateBspline above.
  Eigen::MatrixXd pos_pts(
      3, static_cast<Eigen::Index>(msg->pos_pts.size()));

  Eigen::VectorXd knots(
      static_cast<Eigen::Index>(msg->knots.size()));
  for (std::size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }

  for (std::size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline candidate_position(pos_pts, msg->order, 0.1);
  candidate_position.setKnot(knots);
  UniformBspline candidate_velocity = candidate_position.getDerivative();
  UniformBspline candidate_acceleration = candidate_velocity.getDerivative();
  const auto runtime_validation =
      ego_planner::swarm_validation::validateCubicRuntimeTrajectory(
          candidate_position, candidate_velocity, candidate_acceleration,
          msg->pos_pts.size(), msg->knots.size(), validation.duration);
  if (!runtime_validation.valid())
  {
    RCLCPP_ERROR_THROTTLE(
        rclcpp::get_logger("traj_server"), *node_clock, 2000,
        "reject runtime B-spline candidate: %s",
        ego_planner::swarm_validation::runtimeValidationErrorName(
            runtime_validation.error));
    receive_traj_ = false;
    return;
  }

  // parse yaw traj

  // Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  // for (int i = 0; i < msg->yaw_pts.size(); ++i) {
  //   yaw_pts(i, 0) = msg->yaw_pts[i];
  // }

  //UniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

  const rclcpp::Time candidate_start_time(
      msg->start_time, node_clock->get_clock_type());
  if (candidate_start_time.nanoseconds() == 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("traj_server"), "reject B-spline with zero start_time");
    receive_traj_ = false;
    return;
  }
  const uint32_t candidate_traj_id = static_cast<uint32_t>(msg->traj_id);
  const double candidate_duration = candidate_position.getTimeSum();
  if (trajectory_ack_pub && node_clock &&
    activation_sequence == std::numeric_limits<std::uint64_t>::max())
  {
    // Wrapping to one would make an old ACK indistinguishable from a fresh
    // activation on the same publisher GID. Reject before activation.
    RCLCPP_ERROR(rclcpp::get_logger("traj_server"),
      "trajectory ACK activation_sequence exhausted; reject candidate");
    receive_traj_ = false;
    return;
  }
  std::vector<UniformBspline> candidate_traj;
  candidate_traj.reserve(3);
  candidate_traj.push_back(candidate_position);
  candidate_traj.push_back(candidate_velocity);
  candidate_traj.push_back(candidate_acceleration);

  // Publish/activate only after position, velocity and acceleration have all
  // passed structure, knot, control-point and sample checks.
  start_time_ = candidate_start_time;
  traj_id_ = candidate_traj_id;
  traj_duration_ = candidate_duration;
  traj_ = std::move(candidate_traj);

  receive_traj_ = true;
  trajectory_session_id = msg->navigation_session_id;

  // This is the only recovery ACK.  It is emitted after the complete
  // position/velocity/acceleration candidate has passed validation and has
  // been atomically activated; the periodic PositionCommand stream is not an
  // execution acknowledgement.
  if (trajectory_ack_pub && node_clock) {
    traj_utils::msg::TrajectoryAck ack;
    ack.header.stamp = node_clock->now();
    ack.header.frame_id = output_frame;
    ack.trajectory_id = candidate_traj_id;
    ack.navigation_session_id = trajectory_session_id;
    ack.status = traj_utils::msg::TrajectoryAck::STATUS_ACCEPTED;
    ++activation_sequence;
    ack.activation_sequence = activation_sequence;
    ack.source_node = source_node_name;
    try {
      trajectory_ack_pub->publish(ack);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        rclcpp::get_logger("traj_server"),
        "trajectory ACK publish failed for traj_id=%u: %s", candidate_traj_id, e.what());
    }
  }
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, rclcpp::Time &time_now, rclcpp::Time &time_last)
{
  constexpr double PI = 3.1415926;
  constexpr double YAW_DOT_MAX_PER_SEC = PI;
  // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw = 0;
  double yawdot = 0;
  const double dt = (time_now - time_last).seconds();
  if (dt <= 1e-6)
  {
    return {last_yaw_, 0.0};
  }

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_ ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos : traj_[0].evaluateDeBoorT(traj_duration_) - pos;
  double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
  double max_yaw_change = YAW_DOT_MAX_PER_SEC * dt;
  if (yaw_temp - last_yaw_ > PI)
  {
    if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / dt;
    }
  }
  else if (yaw_temp - last_yaw_ < -PI)
  {
    if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / dt;
    }
  }
  else
  {
    if (yaw_temp - last_yaw_ < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else if (yaw_temp - last_yaw_ > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / dt;
    }
  }

  if (fabs(yaw - last_yaw_) <= max_yaw_change)
    yaw = 0.5 * last_yaw_ + 0.5 * yaw; // nieve LPF
  yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
  last_yaw_ = yaw;
  last_yaw_dot_ = yawdot;

  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  return yaw_yawdot;
}

void cmdCallback()
{
  rclcpp::Time time_now = node_clock->now();
  static rclcpp::Time time_last = time_now;
  /* no publishing before receive traj_ */
  if (!receive_traj_) {
    // Idle/STOP is not a clock jump. Keep the timer baseline current so a
    // later session is not discarded merely because it starts after 0.5s.
    time_last = time_now;
    return;
  }
  double TEM_A=time_now.seconds();
  double TEM_B=start_time_.seconds();
  double t_cur = TEM_A - TEM_B;

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);

  const double clock_step = (time_now - time_last).seconds();
  if (clock_step < -0.001)
  {
    RCLCPP_ERROR(rclcpp::get_logger("traj_server"),
                 "ROS time moved backwards; discard current trajectory");
    receive_traj_ = false;
    time_last = time_now;
    return;
  }
  if (clock_step > 0.5)
  {
    RCLCPP_ERROR(rclcpp::get_logger("traj_server"),
                 "ROS time jumped forward by %.3fs; discard current trajectory", clock_step);
    receive_traj_ = false;
    time_last = time_now;
    return;
  }
  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);

    /*** calculate yaw ***/
    yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last);
    /*** calculate yaw ***/

    double tf = min(traj_duration_, t_cur + 2.0);
    pos_f = traj_[0].evaluateDeBoorT(tf);
  }
  else if (t_cur >= traj_duration_)
  {
    /* hover when finish traj_ */
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();

    yaw_yawdot.first = last_yaw_;
    yaw_yawdot.second = 0;

    pos_f = pos;
  }
  else
  {
    // 轨迹开始时间允许极小的调度提前；明显负值时不发布默认零点。
    if (t_cur >= -0.02)
    {
      pos = traj_[0].evaluateDeBoorT(0.0);
      vel = traj_[1].evaluateDeBoorT(0.0);
      acc = traj_[2].evaluateDeBoorT(0.0);
      pos_f = pos;
    }
    else
    {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("traj_server"), *node_clock, 1000,
                           "trajectory starts in the future (dt=%.3fs); hold publication", t_cur);
      time_last = time_now;
      return;
    }
  }
  time_last = time_now;

  cmd.header.stamp = time_now;
  cmd.header.frame_id = output_frame;
  cmd.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;
  cmd.navigation_session_id = trajectory_session_id;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);

  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  cmd.yaw = yaw_yawdot.first;
  cmd.yaw_dot = yaw_yawdot.second;

  last_yaw_ = cmd.yaw;

  pos_cmd_pub->publish(cmd);
}

int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("traj_server");//这个只是说节点的名字叫这个
  node_clock = nh->get_clock();
  navigation_managed = nh->declare_parameter<bool>("navigation_managed", false);
  auto session_sub = nh->create_subscription<easyfly_msgs::msg::NavigationSession>(
    "/planning/navigation_session_state", rclcpp::QoS(1).reliable().transient_local(),
    [](const easyfly_msgs::msg::NavigationSession::SharedPtr state) {
      if (!navigation_managed || state->session_id.empty() || state->session_id.size() > 128) return;
      if (state->active) {
        // EGO is the sole trusted lifecycle writer. Depth-one QoS may coalesce
        // STOP/START: a newer START must supersede the old session regardless.
        navigation_session_id = state->session_id;
        if (trajectory_session_id != navigation_session_id) receive_traj_ = false;
      } else if (navigation_session_id == state->session_id) {
        receive_traj_ = false;
        navigation_session_id.clear();
        trajectory_session_id.clear();
      }
    });

  nh->declare_parameter<std::string>("traj_server/position_cmd_topic", "position_cmd");
  nh->declare_parameter<std::string>("traj_server/ack_topic", "trajectory_ack");
  nh->declare_parameter<std::string>("traj_server/source_node_name", "");
  nh->get_parameter("traj_server/position_cmd_topic", position_cmd_topic);
  nh->get_parameter("traj_server/ack_topic", trajectory_ack_topic);
  std::string configured_source_node_name;
  nh->get_parameter("traj_server/source_node_name", configured_source_node_name);
  // The ACK source identity is always the actual graph FQN. A bare legacy
  // parameter cannot safely describe a node's namespace, so it is ignored;
  // deployments with hostile peers still require DDS Security beyond this
  // trusted ROS graph boundary.
  source_node_name = nh->get_fully_qualified_name();
  if (!configured_source_node_name.empty() && configured_source_node_name.front() == '/') {
    source_node_name = nh->get_fully_qualified_name();
  }

  // ros::Subscriber bspline_sub = nh.subscribe("planning/bspline", 10, bsplineCallback);
  auto bspline_sub = nh->create_subscription<traj_utils::msg::Bspline>("planning/bspline",10,bsplineCallback);

  pos_cmd_pub = nh->create_publisher<quadrotor_msgs::msg::PositionCommand>(
    position_cmd_topic, 50);
  trajectory_ack_pub = nh->create_publisher<traj_utils::msg::TrajectoryAck>(
    trajectory_ack_topic, rclcpp::QoS(1).reliable().transient_local());

  // ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);
  //TAG 这里是声明在类外的定时器，类内的如tare那个工程。    这里写代码过程中create_wall_timer下面画波浪线有可能是回调函数形参的问题，不一定是格式错误，做为写代码技巧记下了。
  auto cmd_timer = nh->create_wall_timer(std::chrono::milliseconds(10), cmdCallback);
  
  /* control parameter - load from parameters with sensible defaults */
  nh->declare_parameter<double>("traj_server/pos_gain_x", 5.0);
  nh->declare_parameter<double>("traj_server/pos_gain_y", 5.0);
  nh->declare_parameter<double>("traj_server/pos_gain_z", 5.0);
  nh->declare_parameter<double>("traj_server/vel_gain_x", 3.0);
  nh->declare_parameter<double>("traj_server/vel_gain_y", 3.0);
  nh->declare_parameter<double>("traj_server/vel_gain_z", 3.0);
  nh->get_parameter("traj_server/pos_gain_x", pos_gain[0]);
  nh->get_parameter("traj_server/pos_gain_y", pos_gain[1]);
  nh->get_parameter("traj_server/pos_gain_z", pos_gain[2]);
  nh->get_parameter("traj_server/vel_gain_x", vel_gain[0]);
  nh->get_parameter("traj_server/vel_gain_y", vel_gain[1]);
  nh->get_parameter("traj_server/vel_gain_z", vel_gain[2]);

  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  nh->declare_parameter<double>("traj_server/time_forward",1.0);
  nh->get_parameter("traj_server/time_forward",time_forward_);
  nh->declare_parameter<std::string>("traj_server/output_frame", "odom");
  nh->get_parameter("traj_server/output_frame", output_frame);
  last_yaw_ = 0.0;
  last_yaw_dot_ = 0.0;

  // ros::Duration(1.0).sleep();
  std::this_thread::sleep_for(std::chrono::seconds(1));

  RCLCPP_WARN(rclcpp::get_logger("traj_server"), "[Traj server]: ready.");


  // ros::spin();
  rclcpp::spin(nh);

  // Release ROS entities before static/default-context destruction. The global
  // publishers otherwise outlive the middleware and can crash or hang on exit.
  cmd_timer->cancel();
  cmd_timer.reset();
  bspline_sub.reset();
  session_sub.reset();
  trajectory_ack_pub.reset();
  pos_cmd_pub.reset();
  node_clock.reset();
  nh.reset();
  if (rclcpp::ok()) rclcpp::shutdown();

  return 0;
}
