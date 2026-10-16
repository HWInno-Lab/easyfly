#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <iostream>
#include <nav_msgs/msg/path.hpp>

#include <sensor_msgs/msg/imu.hpp>

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/empty.hpp>

#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include <bspline_opt/bspline_optimizer.h>
#include <plan_env/grid_map.h>
#include <traj_utils/msg/bspline.hpp>
#include <traj_utils/msg/trajectory_ack.hpp>
#include <easyfly_msgs/srv/set_navigation_session.hpp>
#include <easyfly_msgs/msg/navigation_session.hpp>

#include <traj_utils/msg/multi_bsplines.hpp>


#include <geometry_msgs/msg/pose_stamped.hpp>
#include <traj_utils/msg/data_disp.hpp>

#include <plan_manage/planner_manager.h>
#include <plan_manage/emergency_stop_gate.h>
#include <plan_manage/navigation_session_contract.h>
#include <traj_utils/planning_visualization.h>

using std::vector;

namespace ego_planner
{

  class EGOReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP,
      SEQUENTIAL_START
    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFENCE_PATH = 3
    };

    /* planning utils */
    EGOPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    traj_utils::msg::DataDisp data_disp_;
    traj_utils::msg::MultiBsplines multi_bspline_msgs_buf_;

    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;
    double waypoints_[50][3];
    int waypoint_num_, wp_id_;
    double planning_horizen_, planning_horizen_time_;
    double emergency_time_{1.0};
    double emergency_recovery_hold_time_{0.5};
    double emergency_recovery_max_speed_{0.1};
    double emergency_recovery_clock_jump_limit_{1.0};
    double odom_freshness_timeout_{1.0};
    double waypoint_future_tolerance_{0.2};
    int emergency_recovery_stable_cycles_{5};
    bool flag_realworld_experiment_;
    bool enable_fail_safe_;
    bool odom_depth_timeout_latched_{false};
    std::string trajectory_ack_topic_{"trajectory_ack"};
    bool navigation_managed_{false};
    std::string navigation_session_id_;
    std::string navigation_trajectory_session_id_;
    double navigation_max_velocity_{0.0};
    double original_manager_velocity_{0.0}, original_optimizer_velocity_{0.0};
    double navigation_requested_speed_{0.0};
    rclcpp::Service<easyfly_msgs::srv::SetNavigationSession>::SharedPtr navigation_service_;
    rclcpp::Publisher<easyfly_msgs::msg::NavigationSession>::SharedPtr navigation_session_pub_;
    rclcpp::Publisher<easyfly_msgs::msg::NavigationSession>::SharedPtr navigation_fault_pub_;
    void navigationSessionCallback(
      const std::shared_ptr<easyfly_msgs::srv::SetNavigationSession::Request> request,
      std::shared_ptr<easyfly_msgs::srv::SetNavigationSession::Response> response);
    std::string traj_server_node_name_{"traj_server"};
    // Full ROS graph identity (name + namespace), e.g. /drone_0_traj_server.
    // The graph endpoint namespace is checked separately from node_name.
    std::string traj_server_node_fqn_{"/traj_server"};

    /* planning data */
    bool have_trigger_, have_target_, have_odom_, have_new_target_, have_recv_pre_agent_;
    std::uint64_t target_generation_{0};
    std::uint64_t emergency_target_generation_{0};
    FSM_EXEC_STATE exec_state_;
    int continously_called_times_{0};
    // A failed terminal-point plan must not be retried on every 10 ms FSM tick.
    // Keep this budget on a monotonic clock; ROS time is reserved for message
    // timestamps and trajectory scheduling.
    std::chrono::steady_clock::time_point gen_retry_not_before_{};
    int gen_failure_count_{0};
    // REPLAN_TRAJ is also driven by a 100 Hz safety timer.  Keep failed
    // terminal-point retries on the same monotonic, bounded budget as
    // GEN_NEW_TRAJ; a safety transition still owns the state and is never
    // delayed by this gate.
    std::chrono::steady_clock::time_point replan_retry_not_before_{};
    int replan_failure_count_{0};

    Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d odom_vel_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d odom_acc_{Eigen::Vector3d::Zero()}; // odometry state
    Eigen::Quaterniond odom_orient_{Eigen::Quaterniond::Identity()};

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state
    std::vector<Eigen::Vector3d> wps_;
    int current_wp_;

    EmergencyStopGate emergency_stop_gate_;
    EmergencyRecoveryGate emergency_recovery_gate_;
    rclcpp::Time last_odom_stamp_;
    rclcpp::Time last_odom_receive_time_;
    bool odom_sample_valid_{false};
    rclcpp::Time last_waypoint_stamp_;
    bool waypoint_stamp_valid_{false};
    std::uint32_t emergency_stop_traj_id_{0};

    /* ROS utils */
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_;
    // ros::Subscriber waypoint_sub_, , trigger_sub_;
    // ros::Publisher replan_pub_, new_pub_, bspline_pub_, data_disp_pub_, , ;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<traj_utils::msg::TrajectoryAck>::SharedPtr trajectory_ack_sub_;
    rclcpp::Subscription<traj_utils::msg::MultiBsplines>::SharedPtr  swarm_trajs_sub_;
    rclcpp::Subscription<traj_utils::msg::Bspline>::SharedPtr broadcast_bspline_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr trigger_sub_;

    rclcpp::Publisher<traj_utils::msg::MultiBsplines>::SharedPtr swarm_trajs_pub_;
    rclcpp::Publisher<traj_utils::msg::Bspline>::SharedPtr broadcast_bspline_pub_;
    rclcpp::Publisher<traj_utils::msg::Bspline>::SharedPtr bspline_pub_;
    rclcpp::Publisher<traj_utils::msg::DataDisp>::SharedPtr data_disp_pub_;

    /* helper functions */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj); // front-end and back-end method
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    void enterEmergencyStop(const char * reason);
    bool planFromGlobalTraj(const int trial_times = 1);
    bool planFromCurrentTraj(const int trial_times = 1);

    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    void readGivenWps();
    void planNextWaypoint(const Eigen::Vector3d next_wp);
    void getLocalTarget();

    /* ROS functions */
    void execFSMCallback();
    void checkCollisionCallback();
    void waypointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void triggerCallback(const geometry_msgs::msg::PoseStamped::SharedPtr  msg);
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr  msg);
    void trajectoryAckCallback(
      const traj_utils::msg::TrajectoryAck::SharedPtr msg,
      const rclcpp::MessageInfo & message_info);
    void swarmTrajsCallback(const traj_utils::msg::MultiBsplines::SharedPtr msg);
    void BroadcastBsplineCallback(const traj_utils::msg::Bspline::SharedPtr msg);

    bool checkCollision();
    bool hasFreshTrustedCloud() const;
    bool hasFreshOdom() const;
    bool hasLiveTrajectoryAckSource() const;
    void publishSwarmTrajs(bool startup_pub);

  public:
    EGOReplanFSM(/* args */)
    {
    }
    ~EGOReplanFSM()
    {
    }

    void init(std::shared_ptr<rclcpp::Node> &nh,
              const std::function<bool()> &stop_requested = {});

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace ego_planner

#endif
