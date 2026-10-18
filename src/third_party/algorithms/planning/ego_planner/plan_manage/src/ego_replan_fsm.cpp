
#include <plan_manage/ego_replan_fsm.h>
#include <bspline_opt/collision_sampling.h>
#include <plan_manage/bspline_runtime_validator.h>
#include <plan_manage/collision_safety.h>
#include <plan_manage/waypoint_admission.h>
#include <plan_manage/swarm_bspline_validator.h>
#include <cmath>
#include <exception>
#include <limits>
#include <string>
#include <utility>

namespace ego_planner
{

  void EGOReplanFSM::init(std::shared_ptr<rclcpp::Node> &nh,
                         const std::function<bool()> &stop_requested)
  {
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    have_recv_pre_agent_ = false;
    node_=nh;
    nh->declare_parameter<int>("fsm/flight_type",-1);
    nh->declare_parameter<double>("fsm/thresh_replan_time",-1.0);
    nh->declare_parameter<double>("fsm/thresh_no_replan_meter",-1.0);
    nh->declare_parameter<double>("fsm/planning_horizon",-1.0);
    nh->declare_parameter<double>("fsm/planning_horizen_time",-1.0);
    nh->declare_parameter<double>("fsm/emergency_time",1.0);
    nh->declare_parameter<double>("fsm/emergency_recovery_hold_time",0.5);
    nh->declare_parameter<int>("fsm/emergency_recovery_stable_cycles",5);
    nh->declare_parameter<double>("fsm/emergency_recovery_max_speed",0.1);
    nh->declare_parameter<double>("fsm/emergency_recovery_clock_jump_limit",1.0);
    nh->declare_parameter<double>("fsm/emergency_recovery_ack_timeout",0.5);
    nh->declare_parameter<double>("fsm/emergency_recovery_ack_retry_period",0.5);
    nh->declare_parameter<double>("fsm/emergency_recovery_ack_retry_max_period",5.0);
    nh->declare_parameter<std::string>("fsm/traj_server_ack_topic","trajectory_ack");
    nh->declare_parameter<std::string>("fsm/traj_server_node_name","traj_server");
    nh->declare_parameter<std::string>("fsm/traj_server_node_fqn","");
    nh->declare_parameter<double>("fsm/odom_freshness_timeout",1.0);
    nh->declare_parameter<double>("fsm/waypoint_future_tolerance",0.2);
    nh->declare_parameter<bool>("fsm/realworld_experiment",false);
    nh->declare_parameter<bool>("fsm/fail_safe",true);
    nh->declare_parameter<int>("fsm/waypoint_num",-1);
    nh->get_parameter("fsm/flight_type",target_type_);
    nh->get_parameter("fsm/thresh_replan_time",replan_thresh_);
    nh->get_parameter("fsm/thresh_no_replan_meter",no_replan_thresh_);
    nh->get_parameter("fsm/planning_horizon",planning_horizen_);
    nh->get_parameter("fsm/planning_horizen_time",planning_horizen_time_);
    nh->get_parameter("fsm/emergency_time",emergency_time_);
    nh->get_parameter("fsm/emergency_recovery_hold_time",emergency_recovery_hold_time_);
    nh->get_parameter("fsm/emergency_recovery_stable_cycles",emergency_recovery_stable_cycles_);
    nh->get_parameter("fsm/emergency_recovery_max_speed",emergency_recovery_max_speed_);
    nh->get_parameter(
      "fsm/emergency_recovery_clock_jump_limit", emergency_recovery_clock_jump_limit_);
    double emergency_recovery_ack_timeout = 0.5;
    double emergency_recovery_ack_retry_period = 0.5;
    double emergency_recovery_ack_retry_max_period = 5.0;
    nh->get_parameter("fsm/emergency_recovery_ack_timeout", emergency_recovery_ack_timeout);
    nh->get_parameter(
      "fsm/emergency_recovery_ack_retry_period", emergency_recovery_ack_retry_period);
    nh->get_parameter(
      "fsm/emergency_recovery_ack_retry_max_period", emergency_recovery_ack_retry_max_period);
    nh->get_parameter("fsm/traj_server_ack_topic", trajectory_ack_topic_);
    nh->get_parameter("fsm/traj_server_node_name", traj_server_node_name_);
    nh->get_parameter("fsm/traj_server_node_fqn", traj_server_node_fqn_);
    if (traj_server_node_fqn_.empty()) {
      traj_server_node_fqn_ = traj_server_node_name_;
    }
    nh->get_parameter("fsm/odom_freshness_timeout",odom_freshness_timeout_);
    nh->get_parameter("fsm/waypoint_future_tolerance",waypoint_future_tolerance_);
    nh->get_parameter("fsm/realworld_experiment",flag_realworld_experiment_);
    nh->get_parameter("fsm/fail_safe",enable_fail_safe_);
    nh->get_parameter("fsm/waypoint_num",waypoint_num_);

    EmergencyRecoveryGate::Params recovery_params;
    recovery_params.stable_cycles = emergency_recovery_stable_cycles_;
    recovery_params.hold_seconds = emergency_recovery_hold_time_;
    recovery_params.max_speed = emergency_recovery_max_speed_;
    recovery_params.max_clock_jump = emergency_recovery_clock_jump_limit_;
    recovery_params.ack_timeout = emergency_recovery_ack_timeout;
    recovery_params.ack_retry_period = emergency_recovery_ack_retry_period;
    recovery_params.ack_retry_max_period = emergency_recovery_ack_retry_max_period;
    emergency_recovery_gate_.configure(recovery_params);
    // The recovery gate rejects malformed safety overrides by falling back to
    // conservative defaults.  Keep the effective values visible to the FSM
    // and logs; never silently widen a safety window by clipping it.
    const auto effective_recovery_params = emergency_recovery_gate_.params();
    emergency_recovery_stable_cycles_ = effective_recovery_params.stable_cycles;
    emergency_recovery_hold_time_ = effective_recovery_params.hold_seconds;
    emergency_recovery_max_speed_ = effective_recovery_params.max_speed;
    emergency_recovery_clock_jump_limit_ = effective_recovery_params.max_clock_jump;
    emergency_recovery_ack_timeout = effective_recovery_params.ack_timeout;
    emergency_recovery_ack_retry_period = effective_recovery_params.ack_retry_period;
    emergency_recovery_ack_retry_max_period = effective_recovery_params.ack_retry_max_period;
    // Keep the local odom freshness guard finite even when a launch file is
    // incomplete or a malformed override is supplied.
    if (!std::isfinite(odom_freshness_timeout_) || odom_freshness_timeout_ <= 0.0 ||
      odom_freshness_timeout_ > 60.0)
    {
      odom_freshness_timeout_ = 1.0;
    }
    waypoint_future_tolerance_ = waypoint_admission::boundedFutureTolerance(
      waypoint_future_tolerance_);
    const double configured_emergency_time = emergency_time_;
    emergency_time_ = collision_safety::sanitizeEmergencyTime(emergency_time_);
    if (configured_emergency_time != emergency_time_) {
      RCLCPP_WARN(
        rclcpp::get_logger("EGOReplanFSM"),
        "fsm/emergency_time=%.3f 无效，回退到安全值 %.3f",
        configured_emergency_time, emergency_time_);
    }
    /*  fsm param  */
    // nh.param("fsm/flight_type", target_type_, -1);
    // nh.param("fsm/thresh_replan_time", replan_thresh_, -1.0);
    // nh.param("fsm/thresh_no_replan_meter", no_replan_thresh_, -1.0);
    // nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    // nh.param("fsm/planning_horizen_time", planning_horizen_time_, -1.0);
    // nh.param("fsm/emergency_time", emergency_time_, 1.0);
    // nh.param("fsm/realworld_experiment", flag_realworld_experiment_, false);
    // nh.param("fsm/fail_safe", enable_fail_safe_, true);
    // nh.param("fsm/waypoint_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh->declare_parameter<double>("fsm/waypoint" + to_string(i) + "_x",-1.0);
      nh->declare_parameter<double>("fsm/waypoint" + to_string(i) + "_y",-1.0);
      nh->declare_parameter<double>("fsm/waypoint" + to_string(i) + "_z",-1.0);
      nh->get_parameter("fsm/waypoint" + to_string(i) + "_x",waypoints_[i][0]);
      nh->get_parameter("fsm/waypoint" + to_string(i) + "_y",waypoints_[i][1]);
      nh->get_parameter("fsm/waypoint" + to_string(i) + "_z",waypoints_[i][2]);
      // nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      // nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      // nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }
    


    have_trigger_ = !flag_realworld_experiment_;
    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);
    planner_manager_->deliverTrajToOptimizer(); // store trajectories
    planner_manager_->setDroneIdtoOpt();
    navigation_managed_ = nh->declare_parameter<bool>("navigation_managed", false);
    if (navigation_managed_ && target_type_ != TARGET_TYPE::MANUAL_TARGET) {
      RCLCPP_WARN(nh->get_logger(), "managed navigation overrides legacy flight_type=%d to MANUAL_TARGET", target_type_);
      target_type_ = TARGET_TYPE::MANUAL_TARGET;
    }
    navigation_max_velocity_ = planner_manager_->pp_.max_vel_;
    double optimizer_max_velocity = navigation_max_velocity_;
    nh->get_parameter("optimization/max_vel", optimizer_max_velocity);
    navigation_max_velocity_ = std::min(navigation_max_velocity_, optimizer_max_velocity);
    original_manager_velocity_ = planner_manager_->pp_.max_vel_;
    original_optimizer_velocity_ = optimizer_max_velocity;
    navigation_session_pub_ = nh->create_publisher<easyfly_msgs::msg::NavigationSession>(
      "/planning/navigation_session_state", rclcpp::QoS(1).reliable().transient_local());
    navigation_fault_pub_ = nh->create_publisher<easyfly_msgs::msg::NavigationSession>(
      "/planning/navigation_fault", rclcpp::QoS(1).reliable().transient_local());
    navigation_service_ = nh->create_service<easyfly_msgs::srv::SetNavigationSession>(
      "/planning/navigation_session", std::bind(&EGOReplanFSM::navigationSessionCallback,
      this, std::placeholders::_1, std::placeholders::_2));

    /* callback */
    // exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);
    exec_timer_ = nh->create_wall_timer(std::chrono::milliseconds(10), std::bind(&EGOReplanFSM::execFSMCallback, this));

    // safety_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this);
    safety_timer_ = nh->create_wall_timer(std::chrono::milliseconds(50), std::bind(&EGOReplanFSM::checkCollisionCallback, this));

    // odom_sub_ = nh.subscribe("odom_world", 1, &EGOReplanFSM::odometryCallback, this);
    odom_sub_ = nh->create_subscription<nav_msgs::msg::Odometry>(
      "odom_world", rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&EGOReplanFSM::odometryCallback, this, std::placeholders::_1));
    // The downstream executor owns this dedicated ACK topic.  It is not the
    // periodic PositionCommand stream: only traj_server publishes an ACK
    // after validating and activating a candidate B-spline.
    auto ack_qos = rclcpp::QoS(1).reliable().transient_local();
    trajectory_ack_sub_ = nh->create_subscription<traj_utils::msg::TrajectoryAck>(
      trajectory_ack_topic_, ack_qos,
      std::bind(
        &EGOReplanFSM::trajectoryAckCallback, this, std::placeholders::_1,
        std::placeholders::_2));

    if (planner_manager_->pp_.drone_id >= 1)
    {
      string sub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id - 1) + string("_planning/swarm_trajs");
      // swarm_trajs_sub_ = nh.subscribe(sub_topic_name.c_str(), 10, &EGOReplanFSM::swarmTrajsCallback, this, ros::TransportHints().tcpNoDelay());
      rclcpp::QoS qos_profile = rclcpp::QoS(10);
      qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);  // 选择最低延迟的可靠性配置
      swarm_trajs_sub_ = nh->create_subscription<traj_utils::msg::MultiBsplines>(sub_topic_name.c_str(),qos_profile,std::bind(&EGOReplanFSM::swarmTrajsCallback, this, std::placeholders::_1));
    }
    string pub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id) + string("_planning/swarm_trajs");
    // swarm_trajs_pub_ = nh.advertise<traj_utils::msg::MultiBsplines>(pub_topic_name.c_str(), 10);
    swarm_trajs_pub_ = nh->create_publisher<traj_utils::msg::MultiBsplines>(pub_topic_name.c_str(), 10);


    // broadcast_bspline_pub_ = nh.advertise<traj_utils::msg::Bspline>("planning/broadcast_bspline_from_planner", 10);
    broadcast_bspline_pub_ = nh->create_publisher<traj_utils::msg::Bspline>("planning/broadcast_bspline_from_planner", 10);

    // broadcast_bspline_sub_ = nh.subscribe("planning/broadcast_bspline_to_planner", 100, &EGOReplanFSM::BroadcastBsplineCallback, this, ros::TransportHints().tcpNoDelay());
    rclcpp::QoS qos_profile = rclcpp::QoS(100);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);  // 选择最低延迟的可靠性配置
    broadcast_bspline_sub_ = nh->create_subscription<traj_utils::msg::Bspline>("planning/broadcast_bspline_to_planner",qos_profile,std::bind(&EGOReplanFSM::BroadcastBsplineCallback, this, std::placeholders::_1));

    // bspline_pub_ = nh.advertise<traj_utils::msg::Bspline>("planning/bspline", 10);
    bspline_pub_ = nh->create_publisher<traj_utils::msg::Bspline>("planning/bspline", 10);

    // data_disp_pub_ = nh.advertise<traj_utils::msg::DataDisp>("planning/data_display", 100);
    data_disp_pub_ = nh->create_publisher<traj_utils::msg::DataDisp>("planning/data_display", 100);

    if (!navigation_managed_ && target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      // waypoint_sub_ = nh.subscribe("/move_base_simple/goal", 1, &EGOReplanFSM::waypointCallback, this);
      waypoint_sub_ = nh->create_subscription<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal",1,std::bind(&EGOReplanFSM::waypointCallback, this, std::placeholders::_1));
    }
    else if (!navigation_managed_ && target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      // trigger_sub_ = nh.subscribe("/traj_start_trigger", 1, &EGOReplanFSM::triggerCallback, this);
      trigger_sub_ = nh->create_subscription<geometry_msgs::msg::PoseStamped>("/traj_start_trigger",1,std::bind(&EGOReplanFSM::triggerCallback, this, std::placeholders::_1));

      // ROS_INFO("Wait for 1 second.");
      RCLCPP_INFO(rclcpp::get_logger("EGOReplanFSM"), "Wait for 1 second.");
      int count = 0;
      while (rclcpp::ok() && count++ < 1000)
      {
        if (stop_requested && stop_requested()) {return;}
        // ros::spinOnce();
        // ros::Duration(0.001).sleep();
        rclcpp::spin_some(nh);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      // ROS_WARN("Waiting for trigger from [n3ctrl] from RC");
      RCLCPP_WARN(rclcpp::get_logger("EGOReplanFSM"), "Waiting for trigger from [n3ctrl] from RC");
      while (rclcpp::ok() && (!have_odom_ || !have_trigger_))
      {
        if (stop_requested && stop_requested()) {return;}
        // ros::spinOnce();
        rclcpp::spin_some(nh);
        // ros::Duration(0.001).sleep();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      if (rclcpp::ok() && !(stop_requested && stop_requested())) {readGivenWps();}
    }
    else if (!navigation_managed_)
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  void EGOReplanFSM::readGivenWps()
  {
    if (waypoint_num_ <= 0)
    {
      // ROS_ERROR("Wrong waypoint_num_ = %d", waypoint_num_);
      RCLCPP_ERROR(rclcpp::get_logger("EGOReplanFSM"), "Wrong waypoint_num_ = %d", waypoint_num_);

      return;
    }

    wps_.resize(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps_[i](0) = waypoints_[i][0];
      wps_[i](1) = waypoints_[i][1];
      wps_[i](2) = waypoints_[i][2];

      // end_pt_ = wps_.back();
    }

    // bool success = planner_manager_->planGlobalTrajWaypoints(
    //   odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    //   wps_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps_[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      // ros::Duration(0.001).sleep();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // plan first global waypoint
    wp_id_ = 0;
    planNextWaypoint(wps_[wp_id_]);

    // if (success)
    // {

    //   /*** display ***/
    //   constexpr double step_size_t = 0.1;
    //   int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    //   std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    //   for (int i = 0; i < i_end; i++)
    //   {
    //     gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    //   }

    //   end_vel_.setZero();
    //   have_target_ = true;
    //   have_new_target_ = true;

    //   /*** FSM ***/
    //   // if (exec_state_ == WAIT_TARGET)
    //   //changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
    //   // trigger_ = true;
    //   // else if (exec_state_ == EXEC_TRAJ)
    //   //   changeFSMExecState(REPLAN_TRAJ, "TRIG");

    //   // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
    //   ros::Duration(0.001).sleep();
    //   visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    //   ros::Duration(0.001).sleep();
    // }
    // else
    // {
    //   ROS_ERROR("Unable to generate global trajectory!");
    // }
  }

  void EGOReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp)
  {
    if (!next_wp.allFinite() || next_wp.cwiseAbs().maxCoeff() > 1000.0 ||
        !std::isfinite(planning_horizen_) || planning_horizen_ <= 0.0) {
      RCLCPP_ERROR(rclcpp::get_logger("EGOReplanFSM"),
                   "拒绝无效航点或规划范围参数");
      return;
    }
    bool success = false;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), next_wp, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    // visualization_->displayGoalPoint(next_wp, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
      end_pt_ = next_wp;

      /*** display ***/
      constexpr double step_size_t = 0.1;
      const double global_duration = planner_manager_->global_data_.global_duration_;
      constexpr double max_visualization_duration = 300.0;
      constexpr int max_visualization_samples = 10000;
      if (!std::isfinite(global_duration) || global_duration <= 0.0 ||
          global_duration > max_visualization_duration) {
        RCLCPP_ERROR(rclcpp::get_logger("EGOReplanFSM"),
                     "全局轨迹时长 %.3f 无效或超出 %.1fs 上限",
                     global_duration, max_visualization_duration);
        return;
      }
      const auto sample_count = static_cast<long long>(std::ceil(global_duration / step_size_t));
      if (sample_count <= 0 || sample_count > max_visualization_samples) {
        RCLCPP_ERROR(rclcpp::get_logger("EGOReplanFSM"),
                     "全局轨迹采样数 %lld 无效或超出 %d 上限",
                     sample_count, max_visualization_samples);
        return;
      }
      const int i_end = static_cast<int>(sample_count);
      vector<Eigen::Vector3d> gloabl_traj(static_cast<size_t>(i_end));
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;
      // Every accepted global target gets a generation.  The emergency-stop
      // recovery path compares this with its entry snapshot before clearing a
      // terminal target, so a newly submitted near goal is never discarded.
      ++target_generation_;
      if (exec_state_ == EMERGENCY_STOP)
      {
        // A newly accepted target is the explicit reset authority for an
        // ODOM_DEPTH_TIMEOUT latch.  Do not let stable evidence from the old
        // target carry into this generation.
        emergency_recovery_gate_.resetStableEvidence();
      }

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      // GEN_NEW_TRAJ/REPLAN_TRAJ/SEQUENTIAL_START are already progressing
      // through the normal timer.  In particular, do not leave
      // EMERGENCY_STOP here: the timer must complete the stop and wait for a
      // fresh trusted cloud before resuming.

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      // ROS_ERROR("Unable to generate global trajectory!");
      RCLCPP_ERROR(rclcpp::get_logger("EGOReplanFSM"), "Unable to generate global trajectory!");

    }
  }

  void EGOReplanFSM::navigationSessionCallback(
    const std::shared_ptr<easyfly_msgs::srv::SetNavigationSession::Request> request,
    std::shared_ptr<easyfly_msgs::srv::SetNavigationSession::Response> response)
  {
    response->success = false;
    if (!navigation_managed_ || request->session_id.empty() || request->session_id.size() > 128) {
      response->message = "invalid_session_or_unmanaged";
      return;
    }
    if (!request->active) {
      if (!navigation_session_id_.empty() && navigation_session_id_ != request->session_id) {
        response->message = "session_mismatch";
        return;
      }
      have_target_ = have_new_target_ = have_trigger_ = false;
      if (exec_state_ != EMERGENCY_STOP) changeFSMExecState(WAIT_TARGET, "SESSION_STOP");
      navigation_session_id_.clear();
      navigation_trajectory_session_id_.clear();
      planner_manager_->setNavigationVelocityLimits(
        original_manager_velocity_, original_optimizer_velocity_);
      planner_manager_->setNavigationFixedEndpoint(false);
      easyfly_msgs::msg::NavigationSession state;
      state.session_id = request->session_id;
      state.active = false;
      navigation_session_pub_->publish(state);
      response->success = true;
      response->message = "stopped";
      return;
    }
    if (!navigation_session_id_.empty()) {
      response->message = "session_busy";
      return;
    }
    const Eigen::Vector3d target(request->target_x, request->target_y, request->target_z);
    const double axis_limit = navigationAxisVelocityLimit(
      request->max_speed, planner_manager_->pp_.feasibility_tolerance_);
    const bool fresh_odom = hasFreshOdom();
    const bool fresh_cloud = fresh_odom && hasFreshTrustedCloud();
    if (!fresh_odom || !fresh_cloud || !target.allFinite() ||
        target.cwiseAbs().maxCoeff() > 1000.0 ||
        !planner_manager_->grid_map_->isInMap(target) ||
        target.z() < planner_manager_->grid_map_->getGroundHeight() ||
        target.z() >= planner_manager_->grid_map_->getVirtualCeilingHeight() ||
        !std::isfinite(navigation_max_velocity_) || navigation_max_velocity_ <= 0.0 ||
        !std::isfinite(request->max_speed) || request->max_speed <= 0.0 ||
        request->max_speed > navigation_max_velocity_ || !std::isfinite(axis_limit)) {
      response->message = "invalid_target_speed_or_stale_sensors";
      RCLCPP_WARN(node_->get_logger(),
        "navigation START rejected: odom=%d cloud=%d target=(%.6f,%.6f,%.6f) "
        "speed=%.6f cap=%.6f axis=%.6f now=%.6f odom_source=%.6f odom_receipt=%.6f",
        fresh_odom ? 1 : 0, fresh_cloud ? 1 : 0, target.x(), target.y(), target.z(),
        request->max_speed, navigation_max_velocity_, axis_limit,
        node_->get_clock()->now().seconds(), last_odom_stamp_.seconds(),
        last_odom_receive_time_.seconds());
      return;
    }
    planner_manager_->setNavigationVelocityLimits(axis_limit, axis_limit);
    planner_manager_->setNavigationFixedEndpoint(true);
    have_target_ = false;
    planNextWaypoint(target);
    if (!have_target_) {
      planner_manager_->setNavigationVelocityLimits(
        original_manager_velocity_, original_optimizer_velocity_);
      planner_manager_->setNavigationFixedEndpoint(false);
      response->message = "global_plan_failed";
      return;
    }
    navigation_session_id_ = request->session_id;
    navigation_trajectory_session_id_.clear();
    navigation_requested_speed_ = request->max_speed;
    have_trigger_ = true;
    if (exec_state_ == EMERGENCY_STOP) {
      emergency_recovery_gate_.beginStop(emergency_target_generation_);
      emergency_stop_gate_.retry();
    }
    easyfly_msgs::msg::NavigationSession state;
    state.session_id = navigation_session_id_;
    state.active = true;
    navigation_session_pub_->publish(state);
    response->success = true;
    response->message = "started";
  }

  void EGOReplanFSM::triggerCallback(const geometry_msgs::msg::PoseStamped::SharedPtr  msg)
  {
    have_trigger_ = true;
    cout << "Triggered!" << endl;
    init_pt_ = odom_pos_;
  }

  void EGOReplanFSM::waypointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (navigation_managed_) return;
    // A manual goal is accepted only in the planner's odom frame and only
    // while both the goal timestamp and the odometry sample are current.  In
    // particular, do these checks before reading odom_pos_ or entering the
    // synchronous planner: an empty/map-frame goal must never be silently
    // interpreted as an odom goal.
    if (!msg || !node_ ||
        !waypoint_admission::validOdomFrame(msg->header.frame_id))
    {
      RCLCPP_WARN(
        rclcpp::get_logger("EGOReplanFSM"),
        "拒绝非 odom frame 航点 (frame='%s')",
        msg ? msg->header.frame_id.c_str() : "<null>");
      return;
    }

    if (!waypoint_admission::validStampFields(
          msg->header.stamp.sec, msg->header.stamp.nanosec))
    {
      RCLCPP_WARN(rclcpp::get_logger("EGOReplanFSM"), "拒绝无效/零时间戳航点");
      return;
    }

    rclcpp::Time waypoint_stamp;
    rclcpp::Time now;
    try
    {
      const auto clock_type = node_->get_clock()->get_clock_type();
      waypoint_stamp = rclcpp::Time(msg->header.stamp, clock_type);
      now = node_->get_clock()->now();
    }
    catch (const std::exception &)
    {
      RCLCPP_WARN(rclcpp::get_logger("EGOReplanFSM"), "无法读取航点 ROS 时间");
      return;
    }

    const double age = (now - waypoint_stamp).seconds();
    if (waypoint_stamp.nanoseconds() <= 0 || now.nanoseconds() <= 0 ||
        !waypoint_admission::validTimestampWindow(
          age, odom_freshness_timeout_, waypoint_future_tolerance_))
    {
      RCLCPP_WARN(
        rclcpp::get_logger("EGOReplanFSM"),
        "拒绝过期/明显未来航点 (age=%.3f, timeout=%.3f, future=%.3f)",
        age, odom_freshness_timeout_, waypoint_future_tolerance_);
      return;
    }

    if (!waypoint_admission::nonDecreasingStamp(
          waypoint_stamp.nanoseconds(), last_waypoint_stamp_.nanoseconds(),
          waypoint_stamp_valid_))
    {
      RCLCPP_WARN(rclcpp::get_logger("EGOReplanFSM"), "拒绝时间戳回退航点");
      return;
    }

    // This guard is independent of whether the request supplies z: a valid
    // explicit altitude must not make stale/absent odometry usable.
    if (!hasFreshOdom())
    {
      RCLCPP_WARN(rclcpp::get_logger("EGOReplanFSM"), "拒绝航点：没有新鲜有效 odom");
      return;
    }

    if (!waypoint_admission::validFinitePosition(
          msg->pose.position.x, msg->pose.position.y, msg->pose.position.z) ||
        std::abs(msg->pose.position.x) > 1000.0 ||
        std::abs(msg->pose.position.y) > 1000.0 ||
        std::abs(msg->pose.position.z) > 1000.0 ||
        msg->pose.position.z < -0.1)
    {
      return;
    }

    cout << "Triggered!" << endl;

    // Honour a finite requested altitude.  RViz's 2-D goal commonly leaves
    // z=0; in that case retain the current safe hover altitude instead of
    // sending a hidden 1.0 m descent into the ground/vertical-inflation band.
    // The acceptance contract supplies the synchronized hover z explicitly;
    // invalid/too-low requests fail safe to the current odometry altitude.
    double goal_z = msg->pose.position.z;
    if (!waypoint_admission::selectGoalAltitude(
          msg->pose.position.z, odom_pos_.z(), goal_z))
    {
      RCLCPP_WARN(rclcpp::get_logger("EGOReplanFSM"), "拒绝航点：目标高度或 odom 高度无效");
      return;
    }

    last_waypoint_stamp_ = waypoint_stamp;
    waypoint_stamp_valid_ = true;
    init_pt_ = odom_pos_;
    Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, goal_z);

    planNextWaypoint(end_wp);
  }

  void EGOReplanFSM::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr  msg)
  {
    const auto invalidateOdom = [this]() {
      have_odom_ = false;
      odom_sample_valid_ = false;
    };
    if (!msg || !std::isfinite(msg->pose.pose.position.x) ||
        !std::isfinite(msg->pose.pose.position.y) ||
        !std::isfinite(msg->pose.pose.position.z) ||
        !std::isfinite(msg->twist.twist.linear.x) ||
        !std::isfinite(msg->twist.twist.linear.y) ||
        !std::isfinite(msg->twist.twist.linear.z) ||
        !std::isfinite(msg->pose.pose.orientation.w) ||
        !std::isfinite(msg->pose.pose.orientation.x) ||
        !std::isfinite(msg->pose.pose.orientation.y) ||
        !std::isfinite(msg->pose.pose.orientation.z)) {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("EGOReplanFSM"),
                           *node_->get_clock(), 1000,
                           "忽略包含非有限值的 odom");
      invalidateOdom();
      return;
    }

    // Keep both the message stamp and node-clock arrival time.  The stop
    // recovery gate must not treat one old/latched odom sample as evidence of
    // a stable vehicle, and a clock-domain/reset error must hold the safety
    // state rather than silently refresh it.
    rclcpp::Time stamp;
    rclcpp::Time receive_time;
    try {
      const auto clock_type = node_->get_clock()->get_clock_type();
      stamp = rclcpp::Time(msg->header.stamp, clock_type);
      receive_time = node_->get_clock()->now();
    } catch (const std::exception &) {
      invalidateOdom();
      return;
    }
    const double age = (receive_time - stamp).seconds();
    if (stamp.nanoseconds() <= 0 || receive_time.nanoseconds() <= 0 ||
      !std::isfinite(age) || age < -0.2 || age > odom_freshness_timeout_)
    {
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("EGOReplanFSM"), *node_->get_clock(), 1000,
        "忽略过旧/未来/无时间戳 odom (age=%.3f)", age);
      invalidateOdom();
      return;
    }
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    last_odom_stamp_ = stamp;
    last_odom_receive_time_ = receive_time;
    odom_sample_valid_ = true;
    have_odom_ = true;
  }

  void EGOReplanFSM::trajectoryAckCallback(
    const traj_utils::msg::TrajectoryAck::SharedPtr msg,
    const rclcpp::MessageInfo & message_info)
  {
    // ego_planner_node uses rclcpp::spin(), so this callback and the FSM
    // timers share the node's mutually-exclusive default callback group.
    // The gate is therefore serialized by the documented single-threaded
    // executor contract; no detached ACK worker may mutate it.
    if (!node_)
    {
      return;
    }

    if (!msg || !matchesNavigationSession(msg->navigation_session_id, navigation_session_id_) ||
      msg->status != traj_utils::msg::TrajectoryAck::STATUS_ACCEPTED ||
      msg->activation_sequence == 0)
    {
      return;
    }

    try
    {
      const auto clock_type = node_->get_clock()->get_clock_type();
      const rclcpp::Time message_stamp(msg->header.stamp, clock_type);
      const rclcpp::Time receive_time = node_->get_clock()->now();
      const auto endpoint_infos = node_->get_publishers_info_by_topic(
        trajectory_ack_sub_ ? trajectory_ack_sub_->get_topic_name() : trajectory_ack_topic_);
      std::size_t publisher_count = 0;
      const rclcpp::TopicEndpointInfo * unique_publisher = nullptr;
      for (const auto & endpoint : endpoint_infos) {
        if (endpoint.endpoint_type() == rclcpp::EndpointType::Publisher) {
          ++publisher_count;
          unique_publisher = &endpoint;
        }
      }
      bool source_valid = false;
      std::string publisher_key;
      if (publisher_count == 1 && unique_publisher != nullptr) {
        const auto & publisher_gid = message_info.get_rmw_message_info().publisher_gid;
        const auto & endpoint_gid = unique_publisher->endpoint_gid();
        const bool gid_matches = std::equal(
          endpoint_gid.begin(), endpoint_gid.end(), publisher_gid.data);
        const bool node_fqn_matches = EmergencyRecoveryGate::endpointFqnMatches(
          unique_publisher->node_name(), unique_publisher->node_namespace(),
          traj_server_node_fqn_);
        source_valid = EmergencyRecoveryGate::validTrajectoryAckSource(
          true, gid_matches, unique_publisher->node_name(),
          unique_publisher->node_namespace(), traj_server_node_fqn_,
          unique_publisher->topic_type(), msg->source_node);
        if (gid_matches && node_fqn_matches) {
          // Use the discovered endpoint GID for both the callback and the
          // per-tick liveness check below.  The graph endpoint is the trusted
          // identity; the MessageInfo GID is only used to authenticate this
          // particular sample against it.
          publisher_key.reserve(endpoint_gid.size());
          for (const auto byte : endpoint_gid) {
            publisher_key.push_back(static_cast<char>(byte));
          }
        }
      }
      emergency_recovery_gate_.acknowledgeStop(
        msg->trajectory_id, msg->activation_sequence, source_valid, publisher_key,
        message_stamp.seconds(), receive_time.seconds());
      if (!source_valid) {
        RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger("EGOReplanFSM"), *node_->get_clock(), 2000,
          "拒绝非唯一 traj_server 来源的急停 ACK (publishers=%zu)", publisher_count);
      }
    }
    catch (const std::exception & e)
    {
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("EGOReplanFSM"), *node_->get_clock(), 2000,
        "忽略无法校验的 traj_server 专用急停 ACK: %s", e.what());
    }
  }

  void EGOReplanFSM::BroadcastBsplineCallback(const traj_utils::msg::Bspline::SharedPtr msg)
  {
    if (!msg)
    {
      if (node_)
      {
        RCLCPP_WARN_THROTTLE(
            rclcpp::get_logger("EGOReplanFSM"), *node_->get_clock(), 2000,
            "[BroadcastBspline] reject null trajectory message");
      }
      else
      {
        RCLCPP_WARN(rclcpp::get_logger("EGOReplanFSM"),
                    "[BroadcastBspline] reject null trajectory message");
      }
      return;
    }
    const auto validation = swarm_validation::validateCubicBspline(*msg);
    if (!validation.valid())
    {
      RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger("EGOReplanFSM"), *node_->get_clock(), 2000,
          "[BroadcastBspline] reject invalid trajectory: %s",
          swarm_validation::validationErrorName(validation.error));
      return;
    }

    const int id_value = msg->drone_id;
    if (id_value == planner_manager_->pp_.drone_id)
      return;
    const size_t id = static_cast<size_t>(id_value);

    const rclcpp::Time msg_start(
        msg->start_time, node_->get_clock()->get_clock_type());
    const double time_difference =
        (node_->get_clock()->now() - msg_start).seconds();
    if (!std::isfinite(time_difference) || std::abs(time_difference) > 0.25)
    {
      // ROS_ERROR("Time difference is too large! Local - Remote Agent %d = %fs",
      //           msg->drone_id, (node_->get_clock()->now() - msg->start_time).seconds());
      RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger("EGOReplanFSM"), *node_->get_clock(), 2000,
          "[BroadcastBspline] time difference is %.3fs for agent %d",
          time_difference, msg->drone_id);

      return;
    }

    // The distance gate below uses the first three control points.  Keep this
    // implementation-specific requirement before any swarm-buffer mutation;
    // the cubic validator already requires four control points for every
    // actual EGO consumer.
    if (msg->pos_pts.size() < 3)
    {
      RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger("EGOReplanFSM"), *node_->get_clock(), 2000,
          "[BroadcastBspline] pos_pts size < 3, ignoring");
      return;
    }

    if (!have_odom_ || !swarm_validation::validPlanningHorizon(planning_horizen_))
    {
      RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger("EGOReplanFSM"), *node_->get_clock(), 2000,
          "[BroadcastBspline] reject candidate before distance gate: odom/plan horizon unavailable");
      return;
    }

    /* Test distance to the agent */
    Eigen::Vector3d cp0(msg->pos_pts[0].x, msg->pos_pts[0].y, msg->pos_pts[0].z);
    Eigen::Vector3d cp1(msg->pos_pts[1].x, msg->pos_pts[1].y, msg->pos_pts[1].z);
    Eigen::Vector3d cp2(msg->pos_pts[2].x, msg->pos_pts[2].y, msg->pos_pts[2].z);
    Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
    if ((swarm_start_pt - odom_pos_).norm() > planning_horizen_ * 4.0f / 3.0f)
    {
      return; // if the current drone is too far to the received agent.
    }

    // Build and fully validate a local candidate before touching the shared
    // buffer.  A stale slot must survive every rejection, including the
    // distance gate and runtime B-spline sampling checks.
    Eigen::MatrixXd pos_pts(
        3, static_cast<Eigen::Index>(msg->pos_pts.size()));
    Eigen::VectorXd knots(static_cast<Eigen::Index>(msg->knots.size()));
    for (size_t j = 0; j < msg->knots.size(); ++j)
    {
      knots(j) = msg->knots[j];
    }
    for (size_t j = 0; j < msg->pos_pts.size(); ++j)
    {
      pos_pts(0, j) = msg->pos_pts[j].x;
      pos_pts(1, j) = msg->pos_pts[j].y;
      pos_pts(2, j) = msg->pos_pts[j].z;
    }

    UniformBspline candidate_position(pos_pts, msg->order, 0.1);
    candidate_position.setKnot(knots);
    UniformBspline candidate_velocity = candidate_position.getDerivative();
    UniformBspline candidate_acceleration = candidate_velocity.getDerivative();
    const auto runtime_validation = swarm_validation::validateCubicRuntimeTrajectory(
        candidate_position, candidate_velocity, candidate_acceleration,
        msg->pos_pts.size(), msg->knots.size(), validation.duration);
    if (!runtime_validation.valid())
    {
      RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger("EGOReplanFSM"), *node_->get_clock(), 2000,
          "[BroadcastBspline] reject runtime candidate: %s",
          swarm_validation::runtimeValidationErrorName(runtime_validation.error));
      return;
    }

    OneTrajDataOfSwarm candidate;
    candidate.drone_id = id_value;
    candidate.duration_ = validation.duration;
    candidate.position_traj_ = candidate_position;
    candidate.start_pos_ = candidate.position_traj_.evaluateDeBoorT(0);
    candidate.start_time_ = rclcpp::Time(
        msg->start_time, node_->get_clock()->get_clock_type());

    const bool committed = swarm_validation::commitSwarmCandidate(
        planner_manager_->swarm_trajs_buf_, id, true, candidate,
        []() {
          OneTrajDataOfSwarm blank;
          blank.drone_id = -1;
          return blank;
        });
    if (!committed)
    {
      RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger("EGOReplanFSM"), *node_->get_clock(), 2000,
          "[BroadcastBspline] candidate commit rejected");
      return;
    }

    /* Check Collision */
    if (planner_manager_->checkCollision(id))
    {
      if (!emergency_stop_gate_.canTransitionToReplan(
          exec_state_ == EMERGENCY_STOP))
        return;
      changeFSMExecState(REPLAN_TRAJ, "TRAJ_CHECK");
    }
  }

  void EGOReplanFSM::swarmTrajsCallback(const traj_utils::msg::MultiBsplines::SharedPtr msg)
  {
    if (!msg)
    {
      if (node_)
      {
        RCLCPP_WARN_THROTTLE(
            rclcpp::get_logger("EGOReplanFSM"), *node_->get_clock(), 2000,
            "[swarmTrajs] reject null message");
      }
      else
      {
        RCLCPP_WARN(rclcpp::get_logger("EGOReplanFSM"),
                    "[swarmTrajs] reject null message");
      }
      return;
    }
    // Validate the complete message before copying it, resizing the shared
    // buffer, converting ids to size_t, or indexing any trajectory.
    const auto validation = swarm_validation::validateMultiBsplines(*msg);
    if (!validation.valid())
    {
      RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger("EGOReplanFSM"), *node_->get_clock(), 2000,
          "[swarmTrajs] reject invalid message: %s",
          swarm_validation::validationErrorName(validation.error));
      return;
    }

    // cout << "\033[45;33mmulti_bspline_msgs_buf.drone_id_from=" << multi_bspline_msgs_buf_.drone_id_from << " multi_bspline_msgs_buf_.traj.size()=" << multi_bspline_msgs_buf_.traj.size() << "\033[0m" << endl;

    // This must precede any expected-count conversion, candidate allocation,
    // message snapshot update, or have_recv_pre_agent_ mutation.  A malformed
    // horizon therefore leaves the previous swarm state intact.
    if (!swarm_validation::validPlanningHorizon(planning_horizen_))
    {
      RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger("EGOReplanFSM"), *node_->get_clock(), 2000,
          "[swarmTrajs] reject invalid planning horizon %.3f", planning_horizen_);
      return;
    }

    if (!have_odom_)
    {
      // ROS_ERROR("swarmTrajsCallback(): no odom!, return.");
      RCLCPP_ERROR(rclcpp::get_logger("EGOReplanFSM"), "swarmTrajsCallback(): no odom!, return.");
      return;
    }

    const std::size_t expected_trajectory_count =
        static_cast<std::size_t>(msg->drone_id_from) + 1;
    if (msg->traj.size() != expected_trajectory_count) // drone_id must start from 0
    {
      // ROS_ERROR("Wrong trajectory size! msg->traj.size()=%d, msg->drone_id_from+1=%d", (int)msg->traj.size(), msg->drone_id_from + 1);
      RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger("EGOReplanFSM"), *node_->get_clock(), 2000,
          "[swarmTrajs] wrong trajectory size: got %zu expected %zu",
          msg->traj.size(), expected_trajectory_count);

      return;
    }

    for (const auto & trajectory : msg->traj)
    {
      if (trajectory.order != 3) // only support B-spline order equals 3.
      {
        RCLCPP_WARN_THROTTLE(
            rclcpp::get_logger("EGOReplanFSM"), *node_->get_clock(), 2000,
            "[swarmTrajs] only order 3 is supported, got %d",
            trajectory.order);
        return;
      }
    }

    // Build the complete replacement off to the side.  The existing swarm
    // buffer and message snapshot remain untouched if any candidate fails a
    // runtime structure, derivative, or sampling check.
    SwarmTrajData candidate_buffer(msg->traj.size());
    for (auto & candidate : candidate_buffer)
    {
      candidate.drone_id = -1;
    }

    for (size_t i = 0; i < msg->traj.size(); i++)
    {
      const auto trajectory_validation =
          swarm_validation::validateCubicBspline(msg->traj[i]);
      if (!trajectory_validation.valid())
      {
        RCLCPP_WARN_THROTTLE(
            rclcpp::get_logger("EGOReplanFSM"), *node_->get_clock(), 2000,
            "[swarmTrajs] reject candidate %zu: %s", i,
            swarm_validation::validationErrorName(trajectory_validation.error));
        return;
      }
      Eigen::Vector3d cp0(msg->traj[i].pos_pts[0].x, msg->traj[i].pos_pts[0].y, msg->traj[i].pos_pts[0].z);
      Eigen::Vector3d cp1(msg->traj[i].pos_pts[1].x, msg->traj[i].pos_pts[1].y, msg->traj[i].pos_pts[1].z);
      Eigen::Vector3d cp2(msg->traj[i].pos_pts[2].x, msg->traj[i].pos_pts[2].y, msg->traj[i].pos_pts[2].z);
      Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
      if ((swarm_start_pt - odom_pos_).norm() > planning_horizen_ * 4.0f / 3.0f)
      {
        continue;
      }

      Eigen::MatrixXd pos_pts(
          3, static_cast<Eigen::Index>(msg->traj[i].pos_pts.size()));
      Eigen::VectorXd knots(
          static_cast<Eigen::Index>(msg->traj[i].knots.size()));
      for (size_t j = 0; j < msg->traj[i].knots.size(); ++j)
      {
        knots(j) = msg->traj[i].knots[j];
      }
      for (size_t j = 0; j < msg->traj[i].pos_pts.size(); ++j)
      {
        pos_pts(0, j) = msg->traj[i].pos_pts[j].x;
        pos_pts(1, j) = msg->traj[i].pos_pts[j].y;
        pos_pts(2, j) = msg->traj[i].pos_pts[j].z;
      }

      UniformBspline candidate_position(pos_pts, msg->traj[i].order, 0.1);
      candidate_position.setKnot(knots);
      UniformBspline candidate_velocity = candidate_position.getDerivative();
      UniformBspline candidate_acceleration = candidate_velocity.getDerivative();
      const auto runtime_validation =
          swarm_validation::validateCubicRuntimeTrajectory(
              candidate_position, candidate_velocity, candidate_acceleration,
              msg->traj[i].pos_pts.size(), msg->traj[i].knots.size(),
              trajectory_validation.duration);
      if (!runtime_validation.valid())
      {
        RCLCPP_WARN_THROTTLE(
            rclcpp::get_logger("EGOReplanFSM"), *node_->get_clock(), 2000,
            "[swarmTrajs] reject runtime candidate %zu: %s", i,
            swarm_validation::runtimeValidationErrorName(runtime_validation.error));
        return;
      }

      candidate_buffer[i].drone_id = static_cast<int>(i);
      candidate_buffer[i].duration_ = trajectory_validation.duration;
      candidate_buffer[i].position_traj_ = candidate_position;
      candidate_buffer[i].start_pos_ = candidate_buffer[i].position_traj_.evaluateDeBoorT(0);
      candidate_buffer[i].start_time_ = rclcpp::Time(
          msg->traj[i].start_time, node_->get_clock()->get_clock_type());
    }

    planner_manager_->swarm_trajs_buf_ = std::move(candidate_buffer);
    multi_bspline_msgs_buf_ = *msg;
    have_recv_pre_agent_ = true;
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    int pre_s = int(exec_state_);
    if (new_state == GEN_NEW_TRAJ && exec_state_ != GEN_NEW_TRAJ)
    {
      // A new target or safety recovery gets one immediate planning attempt.
      // Subsequent failures are exponentially backed off in the FSM case.
      gen_failure_count_ = 0;
      gen_retry_not_before_ = std::chrono::steady_clock::now();
    }
    if (new_state == REPLAN_TRAJ && exec_state_ != REPLAN_TRAJ)
    {
      // A collision-triggered replan gets one immediate attempt.  A failed
      // attempt re-enters the same state below and keeps its backoff; resetting
      // only on a genuine state transition prevents a 100 Hz retry storm.
      replan_failure_count_ = 0;
      replan_retry_not_before_ = std::chrono::steady_clock::now();
    }
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void EGOReplanFSM::execFSMCallback()
  {
    if (navigation_managed_ && navigation_session_id_.empty()) return;
    exec_timer_->cancel(); // To avoid blockage

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!have_target_)
        cout << "wait for goal or trigger." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        goto force_return;
        // return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_ || !have_trigger_ || !hasFreshTrustedCloud())
        goto force_return;
      // return;
      else
      {
        // if ( planner_manager_->pp_.drone_id <= 0 )
        // {
        //   changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        // }
        // else
        // {
        changeFSMExecState(SEQUENTIAL_START, "FSM");
        // }
      }
      break;
    }

    case SEQUENTIAL_START: // for swarm
    {
      // cout << "id=" << planner_manager_->pp_.drone_id << " have_recv_pre_agent_=" << have_recv_pre_agent_ << endl;
      if (planner_manager_->pp_.drone_id <= 0 || (planner_manager_->pp_.drone_id >= 1 && have_recv_pre_agent_))
      {
        if (have_odom_ && have_target_ && have_trigger_)
        {
          bool success = planFromGlobalTraj(10); // zx-todo
          if (success)
          {
            if (exec_state_ == SEQUENTIAL_START)
            {
              changeFSMExecState(EXEC_TRAJ, "FSM");
              publishSwarmTrajs(true);
            }
          }
          else
          {
            // ROS_ERROR("Failed to generate the first trajectory!!!");
            RCLCPP_ERROR(rclcpp::get_logger("EGOReplanFSM"), "Failed to generate the first trajectory!!!");

            if (exec_state_ == SEQUENTIAL_START)
              enterEmergencyStop("FIRST_TRAJ_FAILED");
          }
        }
        else
        {
          // ROS_ERROR("No odom or no target! have_odom_=%d, have_target_=%d", have_odom_, have_target_);
          RCLCPP_ERROR(rclcpp::get_logger("EGOReplanFSM"), "No odom or no target! have_odom_=%d, have_target_=%d", have_odom_, have_target_);

        }
      }

      break;
    }

    case GEN_NEW_TRAJ:
    {

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      // Terminal points can remain occupied for several map updates.  One
      // bounded planning attempt plus monotonic backoff keeps the executor
      // responsive and prevents a 100 Hz retry/log storm.
      if (std::chrono::steady_clock::now() < gen_retry_not_before_)
        break;

      bool success = planFromGlobalTraj(1);
      if (success)
      {
        gen_failure_count_ = 0;
        if (exec_state_ == GEN_NEW_TRAJ)
        {
          changeFSMExecState(EXEC_TRAJ, "FSM");
          publishSwarmTrajs(false);
        }
      }
      else
      {
        if (exec_state_ == GEN_NEW_TRAJ)
        {
          ++gen_failure_count_;
          // Initial generation is deadline-sensitive: retain the first
          // 250 ms retry, then cap this state at 500 ms (about two retries/s).
          // REPLAN_TRAJ keeps its independent 2 s ceiling below.
          const int shift = std::min(gen_failure_count_ - 1, 1);
          gen_retry_not_before_ = std::chrono::steady_clock::now() +
            std::chrono::milliseconds(250 * (1 << shift));
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      // The collision callback/FSM timer can enter this state at 100 Hz while
      // the map still reports an occupied terminal point.  Use steady_clock,
      // not ROS time, and keep the same bounded exponential schedule as the
      // initial-plan path.  EMERGENCY_STOP is never scheduled here: safety
      // transitions are checked after planning and own their state.
      if (std::chrono::steady_clock::now() < replan_retry_not_before_)
        break;

      if (planFromCurrentTraj(1))
      {
        replan_failure_count_ = 0;
        if (exec_state_ == REPLAN_TRAJ)
        {
          changeFSMExecState(EXEC_TRAJ, "FSM");
          publishSwarmTrajs(false);
        }
      }
      else
      {
        if (exec_state_ == REPLAN_TRAJ)
        {
          ++replan_failure_count_;
          const int shift = std::min(replan_failure_count_ - 1, 3);
          replan_retry_not_before_ = std::chrono::steady_clock::now() +
            std::chrono::milliseconds(250 * (1 << shift));
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      rclcpp::Time time_now = node_->get_clock()->now();
      double t_cur = (time_now - info->start_time_).seconds();
      if (!std::isfinite(t_cur) || !std::isfinite(info->duration_) ||
          info->duration_ <= 0.0 || info->duration_ > 300.0) {
        RCLCPP_ERROR(rclcpp::get_logger("EGOReplanFSM"),
                     "执行轨迹时间状态无效，回到等待目标");
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "INVALID_TIME");
        goto force_return;
      }
      t_cur = std::min(std::max(t_cur, 0.0), info->duration_);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      /* && (end_pt_ - pos).norm() < 0.5 */
      if ((target_type_ == TARGET_TYPE::PRESET_TARGET) &&
          (wp_id_ < waypoint_num_ - 1) &&
          (end_pt_ - pos).norm() < no_replan_thresh_)
      {
        wp_id_++;
        planNextWaypoint(wps_[wp_id_]);
      }
      else if ((local_target_pt_ - end_pt_).norm() < 1e-3) // close to the global target
      {
        if (t_cur > info->duration_ - 1e-2)
        {
          have_target_ = false;
          have_trigger_ = false;

          if (target_type_ == TARGET_TYPE::PRESET_TARGET)
          {
            wp_id_ = 0;
            planNextWaypoint(wps_[wp_id_]);
          }

          changeFSMExecState(WAIT_TARGET, "FSM");
          goto force_return;
          // return;
        }
        else if ((end_pt_ - pos).norm() > no_replan_thresh_ && t_cur > replan_thresh_)
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }
      else if (t_cur > replan_thresh_)
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EMERGENCY_STOP:
    {
      // A local ROS publish only proves serialization.  The recovery gate
      // waits for a newer, source-verified traj_server ACK with the exact
      // stop trajectory ID; until then, retry the stop or stay latched.
      const double now = node_->get_clock()->now().seconds();
      if (emergency_recovery_gate_.stopPublicationDue(now))
      {
        emergency_stop_gate_.consume(true);
        const bool published = callEmergencyStop(odom_pos_);
        if (!emergency_recovery_gate_.markStopPublished(
            published, now, emergency_stop_traj_id_))
        {
          // Keep EMERGENCY_STOP and retry on the next bounded FSM tick.  No
          // low-velocity sample may count after a failed local publish or
          // while the downstream executor has not acknowledged the ID.
          emergency_stop_gate_.retry();
        }
      }
      else
      {
        const bool ack_source_alive = hasLiveTrajectoryAckSource();
        // Check executor liveness before the ODOM_DEPTH_TIMEOUT generation
        // gate.  Otherwise a latched timeout could bypass observe() forever,
        // leaving a stale one-shot ACK marked valid after traj_server died.
        if (emergency_recovery_gate_.downstreamAcknowledged() &&
          !ack_source_alive)
        {
          RCLCPP_WARN_THROTTLE(
            rclcpp::get_logger("EGOReplanFSM"), *node_->get_clock(), 2000,
            "急停恢复保持：traj_server ACK 来源已离线，等待新 stop attempt/ACK");
          emergency_recovery_gate_.observe(
            now, false, false, odom_vel_.norm(), false);
          break;
        }
        const bool fresh_odom = hasFreshOdom();
        const bool fresh_cloud = hasFreshTrustedCloud();
        if (!EmergencyRecoveryGate::canRecoverOdomDepthTimeout(
            odom_depth_timeout_latched_, fresh_odom, fresh_cloud,
            have_target_, emergency_target_generation_, target_generation_))
        {
          RCLCPP_WARN_THROTTLE(
            rclcpp::get_logger("EGOReplanFSM"), *node_->get_clock(), 2000,
            "ODOM_DEPTH_TIMEOUT 保持急停：等待新鲜 odom/可信点云和新目标 generation");
          break;
        }
        const bool stable = enable_fail_safe_ && emergency_recovery_gate_.observe(
          now, fresh_odom, fresh_cloud, odom_vel_.norm(),
          ack_source_alive);
        if (!stable) {
          RCLCPP_INFO_THROTTLE(
            rclcpp::get_logger("EGOReplanFSM"), *node_->get_clock(), 2000,
            "急停恢复等待：ack=%d source=%d odom=%d cloud=%d speed=%.3f "
            "stable_cycles=%d hold=%.2f",
            emergency_recovery_gate_.downstreamAcknowledged() ? 1 : 0,
            ack_source_alive ? 1 : 0, fresh_odom ? 1 : 0, fresh_cloud ? 1 : 0,
            odom_vel_.norm(), emergency_recovery_gate_.stableCyclesSeen(),
            emergency_recovery_gate_.params().hold_seconds);
        }
        if (stable)
        {
          const bool reached_unchanged_target =
            EmergencyRecoveryGate::shouldClearReachedTarget(
              emergency_target_generation_, target_generation_, have_target_,
              (end_pt_ - odom_pos_).norm(), no_replan_thresh_);
          if (!have_target_ || reached_unchanged_target)
          {
            // The stop trajectory has already brought the vehicle to the old
            // terminal goal.  Do not feed that completed goal back into
            // GEN_NEW_TRAJ: the resulting loop starves a newly submitted
            // manual goal and delays the next READY trajectory.  Keep the
            // manual trigger latch intact; waypointCallback() will install a
            // fresh target and move WAIT_TARGET to GEN_NEW_TRAJ.
            have_target_ = false;
            have_new_target_ = false;
            local_target_pt_ = odom_pos_;
            local_target_vel_.setZero();
            end_pt_ = odom_pos_;
            end_vel_.setZero();
            odom_depth_timeout_latched_ = false;
            emergency_recovery_gate_.reset();
            changeFSMExecState(WAIT_TARGET, "EMERGENCY_TARGET_REACHED");
          }
          else
          {
            // A changed generation denotes a newer manual target, including
            // a target close to the stopped vehicle.  Preserve it and resume
            // normal planning rather than clearing it as an old terminal
            // goal.
            odom_depth_timeout_latched_ = false;
            emergency_recovery_gate_.reset();
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          }
        }
      }

      break;
    }
    }

    data_disp_.header.stamp = node_->get_clock()->now();
    data_disp_pub_->publish(data_disp_);

  force_return:;
    exec_timer_->reset();
  }

  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/) //zx-todo
  {
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    bool flag_random_poly_init;
    if (timesOfConsecutiveStateCalls().first == 1)
      flag_random_poly_init = false;
    else
      flag_random_poly_init = true;

    for (int i = 0; i < trial_times; i++)
    {
      if (callReboundReplan(true, flag_random_poly_init))
      {
        return true;
      }
      if (exec_state_ == WAIT_TARGET || exec_state_ == EMERGENCY_STOP)
        return false;
    }
    return false;
  }

  bool EGOReplanFSM::planFromCurrentTraj(const int trial_times /*=1*/)
  {

    LocalTrajData *info = &planner_manager_->local_data_;
    rclcpp::Time time_now = node_->get_clock()->now();
    double t_cur = (time_now - info->start_time_).seconds();
    if (!std::isfinite(t_cur) || !std::isfinite(info->duration_) ||
        info->duration_ <= 0.0 || info->duration_ > 300.0) {
      RCLCPP_ERROR(rclcpp::get_logger("EGOReplanFSM"),
                   "重规划时间状态无效");
      return false;
    }
    t_cur = std::min(std::max(t_cur, 0.0), info->duration_);

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    bool success = callReboundReplan(false, false);

    if (!success)
    {
      if (exec_state_ == WAIT_TARGET || exec_state_ == EMERGENCY_STOP)
        return false;
      success = callReboundReplan(true, false);
      //changeFSMExecState(EXEC_TRAJ, "FSM");
      if (!success)
      {
        if (exec_state_ == WAIT_TARGET || exec_state_ == EMERGENCY_STOP)
          return false;
        for (int i = 0; i < trial_times; i++)
        {
          success = callReboundReplan(true, true);
          if (success)
            break;
          if (exec_state_ == WAIT_TARGET || exec_state_ == EMERGENCY_STOP)
            return false;
        }
        if (!success)
        {
          return false;
        }
      }
    }

    return true;
  }

  void EGOReplanFSM::checkCollisionCallback()
  {
    if (navigation_managed_ && navigation_session_id_.empty()) return;

    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    // EMERGENCY_STOP owns the safety trajectory and its cloud-gated recovery
    // in execFSMCallback().  Calling planFromCurrentTraj() synchronously from
    // this 20 Hz timer while already stopped can starve waypoint/odom
    // callbacks (and repeatedly re-enter a stale stop trajectory).  The r6
    // E4 failure showed exactly that: the new goal was accepted by flight
    // control, but the single-threaded EGO executor remained in a safety
    // planning call and never emitted a new trajectory.  Let the FSM timer
    // perform the one bounded transition/replan instead.
    if (exec_state_ == WAIT_TARGET || exec_state_ == EMERGENCY_STOP ||
        info->start_time_.seconds() < 1e-5)
      return;

    if (!hasFreshTrustedCloud()) {
      RCLCPP_ERROR_THROTTLE(
          rclcpp::get_logger("EGOReplanFSM"), *node_->get_clock(), 2000,
          "可信非地面点云已失效，停止当前 EGO 轨迹并等待新鲜点云");
      enterEmergencyStop("NO_TRUSTED_CLOUD");
      return;
    }

    /* ---------- check lost of depth ---------- */
    if (map->getOdomDepthTimeout())
    {
      // ROS_ERROR("Depth Lost! EMERGENCY_STOP");
      odom_depth_timeout_latched_ = true;
      RCLCPP_ERROR(
        rclcpp::get_logger("EGOReplanFSM"),
        "Depth Lost! EMERGENCY_STOP; recovery requires fresh sensors and a new target generation");
      enterEmergencyStop("ODOM_DEPTH_TIMEOUT");
      return;
    }

    // STOP deactivates the previous spline downstream, but local_data_ keeps
    // its ID/time for monotonic planning. A new START must not scan that old
    // session's inactive curve as if it were executing for the new goal.
    // Fresh sensors were checked above; new curves still undergo the full
    // planner collision check, and runtime checks resume after publication.
    if (!runtimeTrajectoryBelongsToSession(navigation_managed_,
        navigation_trajectory_session_id_, navigation_session_id_)) return;

    /* ---------- check trajectory ---------- */
    // Runtime safety is a rolling, bounded look-ahead.  It deliberately does
    // not permanently omit the terminal part: each callback advances the
    // window, while planning/publication performs a separate full-trajectory
    // validation.  The helper always includes scan_end exactly, including the
    // special case t_cur == duration.
    constexpr double kRuntimeLookaheadSeconds = 5.0;
    constexpr double kRuntimeSpatialResolution = 0.10;
    constexpr std::size_t kRuntimeMaxIntervals = 2048;
    // Keep the collision sample's time origin stable across a synchronous
    // replan.  planFromCurrentTraj() may replace local_data_ in place; using
    // info->start_time_ after it returns would mix the old sample time with a
    // new trajectory's clock origin.
    const rclcpp::Time collision_start_time = info->start_time_;
    const double collision_duration = info->duration_;
    double t_cur = std::numeric_limits<double>::quiet_NaN();
    try
    {
      t_cur = (node_->get_clock()->now() - collision_start_time).seconds();
    }
    catch (const std::exception &)
    {
      RCLCPP_ERROR(rclcpp::get_logger("EGOReplanFSM"), "碰撞检查 ROS 时钟域异常，进入急停");
      enterEmergencyStop("INVALID_TRAJECTORY_CLOCK");
      return;
    }
    if (!std::isfinite(t_cur) || t_cur < 0.0) {
      RCLCPP_ERROR(rclcpp::get_logger("EGOReplanFSM"),
                   "碰撞检查 ROS 轨迹时钟回退/无效，进入急停");
      enterEmergencyStop("INVALID_TRAJECTORY_CLOCK");
      return;
    }
    if (!collision_safety::validTrajectoryDuration(info->duration_)) {
      RCLCPP_ERROR(rclcpp::get_logger("EGOReplanFSM"),
                   "碰撞检查轨迹 duration=%.3f 无效，进入急停",
                   info->duration_);
      enterEmergencyStop("INVALID_TRAJECTORY_TIME");
      return;
    }
    t_cur = std::min(std::max(t_cur, 0.0), info->duration_);
    const double CLEARANCE = 1.0 * planner_manager_->getSwarmClearance();
    double t_cur_global = node_->get_clock()->now().seconds();
    const double scan_end = std::min(
        info->duration_, t_cur + kRuntimeLookaheadSeconds);
    const auto collision_samples = collision_sampling::boundedTrajectorySampleTimes(
        t_cur, scan_end, info->position_traj_.getMaxVelocityBound(),
        kRuntimeSpatialResolution, kRuntimeMaxIntervals);
    if (collision_samples.empty())
    {
      RCLCPP_ERROR(rclcpp::get_logger("EGOReplanFSM"),
                   "无效轨迹速度界限或碰撞采样预算，进入急停");
      enterEmergencyStop("INVALID_TRAJECTORY_SAMPLING");
      return;
    }
    const auto trajectoryTimeNow = [this, collision_start_time,
      collision_duration]() -> double {
      if (!node_ || collision_start_time.nanoseconds() <= 0 ||
          !collision_safety::validTrajectoryDuration(collision_duration))
      {
        return std::numeric_limits<double>::quiet_NaN();
      }
      double current = std::numeric_limits<double>::quiet_NaN();
      try
      {
        current = (node_->get_clock()->now() - collision_start_time).seconds();
      }
      catch (const std::exception &)
      {
        return std::numeric_limits<double>::quiet_NaN();
      }
      // A ROS-time rollback is not an old trajectory margin; it is an
      // untrusted clock sample and must not permit planning to continue.
      if (!std::isfinite(current) || current < 0.0)
      {
        return std::numeric_limits<double>::quiet_NaN();
      }
      return std::min(current, collision_duration);
    };
    for (const double t : collision_samples)
    {
      const Eigen::Vector3d sample_pos = info->position_traj_.evaluateDeBoorT(t);
      const int grid_occ = map->getInflateOccupancy(sample_pos);
      bool occ = grid_occ != 0;

      for (size_t id = 0; id < planner_manager_->swarm_trajs_buf_.size(); id++)
      {
        if ((planner_manager_->swarm_trajs_buf_.at(id).drone_id != (int)id) || (planner_manager_->swarm_trajs_buf_.at(id).drone_id == planner_manager_->pp_.drone_id))
        {
          continue;
        }

        double t_global = t_cur_global + (t - t_cur);
        double t_X = t_global - planner_manager_->swarm_trajs_buf_.at(id).start_time_.seconds();
        Eigen::Vector3d swarm_pridicted = planner_manager_->swarm_trajs_buf_.at(id).position_traj_.evaluateDeBoorT(t_X);
        Eigen::Vector3d p_t = info->position_traj_.evaluateDeBoorT(t);
        double dist = (p_t - swarm_pridicted).norm();

        if (dist < CLEARANCE)
        {
          occ = true;
          break;
        }
      }

      if (occ)
      {
        RCLCPP_WARN_THROTTLE(
            rclcpp::get_logger("EGOReplanFSM"), *node_->get_clock(), 1000,
            "collision sample: traj=%u t=%.3f pos=(%.2f,%.2f,%.2f) "
            "odom=(%.2f,%.2f,%.2f) grid_occ=%d trusted_cloud=%d",
            info->traj_id_, t, sample_pos.x(), sample_pos.y(), sample_pos.z(),
            odom_pos_.x(), odom_pos_.y(), odom_pos_.z(), grid_occ,
            hasFreshTrustedCloud() ? 1 : 0);

        // REPLAN_TRAJ is already owned by the FSM timer, which applies the
        // bounded steady-clock retry gate.  Do not let this 20 Hz collision
        // callback bypass that gate with a second synchronous planner call.
        // If the remaining collision horizon becomes imminent, safety still
        // wins immediately and enters the centralized emergency-stop path.
        const double collision_t_cur = trajectoryTimeNow();
        const double collision_remaining = t - collision_t_cur;
        if (collision_safety::emergencyWindowImminent(
              collision_remaining, emergency_time_))
        {
          RCLCPP_ERROR(
            rclcpp::get_logger("EGOReplanFSM"),
            "collision margin %.3f <= emergency_time %.3f，先急停而不规划",
            collision_remaining, emergency_time_);
          enterEmergencyStop(
            exec_state_ == REPLAN_TRAJ ? "REPLAN_COLLISION_IMMINENT" :
            "COLLISION_IMMINENT");
          return;
        }

        if (exec_state_ == REPLAN_TRAJ)
        {
          return;
        }

        const bool planned = planFromCurrentTraj(); // Make a chance
        // Planning may block long enough for the original margin to expire.
        // Never commit/publish EXEC_TRAJ using that stale pre-plan time.
        const double post_plan_t_cur = trajectoryTimeNow();
        const double post_plan_remaining = t - post_plan_t_cur;
        if (collision_safety::emergencyWindowImminent(
              post_plan_remaining, emergency_time_))
        {
          RCLCPP_ERROR(
            rclcpp::get_logger("EGOReplanFSM"),
            "规划返回后 collision margin %.3f <= emergency_time %.3f，急停",
            post_plan_remaining, emergency_time_);
          enterEmergencyStop("COLLISION_IMMINENT_AFTER_PLAN");
          return;
        }

        if (planned)
        {
          if (exec_state_ != EXEC_TRAJ ||
              !emergency_stop_gate_.canCommitExecution(exec_state_ == EMERGENCY_STOP))
            return;
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          publishSwarmTrajs(false);
          return;
        }
        else
        {
          // callReboundReplan() may have entered EMERGENCY_STOP (for example
          // after a freshness failure).  Never overwrite that safety state
          // with a later REPLAN/EXEC decision in this callback.
          if (emergency_stop_gate_.shouldReturnFromCollisionCallback(
              exec_state_ == EMERGENCY_STOP))
            return;
          if (collision_safety::emergencyWindowImminent(
                post_plan_remaining, emergency_time_))
          {
            // ROS_WARN("Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);
            RCLCPP_ERROR(
              rclcpp::get_logger("EGOReplanFSM"),
              "Suddenly discovered obstacles. emergency stop! time=%f",
              post_plan_remaining);
            enterEmergencyStop("COLLISION_PLAN_FAILED");
          }
          else
          {
            //ROS_WARN("current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    if (!hasFreshTrustedCloud()) {
      RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger("EGOReplanFSM"), *node_->get_clock(), 2000,
          "没有新鲜可信非地面点云，禁止生成/重规划轨迹");
      if (exec_state_ == EMERGENCY_STOP) {
        // Preserve an already-entered safety state.  The regular FSM timer
        // owns emergency-stop completion and cloud-gated recovery.
      } else if (exec_state_ == EXEC_TRAJ || exec_state_ == REPLAN_TRAJ) {
        enterEmergencyStop("NO_TRUSTED_CLOUD");
      } else {
        changeFSMExecState(WAIT_TARGET, "NO_TRUSTED_CLOUD");
      }
      return false;
    }

    getLocalTarget();

    bool plan_and_refine_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;

    cout << "refine_success=" << plan_and_refine_success << endl;

    if (plan_and_refine_success)
    {
      if (navigation_managed_) {
        // Refine can replace the spline after its first feasibility check.
        // Bound every derivative control point: the cubic derivative lies in
        // their convex hull, so this protects the complete curve, not samples.
        const auto velocity_points = planner_manager_->local_data_.position_traj_.getDerivative().getControlPoint();
        if (!velocity_points.allFinite() || velocity_points.cols() == 0) return false;
        for (Eigen::Index i = 0; i < velocity_points.cols(); ++i) {
          if (velocity_points.col(i).norm() > navigation_requested_speed_ + 1e-9) {
            RCLCPP_WARN(node_->get_logger(),
              "managed trajectory exceeds requested speed; reject publication: "
              "cp=%ld speed=%.9f requested=%.9f start_speed=%.9f",
              static_cast<long>(i), velocity_points.col(i).norm(),
              navigation_requested_speed_, start_vel_.norm());
            return false;
          }
        }
      }

      auto info = &planner_manager_->local_data_;

      traj_utils::msg::Bspline bspline;
      bspline.navigation_session_id = navigation_session_id_;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::msg::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      // cout << knots.transpose() << endl;
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      /* 1. publish traj to traj_server */
      bspline_pub_->publish(bspline);
      navigation_trajectory_session_id_ = navigation_session_id_;

      /* 2. publish traj to the next drone of swarm */

      /* 3. publish traj for visualization */
      visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
    }

    return plan_and_refine_success;
  }

  bool EGOReplanFSM::hasFreshTrustedCloud() const
  {
    return planner_manager_ && planner_manager_->grid_map_ &&
           planner_manager_->grid_map_->hasFreshTrustedCloud();
  }

  bool EGOReplanFSM::hasFreshOdom() const
  {
    if (!have_odom_ || !odom_sample_valid_ || !node_ ||
      odom_freshness_timeout_ <= 0.0 ||
      last_odom_stamp_.nanoseconds() <= 0 ||
      last_odom_receive_time_.nanoseconds() <= 0)
    {
      return false;
    }
    const rclcpp::Time now = node_->get_clock()->now();
    const double stamp_age = (now - last_odom_stamp_).seconds();
    const double receive_age = (now - last_odom_receive_time_).seconds();
    // Negative age is a rollback/future sample.  A large receive age means
    // the ROS clock jumped forward or the odom publisher stopped; both must
    // reset the recovery evidence and keep EMERGENCY_STOP latched.
    return std::isfinite(stamp_age) && std::isfinite(receive_age) &&
      stamp_age >= -0.2 && stamp_age <= odom_freshness_timeout_ &&
      receive_age >= 0.0 && receive_age <= odom_freshness_timeout_;
  }

  bool EGOReplanFSM::hasLiveTrajectoryAckSource() const
  {
    if (!node_ || !trajectory_ack_sub_ ||
      !emergency_recovery_gate_.downstreamAcknowledged()) {
      return false;
    }
    try {
      const auto endpoint_infos = node_->get_publishers_info_by_topic(
        trajectory_ack_sub_->get_topic_name());
      const rclcpp::TopicEndpointInfo * unique_publisher = nullptr;
      std::size_t publisher_count = 0;
      for (const auto & endpoint : endpoint_infos) {
        if (endpoint.endpoint_type() == rclcpp::EndpointType::Publisher) {
          ++publisher_count;
          unique_publisher = &endpoint;
        }
      }
      if (publisher_count != 1 || unique_publisher == nullptr ||
        unique_publisher->topic_type() != "traj_utils/msg/TrajectoryAck" ||
        !EmergencyRecoveryGate::endpointFqnMatches(
          unique_publisher->node_name(), unique_publisher->node_namespace(),
          traj_server_node_fqn_)) {
        return false;
      }
      std::string publisher_key;
      const auto & endpoint_gid = unique_publisher->endpoint_gid();
      publisher_key.reserve(endpoint_gid.size());
      for (const auto byte : endpoint_gid) {
        publisher_key.push_back(static_cast<char>(byte));
      }
      return emergency_recovery_gate_.acknowledgedPublisherMatches(publisher_key);
    }
    catch (const std::exception &) {
      return false;
    }
  }

  void EGOReplanFSM::publishSwarmTrajs(bool startup_pub)
  {
    auto info = &planner_manager_->local_data_;

    traj_utils::msg::Bspline bspline;
    bspline.navigation_session_id = navigation_session_id_;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.drone_id = planner_manager_->pp_.drone_id;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    // cout << knots.transpose() << endl;
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    if (startup_pub)
    {
      multi_bspline_msgs_buf_.drone_id_from = planner_manager_->pp_.drone_id; // zx-todo
      if ((int)multi_bspline_msgs_buf_.traj.size() == planner_manager_->pp_.drone_id + 1)
      {
        multi_bspline_msgs_buf_.traj.back() = bspline;
      }
      else if ((int)multi_bspline_msgs_buf_.traj.size() == planner_manager_->pp_.drone_id)
      {
        multi_bspline_msgs_buf_.traj.push_back(bspline);
      }
      else
      {
        // ROS_ERROR("Wrong traj nums and drone_id pair!!! traj.size()=%d, drone_id=%d", (int)multi_bspline_msgs_buf_.traj.size(), planner_manager_->pp_.drone_id);
        RCLCPP_ERROR(rclcpp::get_logger("EGOReplanFSM"), "Wrong traj nums and drone_id pair!!! traj.size()=%d, drone_id=%d", (int)multi_bspline_msgs_buf_.traj.size(), planner_manager_->pp_.drone_id);

        // return plan_and_refine_success;
      }
      swarm_trajs_pub_->publish(multi_bspline_msgs_buf_);
    }

    broadcast_bspline_pub_->publish(bspline);
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {
    if (!planner_manager_ || !bspline_pub_ || !stop_pos.allFinite())
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("EGOReplanFSM"),
        "拒绝发布急停轨迹: planner/publisher/stop position invalid");
      return false;
    }

    // Do not advance the planner's trajectory ID while traj_server is absent.
    // A publish with zero matched subscribers is not downstream execution and
    // must leave the FSM in EMERGENCY_STOP for a bounded retry.
    if (bspline_pub_->get_subscription_count() == 0)
    {
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("EGOReplanFSM"), *node_->get_clock(), 1000,
        "急停轨迹无 traj_server 订阅者，保持 EMERGENCY_STOP 等待下游");
      emergency_stop_traj_id_ = 0;
      return false;
    }

    bool stop_ok = false;
    try {
      stop_ok = planner_manager_->EmergencyStop(stop_pos);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        rclcpp::get_logger("EGOReplanFSM"),
        "急停轨迹构造异常: %s", e.what());
      return false;
    }
    if (!stop_ok)
    {
      RCLCPP_ERROR(rclcpp::get_logger("EGOReplanFSM"), "急停轨迹构造失败");
      return false;
    }

    auto info = &planner_manager_->local_data_;
    if (!std::isfinite(info->duration_) || info->duration_ <= 0.0 ||
      info->duration_ > 300.0 || info->position_traj_.get_control_points().cols() < 4)
    {
      RCLCPP_ERROR(rclcpp::get_logger("EGOReplanFSM"), "急停轨迹结构/时长无效");
      return false;
    }
    if (info->traj_id_ <= 0 ||
      static_cast<std::uint64_t>(info->traj_id_) >
        static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max()))
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("EGOReplanFSM"),
        "急停轨迹 ID 无效 (%d)，拒绝等待/伪造下游 ACK", info->traj_id_);
      emergency_stop_traj_id_ = 0;
      return false;
    }
    emergency_stop_traj_id_ = static_cast<std::uint32_t>(info->traj_id_);

    /* publish traj */
    traj_utils::msg::Bspline bspline;
    bspline.navigation_session_id = navigation_session_id_;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
      {
        RCLCPP_ERROR(rclcpp::get_logger("EGOReplanFSM"), "急停控制点包含非有限值");
        return false;
      }
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      if (!std::isfinite(knots(i)))
      {
        RCLCPP_ERROR(rclcpp::get_logger("EGOReplanFSM"), "急停节点包含非有限值");
        return false;
      }
      bspline.knots.push_back(knots(i));
    }

    try {
      bspline_pub_->publish(bspline);
      navigation_trajectory_session_id_ = navigation_session_id_;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        rclcpp::get_logger("EGOReplanFSM"),
        "急停轨迹发布异常: %s", e.what());
      return false;
    }

    // The dedicated traj_server ACK is consumed asynchronously by
    // trajectoryAckCallback().  Returning true here means only that local
    // ROS serialization succeeded; recovery remains gated on that ACK.
    return true;
  }

  void EGOReplanFSM::enterEmergencyStop(const char * reason)
  {
    const auto result = emergency_stop_gate_.request(exec_state_ == EMERGENCY_STOP);
    if (result == EmergencyStopGate::RequestResult::kAlreadyStopped)
      return;

    // Freeze the target identity at the safety transition.  A waypoint
    // callback accepted while stopped advances target_generation_; recovery
    // will then preserve that newer target instead of clearing it as though
    // it were the completed pre-stop goal.
    emergency_target_generation_ = target_generation_;
    // Activation-sequence replay protection is owned by the recovery gate;
    // it binds the first ACK after each stop publication to that attempt's
    // graph-verified publisher GID.  Do not synthesize a receiver-local
    // sequence: only traj_server's activation_sequence is authoritative.
    emergency_recovery_gate_.beginStop(emergency_target_generation_);

    // The FSM timer consumes this latch exactly once and publishes the stop
    // trajectory.  All safety entry points use this helper so a later branch
    // cannot accidentally return to EXEC_TRAJ or plan again in this callback.
    changeFSMExecState(EMERGENCY_STOP, reason);
    if (navigation_managed_ && !navigation_session_id_.empty()) {
      easyfly_msgs::msg::NavigationSession fault;
      fault.session_id = navigation_session_id_;
      fault.active = false;
      navigation_fault_pub_->publish(fault);
    }
  }

  void EGOReplanFSM::getLocalTarget()
  {
    const double global_duration = planner_manager_->global_data_.global_duration_;
    if (!std::isfinite(global_duration) || global_duration <= 0.0 ||
        global_duration > 300.0 || !std::isfinite(planner_manager_->pp_.max_vel_) ||
        planner_manager_->pp_.max_vel_ <= 0.0 ||
        !std::isfinite(planning_horizen_) || planning_horizen_ <= 0.0) {
      local_target_pt_ = end_pt_;
      local_target_vel_.setZero();
      return;
    }
    double t;

    double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_;
    if (!std::isfinite(t_step) || t_step <= 1e-4 || t_step > 10.0) {
      RCLCPP_ERROR(rclcpp::get_logger("EGOReplanFSM"),
                   "局部目标采样步长 %.3f 无效", t_step);
      local_target_pt_ = end_pt_;
      local_target_vel_.setZero();
      return;
    }
    double dist_min = 9999, dist_min_t = 0.0;
    for (t = planner_manager_->global_data_.last_progress_time_; t < global_duration; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();

      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizen_)
      {
        // Important cornor case!
        for (; t < global_duration; t += t_step)
        {
          Eigen::Vector3d pos_t_temp = planner_manager_->global_data_.getPosition(t);
          double dist_temp = (pos_t_temp - start_pt_).norm();
          if (dist_temp < planning_horizen_)
          {
            pos_t = pos_t_temp;
            dist = (pos_t - start_pt_).norm();
            cout << "Escape cornor case \"getLocalTarget\"" << endl;
            break;
          }
        }
      }

      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }

      if (dist >= planning_horizen_)
      {
        local_target_pt_ = pos_t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        break;
      }
    }
    if (t > global_duration) // Last global point
    {
      local_target_pt_ = end_pt_;
      planner_manager_->global_data_.last_progress_time_ = global_duration;
    }

    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(t);
    }
  }

} // namespace ego_planner
