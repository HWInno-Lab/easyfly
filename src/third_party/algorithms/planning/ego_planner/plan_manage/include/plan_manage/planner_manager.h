#ifndef PLAN_MANAGE__PLANNER_MANAGER_H_
#define PLAN_MANAGE__PLANNER_MANAGER_H_

#include <stdlib.h>
#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <plan_env/grid_map.h>
#include <plan_env/obj_predictor.h>
#include <traj_utils/planning_visualization.h>
#include <memory>
#include <vector>
#include <traj_utils_so/msg/data_disp.hpp>
#include <traj_utils/plan_container.hpp>
#include "rclcpp/rclcpp.hpp"

namespace ego_planner
{

// Fast Planner Manager
// Key algorithms of mapping and planning are called

class EGOPlannerManager
{
  // SECTION stable

public:
  EGOPlannerManager();
  ~EGOPlannerManager();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /* main planning interface */
  bool reboundReplan(
    Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
    Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit,
    bool flag_randomPolyTraj);
  bool EmergencyStop(Eigen::Vector3d stop_pos);
  bool planGlobalTraj(
    const Eigen::Vector3d & start_pos, const Eigen::Vector3d & start_vel,
    const Eigen::Vector3d & start_acc,
    const Eigen::Vector3d & end_pos, const Eigen::Vector3d & end_vel,
    const Eigen::Vector3d & end_acc);
  bool planGlobalTrajWaypoints(
    const Eigen::Vector3d & start_pos, const Eigen::Vector3d & start_vel,
    const Eigen::Vector3d & start_acc,
    const std::vector<Eigen::Vector3d> & waypoints, const Eigen::Vector3d & end_vel,
    const Eigen::Vector3d & end_acc);

  void initPlanModules(
    std::shared_ptr<rclcpp::Node> & nh,
    PlanningVisualization::Ptr vis = NULL);

  void deliverTrajToOptimizer(void) {bspline_optimizer_->setSwarmTrajs(&swarm_trajs_buf_);}

  void setDroneIdtoOpt(void) {bspline_optimizer_->setDroneId(pp_.drone_id);}
  void setNavigationVelocityLimits(double planner_velocity, double optimizer_velocity)
  {
    pp_.max_vel_ = planner_velocity;
    bspline_optimizer_->setNavigationMaxVelocity(optimizer_velocity);
  }
  void setNavigationFixedEndpoint(bool enabled)
  {
    navigation_fixed_endpoint_ = enabled;
    bspline_optimizer_->setNavigationFixedEndpoint(enabled);
  }

  double getSwarmClearance(void) {return bspline_optimizer_->getSwarmClearance();}

  bool checkCollision(int drone_id);


  PlanParameters pp_;
  LocalTrajData local_data_;
  GlobalTrajData global_data_;
  GridMap::Ptr grid_map_;
  fast_planner::ObjPredictor::Ptr obj_predictor_;
  SwarmTrajData swarm_trajs_buf_;
  // The planner publishes trajectory timestamps, so it must use the clock
  // attached to the owning node (including /clock when use_sim_time=true).
  rclcpp::Clock::SharedPtr clock_;

private:
  /* main planning algorithms & modules */
  PlanningVisualization::Ptr visualization_;

  // ros::Publisher obj_pub_; //zx-todo

  BsplineOptimizer::Ptr bspline_optimizer_;
  bool navigation_fixed_endpoint_ {false};
  Eigen::Vector3d navigation_endpoint_, navigation_endpoint_velocity_;

  int continous_failures_count_ {0};

  void updateTrajInfo(const UniformBspline & position_traj, const rclcpp::Time time_now);

  bool reparamBspline(
    UniformBspline & bspline, vector<Eigen::Vector3d> & start_end_derivative, double ratio,
    Eigen::MatrixXd & ctrl_pts, double & dt,
    double & time_inc);

  bool refineTrajAlgo(
    UniformBspline & traj, vector<Eigen::Vector3d> & start_end_derivative,
    double ratio, double & ts, Eigen::MatrixXd & optimal_control_points);

  // !SECTION stable

  // SECTION developing

public:
  typedef unique_ptr<EGOPlannerManager> Ptr;

  // !SECTION
};
}  // namespace ego_planner

#endif  // PLAN_MANAGE__PLANNER_MANAGER_H_
