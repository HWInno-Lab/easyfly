// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <plan_manage/fixed_endpoint.h>
#include <plan_manage/trajectory_seed_safety.h>
#include <bspline_opt/collision_sampling.h>
#include <chrono>
#include <cmath>
#include <limits>
#include <thread>
#include "visualization_msgs/msg/marker.hpp"  // zx-todo
#include "rclcpp/logging.hpp"
namespace ego_planner
{

// SECTION interfaces for setup and query

EGOPlannerManager::EGOPlannerManager() {}

EGOPlannerManager::~EGOPlannerManager() {}

void EGOPlannerManager::initPlanModules(
  std::shared_ptr<rclcpp::Node> & nh,
  PlanningVisualization::Ptr vis)
{
  clock_ = nh->get_clock();
  /* read algorithm parameters */
  nh->declare_parameter<double>("manager/max_vel", -1.0);
  nh->declare_parameter<double>("manager/max_acc", -1.0);
  nh->declare_parameter<double>("manager/max_jerk", -1.0);
  nh->declare_parameter<double>("manager/feasibility_tolerance", 0.0);
  nh->declare_parameter<double>("manager/control_points_distance", -1.0);
  nh->declare_parameter<double>("manager/planning_horizon", 5.0);
  nh->declare_parameter<bool>("manager/use_distinctive_trajs", false);
  nh->declare_parameter<int>("manager/drone_id", -1);
  nh->get_parameter("manager/max_vel", pp_.max_vel_);
  nh->get_parameter("manager/max_acc", pp_.max_acc_);
  nh->get_parameter("manager/max_jerk", pp_.max_jerk_);
  nh->get_parameter("manager/feasibility_tolerance", pp_.feasibility_tolerance_);
  nh->get_parameter("manager/control_points_distance", pp_.ctrl_pt_dist);
  nh->get_parameter("manager/planning_horizon", pp_.planning_horizen_);
  nh->get_parameter("manager/use_distinctive_trajs", pp_.use_distinctive_trajs);
  nh->get_parameter("manager/drone_id", pp_.drone_id);
  // nh.param("manager/max_vel", pp_.max_vel_, -1.0);
  // nh.param("manager/max_acc", pp_.max_acc_, -1.0);
  // nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
  // nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
  // nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);
  // nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);
  // nh.param("manager/use_distinctive_trajs", pp_.use_distinctive_trajs, false);
  // nh.param("manager/drone_id", pp_.drone_id, -1);

  local_data_.traj_id_ = 0;
  grid_map_.reset(new GridMap);
  grid_map_->initMap(nh);

  // obj_predictor_.reset(new fast_planner::ObjPredictor(nh));
  // obj_predictor_->init();
  // obj_pub_ = nh.advertise<visualization_msgs::msg::Marker>("/dynamic/obj_prdi", 10); // zx-todo

  bspline_optimizer_.reset(new BsplineOptimizer);
  bspline_optimizer_->setParam(nh);
  bspline_optimizer_->setEnvironment(grid_map_, obj_predictor_);
  bspline_optimizer_->a_star_.reset(new AStar);
  bspline_optimizer_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));

  visualization_ = vis;
}

// !SECTION

// SECTION rebond replanning

bool EGOPlannerManager::reboundReplan(
  Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
  Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
  Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
{
  static int count = 0;
  printf(
    "\033[47;30m\n[drone %d replan %d]==============================================\033[0m\n",
    pp_.drone_id, count++);
  // cout.precision(3);
  // cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" <<
  // local_target_pt.transpose() << ", " << local_target_vel.transpose()
  //      << endl;

  // Managed actions require 0.10 m arrival accuracy, including a short exact
  // endpoint approach after a checked grid-cell centre. Keep planning down to
  // half that radius; collision, endpoint and dynamic validation still run.
  // The legacy unmanaged close-goal policy stays unchanged.
  const double minimum_distance = navigation_fixed_endpoint_ ? 0.05 : 0.2;
  if ((start_pt - local_target_pt).norm() < minimum_distance) {
    cout << "Close to goal" << endl;
    continous_failures_count_++;
    return false;
  }

  bspline_optimizer_->setLocalTargetPt(local_target_pt);

  if (!clock_ || !start_pt.allFinite() || !start_vel.allFinite() ||
    !start_acc.allFinite() || !local_target_pt.allFinite() ||
    !local_target_vel.allFinite() || !std::isfinite(pp_.max_vel_) ||
    !std::isfinite(pp_.max_acc_) || !std::isfinite(pp_.ctrl_pt_dist) ||
    pp_.max_vel_ <= 0.0 || pp_.max_acc_ <= 0.0 || pp_.ctrl_pt_dist <= 0.0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("EGOPlannerManager"),
      "拒绝无效规划输入或动力学参数");
    ++continous_failures_count_;
    return false;
  }

  const auto steady_now = std::chrono::steady_clock::now;
  auto t_start = steady_now();
  double t_init = 0.0;
  double t_opt = 0.0;
  double t_refine = 0.0;

  /*** STEP 1: INIT ***/
  double ts = (start_pt - local_target_pt).norm() >
    // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
    0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.5 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5;
  if (!std::isfinite(ts) || ts <= 1e-4 || ts > 10.0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("EGOPlannerManager"),
      "B-spline 时间步长 %.6f 无效", ts);
    ++continous_failures_count_;
    return false;
  }
  constexpr double max_coordinate = 1000.0;
  constexpr std::size_t max_samples = 10000;
  vector<Eigen::Vector3d> point_set, start_end_derivatives;
  static bool flag_first_call = true, flag_force_polynomial = false;
  bool flag_regenerate = false;
  int regenerate_attempts = 0;
  do{
    if (++regenerate_attempts > 8) {
      RCLCPP_ERROR(
        rclcpp::get_logger("EGOPlannerManager"),
        "轨迹重生成次数超过上限，拒绝继续分配");
      ++continous_failures_count_;
      return false;
    }
    point_set.clear();
    start_end_derivatives.clear();
    flag_regenerate = false;

    if (flag_first_call || flag_polyInit ||
      // Initial path generated from a min-snap traj by order.
      flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/)
    {
      flag_first_call = false;
      flag_force_polynomial = false;

      PolynomialTraj gl_traj;

      double dist = (start_pt - local_target_pt).norm();
      double time =
        pow(
        pp_.max_vel_,
        2) / pp_.max_acc_ >
        dist ? sqrt(dist / pp_.max_acc_) : (dist -
        pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;
      if (!std::isfinite(time) || time <= 0.0 || time > 300.0) {
        RCLCPP_ERROR(
          rclcpp::get_logger("EGOPlannerManager"),
          "全局轨迹时长 %.3f 无效或超出 300s 上限", time);
        ++continous_failures_count_;
        return false;
      }

      if (!flag_randomPolyTraj) {
        gl_traj = PolynomialTraj::one_segment_traj_gen(
          start_pt, start_vel, start_acc,
          local_target_pt, local_target_vel,
          Eigen::Vector3d::Zero(), time);
      } else {
        Eigen::Vector3d horizen_dir =
          ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
        Eigen::Vector3d vertical_dir =
          ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
        Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
          // NOLINTNEXTLINE(runtime/threadsafe_fn): single-thread callback.
          ((static_cast<double>(rand())) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() *
          horizen_dir *
          0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
          // NOLINTNEXTLINE(runtime/threadsafe_fn): single-thread callback.
          ((static_cast<double>(rand())) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() *
          vertical_dir *
          0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);

        // The random polynomial seed is allowed to move sideways around a
        // blocking pillar, but it must not invent a new altitude corridor.
        // With the G8 2 m hard ceiling, the old vertical random offset can
        // put the midpoint above the ceiling.  The optimizer may then
        // classify the resulting control points as occupied; the runtime
        // diagnostic in check_collision_and_rebound distinguishes that
        // case from a real obstacle hit.
        const double seed_z_before_bound = random_inserted_pt.z();
        double hard_lower_bound = -std::numeric_limits<double>::infinity();
        double hard_upper_bound = std::numeric_limits<double>::infinity();
        double seed_lower_bound = -std::numeric_limits<double>::infinity();
        double seed_upper_bound = std::numeric_limits<double>::infinity();
        if (grid_map_) {
          const double resolution = grid_map_->getResolution();
          const double ground_height = grid_map_->getGroundHeight();
          const double virtual_ceiling = grid_map_->getVirtualCeilingHeight();
          Eigen::Vector3d map_origin;
          Eigen::Vector3d map_size;
          grid_map_->getRegion(map_origin, map_size);
          const double map_top = map_origin.z() + map_size.z();
          if (!std::isfinite(resolution) || resolution <= 0.0 ||
            !std::isfinite(ground_height) ||
            !std::isfinite(virtual_ceiling) || !map_origin.allFinite() ||
            !map_size.allFinite() || map_size.z() <= 0.0 ||
            !std::isfinite(map_top))
          {
            RCLCPP_ERROR(
              rclcpp::get_logger("EGOPlannerManager"),
              "拒绝使用无效地图高度约束生成随机初始种子");
            ++continous_failures_count_;
            return false;
          }
          if (!grid_map_->isInMap(start_pt) || !grid_map_->isInMap(local_target_pt) ||
            plan_env::virtualCeilingBlocks(start_pt.z(), virtual_ceiling) ||
            plan_env::virtualCeilingBlocks(local_target_pt.z(), virtual_ceiling))
          {
            RCLCPP_ERROR(
              rclcpp::get_logger("EGOPlannerManager"),
              "拒绝随机初始种子硬边界端点 (start=(%.3f,%.3f,%.3f),"
              "target=(%.3f,%.3f,%.3f),ceiling=%.3f)",
              start_pt.x(), start_pt.y(), start_pt.z(),
              local_target_pt.x(), local_target_pt.y(), local_target_pt.z(),
              virtual_ceiling);
            ++continous_failures_count_;
            return false;
          }
          // Keep endpoint validation on the actual map/corridor boundaries;
          // leave one voxel of margin only for the inserted seed midpoint.
          hard_lower_bound = map_origin.z();
          hard_upper_bound = map_top;
          if (virtual_ceiling > plan_env::kVirtualCeilingDisabledThreshold) {
            hard_upper_bound = std::min(hard_upper_bound, virtual_ceiling);
          }
          seed_lower_bound = hard_lower_bound + resolution;
          seed_upper_bound = hard_upper_bound - resolution;
          if (virtual_ceiling > plan_env::kVirtualCeilingDisabledThreshold) {
            // GridMap marks this exact voxel as the legacy ceiling layer.
            // Keep the random midpoint strictly below its lower edge; an
            // inclusive bound at 1.90 still maps to ceil_id 14 in G8.
            const double resolution_inv = 1.0 / resolution;
            const int ceil_id = static_cast<int>(std::floor(
                (virtual_ceiling - map_origin.z()) * resolution_inv)) - 1;
            const double ceil_voxel_lower =
              map_origin.z() + static_cast<double>(ceil_id) * resolution;
            if (!std::isfinite(ceil_voxel_lower)) {
              RCLCPP_ERROR(
                rclcpp::get_logger("EGOPlannerManager"),
                "拒绝使用无效虚拟天花板体素边界");
              ++continous_failures_count_;
              return false;
            }
            const double ceil_margin = std::min(1e-4, 0.5 * resolution);
            seed_upper_bound = std::nextafter(
              ceil_voxel_lower - ceil_margin,
              -std::numeric_limits<double>::infinity());
          }
        }

        double bounded_seed_z = 0.0;
        if (!trajectory_seed_safety::boundRandomMidpointAltitude(
            start_pt.z(), local_target_pt.z(), seed_z_before_bound,
            hard_lower_bound, hard_upper_bound,
            seed_lower_bound, seed_upper_bound, bounded_seed_z))
        {
          RCLCPP_ERROR(
            rclcpp::get_logger(
              "EGOPlannerManager"),
            "拒绝超出规划高度走廊的随机初始种子 (start_z=%.3f,target_z=%.3f,"
            "raw_z=%.3f,hard=[%.3f,%.3f],seed=[%.3f,%.3f])",
            start_pt.z(), local_target_pt.z(), seed_z_before_bound,
            hard_lower_bound, hard_upper_bound,
            seed_lower_bound, seed_upper_bound);
          ++continous_failures_count_;
          return false;
        }
        if (std::abs(bounded_seed_z - seed_z_before_bound) > 1e-9) {
          RCLCPP_WARN(
            rclcpp::get_logger("EGOPlannerManager"),
            "将随机初始种子高度限制在地图走廊 (raw_z=%.3f,bounded_z=%.3f)",
            seed_z_before_bound, bounded_seed_z);
        }
        random_inserted_pt.z() = bounded_seed_z;

        Eigen::MatrixXd pos(3, 3);
        pos.col(0) = start_pt;
        pos.col(1) = random_inserted_pt;
        pos.col(2) = local_target_pt;
        Eigen::VectorXd t(2);
        t(0) = t(1) = time / 2;
        gl_traj = PolynomialTraj::minSnapTraj(
          pos, start_vel, local_target_vel, start_acc,
          Eigen::Vector3d::Zero(), t);
      }

      double t = 0.0;
      // Fail closed if the first sampling interval check exits before sampling.
      bool flag_too_far = true;
      ts *= 1.5;  // ts will be divided by 1.5 in the next
      int sample_attempts = 0;
      do{
        if (++sample_attempts > 100) {break;}
        ts /= 1.5;
        if (!std::isfinite(ts) || ts <= 1e-4 || ts > 10.0) {break;}
        point_set.clear();
        flag_too_far = false;
        Eigen::Vector3d last_pt = gl_traj.evaluate(0);
        for (t = 0; t < time; t += ts) {
          Eigen::Vector3d pt = gl_traj.evaluate(t);
          if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5) {
            flag_too_far = true;
            break;
          }
          last_pt = pt;
          point_set.push_back(pt);
        }
        if (point_set.size() > max_samples) {
          RCLCPP_ERROR(
            rclcpp::get_logger("EGOPlannerManager"),
            "初始轨迹采样点数超过 %zu，拒绝继续分配", max_samples);
          ++continous_failures_count_;
          return false;
        }
        // To make sure the initial path has enough points.
      } while ((flag_too_far || point_set.size() < 7) && sample_attempts < 100);
      if (flag_too_far || point_set.size() < 7 || !std::isfinite(ts) || ts <= 1e-4) {
        RCLCPP_ERROR(
          rclcpp::get_logger("EGOPlannerManager"),
          "初始轨迹采样失败 (samples=%zu, ts=%.6f)", point_set.size(), ts);
        ++continous_failures_count_;
        return false;
      }
      t -= ts;
      start_end_derivatives.push_back(gl_traj.evaluateVel(0));
      start_end_derivatives.push_back(local_target_vel);
      start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
      start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
    } else {  // Initial path generated from previous trajectory.
      double t = 0.0;
      double t_cur = (clock_->now() - local_data_.start_time_).seconds();
      if (!std::isfinite(t_cur) || !std::isfinite(local_data_.duration_) ||
        local_data_.duration_ <= 0.0 || local_data_.duration_ > 300.0)
      {
        RCLCPP_ERROR(
          rclcpp::get_logger("EGOPlannerManager"),
          "当前轨迹时间状态无效 (t=%.3f,duration=%.3f)",
          t_cur, local_data_.duration_);
        ++continous_failures_count_;
        return false;
      }
      t_cur = std::min(std::max(t_cur, 0.0), local_data_.duration_);

      vector<double> pseudo_arc_length;
      vector<Eigen::Vector3d> segment_point;
      pseudo_arc_length.push_back(0.0);
      for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts) {
        segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
        if (segment_point.size() > max_samples) {
          RCLCPP_ERROR(
            rclcpp::get_logger("EGOPlannerManager"),
            "重规划采样点数超过 %zu，拒绝继续分配", max_samples);
          ++continous_failures_count_;
          return false;
        }
        if (t > t_cur) {
          pseudo_arc_length.push_back(
            (segment_point.back() -
            segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
        }
      }
      t -= ts;

      double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() /
        pp_.max_vel_ * 2;
      if (poly_time > ts) {
        PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(
          local_data_.position_traj_.evaluateDeBoorT(t),
          local_data_.velocity_traj_.evaluateDeBoorT(t),
          local_data_.acceleration_traj_.evaluateDeBoorT(t),
          local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

        for (t = ts; t < poly_time; t += ts) {
          if (!pseudo_arc_length.empty()) {
            segment_point.push_back(gl_traj.evaluate(t));
            if (segment_point.size() > max_samples) {
              RCLCPP_ERROR(
                rclcpp::get_logger("EGOPlannerManager"),
                "补接轨迹采样点数超过 %zu，拒绝继续分配", max_samples);
              ++continous_failures_count_;
              return false;
            }
            pseudo_arc_length.push_back(
              (segment_point.back() -
              segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          } else {
            // ROS_ERROR("pseudo_arc_length is empty, return!");
            RCLCPP_ERROR(
              rclcpp::get_logger(
                "EGOPlannerManager"), "pseudo_arc_length is empty, return!");

            continous_failures_count_++;
            return false;
          }
        }
      }

      double sample_length = 0;
      double cps_dist = pp_.ctrl_pt_dist * 1.5;  // cps_dist will be divided by 1.5 in the next
      size_t id = 0;
      do{
        cps_dist /= 1.5;
        point_set.clear();
        sample_length = 0;
        id = 0;
        while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back()) {
          if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1]) {
            point_set.push_back(
              (sample_length - pseudo_arc_length[id]) /
              (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
              (pseudo_arc_length[id + 1] - sample_length) /
              (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
            sample_length += cps_dist;
          } else {
            id++;
          }
          if (point_set.size() > max_samples) {
            RCLCPP_ERROR(
              rclcpp::get_logger("EGOPlannerManager"),
              "局部轨迹采样点数超过 %zu，拒绝继续分配", max_samples);
            ++continous_failures_count_;
            return false;
          }
        }
        point_set.push_back(local_target_pt);
        // If the start point is very close to end point, this will help
      } while (point_set.size() < 7);

      start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
      start_end_derivatives.push_back(local_target_vel);
      start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
      start_end_derivatives.push_back(Eigen::Vector3d::Zero());

      // The initial path is unnormally too long!
      if (point_set.size() > pp_.planning_horizen_ / pp_.ctrl_pt_dist * 3) {
        flag_force_polynomial = true;
        flag_regenerate = true;
      }
    }
  } while (flag_regenerate);

  // Keep the generated seed range observable.  In particular, this lets a
  // SITL log distinguish a hard-boundary seed from a later optimizer
  // control-point excursion instead of inferring the cause from the generic
  // "terminal point ... obstacle" warning.
  double seed_min_z = std::numeric_limits<double>::infinity();
  double seed_max_z = -std::numeric_limits<double>::infinity();
  for (const auto & seed_point : point_set) {
    if (!seed_point.allFinite()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("EGOPlannerManager"),
        "初始种子包含非有限点，拒绝规划");
      ++continous_failures_count_;
      return false;
    }
    seed_min_z = std::min(seed_min_z, seed_point.z());
    seed_max_z = std::max(seed_max_z, seed_point.z());
  }
  RCLCPP_INFO(
    rclcpp::get_logger("EGOPlannerManager"),
    "初始种子范围: points=%zu z=[%.3f,%.3f] local_target=(%.3f,%.3f,%.3f) random=%s",
    point_set.size(), seed_min_z, seed_max_z,
    local_target_pt.x(), local_target_pt.y(), local_target_pt.z(),
    flag_randomPolyTraj ? "true" : "false");

  Eigen::MatrixXd ctrl_pts, ctrl_pts_temp;
  if (point_set.size() < 7 || point_set.size() > max_samples ||
    !std::isfinite(ts) || ts <= 1e-4 || !std::isfinite(pp_.planning_horizen_) ||
    pp_.planning_horizen_ <= 0.0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("EGOPlannerManager"),
      "B-spline 参数化输入无效 (points=%zu, ts=%.6f)",
      point_set.size(), ts);
    ++continous_failures_count_;
    return false;
  }
  UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
  if (navigation_fixed_endpoint_) {
    navigation_endpoint_ = local_target_pt;
    navigation_endpoint_velocity_ = local_target_vel;
    if (!setFixedCubicEndpoint(ctrl_pts, ts, local_target_pt, local_target_vel)) {return false;}
  }
  if (ctrl_pts.cols() < 6 || !ctrl_pts.allFinite()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("EGOPlannerManager"),
      "B-spline 控制点矩阵无效");
    ++continous_failures_count_;
    return false;
  }

  vector<std::pair<int, int>> segments;
  segments = bspline_optimizer_->initControlPoints(ctrl_pts, true);

  t_init = std::chrono::duration<double>(steady_now() - t_start).count();
  t_start = steady_now();

  /*** STEP 2: OPTIMIZE ***/
  bool flag_step_1_success = false;
  vector<vector<Eigen::Vector3d>> vis_trajs;

  if (pp_.use_distinctive_trajs) {
    // cout << "enter" << endl;
    std::vector<ControlPoints> trajs = bspline_optimizer_->distinctiveTrajs(segments);
    cout << "\033[1;33m"
         << "multi-trajs=" << trajs.size() << "\033[1;0m" << endl;

    double final_cost, min_cost = 999999.0;
    for (int i = trajs.size() - 1; i >= 0; i--) {
      if (bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts_temp, final_cost, trajs[i], ts)) {
        cout << "traj " << trajs.size() - i << " success." << endl;

        flag_step_1_success = true;
        if (final_cost < min_cost) {
          min_cost = final_cost;
          ctrl_pts = ctrl_pts_temp;
        }

        // visualization
        point_set.clear();
        for (int j = 0; j < ctrl_pts_temp.cols(); j++) {
          point_set.push_back(ctrl_pts_temp.col(j));
        }
        vis_trajs.push_back(point_set);
      } else {
        cout << "traj " << trajs.size() - i << " failed." << endl;
      }
    }

    t_opt = std::chrono::duration<double>(steady_now() - t_start).count();

    // This visuallization will take up several milliseconds.
    visualization_->displayMultiInitPathList(vis_trajs, 0.2);
  } else {
    flag_step_1_success = bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
    t_opt = std::chrono::duration<double>(steady_now() - t_start).count();
    // static int vis_id = 0;
    visualization_->displayInitPathList(point_set, 0.2, 0);
  }

  cout << "plan_success=" << flag_step_1_success << endl;
  if (!flag_step_1_success) {
    visualization_->displayOptimalList(ctrl_pts, 0);
    continous_failures_count_++;
    return false;
  }

  t_start = steady_now();

  UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
  pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

  /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
  // Note: Only adjust time in single drone mode. But we still allow drone_0 to adjust its time
  // profile.
  if (pp_.drone_id <= 0) {
    double ratio;
    bool flag_step_2_success = true;
    // Refinement changes control points, not only time.  Its collision-free
    // output can violate dynamics again.  Managed navigation must recheck
    // the final candidate, otherwise ten identical first-plan attempts can
    // all succeed here and then fail the publication speed guard.
    const int max_refinements = navigation_fixed_endpoint_ ? 5 : 1;
    int refinement = 0;
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);
    while (!pos.checkFeasibility(ratio, false)) {
      if (navigation_fixed_endpoint_) {
        RCLCPP_INFO(
          rclcpp::get_logger("EGOPlannerManager"),
          "managed refine dynamics: attempt=%d ratio=%.9f speed_bound=%.9f "
          "axis_limit=%.9f interval=%.9f duration=%.9f",
          refinement, ratio, pos.getMaxVelocityBound(), pp_.max_vel_,
          ts, pos.getTimeSum());
      }
      if (refinement >= max_refinements || !std::isfinite(ratio) || ratio <= 1.0) {
        if (navigation_fixed_endpoint_) {
          RCLCPP_WARN(
            rclcpp::get_logger("EGOPlannerManager"),
            "managed refinement remains dynamically infeasible after %d attempts",
            refinement);
        }
        flag_step_2_success = false;
        break;
      }
      ++refinement;
      cout << "Need to reallocate time." << endl;

      Eigen::MatrixXd optimal_control_points;
      flag_step_2_success = refineTrajAlgo(
        pos, start_end_derivatives, ratio, ts,
        optimal_control_points);
      if (!flag_step_2_success) {break;}
      pos = UniformBspline(optimal_control_points, 3, ts);
      pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);
      // Preserve the legacy single-refinement policy outside managed mode.
      if (!navigation_fixed_endpoint_) {break;}
    }

    if (!flag_step_2_success) {
      RCLCPP_WARN(
        rclcpp::get_logger("EGOPlannerManager"),
        "trajectory refinement rejected (collision, invalid data or dynamics)");
      continous_failures_count_++;
      return false;
    }
  } else {
    static bool print_once = true;
    if (print_once) {
      print_once = false;
      // ROS_ERROR("IN SWARM MODE, REFINE DISABLED!");
      RCLCPP_ERROR(rclcpp::get_logger("EGOPlannerManager"), "IN SWARM MODE, REFINE DISABLED!");
    }
  }

  t_refine = std::chrono::duration<double>(steady_now() - t_start).count();

  // save planned results
  updateTrajInfo(pos, clock_->now());

  static double sum_time = 0;
  static int count_success = 0;
  sum_time += t_init + t_opt + t_refine;
  count_success++;
  cout << "total time:\033[42m" << (t_init + t_opt + t_refine) << "\033[0m,optimize:" <<
    (t_init + t_opt) << ",refine:" << t_refine << ",avg_time=" << sum_time / count_success <<
    endl;

  // success. YoY
  continous_failures_count_ = 0;
  return true;
}

bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
{
  Eigen::MatrixXd control_points(3, 6);
  for (int i = 0; i < 6; i++) {
    control_points.col(i) = stop_pos;
  }

  updateTrajInfo(UniformBspline(control_points, 3, 1.0), clock_->now());

  return true;
}

bool EGOPlannerManager::checkCollision(int drone_id)
{
  if (drone_id < 0 || static_cast<size_t>(drone_id) >= swarm_trajs_buf_.size() ||
    local_data_.start_time_.seconds() < 1e9)  // It means my first planning has not started
  {
    return false;
  }

  double my_traj_start_time = local_data_.start_time_.seconds();
  double other_traj_start_time = swarm_trajs_buf_[drone_id].start_time_.seconds();

  double t_start = max(my_traj_start_time, other_traj_start_time);
  double t_end = min(
    my_traj_start_time + local_data_.duration_,
    other_traj_start_time + swarm_trajs_buf_[drone_id].duration_);

  if (!std::isfinite(t_start) || !std::isfinite(t_end) || t_end < t_start ||
    !std::isfinite(local_data_.duration_) || local_data_.duration_ < 0.0 ||
    !std::isfinite(swarm_trajs_buf_[drone_id].duration_) ||
    swarm_trajs_buf_[drone_id].duration_ < 0.0)
  {
    // Invalid trajectory timing is unsafe to classify as collision-free.
    return true;
  }

  const double local_speed = local_data_.position_traj_.getMaxVelocityBound();
  const double other_speed = swarm_trajs_buf_[drone_id].position_traj_.getMaxVelocityBound();
  const double relative_speed = local_speed + other_speed;
  const double overlap_duration = t_end - t_start;
  const auto samples = collision_sampling::fullTrajectorySampleTimes(
    0.0, overlap_duration, relative_speed, 0.03);
  if (samples.empty()) {
    return true;
  }

  for (const double relative_t : samples) {
    const double t = t_start + relative_t;
    if ((local_data_.position_traj_.evaluateDeBoorT(t - my_traj_start_time) -
      swarm_trajs_buf_[drone_id].position_traj_.evaluateDeBoorT(
        t -
        other_traj_start_time)).norm() < bspline_optimizer_->getSwarmClearance())
    {
      return true;
    }
  }

  return false;
}

bool EGOPlannerManager::planGlobalTrajWaypoints(
  const Eigen::Vector3d & start_pos, const Eigen::Vector3d & start_vel,
  const Eigen::Vector3d & start_acc,
  const std::vector<Eigen::Vector3d> & waypoints, const Eigen::Vector3d & end_vel,
  const Eigen::Vector3d & end_acc)
{
  constexpr double max_coordinate = 1000.0;
  constexpr std::size_t max_intermediate_points = 10000;
  if (waypoints.empty() || !start_pos.allFinite() || !start_vel.allFinite() ||
    !start_acc.allFinite() || !end_vel.allFinite() || !end_acc.allFinite() ||
    !std::isfinite(pp_.max_vel_) || pp_.max_vel_ <= 0.0 ||
    start_pos.cwiseAbs().maxCoeff() > max_coordinate)
  {
    return false;
  }
  for (const auto & waypoint : waypoints) {
    if (!waypoint.allFinite() || waypoint.cwiseAbs().maxCoeff() > 1000.0) {return false;}
  }

  // generate global reference trajectory

  vector<Eigen::Vector3d> points;
  points.push_back(start_pos);

  for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++) {
    points.push_back(waypoints[wp_i]);
  }

  double total_len = 0;
  total_len += (start_pos - waypoints[0]).norm();
  for (size_t i = 0; i < waypoints.size() - 1; i++) {
    total_len += (waypoints[i + 1] - waypoints[i]).norm();
  }

  // insert intermediate points if too far
  vector<Eigen::Vector3d> inter_points;
  double dist_thresh = max(total_len / 8, 4.0);

  for (size_t i = 0; i < points.size() - 1; ++i) {
    inter_points.push_back(points.at(i));
    double dist = (points.at(i + 1) - points.at(i)).norm();

    if (dist > dist_thresh) {
      int id_num = floor(dist / dist_thresh) + 1;

      for (int j = 1; j < id_num; ++j) {
        Eigen::Vector3d inter_pt =
          points.at(i) * (1.0 - static_cast<double>(j) / id_num) + points.at(i + 1) *
          static_cast<double>(j) / id_num;
        inter_points.push_back(inter_pt);
      }
    }
    if (inter_points.size() > max_intermediate_points) {return false;}
  }

  inter_points.push_back(points.back());
  if (inter_points.size() < 2 || inter_points.size() > max_intermediate_points) {return false;}

  // for ( int i=0; i<inter_points.size(); i++ )
  // {
  //   cout << inter_points[i].transpose() << endl;
  // }

  // write position matrix
  int pt_num = inter_points.size();
  Eigen::MatrixXd pos(3, pt_num);
  for (int i = 0; i < pt_num; ++i) {
    pos.col(i) = inter_points[i];
  }

  Eigen::Vector3d zero(0, 0, 0);
  Eigen::VectorXd time(pt_num - 1);
  for (int i = 0; i < pt_num - 1; ++i) {
    time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
  }

  if (!time.allFinite() || time.minCoeff() <= 0.0 || time.sum() > 300.0) {return false;}

  time(0) *= 2.0;
  time(time.rows() - 1) *= 2.0;

  PolynomialTraj gl_traj;
  if (pos.cols() >= 3) {
    gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
  } else if (pos.cols() == 2) {
    gl_traj = PolynomialTraj::one_segment_traj_gen(
      start_pos, start_vel, start_acc, pos.col(
        1), end_vel, end_acc, time(0));
  } else {
    return false;
  }

  auto time_now = clock_->now();
  global_data_.setGlobalTraj(gl_traj, time_now);

  return true;
}

bool EGOPlannerManager::planGlobalTraj(
  const Eigen::Vector3d & start_pos, const Eigen::Vector3d & start_vel,
  const Eigen::Vector3d & start_acc,
  const Eigen::Vector3d & end_pos, const Eigen::Vector3d & end_vel, const Eigen::Vector3d & end_acc)
{
  constexpr double max_coordinate = 1000.0;
  constexpr std::size_t max_intermediate_points = 10000;
  if (!start_pos.allFinite() || !start_vel.allFinite() || !start_acc.allFinite() ||
    !end_pos.allFinite() || !end_vel.allFinite() || !end_acc.allFinite() ||
    !std::isfinite(pp_.max_vel_) || pp_.max_vel_ <= 0.0 ||
    start_pos.cwiseAbs().maxCoeff() > max_coordinate ||
    end_pos.cwiseAbs().maxCoeff() > max_coordinate)
  {
    return false;
  }

  // generate global reference trajectory

  vector<Eigen::Vector3d> points;
  points.push_back(start_pos);
  points.push_back(end_pos);

  // insert intermediate points if too far
  vector<Eigen::Vector3d> inter_points;
  const double dist_thresh = 4.0;

  for (size_t i = 0; i < points.size() - 1; ++i) {
    inter_points.push_back(points.at(i));
    double dist = (points.at(i + 1) - points.at(i)).norm();

    if (dist > dist_thresh) {
      int id_num = floor(dist / dist_thresh) + 1;

      for (int j = 1; j < id_num; ++j) {
        Eigen::Vector3d inter_pt =
          points.at(i) * (1.0 - static_cast<double>(j) / id_num) + points.at(i + 1) *
          static_cast<double>(j) / id_num;
        inter_points.push_back(inter_pt);
      }
    }
    if (inter_points.size() > max_intermediate_points) {return false;}
  }

  inter_points.push_back(points.back());
  if (inter_points.size() < 2 || inter_points.size() > max_intermediate_points) {return false;}

  // write position matrix
  int pt_num = inter_points.size();
  Eigen::MatrixXd pos(3, pt_num);
  for (int i = 0; i < pt_num; ++i) {
    pos.col(i) = inter_points[i];
  }

  Eigen::Vector3d zero(0, 0, 0);
  Eigen::VectorXd time(pt_num - 1);
  for (int i = 0; i < pt_num - 1; ++i) {
    time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
  }

  if (!time.allFinite() || time.minCoeff() <= 0.0 || time.sum() > 300.0) {return false;}

  time(0) *= 2.0;
  time(time.rows() - 1) *= 2.0;

  PolynomialTraj gl_traj;
  if (pos.cols() >= 3) {
    gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
  } else if (pos.cols() == 2) {
    gl_traj = PolynomialTraj::one_segment_traj_gen(
      start_pos, start_vel, start_acc, end_pos,
      end_vel, end_acc, time(0));
  } else {
    return false;
  }

  auto time_now = clock_->now();
  global_data_.setGlobalTraj(gl_traj, time_now);

  return true;
}

bool EGOPlannerManager::refineTrajAlgo(
  UniformBspline & traj,
  vector<Eigen::Vector3d> & start_end_derivative, double ratio,
  double & ts, Eigen::MatrixXd & optimal_control_points)
{
  double t_inc;

  Eigen::MatrixXd ctrl_pts;  // = traj.getControlPoint()

  // std::cout << "ratio: " << ratio << std::endl;
  if (!reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc)) {return false;}
  if (navigation_fixed_endpoint_ && !setFixedCubicEndpoint(
      ctrl_pts, ts, navigation_endpoint_, navigation_endpoint_velocity_)) {return false;}

  traj = UniformBspline(ctrl_pts, 3, ts);

  double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
  bspline_optimizer_->ref_pts_.clear();
  for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step) {
    bspline_optimizer_->ref_pts_.push_back(traj.evaluateDeBoorT(t));
  }

  bool success =
    bspline_optimizer_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

  return success;
}

void EGOPlannerManager::updateTrajInfo(
  const UniformBspline & position_traj,
  const rclcpp::Time time_now)
{
  const double duration = position_traj.getTimeSum();
  if (!time_now.nanoseconds() || !std::isfinite(duration) || duration <= 0.0 ||
    duration > 300.0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("EGOPlannerManager"),
      "拒绝无效轨迹时间戳或时长 (stamp=%ld,duration=%.3f)",
      time_now.nanoseconds(), duration);
    local_data_.duration_ = 0.0;
    return;
  }
  local_data_.start_time_ = time_now;
  local_data_.position_traj_ = position_traj;
  local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
  local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
  local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
  local_data_.duration_ = duration;
  local_data_.traj_id_ += 1;
}

bool EGOPlannerManager::reparamBspline(
  UniformBspline & bspline, vector<Eigen::Vector3d> & start_end_derivative, double ratio,
  Eigen::MatrixXd & ctrl_pts, double & dt, double & time_inc)
{
  double time_origin = bspline.getTimeSum();
  int seg_num = bspline.getControlPoint().cols() - 3;
  // double length = bspline.getLength(0.1);
  // int seg_num = ceil(length / pp_.ctrl_pt_dist);

  if (!std::isfinite(time_origin) || time_origin <= 0.0 || seg_num < 4 ||
    !std::isfinite(ratio) || ratio <= 1.0 ||
    !std::isfinite(time_origin * ratio) || time_origin * ratio > 300.0) {return false;}

  if (navigation_fixed_endpoint_) {
    // The legacy lengthenTime stretches only middle knot spans.  On a short
    // spline it changes much less than the requested feasibility ratio.
    // Sample the old geometry on a uniformly scaled full time domain, then
    // parameterize with the ORIGINAL boundary derivatives (do not silently
    // divide the measured start velocity).  Refinement checks collisions
    // again, and the caller rechecks dynamics on the resulting control points.
    dt = time_origin * ratio / static_cast<double>(seg_num);
    if (!std::isfinite(dt) || dt <= 1e-4 || dt > 10.0) {return false;}
    time_inc = time_origin * (ratio - 1.0);
    vector<Eigen::Vector3d> points;
    points.reserve(seg_num + 1);
    for (int i = 0; i <= seg_num; ++i) {
      points.push_back(
        bspline.evaluateDeBoorT(
          time_origin * static_cast<double>(i) / static_cast<double>(seg_num)));
    }
    UniformBspline::parameterizeToBspline(dt, points, start_end_derivative, ctrl_pts);
    return ctrl_pts.cols() >= 7 && ctrl_pts.allFinite();
  }

  bspline.lengthenTime(ratio);
  double duration = bspline.getTimeSum();
  dt = duration / static_cast<double>(seg_num);
  time_inc = duration - time_origin;

  vector<Eigen::Vector3d> point_set;
  for (double time = 0.0; time <= duration + 1e-4; time += dt) {
    point_set.push_back(bspline.evaluateDeBoorT(time));
  }
  UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  return ctrl_pts.cols() >= 7 && ctrl_pts.allFinite();
}

}  // namespace ego_planner
