// Copyright 2026 EasyFly Project
//
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

#include "easyfly_trajectory_optimizer/plan.hpp"
#include "easyfly_trajectory_optimizer/observation_grid.hpp"
#include <iostream>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <easyfly_msgs/msg/voxel_grid.hpp>
#include <easyfly_msgs/msg/uav_state.hpp>
#include <easyfly_msgs/msg/navigation_session.hpp>
#include <easyfly_msgs/msg/polynomial_trajectory.hpp>
#include <easyfly_msgs/srv/set_navigation_session.hpp>
#include <easyfly_msgs/srv/set_exploration_view.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <spawn.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <unistd.h>
#include <filesystem>
#include <fstream>
#include <thread>
extern char ** environ;
namespace et = easyfly::trajectory;
class GcopterNavigation : public rclcpp::Node
{
  using Session = easyfly_msgs::srv::SetNavigationSession;
  using Map = easyfly_msgs::msg::VoxelGrid;
  using State = easyfly_msgs::msg::UAVState;

public:
  GcopterNavigation()
  : Node("gcopter_navigation")
  {
    rcl_interfaces::msg::ParameterDescriptor d; d.read_only = true;
    floor_ = declare_parameter<double>("z_min", .5, d); ceiling_ = declare_parameter<double>(
      "z_max", 2.5, d);
    allow_idle_policy_epoch_ = declare_parameter<bool>("allow_idle_policy_map_epoch", false, d);
    if (!std::isfinite(floor_) || !std::isfinite(ceiling_) || floor_ < .3 || ceiling_ > 3.5 ||
      ceiling_ - floor_ < .5)
    {
      throw std::invalid_argument("GCOPTER flight bounds");
    }
    dynamic_map_ = declare_parameter<bool>("dynamic_map", false, d);
    native_fuel_ = declare_parameter<bool>("fuel_native_trajectory", false, d);
    if (native_fuel_ && (!allow_idle_policy_epoch_ || !dynamic_map_)) {
      throw std::invalid_argument("native FUEL requires the explicit observed policy-map profile");
    }
    auto directory = declare_parameter<std::string>("evidence_directory", "", d);
    if (directory.empty()) {directory = std::filesystem::temp_directory_path();}
    if (!std::filesystem::is_directory(directory)) {
      throw std::invalid_argument("evidence parent directory");
    }
    auto pattern = directory + "/gcopter-XXXXXX";
    std::vector<char> name(pattern.begin(), pattern.end()); name.push_back(0);
    char * created = mkdtemp(name.data()); if (!created) {
      throw std::runtime_error("private job directory");
    }
    directory_ = created;
    worker_exe_ = native_fuel_ ?
      ament_index_cpp::get_package_prefix("easyfly_fuel_exploration") +
      "/lib/easyfly_fuel_exploration/fuel_bspline_worker" :
      ament_index_cpp::get_package_prefix("easyfly_trajectory_optimizer") +
      "/lib/easyfly_trajectory_optimizer/gcopter_worker";
    RCLCPP_INFO(get_logger(), "GCOPTER jobs: %s", directory_.c_str());
    state_sub_ = create_subscription<State>(
      "/uav_state", 10, [this](State::ConstSharedPtr p) {
        state_ = p;state_received_ = et::Steady::now();
      });
    map_sub_ = create_subscription<Map>(
      "/planning/voxel_grid", rclcpp::QoS(1).reliable().transient_local(),
      [this](Map::ConstSharedPtr p) {
        // FUEL publishes a new policy UUID only for an explicitly admitted new
        // parent mission, after exact STOP/reap/release of the previous one.
        // Never rebase an active session, a same-UUID source reset, or a clock fault.
        if (allow_idle_policy_epoch_ && !clock_reset_ && session_.empty() &&
        worker_ == 0 && stopped_ && p->valid && !p->session_id.empty() &&
        p->session_id.size() <= 128 && p->session_id != map_session_)
        {
          map_.reset();map_session_ = p->session_id;generation_ = p->generation;
          reset_required_ = false;
        }
        if (!map_session_.empty() && (p->session_id != map_session_ || p->generation != generation_)) {
          reset_required_ = true;
        }
        if (map_ && map_->valid && p->valid && p->session_id == map_->session_id &&
        p->generation == map_->generation)
        {
          const auto source_ns = [](const auto & stamp) {
            return int64_t(stamp.sec) * 1000000000LL + stamp.nanosec;
          };
          const bool layout = p->size_x != map_->size_x || p->size_y != map_->size_y ||
          p->size_z != map_->size_z || !std::isfinite(p->resolution) ||
          std::abs(p->resolution - map_->resolution) > 1e-8;
          bool origin_changed = false, lattice_changed = false;
          if (!layout && p->resolution >= .05) {
            const double shifts[]{(p->origin.x - map_->origin.x) / p->resolution,
              (p->origin.y - map_->origin.y) / p->resolution,
              (p->origin.z - map_->origin.z) / p->resolution};
            for (double shift:shifts) {
              origin_changed = origin_changed || std::abs(shift) > 1e-8;
              lattice_changed = lattice_changed || !std::isfinite(shift) ||
              std::abs(shift - std::round(shift)) > 1e-8;
            }
          }
          if (layout || lattice_changed || (origin_changed && !dynamic_map_) ||
          p->revision < map_->revision ||
          source_ns(p->header.stamp) < source_ns(map_->header.stamp))
          {
            reset_required_ = true;
          }
        }
        if (map_session_.empty() && !p->session_id.empty()) {
          map_session_ = p->session_id;generation_ = p->generation;
        }
        map_ = p; map_received_ = et::Steady::now(); grid_revision_ = std::numeric_limits<uint64_t>::max();
      });
    commands_ = create_publisher<quadrotor_msgs::msg::PositionCommand>(
      "/position_cmd",
      rclcpp::SensorDataQoS());
    faults_ = create_publisher<easyfly_msgs::msg::NavigationSession>(
      "/planning/navigation_fault", rclcpp::QoS(
        1).reliable().transient_local());
    replans_ = create_publisher<easyfly_msgs::msg::NavigationSession>(
      "/planning/gcopter_replan", rclcpp::QoS(1).reliable().transient_local());
    sessions_ = create_publisher<easyfly_msgs::msg::NavigationSession>(
      "/planning/navigation_session_state", rclcpp::QoS(1).reliable().transient_local());
    polynomials_ = create_publisher<easyfly_msgs::msg::PolynomialTrajectory>(
      "/planning/gcopter_trajectory", rclcpp::QoS(1).reliable().transient_local());
    if (native_fuel_) {
      using View = easyfly_msgs::srv::SetExplorationView;
      fuel_view_service_ = create_service<View>(
        "/planning/fuel_viewpoint",
        [this](View::Request::SharedPtr req, View::Response::SharedPtr res) {
          const auto uuid = [](const std::string & id) {
            return id.size() == 32 && id.find_first_not_of("0123456789abcdef") == std::string::npos;
          };
          res->success = false;res->message = "invalid_or_owned_viewpoint";
          if (!uuid(req->mission_id) || !uuid(req->navigation_goal_id)) {return;}
          const bool same = fuel_view_ && req->mission_id == fuel_view_->mission_id &&
          req->navigation_goal_id == fuel_view_->navigation_goal_id;
          if (!req->active) {
            if ((!fuel_view_ || same) && session_.empty() && !worker_ && stopped_) {
              fuel_view_.reset();fuel_view_started_ = false;
              res->success = true;res->message = "viewpoint_released";
            }
            return;
          }
          if (fuel_view_) {
            res->success = same && req->target_x == fuel_view_->target_x &&
            req->target_y == fuel_view_->target_y && req->target_z == fuel_view_->target_z &&
            req->target_yaw == fuel_view_->target_yaw;
            if (res->success) {res->message = "same_viewpoint_already_bound";}
            return;
          }
          if (!session_.empty() || worker_ || !stopped_ || !ready() ||
          map_->session_id != req->mission_id || req->header.frame_id != "odom" ||
          !fresh(req->header.stamp) || !std::isfinite(req->target_x) ||
          !std::isfinite(req->target_y) || !std::isfinite(req->target_z) ||
          !std::isfinite(req->target_yaw) || std::abs(req->target_yaw) > M_PI ||
          req->target_z < floor_ || req->target_z > ceiling_) {return;}
          fuel_view_ = req;fuel_view_created_ = et::Steady::now();fuel_view_started_ = false;
          res->success = true;res->message = "viewpoint_bound_without_motion";
        });
    }
    service_ = create_service<Session>(
      "/planning/gcopter_session",
      [this](Session::Request::SharedPtr req, Session::Response::SharedPtr response) {
        response->success = false;
        if (!req->active) {
          if (!session_.empty() && req->session_id != session_) {
            response->message = "session_mismatch";return;
          }
          stopped_ = true; plan_.reset();
          if (!stopWorker()) {response->message = "worker_not_reaped";return;}
          if (!session_.empty()) {publishSession(false);}
          session_.clear(); response->success = true;response->message = "stopped_and_worker_reaped";return;
        }
        if (!session_.empty() || worker_ > 0 || reset_required_ || req->session_id.empty() ||
        req->session_id.size() > 128 ||
        !std::isfinite(req->target_x) || !std::isfinite(req->target_y) ||
        !std::isfinite(req->target_z) ||
        req->target_z < floor_ || req->target_z > ceiling_ || !std::isfinite(req->max_speed) ||
        req->max_speed < .1 || req->max_speed > 3. || !ready())
        {
          response->message = "invalid_session_or_inputs";return;
        }
        if (native_fuel_ && (!fuel_view_ ||
        req->session_id.rfind(fuel_view_->navigation_goal_id + ":", 0) != 0 ||
        map_->session_id != fuel_view_->mission_id ||
        (!fuel_view_started_ && age(fuel_view_created_) > 10.) ||
        std::abs(req->target_x - fuel_view_->target_x) > 1e-7 ||
        std::abs(req->target_y - fuel_view_->target_y) > 1e-7 ||
        std::abs(req->target_z - fuel_view_->target_z) > 1e-7))
        {
          response->message = "native_fuel_viewpoint_binding_required";return;
        }
        try {
          updateGrid();
          request_.grid = grid_;request_.start = position();request_.velocity = {state_->velocity_x, state_->velocity_y, state_->velocity_z};
          request_.goal = {req->target_x, req->target_y, req->target_z};request_.max_speed = req->max_speed;
          if (request_.velocity.norm() > .3 || !grid_.free(request_.start) ||
          !grid_.free(request_.goal)) {throw std::runtime_error("occupied_or_unknown_endpoint");}
          if (get_publishers_info_by_topic("/position_cmd").size() != 1) {
            throw std::runtime_error(
              "command_publisher_conflict");
          }
          session_ = req->session_id;stopped_ = false;failed_ = false;yaw_ = state_->yaw;state_stamp_ = state_->header.stamp;
          source_map_stamp_ = map_->header.stamp;source_map_revision_ = map_->revision;
          yaw_delta_ = native_fuel_ ? std::remainder(fuel_view_->target_yaw - yaw_, 2. * M_PI) : 0.;
          yaw_duration_ = std::max(
            std::abs(yaw_delta_) * 1.875 / .3,
            std::sqrt(std::abs(yaw_delta_) * 5.78 / .3));
          fuel_view_started_ = native_fuel_;
          startWorker();publishSession(
            true);response->success = true;response->message = "owned_worker_started";
        } catch (const std::exception & e) {
          stopWorker();session_.clear();stopped_ = true;response->message = e.what();
        }
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] {tick();});
  }
  ~GcopterNavigation() override
  {
    stopped_ = true;plan_.reset();
    if (!stopWorker() && worker_ > 0) {
      kill(worker_, SIGKILL); int status = 0;
      while (waitpid(worker_, &status, 0) < 0 && errno == EINTR) {} worker_ = 0;
    }
  }

private:
  double age(et::Steady::time_point stamp) const
  {
    return std::chrono::duration<double>(et::Steady::now() - stamp).count();
  }
  bool fresh(const builtin_interfaces::msg::Time & stamp) const
  {
    if (stamp.sec < 0 || stamp.nanosec >= 1000000000 || (stamp.sec == 0 && stamp.nanosec == 0)) {
      return false;
    }
    const double value = (now() - rclcpp::Time(stamp, get_clock()->get_clock_type())).seconds();
    return value >= -.2 && value <= 1.;
  }
  et::Vec position() const {return {state_->position_x, state_->position_y, state_->position_z};}
  bool ready() const
  {
    return !reset_required_ && state_ && map_ && map_->valid && map_->header.frame_id == "odom" &&
           !map_->session_id.empty() && map_->session_id.size() <= 128 &&
           fresh(map_->header.stamp) && age(map_received_) <= 1. &&
           get_publishers_info_by_topic("/planning/voxel_grid").size() == 1 &&
           state_->header.frame_id == "odom" && fresh(state_->header.stamp) &&
           age(state_received_) <= .5 &&
           state_->connected && state_->armed && state_->flight_mode == 3 && state_->pose_valid &&
           state_->velocity_valid && state_->attitude_valid &&
           get_publishers_info_by_topic(state_sub_->get_topic_name()).size() == 1 &&
           position().allFinite() && std::isfinite(state_->velocity_x) && std::isfinite(
      state_->velocity_y) &&
           std::isfinite(state_->velocity_z) && std::isfinite(state_->yaw);
  }
  void updateGrid()
  {
    if (grid_revision_ == map_->revision) {return;}
    grid_.size = {int(map_->size_x), int(map_->size_y), int(map_->size_z)};
    grid_.origin = {map_->origin.x, map_->origin.y, map_->origin.z};
    grid_.resolution = map_->resolution;grid_.data = map_->data;
    if (!grid_.valid() || map_->inflation_radius < .65 || map_->inflation_radius > 1. ||
      !std::isfinite(map_->inflation_radius))
    {
      throw std::runtime_error("invalid_voxel_layout");
    }
    if (dynamic_map_) {et::reserveUnknown(grid_, map_->inflation_radius);}
    // Apply flight bounds to whole cell volumes, not just their centers.
    for (int z = 0; z < grid_.size.z(); ++z) {
      if (grid_.origin.z() + z * grid_.resolution < floor_ - 1e-8 ||
        grid_.origin.z() + (z + 1) * grid_.resolution > ceiling_ + 1e-8)
      {
        for (int y = 0; y < grid_.size.y(); ++y) {
          for (int x = 0; x < grid_.size.x(); ++x) {
            grid_.data[grid_.index({x, y, z})] = 100;
          }
        }
      }
    }
    // Keep the complete vehicle/tracking envelope inside the mapped XY box.
    const auto corner = (grid_.origin + grid_.size.cast<double>() * grid_.resolution).eval();
    for (int y = 0; y < grid_.size.y(); ++y) {
      for (int x = 0; x < grid_.size.x(); ++x) {
        const double low_x = grid_.origin.x() + x * grid_.resolution,
          high_x = low_x + grid_.resolution;
        const double low_y = grid_.origin.y() + y * grid_.resolution,
          high_y = low_y + grid_.resolution;
        if (low_x < grid_.origin.x() + map_->inflation_radius ||
          high_x > corner.x() - map_->inflation_radius ||
          low_y < grid_.origin.y() + map_->inflation_radius ||
          high_y > corner.y() - map_->inflation_radius)
        {
          for (int z = 0; z < grid_.size.z(); ++z) {grid_.data[grid_.index({x, y, z})] = 100;}}
      }
    }
    grid_revision_ = map_->revision;
  }
  void publishSession(bool active)
  {
    easyfly_msgs::msg::NavigationSession msg;msg.session_id = session_;msg.active = active;
    sessions_->publish(msg);
  }
  void fail(const std::string & reason, bool retryable_map = false)
  {
    if (failed_ || session_.empty()) {return;}
    stopped_ = true;plan_.reset();failed_ = true;stopWorker();
    easyfly_msgs::msg::NavigationSession msg;msg.session_id = session_;msg.active = false;
    if (retryable_map) {replans_->publish(msg);} else {faults_->publish(msg);}
    RCLCPP_ERROR(get_logger(), "GCOPTER session stopped: %s", reason.c_str());
    std::ofstream(directory_ + "/failure-" + std::to_string(job_) + ".txt") << reason << '\n';
  }
  void startWorker()
  {
    ++job_; input_ = directory_ + "/job-" + std::to_string(job_) + ".request";
    output_ = directory_ + "/job-" + std::to_string(job_) + ".polynomial";
    et::writeRequest(input_, request_);
    const std::string log = directory_ + "/job-" + std::to_string(job_) + ".log";
    posix_spawn_file_actions_t actions;posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_addopen(
      &actions, STDOUT_FILENO,
      log.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0600);
    posix_spawn_file_actions_adddup2(&actions, STDOUT_FILENO, STDERR_FILENO);
    std::string yaw_minimum = std::to_string(2. * yaw_duration_);
    std::vector<char *> argv{worker_exe_.data(), input_.data(), output_.data()};
    if (native_fuel_) {argv.push_back(yaw_minimum.data());}
    argv.push_back(nullptr);
    const int code = posix_spawn(
      &worker_, worker_exe_.c_str(), &actions, nullptr,
      argv.data(), environ);
    posix_spawn_file_actions_destroy(&actions);
    if (code) {worker_ = 0;throw std::runtime_error("worker_spawn_failed");}
    worker_started_ = et::Steady::now();
    std::ofstream(directory_ + "/job-" + std::to_string(job_) + ".owner") << worker_ << '\n';
  }
  bool stopWorker()
  {
    if (worker_ <= 0) {return true;}
    kill(worker_, SIGTERM);
    const auto deadline = et::Steady::now() + std::chrono::milliseconds(250);
    do {
      int status = 0;const auto result = waitpid(worker_, &status, WNOHANG);
      if (result == worker_ || (result < 0 && errno == ECHILD)) {worker_ = 0;return true;}
      if (result < 0 && errno != EINTR) {return false;}
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while(et::Steady::now() < deadline);
    return false;
  }
  void announce()
  {
    easyfly_msgs::msg::PolynomialTrajectory message;message.header.frame_id = "odom";
    message.header.stamp = start_time_;
    message.navigation_session_id = session_;message.trajectory_id = trajectory_id_;
    message.map_session_id = map_session_;message.map_generation = generation_;
    message.map_revision = source_map_revision_;
    message.map_stamp = source_map_stamp_;message.state_stamp = state_stamp_;
    message.validated_map_revision = map_->revision;
    message.validated_map_stamp = map_->header.stamp;
    message.max_speed = plan_->maximum_speed;message.max_acceleration = plan_->maximum_acceleration;
    message.max_jerk = plan_->maximum_jerk;
    for (const auto & p:plan_->trajectory) {
      message.durations.push_back(p.getDuration());
      for (int axis = 0; axis < 3; ++axis) {
        for (int j = 0; j < 6; ++j) {
          message.coefficients.push_back(p.getCoeffMat()(axis, j));
        }
      }
    }
    polynomials_->publish(message);
  }
  void tick()
  {
    if (!rclcpp::ok()) {return;}
    const auto current = now().nanoseconds();
    if (current < last_clock_) {reset_required_ = true;clock_reset_ = true;}
    last_clock_ = current;
    if (session_.empty()) {return;}
    if (stopped_) {if (worker_ > 0) {stopWorker();} return;}
    if (!ready()) {fail("input_invalid_or_stale");return;}
    try {
      updateGrid();
      if (worker_ > 0) {
        int status = 0;const auto result = waitpid(worker_, &status, WNOHANG);
        if (result == 0) {if (age(worker_started_) >= 3.) {fail("worker_wall_deadline");} return;}
        if (result < 0 && errno == EINTR) {return;}
        worker_ = 0;
        if (result < 0 || !WIFEXITED(status) || WEXITSTATUS(status) != 0) {
          fail("worker_rejected_or_exited");return;
        }
        auto plan = std::make_unique<et::Plan>(et::readPlan(output_));
        if ((position() - request_.start).norm() > .15) {
          fail("stale_result_or_current_map_collision");return;
        }
        et::validate(request_, *plan, grid_, et::Steady::now() + std::chrono::milliseconds(40));
        // Recheck serialized result boundaries and finite bounds in the owner.
        if ((plan->trajectory.getPos(0) - request_.start).norm() > 1e-5 ||
          (plan->trajectory.getVel(0) - request_.velocity).norm() > 1e-4 ||
          (plan->trajectory.getPos(plan->trajectory.getTotalDuration()) - request_.goal).norm() >
          1e-5 ||
          !std::isfinite(plan->maximum_speed) || plan->maximum_speed > request_.max_speed ||
          !std::isfinite(plan->maximum_acceleration) ||
          plan->maximum_acceleration > request_.max_acceleration ||
          plan->maximum_acceleration > et::kTrackingAcceleration ||
          !std::isfinite(plan->maximum_jerk) || plan->maximum_jerk > request_.max_jerk ||
          plan->maximum_jerk > et::kTrackingJerk ||
          (native_fuel_ && plan->trajectory.getTotalDuration() + 1e-6 < 2. * yaw_duration_))
        {
          fail("serialized_plan_bounds");return;
        }
        if (trajectory_id_ == std::numeric_limits<uint32_t>::max()) {
          fail("trajectory_id_exhausted");return;
        }
        ++trajectory_id_;plan_ = std::move(plan);
        start_time_ = now() + rclcpp::Duration::from_seconds(.05);announce();
      }
      if (!plan_) {return;}
      const auto command_time = now();
      const double time = std::clamp(
        (command_time - start_time_).seconds(), 0., plan_->trajectory.getTotalDuration());
      const auto p = plan_->trajectory.getPos(time), v = plan_->trajectory.getVel(time),
        a = plan_->trajectory.getAcc(time);
      if (!et::collisionFree(
          grid_, plan_->trajectory, std::min(time, plan_->trajectory.getTotalDuration() - 1e-6),
          et::Steady::now() + std::chrono::milliseconds(40)))
      {
        // Stop output first. Only a map change with safe current state and
        // reference may request the coordinator's bounded stop/replan cycle.
        const bool safe_to_replan = grid_.free(position()) && grid_.free(p) &&
          (p - position()).norm() <= .2 &&
          get_publishers_info_by_topic("/position_cmd").size() == 1;
        fail("current_map_collision_or_unknown", safe_to_replan);return;
      }
      // Compare the measured pose with the reference at its SOURCE stamp.
      // Comparing against command_time also counts delivery age as position error.
      const double state_time = std::clamp(
        (rclcpp::Time(state_->header.stamp, get_clock()->get_clock_type()) - start_time_).seconds(),
        0., plan_->trajectory.getTotalDuration());
      const double tracking_error = (plan_->trajectory.getPos(state_time) - position()).norm();
      if (tracking_error > .2) {
        RCLCPP_ERROR(
          get_logger(), "Tracking error %.6f m at trajectory time %.6f s",
          tracking_error, state_time);
        fail("tracking_error");return;
      }
      if (!p.allFinite() || !v.allFinite() || !a.allFinite() ||
        v.norm() > request_.max_speed || a.norm() > request_.max_acceleration ||
        get_publishers_info_by_topic("/position_cmd").size() != 1)
      {
        fail("tracking_or_command_bounds");return;
      }
      quadrotor_msgs::msg::PositionCommand command;command.header.frame_id = "odom";
      command.header.stamp = command_time;
      command.navigation_session_id = session_;command.trajectory_id = trajectory_id_;
      command.trajectory_flag = command.TRAJECTORY_STATUS_READY;
      command.position.x = p.x();command.position.y = p.y();command.position.z = p.z();
      command.velocity.x = v.x();command.velocity.y = v.y();command.velocity.z = v.z();
      command.acceleration.x = a.x();command.acceleration.y = a.y();command.acceleration.z = a.z();
      const double yaw_fraction = yaw_duration_ >
        1e-6 ? std::clamp(time / yaw_duration_, 0., 1.) : 1.;
      const double yaw_progress = yaw_fraction * yaw_fraction * yaw_fraction *
        (10. + yaw_fraction * (-15. + 6. * yaw_fraction));
      command.yaw =
        native_fuel_ ? std::remainder(yaw_ + yaw_delta_ * yaw_progress, 2. * M_PI) : yaw_;
      commands_->publish(command);
    } catch (const std::exception & e) {fail(e.what());}
  }
  std::string directory_, worker_exe_, input_, output_, session_, map_session_;
  bool dynamic_map_{false}, native_fuel_{false}, fuel_view_started_{false};
  double yaw_delta_{0.}, yaw_duration_{0.};
  et::Steady::time_point fuel_view_created_;
  easyfly_msgs::srv::SetExplorationView::Request::ConstSharedPtr fuel_view_;
  rclcpp::Service<easyfly_msgs::srv::SetExplorationView>::SharedPtr fuel_view_service_;
  bool stopped_{true}, failed_{false}, reset_required_{false};
  bool allow_idle_policy_epoch_{false}, clock_reset_{false};pid_t worker_{0};
  uint32_t trajectory_id_{0};
  uint64_t job_{0}, generation_{0}, source_map_revision_{0},
  grid_revision_{std::numeric_limits<uint64_t>::max()};
  int64_t last_clock_{0};double floor_, ceiling_, yaw_{0};et::Grid grid_;et::Request request_;
  builtin_interfaces::msg::Time state_stamp_, source_map_stamp_;
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  et::Steady::time_point map_received_, state_received_, worker_started_;
  Map::ConstSharedPtr map_;State::ConstSharedPtr state_;std::unique_ptr<et::Plan> plan_;
  rclcpp::Service<Session>::SharedPtr service_;rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<Map>::SharedPtr map_sub_;rclcpp::Subscription<State>::SharedPtr state_sub_;
  rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr commands_;
  rclcpp::Publisher<easyfly_msgs::msg::NavigationSession>::SharedPtr faults_, sessions_, replans_;
  rclcpp::Publisher<easyfly_msgs::msg::PolynomialTrajectory>::SharedPtr polynomials_;
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<GcopterNavigation>();
    rclcpp::spin(node);
  } catch (const rclcpp::exceptions::RCLError & e) {
    if (rclcpp::ok()) {std::cerr << e.what() << '\n'; result = 1;}
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n'; result = 1;
  }
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return result;
}
