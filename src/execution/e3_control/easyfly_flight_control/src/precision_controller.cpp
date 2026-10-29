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

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <limits>

#include "easyfly_flight_control/flight_controller.hpp"

namespace easyfly_flight_control
{
namespace
{
int64_t source_stamp(const std_msgs::msg::Header & header)
{
  if (header.stamp.sec < 0 || header.stamp.nanosec >= 1000000000u) {return 0;}
  return static_cast<int64_t>(header.stamp.sec) * 1000000000LL + header.stamp.nanosec;
}

double steady_seconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

void FlightController::initialize_precision_landing()
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  precision_timing_diagnostics_ = declare_parameter(
    "precision_timing_diagnostics", false, descriptor);
  precision_enabled_ = declare_parameter("precision_enabled", false, descriptor);
  precision_tag_id_ = declare_parameter("precision_tag_id", -1, descriptor);
  precision_tag_size_m_ = declare_parameter("precision_tag_size_m", 0.0, descriptor);
  precision_optical_frame_ = declare_parameter(
    "precision_optical_frame", "camera_down_optical_frame", descriptor);
  auto parameter = [this, &descriptor](const std::string & name, double value) {
      return declare_parameter("precision_" + name, value, descriptor);
    };
  auto & config = precision_config_;
  config.moving_target = declare_parameter("precision_moving_target", false, descriptor);
  config.moving_max_speed = parameter("moving_max_speed", config.moving_max_speed);
  config.moving_fit_error = parameter("moving_fit_error", config.moving_fit_error);
  config.moving_land_timeout = parameter("moving_land_timeout", config.moving_land_timeout);
  config.moving_usable_radius = parameter("moving_usable_radius", config.moving_usable_radius);
  config.horizontal_speed = parameter("horizontal_speed", config.horizontal_speed);
  config.descent_speed = parameter("descent_speed", config.descent_speed);
  config.align_tolerance = parameter("align_tolerance", config.align_tolerance);
  config.handoff_height = parameter("handoff_height", config.handoff_height);
  config.descent_floor_height = parameter("descent_floor_height", config.descent_floor_height);
  config.max_height = parameter("max_height", config.max_height);
  config.max_radius = parameter("max_radius", config.max_radius);
  config.plane_z = parameter("plane_z", config.plane_z);
  config.plane_tolerance = parameter("plane_tolerance", config.plane_tolerance);
  config.tilt_limit = parameter("tilt_limit", config.tilt_limit);
  config.target_jump = parameter("target_jump", config.target_jump);
  config.target_max_age = parameter("target_max_age", config.target_max_age);
  config.acquire_timeout = parameter("acquire_timeout", config.acquire_timeout);
  config.lost_timeout = parameter("lost_timeout", config.lost_timeout);
  config.settle_observations = declare_parameter(
    "precision_settle_observations", config.settle_observations, descriptor);
  if (precision_enabled_ && (!config.valid() || precision_tag_id_ < 0 ||
    !std::isfinite(precision_tag_size_m_) || precision_tag_size_m_ <= 0 ||
    precision_optical_frame_.empty() || config.plane_z + config.max_height > max_altitude_))
  {
    throw std::invalid_argument("invalid explicit precision landing configuration");
  }
  precision_policy_ = std::make_unique<easyfly_precision_landing::Policy>(config);
  precision_receiver_progress_time_ = std::chrono::steady_clock::now();
  create_precision_receiver();
  rclcpp::SubscriptionOptions status_options;
  status_options.callback_group = state_cb_group_;
  sub_flight_status_ = create_subscription<easyfly_msgs::msg::FlightStatus>(
    "/flight_status", rclcpp::QoS(1).best_effort(),
    [this](easyfly_msgs::msg::FlightStatus::ConstSharedPtr status) {
      std::lock_guard<std::mutex> lock(state_mtx_);
      flight_status_ = *status;
      flight_status_receipt_ = std::chrono::steady_clock::now();
    }, status_options);
  if (precision_timing_diagnostics_) {
    timer_precision_timing_ = create_wall_timer(
      std::chrono::seconds(1), [this]() {flush_precision_timing();}, state_cb_group_);
  }
  if (precision_enabled_) {
    // Sharing the receiver group is essential: control progress on another
    // group must not hide a blocked reader or a callback waiting on flight_mtx_.
    timer_precision_receiver_ = create_wall_timer(
      std::chrono::milliseconds(50), [this]() {
        std::lock_guard<std::recursive_mutex> lock(flight_mtx_);
        precision_receiver_progress();
      }, state_cb_group_);
  }
  cli_precision_land_ = rclcpp_action::create_client<Land>(
    this, "/land", cli_cb_group_);
  using std::placeholders::_1;
  using std::placeholders::_2;
  as_precision_ = rclcpp_action::create_server<PrecisionLand>(
    this, "/precision_land",
    std::bind(&FlightController::handle_precision_goal, this, _1, _2),
    std::bind(&FlightController::handle_precision_cancel, this, _1),
    std::bind(&FlightController::handle_precision_accepted, this, _1),
    rcl_action_server_get_default_options(), action_cb_group_);
  srv_precision_ownership_ = create_service<easyfly_msgs::srv::GetActionOwnership>(
    "/precision_landing/ownership", [this](
      easyfly_msgs::srv::GetActionOwnership::Request::SharedPtr request,
      easyfly_msgs::srv::GetActionOwnership::Response::SharedPtr response) {
      std::lock_guard<std::recursive_mutex> lock(flight_mtx_);
      response->goal_id = request->goal_id;
      if (request->goal_id.size() != 32 ||
      !std::all_of(
        request->goal_id.begin(), request->goal_id.end(), [](char value) {
          return (value >= '0' && value <= '9') || (value >= 'a' && value <= 'f');
        })) {return;}
      const bool released = std::find(
        precision_released_.begin(), precision_released_.end(),
        request->goal_id) != precision_released_.end();
      if (precision_reserved_ && request->goal_id == precision_session()) {
        response->state = easyfly_msgs::srv::GetActionOwnership::Response::OWNED;
      } else if (released) {
        response->state = easyfly_msgs::srv::GetActionOwnership::Response::RELEASED;
      }
    });
  srv_precision_capabilities_ = create_service<easyfly_msgs::srv::GetCapabilities>(
    "/platform/precision_landing/capabilities", [this](
      easyfly_msgs::srv::GetCapabilities::Request::SharedPtr,
      easyfly_msgs::srv::GetCapabilities::Response::SharedPtr response) {
      std::lock_guard<std::recursive_mutex> lock(flight_mtx_);
      std::chrono::steady_clock::time_point receipt;
      const auto uav = get_uav(&receipt);
      response->header.stamp = now();
      response->header.frame_id = "odom";
      response->names = {"precision_landing"};
      response->implemented = {true};
      response->ready = {precision_enabled_ && !precision_reserved_ &&
        !action_reserved_ && active_ == HOVER && !command_uncertain_ &&
        flight_state_ready(uav, receipt) && uav.header.frame_id == "odom" &&
        uav.armed && uav.flight_mode == 3 &&
        precision_target_ready()};
      response->details = {precision_enabled_ ?
        "configured; readiness requires owned OFFBOARD hover and fresh selected target" :
        "disabled; explicit startup simulation configuration required"};
    });
}

bool FlightController::precision_receiver_progress()
{
  if (!precision_enabled_) {return true;}
  const auto current = std::chrono::steady_clock::now();
  const double gap = std::chrono::duration<double>(
    current - precision_receiver_progress_time_).count();
  precision_receiver_progress_time_ = current;
  if (gap >= 0 && gap <= precision_config_.target_max_age) {return true;}
  precision_target_.valid = false;
  precision_target_receipt_ = {};
  ++precision_receiver_generation_;
  // Preserve source watermark, policy and the first-loss deadline. A queued
  // ROS now is not an acquisition barrier. Retire only this guidance reader.
  sub_precision_.reset();
  create_precision_receiver();
  RCLCPP_WARN(
    get_logger(), "precision_receiver_processing_gap: volatile reader rebuilt; generation=%" PRIu64,
    precision_receiver_generation_);
  return false;
}

void FlightController::create_precision_receiver()
{
  rclcpp::SubscriptionOptions options;
  options.callback_group = state_cb_group_;
  const auto generation = precision_receiver_generation_;
  sub_precision_ = create_subscription<easyfly_msgs::msg::LandingGuidanceTarget>(
    "/landing/guidance_target", rclcpp::QoS(1).durability_volatile(),
    [this, generation](easyfly_msgs::msg::LandingGuidanceTarget::SharedPtr target,
    const rclcpp::MessageInfo & info) {
      const double entered = precision_timing_diagnostics_ ? steady_seconds() : 0.0;
      const int64_t entry_clock = precision_timing_diagnostics_ ? now().nanoseconds() : 0;
      const int64_t entry_wall = precision_timing_diagnostics_ ?
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count() : 0;
      std::lock_guard<std::recursive_mutex> lock(flight_mtx_);
      const double acquired = precision_timing_diagnostics_ ? steady_seconds() : 0.0;
      const auto record = [&](const char * outcome) {
        record_precision_timing(
          outcome, *target, info, entered, acquired, entry_clock, entry_wall, generation);
      };
      // An already taken callback may have waited on the lock across a reset.
      if (generation != precision_receiver_generation_) {record("retired"); return;}
      const int64_t stamp = source_stamp(target->header);
      const int64_t clock = now().nanoseconds();
      const double age = static_cast<double>(clock - stamp) * 1e-9;
      const bool source_valid = stamp > 0 && stamp == source_stamp(target->source_header) &&
      target->header.frame_id == "odom" &&
      target->source_header.frame_id == precision_optical_frame_ &&
      target->tag_id == precision_tag_id_ && clock > 0 &&
      age >= -std::min(0.2, precision_config_.target_max_age) &&
      age <= precision_config_.target_max_age;
      if (!precision_receiver_progress()) {
        if (source_valid) {precision_last_stamp_ = std::max(precision_last_stamp_, stamp);}
        record("processing_gap");
        return;
      }
      if (!source_valid || stamp < precision_last_stamp_) {
        precision_target_.valid = false;
        record("source_invalid_or_old");
        return;
      }
      if (stamp == precision_last_stamp_) {
        if (!target->valid) {precision_target_.valid = false;}
        record("duplicate");
        return;
      }
      precision_last_stamp_ = stamp;
      precision_target_ = *target;
      precision_target_receipt_ = std::chrono::steady_clock::now();
      ++precision_target_sequence_;
      record("stored");
    }, options);
}

void FlightController::record_precision_timing(
  const char * outcome, const easyfly_msgs::msg::LandingGuidanceTarget & target,
  const rclcpp::MessageInfo & info, double entered, double acquired,
  int64_t entry_clock, int64_t entry_wall, uint64_t callback_generation)
{
  if (!precision_timing_diagnostics_) {return;}
  const auto & native = info.get_rmw_message_info();
  // Native timestamps use system time, not ROS or steady. Missing/backward
  // differences are unknown. The callback timestamp is NOT a native take time.
  const auto received = native.received_timestamp;
  std::ostringstream event;
  event << std::setprecision(17) << std::boolalpha <<
    "{\"stage\":\"guidance_received\",\"source_ns\":" << source_stamp(target.header) <<
    ",\"steady_s\":" << entered << ",\"clock_ns\":" << entry_clock <<
    ",\"callback_wall_ns\":" << entry_wall <<
    ",\"source_wall_ns\":" << native.source_timestamp <<
    ",\"received_wall_ns\":" << received << ",\"receive_to_callback_ms\":";
  if (received > 0 && received <= entry_wall) {
    event << static_cast<double>(entry_wall - received) * 1e-6;
  } else {event << "null";}
  event << ",\"lock_wait_ms\":" << (acquired - entered) * 1000.0 <<
    ",\"generation\":" << precision_receiver_generation_ <<
    ",\"callback_generation\":" << callback_generation <<
    ",\"outcome\":\"" << outcome << "\",\"target_valid\":" << target.valid <<
    ",\"ready_at_exit\":" << precision_target_ready() <<
    ",\"finish_steady_s\":" << steady_seconds() <<
    ",\"finish_clock_ns\":" << now().nanoseconds() <<
    ",\"session_active\":" << precision_reserved_ <<
    ",\"session_uuid\":\"" << precision_session() << "\"}";
  if (precision_timing_events_.size() == 256) {
    precision_timing_events_.pop_front();
    ++precision_timing_dropped_;
  }
  precision_timing_events_.push_back(event.str());
}

void FlightController::flush_precision_timing()
{
  std::lock_guard<std::recursive_mutex> lock(flight_mtx_);
  if (!precision_timing_diagnostics_) {return;}
  std::ostringstream snapshot;
  snapshot << "{\"dropped_events\":" << precision_timing_dropped_ << ",\"events\":[";
  for (size_t index = 0; index < precision_timing_events_.size(); ++index) {
    if (index) {snapshot << ',';}
    snapshot << precision_timing_events_[index];
  }
  snapshot << "]}";
  precision_timing_events_.clear();
  RCLCPP_INFO(get_logger(), "precision_frame_diag %s", snapshot.str().c_str());
}

bool FlightController::precision_target_ready() const
{
  const auto & target = precision_target_;
  const int64_t stamp = source_stamp(target.header);
  const int64_t clock = now().nanoseconds();
  const double age = static_cast<double>(clock - stamp) * 1e-9;
  const auto & normal = target.plane_normal;
  const auto & point = target.position;
  return target.valid && stamp > 0 && source_stamp(target.source_header) == stamp &&
         target.tag_id == precision_tag_id_ && target.header.frame_id == "odom" &&
         std::isfinite(target.tag_size_m) &&
         std::abs(target.tag_size_m - precision_tag_size_m_) <= 1e-6 &&
         target.source_header.frame_id == precision_optical_frame_ && clock > 0 &&
         age >= -std::min(0.2, precision_config_.target_max_age) &&
         age <= precision_config_.target_max_age &&
         precision_target_receipt_ != std::chrono::steady_clock::time_point{} &&
         std::chrono::duration<double>(
    std::chrono::steady_clock::now() -
    precision_target_receipt_).count() <= precision_config_.target_max_age &&
         std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
         std::isfinite(normal.x) && std::isfinite(normal.y) && std::isfinite(normal.z) &&
         std::abs(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z - 1.0) <
         1e-4 && std::abs(normal.z) >= std::cos(precision_config_.tilt_limit) &&
         std::abs(point.z - precision_config_.plane_z) <= precision_config_.plane_tolerance;
}

rclcpp_action::GoalResponse FlightController::handle_precision_goal(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const PrecisionLand::Goal> goal)
{
  std::lock_guard<std::recursive_mutex> lock(flight_mtx_);
  std::chrono::steady_clock::time_point receipt;
  const auto uav = get_uav(&receipt);
  if (!precision_enabled_ || !precision_config_.valid() ||
    goal->tag_id != precision_tag_id_ || !std::isfinite(goal->timeout) || goal->timeout <= 0 ||
    precision_reserved_ || action_reserved_ || active_ != HOVER || command_uncertain_ ||
    !flight_state_ready(uav, receipt) || uav.header.frame_id != "odom" ||
    !uav.armed || uav.flight_mode != 3 ||
    uav.position_z < precision_config_.plane_z ||
    uav.position_z - precision_config_.plane_z > precision_config_.max_height)
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  precision_uuid_ = uuid;
  precision_reserved_ = true;
  precision_terminal_ = false;
  precision_handoff_aligned_ = false;
  precision_handoff_pending_ = false;
  precision_handoff_barrier_ns_ = 0;
  precision_cancel_requested_ = false;
  precision_completion_deferred_ = false;
  precision_stop_reason_.clear();
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void FlightController::handle_precision_accepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<PrecisionLand>> goal)
{
  std::lock_guard<std::recursive_mutex> lock(flight_mtx_);
  if (!precision_reserved_ || goal->get_goal_id() != precision_uuid_) {
    auto result = std::make_shared<PrecisionLand::Result>();
    result->message = "precision reservation superseded";
    goal->abort(result);
    return;
  }
  gh_precision_ = goal;
  std::chrono::steady_clock::time_point receipt;
  const auto uav = get_uav(&receipt);
  if (command_uncertain_ || active_ != HOVER || action_reserved_ ||
    !flight_state_ready(uav, receipt) || uav.header.frame_id != "odom" ||
    !uav.armed || uav.flight_mode != 3 ||
    uav.position_z < precision_config_.plane_z ||
    uav.position_z - precision_config_.plane_z > precision_config_.max_height)
  {
    active_ = NONE;
    finish_precision(false, "precision admission evidence lost");
    return;
  }
  precision_start_ = precision_last_control_ = std::chrono::steady_clock::now();
  precision_start_source_ns_ = now().nanoseconds();
  precision_policy_->start({uav.position_x, uav.position_y, uav.position_z}, steady_seconds());
  active_ = PRECISION_GUIDANCE;
  hover_yaw_ = uav.yaw;
}

bool FlightController::precision_control_loop()
{
  if (!precision_reserved_ || !gh_precision_) {return false;}
  if (precision_completion_deferred_ && gh_precision_->is_canceling()) {
    precision_completion_deferred_ = false;
    finish_precision(
      precision_deferred_success_, precision_deferred_message_,
      precision_deferred_release_);
    return !precision_handoff_;
  }
  std::chrono::steady_clock::time_point receipt;
  const auto uav = get_uav(&receipt);
  const auto current = std::chrono::steady_clock::now();
  const bool trusted = !command_uncertain_ &&
    rclcpp::ok(get_node_base_interface()->get_context()) &&
    uav.header.frame_id == "odom" && flight_state_ready(uav, receipt) &&
    (precision_handoff_ || !precision_mode_control_lost());
  const bool timed_out = std::chrono::duration<double>(current - precision_start_).count() >=
    gh_precision_->get_goal()->timeout;
  if (precision_stop_reason_.empty()) {
    if ((!trusted && !(precision_handoff_ && land_disarm_confirmed(uav, receipt))) ||
      (!precision_handoff_ && (!uav.armed || uav.flight_mode != 3)))
    {
      precision_stop_reason_ = "flight evidence/control lost; precision stream stopped";
    } else if (gh_precision_->is_canceling()) {
      precision_stop_reason_ = "precision landing canceled";
    } else if (timed_out) {
      precision_stop_reason_ = "precision landing timeout";
    }
  }
  if (precision_handoff_) {
    if (precision_config_.moving_target && precision_stop_reason_.empty() &&
      !land_disarm_confirmed(uav, receipt))
    {
      const double source_elapsed = static_cast<double>(
        now().nanoseconds() - precision_handoff_source_ns_) * 1e-9;
      const double wall_elapsed = std::chrono::duration<double>(
        current - precision_handoff_started_).count();
      if (source_elapsed < 0 || source_elapsed > precision_config_.moving_land_timeout ||
        wall_elapsed > 2 * precision_config_.moving_land_timeout)
      {
        precision_stop_reason_ = "moving pad bounded Land interval expired";
      } else if (precision_target_ready() && std::hypot(
          precision_target_.position.x - precision_handoff_board_.x,
          precision_target_.position.y - precision_handoff_board_.y) >
        precision_config_.moving_max_speed * source_elapsed +
        2 * precision_config_.moving_fit_error)
      {
        precision_stop_reason_ = "moving pad left bounded handoff corridor";
      }
    }
    if (!precision_terminal_) {
      auto feedback = std::make_shared<PrecisionLand::Feedback>();
      feedback->phase = "land_handoff";
      feedback->target_valid = precision_target_ready();
      if (feedback->target_valid) {
        feedback->horizontal_error_m = std::hypot(
          uav.position_x - precision_target_.position.x,
          uav.position_y - precision_target_.position.y);
        feedback->height_above_target_m = uav.position_z - precision_target_.position.z;
      }
      gh_precision_->publish_feedback(feedback);
    }
    if (!precision_stop_reason_.empty()) {precision_cancel_child();}
    return false;  // Existing Land remains sole controller of the handoff.
  }
  if (!precision_stop_reason_.empty()) {
    if (trusted && uav.armed && uav.flight_mode == 3) {
      enter_hover_at(uav.position_x, uav.position_y, uav.position_z, uav.yaw);
    } else {active_ = NONE;}
    // Cancel callback return and action CANCELING application can race this loop.
    if (precision_stop_reason_ == "precision landing canceled" &&
      !gh_precision_->is_canceling()) {return true;}
    finish_precision(false, precision_stop_reason_);
    return true;
  }
  easyfly_precision_landing::Observation observation;
  observation.valid = precision_target_ready();
  observation.stamp_ns = source_stamp(precision_target_.header);
  observation.x = precision_target_.position.x;
  observation.y = precision_target_.position.y;
  observation.z = precision_target_.position.z;
  observation.normal_x = precision_target_.plane_normal.x;
  observation.normal_y = precision_target_.plane_normal.y;
  observation.normal_z = precision_target_.plane_normal.z;
  const double dt = std::chrono::duration<double>(current - precision_last_control_).count();
  precision_last_control_ = current;
  auto decision = precision_policy_->step(
    {uav.position_x, uav.position_y, uav.position_z}, observation,
    steady_seconds(), std::max(1e-6, dt));
  if (decision.handoff) {
    if (!precision_handoff_pending_) {
      precision_handoff_pending_ = true;
      precision_handoff_barrier_ns_ = now().nanoseconds();
      precision_handoff_hold_ = {uav.position_x, uav.position_y, uav.position_z};
    }
    // Never chase measured altitude while awaiting a distinct post-decision
    // mode observation. Aggregate state publication does not provide one.
    decision.x = precision_handoff_hold_.x;
    decision.y = precision_handoff_hold_.y;
    decision.z = precision_handoff_hold_.z;
    decision.handoff = precision_handoff_mode_ready();
    decision.phase = decision.handoff ? "land_handoff" : "handoff_wait_mode";
  } else {
    precision_handoff_pending_ = false;
    precision_handoff_barrier_ns_ = 0;
  }
  auto feedback = std::make_shared<PrecisionLand::Feedback>();
  feedback->phase = decision.phase;
  feedback->target_valid = decision.target_valid;
  feedback->horizontal_error_m = decision.horizontal_error;
  feedback->height_above_target_m = decision.height_above_target;
  gh_precision_->publish_feedback(feedback);
  if (decision.failed) {
    enter_hover_at(uav.position_x, uav.position_y, uav.position_z, uav.yaw);
    finish_precision(false, decision.reason);
    return true;
  }
  hover_x_ = decision.x;
  hover_y_ = decision.y;
  hover_z_ = decision.z;
  if (decision.handoff) {
    precision_handoff_aligned_ = true;
    precision_handoff_board_ = {observation.x, observation.y, observation.z};
    precision_handoff();
    return false;
  }
  easyfly_msgs::msg::UAVCommand command;
  command.control_mode = easyfly_msgs::msg::UAVCommand::POSITION;
  stamp_command(command);
  command.position_x = decision.x;
  command.position_y = decision.y;
  command.position_z = decision.z;
  command.yaw = hover_yaw_;
  pub_uav_cmd_->publish(command);
  return true;
}

std::string FlightController::precision_session() const
{
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (auto byte : precision_uuid_) {
    stream << std::setw(2) << static_cast<int>(byte);
  }
  return stream.str();
}

bool FlightController::precision_land_authorized(const std::string & session) const
{
  if (!precision_reserved_) {
    return session.empty();
  }
  return precision_handoff_ &&
         session == precision_session();
}

bool FlightController::precision_handoff_ready() const
{
  std::chrono::steady_clock::time_point receipt;
  const auto uav = get_uav(&receipt);
  const auto & target = precision_target_.position;
  const double height = uav.position_z - target.z;
  return precision_handoff_aligned_ && precision_handoff_mode_ready() &&
         precision_stop_reason_.empty() &&
         !precision_cancel_requested_ && precision_target_ready() &&
         flight_state_ready(uav, receipt) && uav.header.frame_id == "odom" &&
         uav.armed && uav.flight_mode == 3 && height >= 0 &&
         height <= precision_config_.handoff_height &&
         std::hypot(uav.position_x - target.x, uav.position_y - target.y) <=
         precision_config_.align_tolerance &&
         std::hypot(
    target.x - precision_handoff_board_.x,
    target.y - precision_handoff_board_.y) <= precision_config_.target_jump;
}

bool FlightController::precision_handoff_mode_ready() const
{
  std::lock_guard<std::mutex> lock(state_mtx_);
  const auto & status = flight_status_;
  const int64_t source = source_stamp(status.header);
  const double source_age = static_cast<double>(now().nanoseconds() - source) * 1e-9;
  const double receipt_age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - flight_status_receipt_).count();
  return precision_handoff_pending_ && precision_handoff_barrier_ns_ > 0 &&
         source > precision_handoff_barrier_ns_ && status.valid && status.connected &&
         status.armed && status.flight_mode == 3 && status.flight_mode_name == "OFFBOARD" &&
         std::isfinite(status.receipt_age_s) && status.receipt_age_s >= 0 &&
         status.receipt_age_s <= 1.0 && source_age >= -0.2 && source_age <= 1.0 &&
         flight_status_receipt_ != std::chrono::steady_clock::time_point{} &&
         receipt_age >= 0 && status.receipt_age_s + receipt_age <= 1.0;
}

bool FlightController::precision_mode_control_lost() const
{
  std::lock_guard<std::mutex> lock(state_mtx_);
  const auto & status = flight_status_;
  const int64_t source = source_stamp(status.header);
  const double source_age = static_cast<double>(now().nanoseconds() - source) * 1e-9;
  const double receipt_age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - flight_status_receipt_).count();
  return precision_start_source_ns_ > 0 && source > precision_start_source_ns_ &&
         status.valid && status.connected &&
         (!status.armed || status.flight_mode != 3 || status.flight_mode_name != "OFFBOARD") &&
         std::isfinite(status.receipt_age_s) && status.receipt_age_s >= 0 &&
         source_age >= -0.2 && source_age <= 1.0 &&
         flight_status_receipt_ != std::chrono::steady_clock::time_point{} &&
         receipt_age >= 0 && status.receipt_age_s + receipt_age <= 1.0;
}

rclcpp_action::CancelResponse FlightController::handle_precision_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<PrecisionLand>> goal)
{
  std::lock_guard<std::recursive_mutex> lock(flight_mtx_);
  if (!precision_reserved_ || !goal || goal->get_goal_id() != precision_uuid_) {
    return rclcpp_action::CancelResponse::REJECT;
  }
  // The action framework applies CANCELING only after this callback returns.
  precision_stop_reason_ = "precision landing canceled";
  precision_cancel_requested_ = true;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void FlightController::finish_precision(
  bool success, const std::string & message, bool release)
{
  std::lock_guard<std::recursive_mutex> lock(flight_mtx_);
  if (gh_precision_ && !precision_terminal_ && precision_cancel_requested_ &&
    !gh_precision_->is_canceling())
  {
    precision_completion_deferred_ = true;
    precision_deferred_success_ = success;
    precision_deferred_message_ = message;
    precision_deferred_release_ = release;
    return;
  }
  if (gh_precision_ && !precision_terminal_) {
    auto result = std::make_shared<PrecisionLand::Result>();
    result->success = success;
    result->message = message;
    if (gh_precision_->is_canceling()) {
      gh_precision_->canceled(result);
    } else if (success) {
      gh_precision_->succeed(result);
    } else {
      gh_precision_->abort(result);
    }
    precision_terminal_ = true;
  }
  if (!release) {
    return;
  }
  precision_released_.push_back(precision_session());
  if (precision_released_.size() > 64) {
    precision_released_.pop_front();
  }
  precision_reserved_ = false;
  precision_handoff_ = false;
  precision_handoff_pending_ = false;
  precision_handoff_barrier_ns_ = 0;
  precision_child_admission_pending_ = false;
  precision_child_cancel_pending_ = false;
  precision_child_cancel_accepted_ = false;
  precision_stop_reason_.clear();
  precision_child_.reset();
  gh_precision_.reset();
}

void FlightController::precision_cancel_child()
{
  if (!precision_child_ || precision_child_cancel_pending_ ||
    precision_child_cancel_accepted_)
  {
    return;
  }
  precision_child_cancel_pending_ = true;
  const auto session = precision_session();
  try {
    cli_precision_land_->async_cancel_goal(
      precision_child_,
      [this, session](auto response) {
        std::lock_guard<std::recursive_mutex> lock(flight_mtx_);
        if (!precision_reserved_ || session != precision_session()) {
          return;
        }
        precision_child_cancel_pending_ = false;
        precision_child_cancel_accepted_ = response &&
        !response->goals_canceling.empty();
        // A preparation-phase rejection is not a child terminal state.
        // Retry the same exact goal on a later control cycle.
      });
  } catch (const std::exception & error) {
    // A cancel send exception does not prove remote cancellation or release.
    precision_child_cancel_pending_ = true;
    precision_stop_reason_ = std::string("child cancel outcome unknown: ") + error.what();
  }
}

void FlightController::precision_handoff()
{
  if (precision_handoff_) {
    return;
  }
  if (!cli_precision_land_->action_server_is_ready()) {
    precision_stop_reason_ = "Land server unavailable before handoff";
    return;
  }
  precision_handoff_ = true;
  precision_handoff_source_ns_ = now().nanoseconds();
  precision_handoff_started_ = std::chrono::steady_clock::now();
  precision_child_admission_pending_ = true;
  active_ = HOVER;
  Land::Goal request;
  request.landing_speed = 0.0;
  request.precision_landing_session_id = precision_session();
  const auto session = precision_session();
  rclcpp_action::Client<Land>::SendGoalOptions options;
  options.goal_response_callback = [this, session](auto child) {
      std::lock_guard<std::recursive_mutex> lock(flight_mtx_);
      if (!precision_reserved_ || session != precision_session()) {
        return;
      }
      precision_child_admission_pending_ = false;
      if (!child) {
        finish_precision(false, "internal Land rejected");
        return;
      }
      precision_child_ = child;
      if (!precision_stop_reason_.empty()) {
        precision_cancel_child();
      }
    };
  options.result_callback = [this, session](const auto & result) {
      std::lock_guard<std::recursive_mutex> lock(flight_mtx_);
      if (!precision_reserved_ || session != precision_session()) {
        return;
      }
      if (!precision_stop_reason_.empty()) {
        finish_precision(false, precision_stop_reason_ + "; child Land drained");
      } else {
        const bool success = result.code == rclcpp_action::ResultCode::SUCCEEDED &&
          result.result && result.result->success;
        finish_precision(
          success,
          result.result ? result.result->message : "missing Land result");
      }
    };
  try {
    cli_precision_land_->async_send_goal(request, options);
  } catch (const std::exception & error) {
    // Submission can have reached the middleware. Never reopen admission.
    precision_stop_reason_ = std::string("Land admission outcome unknown: ") + error.what();
    active_ = NONE;
    finish_precision(false, precision_stop_reason_, false);
  }
}
}  // namespace easyfly_flight_control
