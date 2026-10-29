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

#include <json/json.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <initializer_list>
#include <set>
#include <utility>

#include "easyfly_ui_bridge/model.hpp"

namespace easyfly_ui_bridge
{
ClockWatch::ClockWatch(bool simulation)
: simulation_(simulation) {}

bool ClockWatch::observe(int64_t ns, double steady)
{
  if (!simulation_) {return false;}
  const bool backwards = received_ && ns < now_ns_;
  if (backwards) {++epoch_; reset_ = true;}
  if (!received_ || ns > now_ns_) {
    progressed_at_ = steady;
    if (!backwards) {reset_ = false;}
  }
  received_ = true;
  now_ns_ = ns;
  return backwards;
}

ClockState ClockWatch::state(double steady, int64_t system_ns) const
{
  if (!simulation_) {return {Quality::ValidAtObservation, system_ns, epoch_};}
  if (!received_ || now_ns_ <= 0) {return {Quality::ClockNotReady, now_ns_, epoch_};}
  if (reset_) {return {Quality::ClockReset, now_ns_, epoch_};}
  return {steady - progressed_at_ > 1.0 ? Quality::ClockPaused :
    Quality::ValidAtObservation, now_ns_, epoch_};
}

namespace detail
{
namespace
{
bool finite(std::initializer_list<double> values)
{
  return std::all_of(values.begin(), values.end(), [](double x) {return std::isfinite(x);});
}
bool json(const std::string & text, Json::Value & root)
{
  if (text.empty() || text.size() > 4096) {return false;}
  Json::CharReaderBuilder builder;
  builder["collectComments"] = false;
  builder["allowComments"] = false;
  builder["allowTrailingCommas"] = false;
  builder["failIfExtra"] = true;
  builder["rejectDupKeys"] = true;
  builder["allowSpecialFloats"] = false;
  builder["stackLimit"] = 32;
  std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
  std::string errors;
  if (!reader->parse(text.data(), text.data() + text.size(), &root, &errors)) {return false;}
  const auto finite_tree = [](const auto & self, const Json::Value & v) -> bool {
      if (v.isDouble() && !std::isfinite(v.asDouble())) {return false;}
      if (v.isArray() || v.isObject()) {
        for (const auto & child : v) {
          if (!self(self, child)) {
            return false;
          }
        }
      }
      return true;
    };
  return finite_tree(finite_tree, root);
}
}  // namespace

double steady_seconds()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
{
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000U) {return -1;}
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}
Quality source_quality(
  const std_msgs::msg::Header & header, const ClockState & clock,
  double receipt_age, double lifetime, bool require_odom)
{
  const int64_t source = stamp_ns(header.stamp);
  if (source <= 0 || header.frame_id.size() > 256 ||
    (require_odom && header.frame_id != "odom")) {return Quality::Invalid;}
  if (clock.quality != Quality::ValidAtObservation) {return clock.quality;}
  const double age = static_cast<double>(clock.now_ns - source) / 1e9;
  if (age < -.2) {return Quality::Invalid;}
  if (receipt_age > lifetime || age > lifetime) {return Quality::Stale;}
  return Quality::ValidAtObservation;
}
Quality validate_localization(
  const easyfly_msgs::msg::LocalizationStatus & m, const ClockState & clock, double receipt_age)
{
  using Status = easyfly_msgs::msg::LocalizationStatus;
  if (m.session_id.empty() || m.session_id.size() > 128 || m.reason.size() > 2048 ||
    m.last_rejection.size() > 2048 ||
    (m.backend_id != "openvins" && m.backend_id != "fast_livo2" && m.backend_id != "orbslam3") ||
    m.header.frame_id !=
    (m.backend_id == "orbslam3" ? "orbslam3_map" : m.backend_id ==
    "fast_livo2" ? "fastlivo_odom" : "openvins_odom") ||
    m.tracked_frame != (m.backend_id == "orbslam3" ? "base_link" : "livox_imu_frame") ||
    m.state > Status::FAILED || stamp_ns(m.pose_stamp) < 0 ||
    stamp_ns(m.camera_stamp) < 0 || stamp_ns(m.imu_stamp) < 0 ||
    m.velocity_valid || (m.backend_id == "openvins" && m.tracking_quality_available) ||
    (m.pose_valid != (m.state == Status::ESTIMATING)) ||
    (m.pose_valid &&
    (stamp_ns(m.pose_stamp) <= 0 || m.covariance_available != (m.backend_id != "orbslam3"))))
  {return Quality::Invalid;}
  if (m.state == Status::RESET_REQUIRED || m.state == Status::FAILED) {return Quality::Invalid;}
  const auto quality = source_quality(m.header, clock, receipt_age, .5, false);
  if (quality != Quality::ValidAtObservation) {return quality;}
  if (m.state == Status::STALE) {return Quality::Stale;}
  if (m.state != Status::ESTIMATING) {return Quality::Unknown;}
  auto pose_header = m.header;
  pose_header.stamp = m.pose_stamp;
  return source_quality(pose_header, clock, receipt_age, .5, false);
}
Quality validate_navigation_localization(
  const easyfly_msgs::msg::LocalizationStatus & m, const std::string & backend,
  const ClockState & clock, double receipt_age)
{
  using Status = easyfly_msgs::msg::LocalizationStatus;
  if ((backend != "openvins" && backend != "fast_livo2" && backend != "orbslam3") ||
    m.backend_id != backend + "_navigation" || m.session_id.empty() ||
    m.session_id.size() > 128 || m.reason.size() > 2048 || m.last_rejection.size() > 2048 ||
    m.header.frame_id != "odom" || m.tracked_frame != "base_link" ||
    m.state > Status::FAILED || m.pose_valid != (m.state == Status::ESTIMATING) ||
    m.velocity_valid || m.covariance_available || stamp_ns(m.pose_stamp) < 0 ||
    (m.pose_valid && (!m.tracking_quality_available || stamp_ns(m.pose_stamp) <= 0)))
  {return Quality::Invalid;}
  if (m.state == Status::RESET_REQUIRED || m.state == Status::FAILED) {return Quality::Invalid;}
  const auto quality = source_quality(m.header, clock, receipt_age, .5);
  if (quality != Quality::ValidAtObservation) {return quality;}
  if (m.state == Status::STALE) {return Quality::Stale;}
  if (m.state != Status::ESTIMATING) {return Quality::Unknown;}
  auto pose_header = m.header; pose_header.stamp = m.pose_stamp;
  return source_quality(pose_header, clock, receipt_age, .5);
}
StateView state_view(
  const easyfly_core::StateSnapshot & sdk, const ClockState & clock,
  double first_received, double steady, bool conflict)
{
  StateView result;
  result.sdk = sdk;
  result.evidence.endpoint = "/uav_state (SDK)";
  result.evidence.received_steady = steady - sdk.receipt_age_seconds;
  result.evidence.first_source_steady = first_received;
  result.evidence.receipt_age = steady - first_received;
  result.evidence.source_age = sdk.source_age_seconds;
  result.evidence.source_age_available = sdk.source_time_valid;
  result.evidence.time_epoch = clock.epoch;
  if (!sdk.received) {return result;}
  result.evidence.quality = conflict ? Quality::Invalid :
    source_quality(sdk.value.header, clock, steady - first_received, 1.0);
  if (!sdk.fresh && result.evidence.quality == Quality::ValidAtObservation) {
    result.evidence.quality = sdk.source_time_valid ? Quality::Stale : Quality::Invalid;
  }
  if (sdk.value.flight_mode_name.size() > 2048) {
    result.evidence.quality = Quality::BudgetExceeded;
  }
  result.evidence.reason = conflict ? "same source stamp with different payload" :
    "source and first distinct receipt checked; per-field validity is separate";
  const auto & v = sdk.value;
  const bool current = result.evidence.quality == Quality::ValidAtObservation;
  result.position_valid = current && v.pose_valid &&
    finite({v.position_x, v.position_y, v.position_z});
  result.velocity_valid = current && v.velocity_valid &&
    finite({v.velocity_x, v.velocity_y, v.velocity_z});
  result.attitude_valid = current && v.attitude_valid &&
    finite({v.roll, v.pitch, v.yaw, v.rollspeed, v.pitchspeed, v.yawspeed});
  result.battery_valid = current && v.battery_valid &&
    finite({v.battery_remaining, v.battery_voltage}) && v.battery_voltage >= 0 &&
    v.battery_remaining >= 0 && v.battery_remaining <= 1;
  result.gps_valid = current && v.gps_valid &&
    finite({v.latitude, v.longitude, v.altitude, v.relative_altitude, v.eph, v.epv}) &&
    std::abs(v.latitude) <= 90 && std::abs(v.longitude) <= 180 && v.eph >= 0 && v.epv >= 0;
  if (result.velocity_valid) {
    const double speed = std::hypot(v.velocity_x, v.velocity_y, v.velocity_z);
    if (std::isfinite(speed)) {result.speed_mps = speed;} else {result.velocity_valid = false;}
  }
  if (result.battery_valid) {result.battery_percent = v.battery_remaining * 100;}
  return result;
}
Quality validate_image(const sensor_msgs::msg::Image & v)
{
  if (v.width > 1920 || v.height > 1080 || v.data.size() > 8U * 1024U * 1024U) {
    return Quality::BudgetExceeded;
  }
  const uint32_t channels = v.encoding == "mono8" ? 1 :
    ((v.encoding == "rgb8" || v.encoding == "bgr8") ? 3 : 0);
  if (!channels) {return Quality::Unsupported;}
  if (!v.width || !v.height || v.is_bigendian > 1 || v.header.frame_id.empty() ||
    v.header.frame_id.size() > 256 || stamp_ns(v.header.stamp) <= 0 ||
    v.step != v.width * channels || static_cast<uint64_t>(v.step) * v.height != v.data.size())
  {return Quality::Invalid;}
  return Quality::ValidAtObservation;
}
Quality validate_capabilities(const easyfly_msgs::srv::GetCapabilities::Response & v)
{
  if (v.names.size() > 64) {return Quality::BudgetExceeded;}
  if (v.names.size() != v.ready.size() || v.names.size() != v.implemented.size() ||
    v.names.size() != v.details.size()) {return Quality::Invalid;}
  std::set<std::string> names;
  std::size_t total = 0;
  for (std::size_t i = 0; i < v.names.size(); ++i) {
    total += v.details[i].size();
    if (v.names[i].size() > 256 || v.details[i].size() > 4096 || total > 65536) {
      return Quality::BudgetExceeded;
    }
    if (v.names[i].empty() || !names.insert(v.names[i]).second) {return Quality::Invalid;}
  }
  return Quality::ValidAtObservation;
}
bool basename(const std::string & s)
{
  return !s.empty() && s.size() <= 255 && s != "." && s != ".." &&
         s.find('/') == std::string::npos && s.find('\\') == std::string::npos &&
         std::none_of(s.begin(), s.end(), [](unsigned char c) {return c < 32 || c == 127;});
}
bool execution_id(const std::string & s)
{
  return s.size() == 32 && std::all_of(
    s.begin(), s.end(), [](char c) {
      return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
    });
}
Quality validate_event(const easyfly_msgs::msg::MissionStatus & v)
{
  if (v.message.size() > 2048 || v.phase_name.size() > 256) {return Quality::BudgetExceeded;}
  if (stamp_ns(v.header.stamp) <= 0 || v.header.frame_id.size() > 256 ||
    v.mission_phase > 8 || !finite({v.progress, v.estimated_time_remaining}) ||
    v.progress < 0 || v.progress > 1 || v.estimated_time_remaining < 0 ||
    v.current_waypoint < -1 || v.total_waypoints<0 ||
    v.current_waypoint> v.total_waypoints) {return Quality::Invalid;}
  return Quality::ValidAtObservation;
}
Quality validate_mission(
  const easyfly_msgs::srv::ManageMission::Response & v,
  Key key, const std::string & requested_id)
{
  if (v.message.size() > 2048 || v.status.size() > 256 || v.missions.size() > 256 ||
    v.current_status.message.size() > 2048 || v.current_status.phase_name.size() > 256 ||
    v.current_status.header.frame_id.size() > 256)
  {
    return Quality::BudgetExceeded;
  }
  std::set<std::string> names;
  for (const auto & name : v.missions) {
    if (!basename(name) || !names.insert(name).second) {return Quality::Invalid;}
  }
  if (v.execution_id.size() > 32) {return Quality::Invalid;}
  if (key == Key::MissionQuery) {
    if (!v.success) {return Quality::Unknown;}  // not_found proves no release.
    if (v.execution_id != requested_id || !execution_id(v.execution_id)) {return Quality::Invalid;}
    if (v.feedback_valid) {return validate_event(v.current_status);}
  }
  return v.success ? Quality::ValidAtObservation : Quality::Unknown;
}
Quality validate_perception(
  const easyfly_msgs::srv::GetPerception::Response & v,
  Key key, const ClockState & clock)
{
  if (v.message.size() > 2048 || v.source_topic.size() > 256 ||
    v.targets.targets.size() > 2000 || v.obstacles.obstacles.size() > 2000)
  {return Quality::BudgetExceeded;}
  if (v.status == v.UNSUPPORTED) {return Quality::Unsupported;}
  if (v.status == v.NOT_READY) {return Quality::NotReceived;}
  if (v.status == v.STALE) {return Quality::Stale;}
  if (v.status != v.VALID || !std::isfinite(v.age_sec) || v.age_sec < -.2) {
    return Quality::Invalid;
  }
  if (v.age_sec > 1.0) {return Quality::Stale;}
  const auto & header = key == Key::Targets ? v.targets.header : v.obstacles.header;
  const auto time_quality = source_quality(header, clock, 0, 1.0);
  if (time_quality != Quality::ValidAtObservation) {return time_quality;}
  if (v.observed_at != header.stamp) {return Quality::Invalid;}
  if (key == Key::Targets) {
    for (const auto & t : v.targets.targets) {
      if (t.class_name.size() > 256) {return Quality::BudgetExceeded;}
      if (source_quality(t.header, clock, 0, 1.0) != Quality::ValidAtObservation ||
        !finite(
          {t.confidence, t.position_x, t.position_y,
            t.position_z, t.distance}) || t.confidence < 0 || t.confidence > 1 ||
        !t.position_valid || t.distance < 0)
      {return Quality::Invalid;}
    }
  } else {
    for (const auto & o : v.obstacles.obstacles) {
      if (o.obstacle_type.size() > 256) {return Quality::BudgetExceeded;}
      if (source_quality(o.header, clock, 0, 1.0) != Quality::ValidAtObservation ||
        !finite(
          {o.center_x, o.center_y, o.center_z,
            o.size_x, o.size_y, o.size_z, o.velocity_x, o.velocity_y, o.velocity_z}) ||
        o.size_x < 0 || o.size_y < 0 || o.size_z < 0) {return Quality::Invalid;}
    }
  }
  return Quality::ValidAtObservation;
}
bool valid_json(const std::string & text)
{
  Json::Value root;
  return json(text, root);
}
bool parameter_json(const std::string & text, Key key)
{
  Json::Value root;
  if (!json(text, root)) {return false;}
  return (key == Key::Profile || key == Key::OperatorNote) ? root.isString() : root.isBool();
}
std::optional<bool> command_uncertain(const std::string & text)
{
  Json::Value root;
  if (!json(text, root) || !root.isObject() || !root["command_uncertain"].isBool()) {
    return std::nullopt;
  }
  return root["command_uncertain"].asBool();
}
void append_event(std::deque<Event> & events, Event event)
{
  event.text = event.text.substr(0, 2048);
  events.push_back(std::move(event));
  while (events.size() > 1000) {events.pop_front();}
}
void prune_trajectory(std::deque<TrajectoryPoint> & points, double now)
{
  while (points.size() > 4096 || (!points.empty() && now - points.front().steady > 300)) {
    points.pop_front();
  }
}
void append_trajectory(std::deque<TrajectoryPoint> & points, TrajectoryPoint point)
{
  points.push_back(point);
  prune_trajectory(points, point.steady);
}
}  // namespace detail
}  // namespace easyfly_ui_bridge
