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
bool json(const std::string & text, Json::Value & root, size_t budget = 4096)
{
  if (text.empty() || text.size() > budget) {return false;}
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
namespace
{
Quality execution_source_quality(int64_t source, const ClockState & clock, double receipt_age)
{
  if (source <= 0) {return Quality::Invalid;}
  if (clock.quality != Quality::ValidAtObservation) {return clock.quality;}
  const double age = static_cast<double>(clock.now_ns - source) / 1e9;
  if (age < -.2) {return Quality::Invalid;}
  return receipt_age > 2. || age > 2. ? Quality::Stale : Quality::ValidAtObservation;
}
}  // namespace
Quality validate_execution_overview(
  const std::string & text, MissionExecutionView * output, int64_t * source_ns)
{
  if (output) {*output = {};}
  if (source_ns) {*source_ns = 0;}
  if (text.size() > 400U * 1024U) {return Quality::BudgetExceeded;}
  Json::Value v;
  if (!json(text, v, 400U * 1024U) || !v.isObject() ||
    !v["schema_version"].isInt() || v["schema_version"].asInt() != 1 ||
    !v["stamp_ns"].isInt64() || v["stamp_ns"].asInt64() <= 0)
  {return Quality::Invalid;}
  for (const auto * field : {"goal_id", "name", "sha256", "content", "frame_id", "state"}) {
    if (!v[field].isString()) {return Quality::Invalid;}
  }
  for (const auto * field : {"current_step", "completed_steps", "total_steps"}) {
    if (!v[field].isInt()) {return Quality::Invalid;}
  }
  MissionExecutionView result;
  result.goal_id = v["goal_id"].asString(); result.name = v["name"].asString();
  result.hash = v["sha256"].asString(); result.content = v["content"].asString();
  result.frame = v["frame_id"].asString(); result.state = v["state"].asString();
  result.current_step = v["current_step"].asInt();
  result.completed_steps = v["completed_steps"].asInt();
  result.total_steps = v["total_steps"].asInt();
  if (!execution_id(result.goal_id) || !basename(result.name) || result.name.size() > 128 ||
    !easyfly_mission_fsm::missionTextUtf8(result.name) || result.frame != "odom" ||
    (result.state != "active" && result.state != "succeeded" &&
    result.state != "failed" && result.state != "canceled") ||
    result.total_steps < 1 || result.total_steps > 256 ||
    result.current_step < -1 || result.current_step >= result.total_steps ||
    result.completed_steps < 0 || result.completed_steps > result.total_steps ||
    (result.current_step >= 0 && result.current_step < result.completed_steps))
  {return Quality::Invalid;}
  if (result.content.size() > 65536 || result.hash.size() > 64) {return Quality::BudgetExceeded;}
  if (!easyfly_mission_fsm::missionTextUtf8(result.content) || result.hash.size() != 64 ||
    result.hash != easyfly_mission_fsm::missionContentSha256(result.content))
  {
    return Quality::Invalid;
  }
  auto plan = std::make_shared<easyfly_mission_fsm::MissionValidation>(
    easyfly_mission_fsm::validateMissionText(result.content));
  if (!plan->valid()) {return Quality::Invalid;}
  if (plan->steps.size() != static_cast<size_t>(result.total_steps)) {return Quality::Invalid;}
  result.plan = std::move(plan);
  if (source_ns) {*source_ns = v["stamp_ns"].asInt64();}
  if (output) {*output = std::move(result);}
  return Quality::ValidAtObservation;
}
void MissionExecutionBuffer::clear()
{
  observation_ = {}; source_.clear(); payload_.clear();
  source_ns_ = seen_ns_ = 0; epoch_ = 0; blocked_ = false;
}
void MissionExecutionBuffer::origin(const ClockState & clock, const std::string & source)
{
  const bool lost_source = !source_.empty() && source.empty();
  if (clock.epoch != epoch_ || source != source_) {
    clear(); epoch_ = clock.epoch; source_ = source;
  }
  observation_.evidence.endpoint = "/mission/execution_overview";
  observation_.evidence.time_epoch = clock.epoch;
  if (lost_source) {
    observation_.evidence.quality = Quality::Invalid;
    observation_.evidence.reason = "publisher disappeared or became ambiguous; origin cleared";
  }
}
void MissionExecutionBuffer::observe(
  const std::string & text, const ClockState & clock,
  double steady, const std::string & source)
{
  origin(clock, source);
  auto & e = observation_.evidence;
  e.received_steady = steady;
  if (!payload_.empty() && text == payload_) {return;}
  MissionExecutionView parsed; int64_t stamp = 0;
  const auto quality = validate_execution_overview(text, &parsed, &stamp);
  const auto timing = execution_source_quality(stamp, clock, 0);
  bool conflict = stamp <= seen_ns_;
  if (observation_.value && parsed.goal_id == observation_.value->goal_id) {
    const auto & previous = *observation_.value;
    conflict = conflict || parsed.name != previous.name || parsed.hash != previous.hash ||
      parsed.content != previous.content || parsed.completed_steps < previous.completed_steps ||
      (previous.state != "active" && parsed.state != previous.state) ||
      (parsed.state == "active" && parsed.current_step >= 0 &&
      parsed.current_step < previous.current_step);
  }
  if (quality != Quality::ValidAtObservation || timing != Quality::ValidAtObservation ||
    source.empty() || conflict)
  {
    blocked_ = true; e.quality = quality != Quality::ValidAtObservation ? quality :
      timing != Quality::ValidAtObservation ? timing : Quality::Invalid;
    e.reason = "execution context failed source, format, hash, progress or identity validation";
    if (timing == Quality::ValidAtObservation) {seen_ns_ = std::max(seen_ns_, stamp);}
    return;
  }
  source_ns_ = seen_ns_ = stamp; payload_ = text; blocked_ = false;
  observation_.value = std::make_shared<const MissionExecutionView>(std::move(parsed));
  e.first_source_steady = steady; e.quality = Quality::ValidAtObservation;
  e.reason = "native execution identity and exact content observed; display only";
}
Observation<MissionExecutionView> MissionExecutionBuffer::snapshot(
  const ClockState & clock, double steady, const std::string & source)
{
  origin(clock, source);
  auto & e = observation_.evidence;
  if (observation_.value) {
    e.receipt_age = steady - e.first_source_steady;
    e.source_age_available = clock.now_ns > 0 && source_ns_ > 0;
    e.source_age = e.source_age_available ? static_cast<double>(clock.now_ns - source_ns_) /
      1e9 : 0.;
    if (!blocked_) {e.quality = execution_source_quality(source_ns_, clock, e.receipt_age);}
    if (e.quality != Quality::ValidAtObservation) {
      e.reason = "historical execution context; no current target may be inferred";
    }
  }
  return observation_;
}
namespace
{
Quality grid_structure(const nav_msgs::msg::OccupancyGrid & m)
{
  const auto & p = m.info.origin.position;
  const auto & q = m.info.origin.orientation;
  const uint64_t cells = uint64_t(m.info.width) * m.info.height;
  if (m.info.width > 4096 || m.info.height > 4096 || cells > 1000000) {
    return Quality::BudgetExceeded;
  }
  if (!cells || cells != m.data.size() || m.header.frame_id != "odom" ||
    !finite({m.info.resolution, p.x, p.y, p.z, q.x, q.y, q.z, q.w}) ||
    m.info.resolution <= 0 || m.info.resolution > 100 ||
    std::abs(p.x) > 1e6 || std::abs(p.y) > 1e6 || std::abs(p.z) > 1e6 ||
    std::abs(q.x) > 1e-6 || std::abs(q.y) > 1e-6 ||
    std::abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1.) > .001)
  {return Quality::Invalid;}
  if (std::any_of(m.data.begin(), m.data.end(), [](int8_t v) {return v < -1 || v > 100;})) {
    return Quality::Invalid;
  }
  return Quality::ValidAtObservation;
}
}  // namespace
Quality validate_occupancy_grid(
  const nav_msgs::msg::OccupancyGrid & m, const ClockState & clock, double receipt_age)
{
  const auto structure = grid_structure(m);
  if (structure != Quality::ValidAtObservation) {return structure;}
  if (!std::isfinite(receipt_age) || receipt_age < 0) {return Quality::Invalid;}
  return source_quality(m.header, clock, receipt_age, 2.0);
}
void OccupancyGridBuffer::clear()
{
  candidate_.reset(); evidence_ = {}; source_.clear(); epoch_ = 0;
  high_water_ = 0; blocked_ = false;
}
void OccupancyGridBuffer::origin(const ClockState & clock, const std::string & source)
{
  const bool changed = source != source_ || clock.epoch != epoch_;
  const bool had_source = !source_.empty();
  if (changed) {clear(); source_ = source; epoch_ = clock.epoch;}
  evidence_.endpoint = "/occupancy_grid"; evidence_.time_epoch = clock.epoch;
  if (source.empty() && (had_source || changed)) {
    evidence_.quality = Quality::Invalid;
    evidence_.reason = "grid publisher missing or ambiguous; origin cleared";
  }
}
void OccupancyGridBuffer::observe(
  std::shared_ptr<const nav_msgs::msg::OccupancyGrid> message,
  const ClockState & clock, double steady, const std::string & source)
{
  origin(clock, source);
  if (!message) {return;}
  const auto structure = grid_structure(*message);
  const auto stamp = stamp_ns(message->header.stamp);
  const bool reset = structure == Quality::ValidAtObservation && stamp == 0 &&
    std::all_of(message->data.begin(), message->data.end(), [](int8_t v) {return v == -1;});
  if (reset || source.empty() || structure != Quality::ValidAtObservation || stamp <= 0 ||
    !std::isfinite(steady) || steady < 0)
  {
    candidate_.reset(); blocked_ = true;
    evidence_.received_steady = steady;
    evidence_.first_source_steady = evidence_.receipt_age = evidence_.source_age = 0;
    evidence_.source_age_available = false;
    if (reset) {high_water_ = 0;}
    evidence_.quality = structure == Quality::BudgetExceeded ? structure : Quality::Invalid;
    evidence_.reason = reset ? "upstream grid reset: zero stamp and all unknown; cleared" :
      "grid requires bounded planar odom geometry, positive time and a unique publisher";
    return;
  }
  if (stamp <= high_water_) {
    if (stamp < high_water_ || !candidate_ || *candidate_ != *message) {
      blocked_ = true; evidence_.quality = Quality::Invalid;
      evidence_.reason = "grid source stamp regressed or duplicate payload conflicts";
    }
    // Identical duplicates never renew either receipt timestamp or unblock conflicts.
    return;
  }
  high_water_ = stamp; candidate_ = std::move(message); blocked_ = false;
  evidence_.received_steady = evidence_.first_source_steady = steady;
  evidence_.receipt_age = 0;
  evidence_.quality = source_quality(candidate_->header, clock, 0, 2.0);
  blocked_ = evidence_.quality == Quality::Invalid;
  evidence_.reason =
    "original obstacle grid; includes upstream inflation/persistence; display only";
}
Observation<nav_msgs::msg::OccupancyGrid> OccupancyGridBuffer::snapshot(
  const ClockState & clock, double steady, const std::string & source)
{
  origin(clock, source);
  if (candidate_) {
    evidence_.receipt_age = steady - evidence_.first_source_steady;
    evidence_.source_age_available = clock.now_ns > 0;
    evidence_.source_age = evidence_.source_age_available ?
      static_cast<double>(clock.now_ns - stamp_ns(candidate_->header.stamp)) / 1e9 : 0.;
    if (!blocked_ && !source.empty()) {
      evidence_.quality = !std::isfinite(evidence_.receipt_age) || evidence_.receipt_age < 0 ?
        Quality::Invalid : source_quality(candidate_->header, clock, evidence_.receipt_age, 2.0);
      if (evidence_.quality == Quality::Invalid) {blocked_ = true;}
    }
  }
  return {evidence_.quality == Quality::ValidAtObservation ? candidate_ : nullptr, evidence_};
}
Quality validate_global_position(
  const sensor_msgs::msg::NavSatFix & m, const ClockState & clock, double receipt_age)
{
  using Status = sensor_msgs::msg::NavSatStatus;
  if (m.status.status < Status::STATUS_FIX || m.status.status > Status::STATUS_GBAS_FIX ||
    !finite({m.latitude, m.longitude, m.altitude}) ||
    std::abs(m.latitude) > 90 || std::abs(m.longitude) > 180 ||
    m.header.frame_id.size() > 256)
  {return Quality::Invalid;}
  return source_quality(m.header, clock, receipt_age, 2.0, false);
}
void GlobalPositionBuffer::clear()
{
  observation_ = {}; points_.clear(); source_.clear(); epoch_ = 0;
  segment_ = 0; last_source_ = 0; discontinuity_ = true; blocked_ = false;
}
void GlobalPositionBuffer::origin(const ClockState & clock, const std::string & source)
{
  const bool lost_source = !source_.empty() && source.empty();
  if (clock.epoch != epoch_ || source != source_) {
    clear(); epoch_ = clock.epoch; source_ = source;
  }
  observation_.evidence.endpoint = "/mavros/global_position/global";
  observation_.evidence.time_epoch = clock.epoch;
  if (lost_source) {
    observation_.evidence.quality = Quality::Invalid;
    observation_.evidence.reason = "publisher disappeared or became ambiguous; origin cleared";
  }
}
void GlobalPositionBuffer::observe(
  std::shared_ptr<const sensor_msgs::msg::NavSatFix> message,
  const ClockState & clock, double steady, const std::string & source)
{
  origin(clock, source);
  if (!message) {return;}
  const auto stamp = stamp_ns(message->header.stamp);
  auto & e = observation_.evidence;
  e.received_steady = steady;
  if (observation_.value && message->header.frame_id != observation_.value->header.frame_id) {
    points_.clear(); discontinuity_ = true; last_source_ = 0;
  }
  if (stamp <= last_source_ && observation_.value) {
    discontinuity_ = true;
    if (stamp < last_source_ || *message != *observation_.value) {
      blocked_ = true; e.quality = Quality::Invalid;
      e.reason = "GPS source stamp regressed or duplicate payload conflicts";
    }
    return;
  }
  observation_.value = std::move(message);
  e.first_source_steady = steady;
  e.quality = source.empty() ? Quality::Invalid :
    validate_global_position(*observation_.value, clock, 0);
  blocked_ = e.quality != Quality::ValidAtObservation;
  if (source_quality(observation_.value->header, clock, 0, 2., false) ==
    Quality::ValidAtObservation) {last_source_ = stamp;}
  if (blocked_) {
    discontinuity_ = true; e.reason = "GPS requires a unique source, valid fix and source time";
    return;
  }
  if (!points_.empty() && (steady - points_.back().steady > 2. ||
    stamp - points_.back().source_ns > 2000000000LL)) {discontinuity_ = true;}
  if (discontinuity_) {++segment_; discontinuity_ = false;}
  points_.push_back(
    {stamp, steady, observation_.value->latitude,
      observation_.value->longitude, observation_.value->altitude, segment_});
  e.reason = "original GPS fix and acquisition time checked; display only";
  while (points_.size() > 4096 || (!points_.empty() && steady - points_.front().steady > 300.)) {
    points_.pop_front();
  }
}
Observation<sensor_msgs::msg::NavSatFix> GlobalPositionBuffer::snapshot(
  const ClockState & clock, double steady, const std::string & source)
{
  origin(clock, source);
  auto & e = observation_.evidence;
  if (observation_.value) {
    e.receipt_age = steady - e.first_source_steady;
    e.source_age_available = clock.now_ns > 0 && stamp_ns(observation_.value->header.stamp) > 0;
    e.source_age = e.source_age_available ?
      static_cast<double>(clock.now_ns - stamp_ns(observation_.value->header.stamp)) / 1e9 : 0.;
    e.quality = source.empty() || blocked_ ? Quality::Invalid :
      validate_global_position(*observation_.value, clock, e.receipt_age);
    if (e.quality != Quality::ValidAtObservation) {discontinuity_ = true;}
  }
  while (!points_.empty() && steady - points_.front().steady > 300.) {points_.pop_front();}
  return observation_;
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
    if (std::isfinite(speed)) {
      result.speed_mps = speed;
      result.horizontal_speed_mps = std::hypot(v.velocity_x, v.velocity_y);
      result.vertical_speed_mps = v.velocity_z;
    } else {result.velocity_valid = false;}
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
