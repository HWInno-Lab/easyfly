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

#ifndef EASYFLY_UI_BRIDGE__MODEL_HPP_
#define EASYFLY_UI_BRIDGE__MODEL_HPP_

#include <array>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <string>

#include <easyfly_core/client.hpp>
#include <easyfly_msgs/msg/mission_status.hpp>
#include <easyfly_msgs/srv/get_capabilities.hpp>
#include <easyfly_msgs/srv/get_perception.hpp>
#include <easyfly_msgs/srv/manage_mission.hpp>
#include <easyfly_msgs/srv/get_mission_details.hpp>
#include <easyfly_mission_format/mission_format.hpp>
#include <easyfly_msgs/srv/query_params.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <easyfly_msgs/msg/detection_info_array.hpp>
#include <easyfly_msgs/msg/track_info_array.hpp>
#include <easyfly_msgs/msg/target3_d_array.hpp>
#include <easyfly_msgs/msg/ellipse_observation_array.hpp>
#include <easyfly_msgs/msg/color_line_observation_array.hpp>
#include <easyfly_msgs/msg/localization_status.hpp>

namespace easyfly_ui_bridge
{
// Display evidence only; none of these values authorize a flight operation.
enum class Quality
{
  NotReceived, ValidAtObservation, Stale, Invalid, ClockNotReady, ClockPaused,
  ClockReset, Unavailable, Unsupported, BudgetExceeded, Unknown
};
enum class Key
{
  FlightCapabilities, MissionCapabilities, PerceptionCapabilities, LandingCapabilities,
  MissionList, MissionValidation, MissionQuery, MissionDetails, Profile, UseSimTime, OperatorNote,
  ExportPretty, Targets, Obstacles
};
struct Evidence
{
  Quality quality{Quality::NotReceived};
  std::string endpoint;
  std::string reason;
  uint64_t request_id{0};
  uint64_t time_epoch{0};
  double received_steady{0};
  double first_source_steady{0};
  double receipt_age{0};
  double source_age{0};
  bool source_age_available{false};  // Header-less/unknown source age must display as unavailable.
};
template<typename T>
struct Observation
{
  std::shared_ptr<const T> value;
  Evidence evidence;
};
struct ClockState
{
  Quality quality{Quality::ClockNotReady};
  int64_t now_ns{0};
  uint64_t epoch{0};
};
class ClockWatch
{
public:
  explicit ClockWatch(bool simulation);
  bool observe(int64_t source_ns, double steady);  // true on backwards jump
  ClockState state(double steady, int64_t system_ns = 0) const;

private:
  bool simulation_;
  bool received_{false};
  bool reset_{false};
  int64_t now_ns_{0};
  double progressed_at_{0};
  uint64_t epoch_{0};
};
struct StateView
{
  easyfly_core::StateSnapshot sdk;
  Evidence evidence;
  bool position_valid{false};
  bool velocity_valid{false};
  bool attitude_valid{false};
  bool battery_valid{false};
  bool gps_valid{false};
  std::optional<double> speed_mps;
  std::optional<double> battery_percent;
};
struct TrajectoryPoint
{
  int64_t source_ns{0};
  double steady{0};
  double x{0}, y{0}, z{0};
};
struct Event
{
  double steady{0};
  uint64_t epoch{0};
  std::string text;
  uint64_t observation_session_id{0};
};
struct Counters
{
  uint64_t rejected_requests{0}, timed_out{0}, discarded_replies{0};
  uint64_t dropped_images{0}, duplicate_samples{0}, invalid_samples{0};
};
struct PerceptionReport
{
  bool received{false};
  uint8_t upstream_status{0};
  std::string source_topic, message;
  builtin_interfaces::msg::Time observed_at;
  double reported_age_sec{0};
  Evidence evidence;
};
// Explicit one-camera pipeline association. Topic names alone are not proof
// that unrelated producers refer to the same camera. No automatic remapping.
struct CameraPerceptionBinding
{
  std::string image_topic, detections_topic, tracks_topic, targets_topic;
  std::string target_frame{"odom"};
  std::string ellipse_topic, color_line_topic;
};
struct CameraObservations
{
  CameraPerceptionBinding binding;
  Observation<easyfly_msgs::msg::DetectionInfoArray> detections;
  Observation<easyfly_msgs::msg::TrackInfoArray> tracks;
  Observation<easyfly_msgs::msg::Target3DArray> targets;
  Observation<sensor_msgs::msg::Image> detection_image, track_image;
  Observation<easyfly_msgs::msg::EllipseObservationArray> ellipses;
  Observation<easyfly_msgs::msg::ColorLineObservationArray> color_lines;
  Observation<sensor_msgs::msg::Image> ellipse_image, color_line_image;
  size_t cached_images{0}, cached_bytes{0};
};
struct UiSnapshot
{
  uint64_t observation_session_id{0}, time_epoch{0}, revision{0};
  double produced_steady{0};
  bool running{false}, stopped{false};
  std::string local_error;
  std::string selected_profile;  // Not a confirmed backend profile.
  ClockState clock;
  // Read-only localization telemetry; never navigation or flight admission.
  std::string selected_localization_backend;
  Observation<easyfly_msgs::msg::LocalizationStatus> localization;
  StateView state;
  // Read-only DDS graph evidence, sampled on the bridge worker at most once/second.
  double state_graph_observed{0};
  double backend_graph_observed{0};
  std::map<std::string, bool> task_backends;
  std::size_t state_publishers{0}, state_matching_publishers{0};
  std::map<Key, Observation<easyfly_msgs::srv::GetCapabilities::Response>> capabilities;
  std::map<Key, Observation<easyfly_msgs::srv::ManageMission::Response>> missions;
  Observation<easyfly_msgs::srv::GetMissionDetails::Response> mission_details;
  std::shared_ptr<const easyfly_mission_fsm::MissionValidation> mission_plan;
  std::string mission_format;  // Classified on the bridge worker, not the GUI.
  std::map<Key, Observation<easyfly_msgs::srv::QueryParams::Response>> parameters;
  std::map<Key, Observation<easyfly_msgs::srv::GetPerception::Response>> perception;
  // Retain bounded upstream envelope even when geometry cannot be retained.
  std::map<Key, PerceptionReport> perception_reports;
  Observation<easyfly_msgs::msg::MissionStatus> last_mission_event;
  std::array<Observation<sensor_msgs::msg::Image>, 2> images;
  std::array<CameraObservations, 2> camera_perception;
  std::deque<TrajectoryPoint> trajectory;
  std::deque<Event> events;
  // Historical warning, never cleared by telemetry, invalid JSON or disconnect.
  std::optional<Event> command_uncertain_observed;
  std::size_t pending_queries{0};
  Counters counters;
};

namespace detail
{
double steady_seconds();
int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp);
Quality source_quality(
  const std_msgs::msg::Header & header, const ClockState & clock,
  double first_receipt_age, double lifetime, bool require_odom = true);
StateView state_view(
  const easyfly_core::StateSnapshot & sdk, const ClockState & clock,
  double first_received, double steady, bool duplicate_conflict);
Quality validate_localization(
  const easyfly_msgs::msg::LocalizationStatus & message,
  const ClockState & clock, double receipt_age);
Quality validate_image(const sensor_msgs::msg::Image & image);
Quality validate_capabilities(const easyfly_msgs::srv::GetCapabilities::Response & response);
Quality validate_mission(
  const easyfly_msgs::srv::ManageMission::Response & response,
  Key key, const std::string & requested_id);
Quality validate_event(const easyfly_msgs::msg::MissionStatus & message);
Quality validate_mission_details(
  const easyfly_msgs::srv::GetMissionDetails::Response & response,
  const std::string & requested_name, easyfly_mission_fsm::MissionValidation * plan = nullptr);
bool catalog_name(const std::string & text);
Quality validate_perception(
  const easyfly_msgs::srv::GetPerception::Response & response,
  Key key, const ClockState & clock);
bool valid_json(const std::string & text);
bool parameter_json(const std::string & text, Key key);
std::optional<bool> command_uncertain(const std::string & text);
bool basename(const std::string & text);
bool execution_id(const std::string & text);
void append_event(std::deque<Event> & events, Event event);
void append_trajectory(std::deque<TrajectoryPoint> & points, TrajectoryPoint point);
void prune_trajectory(std::deque<TrajectoryPoint> & points, double now);
}  // namespace detail
}  // namespace easyfly_ui_bridge
#endif  // EASYFLY_UI_BRIDGE__MODEL_HPP_
