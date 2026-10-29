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

#ifndef EASYFLY_UI_BRIDGE__OBSERVER_HPP_
#define EASYFLY_UI_BRIDGE__OBSERVER_HPP_

#include <memory>
#include <string>

#include "easyfly_ui_bridge/model.hpp"

namespace easyfly_ui_bridge
{
struct BridgeConfig
{
  bool use_sim_time{true};
  bool poll_observations{true};
  std::string selected_profile{"navigation_dev"};
  std::string selected_localization_backend{"external"};
  bool localization_navigation_enabled{false};
  std::string landing_optical_frame{"camera_down_optical_frame"};
  std::array<std::string, 2> image_topics{
    "/camera/color/image_raw", "/camera_down/color/image_raw"};
  std::array<CameraPerceptionBinding, 2> camera_perception{{
    {"/camera/color/image_raw", "/detections", "/tracks_raw", "/target_positions", "odom", "", ""},
    {}}};
};
enum class RequestStatus {Queued, Busy, RateLimited, Invalid, Closed};
struct RequestReceipt
{
  uint64_t id{0};
  RequestStatus status{RequestStatus::Closed};
};
enum class CapabilitySource {Flight, Mission, Perception, Landing};
enum class Parameter {Profile, UseSimTime, OperatorNote, ExportPretty};
enum class PerceptionKind {Targets, Obstacles};

// Starts local observers only. ROS domain is inherited from the launch environment
// by both contexts, never changed by this library. Reconfigure by stopping and
// constructing a new Observer (a new session ID). No SDK/control facade is exposed.
class Observer
{
public:
  // Reconnect with a stopped previous snapshot to retain bounded warning/event
  // history only, never telemetry, service cache, identity or pending requests.
  explicit Observer(
    const BridgeConfig & config = BridgeConfig{},
    std::shared_ptr<const UiSnapshot> previous_session = nullptr);
  ~Observer();
  Observer(const Observer &) = delete;
  Observer & operator=(const Observer &) = delete;
  std::shared_ptr<const UiSnapshot> snapshot() const;
  RequestReceipt capabilities(CapabilitySource source);
  RequestReceipt list_missions();
  RequestReceipt validate_mission(const std::string & basename);
  RequestReceipt mission_details(const std::string & basename);
  RequestReceipt query_mission(const std::string & service_execution_id);
  RequestReceipt read_parameter(Parameter parameter);
  RequestReceipt perception(PerceptionKind kind);
  // Nonblocking: poll snapshot()->stopped for actual cleanup completion. The last
  // owner/destructor joins local cleanup; desktop must release it after stopped,
  // or off the GUI thread. No remote operation exists in this read-only facade.
  void stop();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace easyfly_ui_bridge
#endif  // EASYFLY_UI_BRIDGE__OBSERVER_HPP_
