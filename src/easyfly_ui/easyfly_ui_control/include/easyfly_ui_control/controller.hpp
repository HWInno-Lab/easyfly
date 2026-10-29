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
#ifndef EASYFLY_UI_CONTROL__CONTROLLER_HPP_
#define EASYFLY_UI_CONTROL__CONTROLLER_HPP_
#include <memory>
#include <optional>
#include <string>
#include <easyfly_msgs/srv/get_mission_details.hpp>
namespace easyfly_ui_control
{
enum class Phase {Idle, Sent, Uncertain, Running, CancelPending, DrainPending, Completed, Blocked};
struct Intent
{
  std::string scope, instance, request, name, hash, execution, goal;
  bool cancel_dispatched{false}, resolved{false};
};
struct ControlSnapshot
{
  Phase phase{Phase::Idle};
  bool stopped{false}, session_ready{false}, request_pending{false};
  std::string server_instance, status, message;
  double session_received{0};
  std::optional<Intent> intent;
};
struct ControlConfig
{
  // Explicit dedicated absolute directory. Created mode0700; never use an ephemeral default.
  std::string journal_directory;
  double response_timeout{2.0};
};
double steady_seconds();
// Diagnostic correlation only: bounded identifiers, no mission body/path or credentials.
// Logs are not authentication, durable flight authorization, or remote execution evidence.
void audit_event(
  const char * event, const std::string & confirmation = {}, const std::string & instance = {},
  const std::string & request = {}, const std::string & content_hash = {});
// Separate from the read-only Observer. The desktop creates this only on explicit opt-in.
// Construction/restoration sends reads only.
class MissionControl
{
public:
  // Local directory/lock acquisition errors throw before creating a ROS context or worker.
  // Existing record/schema/scope failures still surface as Blocked during restoration.
  explicit MissionControl(const ControlConfig & config);
  ~MissionControl();
  MissionControl(const MissionControl &) = delete;
  MissionControl & operator=(const MissionControl &) = delete;
  std::shared_ptr<const ControlSnapshot> snapshot() const;
  // For explicit operator confirmation, not discovery/telemetry callbacks.
  // Review must have been received <=2 steady seconds ago. Revalidated in worker.
  // Legacy overload refuses dispatch without a confirmation identity.
  bool start_confirmed(
    const easyfly_msgs::srv::GetMissionDetails::Response & review,
    double review_received, const std::string & confirmed_server_instance);
  // Required confirmation identity, consumed once per facade; empty IDs are refused.
  bool start_confirmed(
    const easyfly_msgs::srv::GetMissionDetails::Response & review,
    double review_received, const std::string & confirmed_server_instance,
    const std::string & confirmation_id);
  // Explicit confirmation, exact locally registered instance/request/execution only.
  bool cancel_confirmed();
  // Stops local communication only; never cancels, clears the journal, or infers remote stop.
  void stop();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace easyfly_ui_control
#endif  // EASYFLY_UI_CONTROL__CONTROLLER_HPP_
