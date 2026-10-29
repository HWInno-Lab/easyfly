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
#pragma once
#include <map>
#include <memory>
#include <string>
#include <easyfly_msgs/action/takeoff.hpp>
#include <easyfly_msgs/action/land.hpp>
#include <easyfly_msgs/action/precision_land.hpp>
#include <easyfly_msgs/action/go_to_waypoint.hpp>
#include <easyfly_msgs/action/navigate_to.hpp>
#include <easyfly_msgs/action/follow_trajectory.hpp>
#include <easyfly_msgs/action/track_target.hpp>
#include <easyfly_msgs/action/execute_mission.hpp>
#include <easyfly_msgs/action/search_area.hpp>
#include <easyfly_msgs/action/explore.hpp>
#include <easyfly_msgs/msg/uav_state.hpp>
namespace easyfly_core
{
enum class Admission { Pending, Accepted, Rejected, Unavailable, Unsupported, CapacityExceeded,
  Closed };
enum class Status { Pending, Succeeded, Failed, Canceled, Rejected, Unavailable, Unsupported,
  CapacityExceeded, NotSent };
enum class LocalStatus { Active, AdmissionTimedOut, Closed, TransportError };
enum class CancelStatus { None, Requested, Accepted, Rejected, TransportError };
struct Feedback
{
  std::map<std::string, double> numbers;
  std::map<std::string, bool> flags;
  std::map<std::string, std::string> strings;
};
struct OperationSnapshot
{
  Admission admission{Admission::Pending};
  Status status{Status::Pending};
  LocalStatus local_status{LocalStatus::Active};
  CancelStatus cancel_status{CancelStatus::None};
  bool remote_execution_pending{true};
  bool success{false};
  bool action_terminal_received{false};
  double result_received_seconds{0.0};  // monotonic steady-clock epoch, not ROS time
  std::string goal_id;
  std::string message, local_message, cancel_message;
  Feedback feedback;
  double feedback_received_seconds{0.0};  // steady-clock receipt; never a source timestamp
};
struct StateSnapshot
{
  easyfly_msgs::msg::UAVState value;
  bool received{false}, fresh{false}, receipt_fresh{false}, source_time_valid{false};
  double receipt_age_seconds{0.0};
  double source_age_seconds{0.0};
};
struct ClientOptions
{
  std::size_t max_pending_operations{32};
  double state_freshness_seconds{1.0};
  double admission_timeout_seconds{5.0};
  // Independent admission policy, not Operation::wait's observation timeout.
  bool cancel_on_admission_timeout{false};
  bool use_sim_time{false};
  bool require_managed_ownership{false};
};
struct Core;
class Operation
{
public:
  OperationSnapshot snapshot() const;
  // Observation only. On a closed client, the last terminal operation also
  // observes runtime cleanup within this same budget; timeout never fabricates
  // a remote result or sends cancellation.
  bool wait(double timeout_seconds) const;
  bool cancel();  // queued request, NEVER a statement that execution stopped

private:
  friend struct Core;
  friend class Client;
  struct Data;
  explicit Operation(std::shared_ptr<Data> data);
  std::shared_ptr<Data> data_;
};
class Client
{
public:
  explicit Client(const ClientOptions & options = ClientOptions{});
  ~Client();
  Client(const Client &) = delete;
  Client & operator=(const Client &) = delete;
  bool running() const;
  bool server_ready(const std::string & action) const;
  StateSnapshot state() const;
  std::size_t pending_operations() const;
  // Freeze/unsubscribe state, stop admission, cancel exact owned handles.
  // False means remote ownership or runtime cleanup is still unresolved.
  bool close(double timeout_seconds = 0.0);
  std::shared_ptr<Operation> takeoff(const easyfly_msgs::action::Takeoff::Goal &);
  std::shared_ptr<Operation> land(const easyfly_msgs::action::Land::Goal &);
  std::shared_ptr<Operation> precision_land(const easyfly_msgs::action::PrecisionLand::Goal &);
  std::shared_ptr<Operation> go_to(const easyfly_msgs::action::GoToWaypoint::Goal &);
  std::shared_ptr<Operation> navigate(const easyfly_msgs::action::NavigateTo::Goal &);
  std::shared_ptr<Operation> follow(const easyfly_msgs::action::FollowTrajectory::Goal &);
  std::shared_ptr<Operation> track(const easyfly_msgs::action::TrackTarget::Goal &);
  std::shared_ptr<Operation> execute_mission(const easyfly_msgs::action::ExecuteMission::Goal &);
  std::shared_ptr<Operation> search(const easyfly_msgs::action::SearchArea::Goal &);
  std::shared_ptr<Operation> explore(const easyfly_msgs::action::Explore::Goal &);
  std::shared_ptr<Operation> unsupported(const std::string & capability);

private:
  struct Worker;
  std::unique_ptr<Worker> worker_;
  std::shared_ptr<Core> core_;
};
}  // namespace easyfly_core
