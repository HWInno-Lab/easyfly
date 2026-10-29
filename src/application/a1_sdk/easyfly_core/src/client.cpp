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
#include <chrono>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <atomic>
#include "easyfly_core/client.hpp"
#include "easyfly_core/ownership_registry.hpp"
#include <easyfly_msgs/srv/get_action_ownership.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
namespace easyfly_core
{
using Clock = std::chrono::steady_clock;
struct Operation::Data
{
  mutable std::mutex mutex;
  mutable std::condition_variable changed;
  OperationSnapshot value;
  std::weak_ptr<Core> core;
  bool cancel_requested{false}, cancel_sent{false};
  Clock::time_point cancel_retry_not_before{};
  Clock::time_point submitted{Clock::now()};
  std::function<void()> cancel_exact;
  bool managed_ownership{false}, ownership_inflight{false};
  int64_t ownership_request_id{0};
  Clock::time_point ownership_deadline{}, ownership_retry{};
  std::weak_ptr<rclcpp::Client<easyfly_msgs::srv::GetActionOwnership>> ownership_client;
};
static Feedback feedback(const easyfly_msgs::action::Takeoff::Feedback & f)
{
  return {{{"current_altitude", f.current_altitude}, {"progress", f.progress}}, {}, {}};
}
static Feedback feedback(const easyfly_msgs::action::Land::Feedback & f)
{
  return {{{"current_altitude", f.current_altitude}}, {}, {}};
}
static Feedback feedback(const easyfly_msgs::action::PrecisionLand::Feedback & f)
{
  return {{{"horizontal_error_m", f.horizontal_error_m},
    {"height_above_target_m", f.height_above_target_m}},
    {{"target_valid", f.target_valid}}, {{"phase", f.phase}}};
}
static Feedback feedback(const easyfly_msgs::action::GoToWaypoint::Feedback & f)
{
  return {{{"distance_remaining", f.distance_remaining}, {"progress", f.progress}}, {}, {}};
}
static Feedback feedback(const easyfly_msgs::action::NavigateTo::Feedback & f)
{
  return {{{"distance_remaining", f.distance_remaining}, {"current_segment", f.current_segment},
    {"total_segments", f.total_segments}}, {}, {}};
}
static Feedback feedback(const easyfly_msgs::action::FollowTrajectory::Feedback & f)
{
  return {{{"distance_remaining", f.distance_remaining}}, {{"stream_active", f.stream_active}}, {}};
}
static Feedback feedback(const easyfly_msgs::action::TrackTarget::Feedback & f)
{
  return {{{"distance_to_target", f.distance_to_target},
    {"current_target_id", f.current_target_id}},
    {{"target_visible", f.target_visible}}, {}};
}
static Feedback feedback(const easyfly_msgs::action::ExecuteMission::Feedback & f)
{
  return {{{"mission_phase", f.current_status.mission_phase},
    {"progress", f.current_status.progress},
    {"current_waypoint", f.current_status.current_waypoint},
    {"total_waypoints", f.current_status.total_waypoints}},
    {{"mission_success", f.current_status.mission_success}},
    {{"phase_name", f.current_status.phase_name}, {"message", f.current_status.message}}};
}
static Feedback feedback(const easyfly_msgs::action::SearchArea::Feedback & f)
{
  return {{{"area_covered", f.area_covered}, {"progress", f.progress},
    {"targets_found", f.targets_found}}, {}, {}};
}
static Feedback feedback(const easyfly_msgs::action::Explore::Feedback & f)
{
  return {{{"coverage", f.coverage}, {"progress", f.coverage},
    {"completed_viewpoints", f.completed_viewpoints}, {"next_x", f.next_viewpoint.x},
    {"next_y", f.next_viewpoint.y}, {"next_z", f.next_viewpoint.z}}, {}, {{"phase", f.phase}}};
}
template<typename Result>
static std::string result_message(const Result & result) {return result.message;}
static std::string result_message(const easyfly_msgs::action::Explore::Result & result)
{return result.reason;}
struct Core : std::enable_shared_from_this<Core>
{
  ClientOptions options;
  std::shared_ptr<rclcpp::Context> context;
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  rclcpp::Subscription<easyfly_msgs::msg::UAVState>::SharedPtr subscription;
  mutable std::mutex mutex;
  std::condition_variable stopped_event;
  bool closed{false}, stopped{false};
  std::thread::id executor_thread;
  StateSnapshot state_value;
  Clock::time_point state_receipt;
  std::vector<std::shared_ptr<Operation::Data>> pending;
  std::deque<std::function<void()>> inbox;
  std::map<std::string, std::shared_ptr<void>> clients;
  std::map<std::string, std::function<bool()>> ready;
  std::map<std::string, rclcpp::Client<easyfly_msgs::srv::GetActionOwnership>::SharedPtr> ownership;
  template<class Action> void add(const std::string & name, const std::string & topic)
  {
    auto client = rclcpp_action::create_client<Action>(node, topic);
    clients[name] = client;
    ready[name] = [client] {return client->action_server_is_ready();};
  }
  void initialize()
  {
    context = std::make_shared<rclcpp::Context>();
    context->init(0, nullptr);
    rclcpp::NodeOptions no;
    no.context(context).use_global_arguments(false);
    no.parameter_overrides({rclcpp::Parameter("use_sim_time", options.use_sim_time)});
    static std::atomic<unsigned> serial{0};
    node = std::make_shared<rclcpp::Node>("easyfly_sdk_" + std::to_string(++serial), no);
    rclcpp::ExecutorOptions eo;
    eo.context = context;
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(eo);
    executor->add_node(node);
    add<easyfly_msgs::action::Takeoff>("takeoff", "/takeoff");
    add<easyfly_msgs::action::Land>("land", "/land");
    add<easyfly_msgs::action::PrecisionLand>("precision_land", "/precision_land");
    add<easyfly_msgs::action::GoToWaypoint>("goto", "/goto_waypoint");
    add<easyfly_msgs::action::NavigateTo>("navigate", "/navigate");
    add<easyfly_msgs::action::FollowTrajectory>("follow", "/follow_trajectory");
    add<easyfly_msgs::action::TrackTarget>("track", "/track_target");
    add<easyfly_msgs::action::ExecuteMission>("execute_mission", "/execute_mission");
    add<easyfly_msgs::action::SearchArea>("search", "/search_area");
    add<easyfly_msgs::action::Explore>("explore", "/explore");
    for (const auto & entry:
      std::map<std::string, std::string>{{"navigate", "/navigation/ownership"},
        {"search", "/search/ownership"}, {"execute_mission", "/mission/ownership"},
        {"precision_land", "/precision_landing/ownership"},
        {"explore", "/exploration/ownership"}})
    {
      auto service = node->create_client<easyfly_msgs::srv::GetActionOwnership>(entry.second);
      ownership[entry.first] = service;
      auto action_ready = ready.at(entry.first);
      const bool required = options.require_managed_ownership;
      ready[entry.first] = [service, action_ready, required] {
          return action_ready() && (!required || service->service_is_ready());
        };
    }
    const auto weak = weak_from_this();
    subscription = node->create_subscription<easyfly_msgs::msg::UAVState>(
      "/uav_state",
      rclcpp::SensorDataQoS(), [weak](easyfly_msgs::msg::UAVState::SharedPtr message) {
        if (auto self = weak.lock()) {
          std::lock_guard<std::mutex> lock(self->mutex);
          if (self->closed) {return;}
          self->state_value.value = *message;
          self->state_value.received = true;
          self->state_receipt = Clock::now();
        }
      });
  }
  static void finish(
    const std::shared_ptr<Operation::Data> & data, Status status,
    const std::string & message, bool actual_result = false)
  {
    std::lock_guard<std::mutex> lock(data->mutex);
    data->value.status = status;
    data->value.message = message;
    data->value.success = status == Status::Succeeded;
    data->value.action_terminal_received = actual_result;
    if (actual_result) {
      data->value.result_received_seconds = std::chrono::duration<double>(
        Clock::now().time_since_epoch()).count();
    }
    data->value.remote_execution_pending = actual_result && data->managed_ownership;
    data->cancel_exact = {};
    data->changed.notify_all();
  }
  std::shared_ptr<Operation> immediate(
    Admission admission, Status status,
    const std::string & message)
  {
    auto data = std::make_shared<Operation::Data>();
    data->value.admission = admission;
    finish(data, status, message);
    return std::shared_ptr<Operation>(new Operation(data));
  }
  template<class Action> std::shared_ptr<Operation> submit(
    const std::string & name,
    typename Action::Goal goal)
  {
    std::lock_guard<std::mutex> lock(mutex);
    if (closed) {return immediate(Admission::Closed, Status::NotSent, "client_closed");}
    pending.erase(
      std::remove_if(
        pending.begin(), pending.end(), [](const auto & op) {
          std::lock_guard<std::mutex> lock(op->mutex);
          return !op->value.remote_execution_pending;
        }), pending.end());
    if (pending.size() >= options.max_pending_operations) {
      return immediate(
        Admission::CapacityExceeded, Status::CapacityExceeded,
        "pending_capacity_exceeded");
    }
    auto data = std::make_shared<Operation::Data>();
    data->core = shared_from_this();
    pending.push_back(data);
    const auto weak = weak_from_this();
    inbox.push_back(
      [weak, data, goal, name] {
        if (auto self = weak.lock()) {self->send<Action>(name, goal, data);}
      });
    return std::shared_ptr<Operation>(new Operation(data));
  }
  template<class Action> void send(
    const std::string & name, const typename Action::Goal & goal,
    const std::shared_ptr<Operation::Data> & data)
  {
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (closed) {
        {std::lock_guard<std::mutex> op_lock(data->mutex);
          data->value.admission = Admission::Closed;}
        finish(data, Status::NotSent, "closed_before_send");
        return;
      }
    }
    auto client = std::static_pointer_cast<rclcpp_action::Client<Action>>(clients.at(name));
    if (!ready.at(name)()) {
      {std::lock_guard<std::mutex> lock(data->mutex);
        data->value.admission = Admission::Unavailable;}
      finish(data, Status::Unavailable, "action_server_unavailable");
      return;
    }
    if (ownership.count(name)) {
      std::lock_guard<std::mutex> lock(data->mutex);
      auto service = ownership.at(name);
      data->ownership_client = service;
      data->managed_ownership = options.require_managed_ownership || service->service_is_ready();
    }
    using Handle = rclcpp_action::ClientGoalHandle<Action>;
    typename rclcpp_action::Client<Action>::SendGoalOptions so;
    const std::weak_ptr<Operation::Data> weak_data(data);
    so.goal_response_callback = [weak_data, client](typename Handle::SharedPtr handle) {
        const auto op = weak_data.lock();
        if (!op) {return;}
        if (!handle) {
          {std::lock_guard<std::mutex> lock(op->mutex); op->value.admission = Admission::Rejected;}
          Core::finish(op, Status::Rejected, "goal_rejected");
          return;
        }
        std::lock_guard<std::mutex> lock(op->mutex);
        op->value.admission = Admission::Accepted;
        op->value.goal_id = goalIdentity(handle->get_goal_id());
        op->cancel_exact = [weak_data, client, handle] {
            const auto current = weak_data.lock();
            if (!current) {return;}
            try {
              const auto owned_id = handle->get_goal_id();
              client->async_cancel_goal(
                handle, [weak_data, owned_id](auto response) {
                  if (auto current = weak_data.lock()) {
                    std::lock_guard<std::mutex> lock(current->mutex);
                    const bool accepted = response && response->return_code == 0 &&
                    std::any_of(
                      response->goals_canceling.begin(), response->goals_canceling.end(),
                      [&](
                        const auto & goal_info) {return goal_info.goal_id.uuid == owned_id;});
                    current->value.cancel_status = accepted ?
                    CancelStatus::Accepted : CancelStatus::Rejected;
                    current->value.cancel_message = accepted ?
                    "cancel_accepted_waiting_terminal" : "cancel_rejected";
                    current->changed.notify_all();
                  }
                });
            } catch (const std::exception & e) {
              std::lock_guard<std::mutex> lock(current->mutex);
              current->value.cancel_status = CancelStatus::TransportError;
              current->value.cancel_message = e.what();
              current->cancel_sent = false;
              current->cancel_retry_not_before = Clock::now() + std::chrono::milliseconds(500);
            }
          };
        op->changed.notify_all();
      };
    so.feedback_callback = [weak_data](typename Handle::SharedPtr,
        std::shared_ptr<const typename Action::Feedback> f) {
        if (auto op = weak_data.lock()) {
          std::lock_guard<std::mutex> lock(op->mutex);
          if (op->value.remote_execution_pending && !op->value.action_terminal_received && f) {
            op->value.feedback = feedback(*f);
            op->value.feedback_received_seconds =
              std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
          }
        }
      };
    so.result_callback = [weak_data](const typename Handle::WrappedResult & result) {
        if (auto op = weak_data.lock()) {
          Status status = Status::Failed;
          const bool succeeded = result.code == rclcpp_action::ResultCode::SUCCEEDED &&
            result.result && result.result->success;
          if (result.code == rclcpp_action::ResultCode::CANCELED) {
            status = Status::Canceled;
          } else if (succeeded) {
            status = Status::Succeeded;
          }
          const std::string message =
            result.result ? result_message(*result.result) : "missing_action_result";
          if (status == Status::Failed &&
            (message == "unsupported" || message.rfind("unsupported:", 0) == 0))
          {
            status = Status::Unsupported;
          }
          Core::finish(op, status, message, true);
        }
      };
    try {client->async_send_goal(goal, so);} catch (const std::exception & e) {
      // Transport ambiguity is not evidence that no remote goal exists.
      std::lock_guard<std::mutex> lock(data->mutex);
      data->value.local_status = LocalStatus::TransportError;
      data->value.local_message = e.what();
    }
  }
  void poll_ownership(const std::shared_ptr<Operation::Data> & data)
  {
    using Ownership = easyfly_msgs::srv::GetActionOwnership;
    std::lock_guard<std::mutex> lock(data->mutex);
    if (!data->value.remote_execution_pending || !data->value.action_terminal_received ||
      !data->managed_ownership || data->value.goal_id.empty()) {return;}
    auto service = data->ownership_client.lock();
    if (!service) {
      return;              // disappearing runtime is not evidence of remote release
    }
    const auto now = Clock::now();
    if (data->ownership_inflight) {
      if (now < data->ownership_deadline) {return;}
      service->remove_pending_request(data->ownership_request_id);
      data->ownership_inflight = false;
      data->ownership_retry = now + std::chrono::milliseconds(250);
    }
    if (now < data->ownership_retry || !service->service_is_ready()) {return;}
    auto request = std::make_shared<Ownership::Request>(); request->goal_id = data->value.goal_id;
    const auto weak = std::weak_ptr<Operation::Data>(data);
    data->ownership_inflight = true;
    data->ownership_deadline = now + std::chrono::milliseconds(500);
    data->ownership_retry = now + std::chrono::milliseconds(250);
    try {
      auto future = service->async_send_request(
        request, [weak](rclcpp::Client<Ownership>::SharedFuture future) {
          if (auto operation = weak.lock()) {
            std::lock_guard<std::mutex> lock(operation->mutex);
            operation->ownership_inflight = false;
            operation->ownership_retry = Clock::now() + std::chrono::milliseconds(250);
            try {
              auto response = future.get();
              if (response && response->goal_id == operation->value.goal_id &&
              response->state == 2)
              {
                operation->value.remote_execution_pending = false;
                operation->changed.notify_all();
              }
            } catch (const std::exception &) {}
          }
        });
      data->ownership_request_id = future.request_id;
    } catch (const std::exception & e) {
      data->ownership_inflight = false;
      data->value.local_message = std::string("ownership_query_failed:") + e.what();
    }
  }
  void loop()
  {
    {std::lock_guard<std::mutex> lock(mutex); executor_thread = std::this_thread::get_id();}
    for (;; ) {
      std::deque<std::function<void()>> tasks;
      std::vector<std::shared_ptr<Operation::Data>> active;
      {std::lock_guard<std::mutex> lock(mutex); tasks.swap(inbox); active = pending;}
      for (auto & task : tasks) {task();}
      try {executor->spin_some(std::chrono::milliseconds(5));} catch (const std::exception & e) {
        for (const auto & data : active) {
          std::lock_guard<std::mutex> lock(data->mutex);
          if (data->value.remote_execution_pending) {
            data->value.local_status = LocalStatus::TransportError;
            data->value.local_message = e.what();
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      for (const auto & data : active) {
        std::function<void()> cancel;
        {
          std::lock_guard<std::mutex> lock(data->mutex);
          if (!data->value.remote_execution_pending) {continue;}
          if (data->value.admission == Admission::Pending &&
            data->value.local_status == LocalStatus::Active &&
            options.admission_timeout_seconds > 0.0 &&
            std::chrono::duration<double>(Clock::now() - data->submitted).count() >=
            options.admission_timeout_seconds)
          {
            data->value.local_status = LocalStatus::AdmissionTimedOut;
            data->value.local_message = "admission_response_timeout_remote_unknown";
            if (options.cancel_on_admission_timeout) {
              data->cancel_requested = true;
              data->value.cancel_status = CancelStatus::Requested;
            }
          }
          if (data->cancel_requested && !data->cancel_sent && data->cancel_exact &&
            Clock::now() >= data->cancel_retry_not_before)
          {
            data->cancel_sent = true;
            cancel = data->cancel_exact;
          }
        }
        if (cancel) {cancel();}
        poll_ownership(data);
      }
      bool finish_worker = false;
      {
        std::lock_guard<std::mutex> lock(mutex);
        pending.erase(
          std::remove_if(
            pending.begin(), pending.end(), [](const auto & data) {
              std::lock_guard<std::mutex> lock(data->mutex);
              return !data->value.remote_execution_pending;
            }), pending.end());
        finish_worker = closed && pending.empty() && inbox.empty();
      }
      if (finish_worker) {break;}
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    subscription.reset();
    executor->remove_node(node);
    executor.reset();
    ready.clear();
    clients.clear();
    ownership.clear();
    node.reset();
    context->shutdown("SDK exact goal drain completed");
    context.reset();  // Complete DDS teardown before publishing the cleanup barrier.
    {std::lock_guard<std::mutex> lock(mutex); stopped = true;}
    stopped_event.notify_all();
  }
};
struct Client::Worker
{
  std::thread thread;
  std::shared_ptr<Core> state;
  explicit Worker(std::shared_ptr<Core> core)
  : state(core)
  {
    thread = std::thread([core] {core->loop();});
  }
  ~Worker()
  {
    if (!thread.joinable()) {return;}
    bool stopped;
    {std::lock_guard<std::mutex> lock(state->mutex); stopped = state->stopped;}
    if (stopped && thread.get_id() != std::this_thread::get_id()) {thread.join();} else {
      // thread owns Core, never Client/Python; pending ownership survives facade
      thread.detach();
    }
  }
};
Operation::Operation(std::shared_ptr<Data> data)
: data_(std::move(data)) {}
OperationSnapshot Operation::snapshot() const
{
  std::lock_guard<std::mutex> lock(data_->mutex);
  return data_->value;
}
bool Operation::wait(double seconds) const
{
  if (!std::isfinite(seconds) || seconds < 0) {
    throw std::invalid_argument("wait timeout must be finite/nonnegative");
  }
  if (auto core = data_->core.lock()) {
    std::lock_guard<std::mutex> lock(core->mutex);
    if (core->executor_thread == std::this_thread::get_id()) {
      throw std::logic_error("wait from SDK executor forbidden");
    }
  }
  const auto deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(
      seconds));
  std::unique_lock<std::mutex> lock(data_->mutex);
  if (!data_->changed.wait_until(
      lock, deadline,
      [&] {return !data_->value.remote_execution_pending;})) {return false;}
  lock.unlock();
  if (auto core = data_->core.lock()) {
    std::unique_lock<std::mutex> core_lock(core->mutex);
    if (core->closed) {
      bool remaining = false;
      for (const auto & operation : core->pending) {
        std::lock_guard<std::mutex> operation_lock(operation->mutex);
        remaining |= operation->value.remote_execution_pending;
      }
      // The last drained operation must not return into process exit while its
      // self-owned executor is still destroying DDS resources. Other live
      // operations remain explicitly outside this operation's wait guarantee.
      if (!remaining) {
        return core->stopped_event.wait_until(core_lock, deadline, [&] {return core->stopped;});
      }
    }
  }
  return true;
}
bool Operation::cancel()
{
  std::lock_guard<std::mutex> lock(data_->mutex);
  if (!data_->value.remote_execution_pending) {return false;}
  data_->cancel_requested = true;
  if (data_->value.cancel_status == CancelStatus::Rejected ||
    data_->value.cancel_status == CancelStatus::TransportError)
  {
    data_->cancel_sent = false;
  }
  if (data_->value.cancel_status == CancelStatus::None) {
    data_->value.cancel_status = CancelStatus::Requested;
  }
  return true;
}
Client::Client(const ClientOptions & options)
: core_(std::make_shared<Core>())
{
  if (!options.max_pending_operations || options.max_pending_operations > 1024 ||
    !std::isfinite(options.state_freshness_seconds) || options.state_freshness_seconds <= 0 ||
    !std::isfinite(options.admission_timeout_seconds) || options.admission_timeout_seconds < 0)
  {
    throw std::invalid_argument("invalid SDK options");
  }
  core_->options = options;
  core_->initialize();
  worker_ = std::make_unique<Worker>(core_);
}
Client::~Client() {close(pending_operations() == 0 ? 2.0 : 0.0);}
bool Client::running() const
{
  std::lock_guard<std::mutex> lock(core_->mutex); return !core_->closed && !core_->stopped;
}
bool Client::server_ready(const std::string & name) const
{
  std::lock_guard<std::mutex> lock(core_->mutex);
  if (core_->closed) {return false;}
  auto it = core_->ready.find(name);
  return it != core_->ready.end() && it->second();
}
StateSnapshot Client::state() const
{
  std::lock_guard<std::mutex> lock(core_->mutex);
  auto result = core_->state_value;
  if (result.received) {
    result.receipt_age_seconds =
      std::chrono::duration<double>(Clock::now() - core_->state_receipt).count();
  }
  result.receipt_fresh = result.received && !core_->closed &&
    result.receipt_age_seconds <= core_->options.state_freshness_seconds;
  const auto & stamp = result.value.header.stamp;
  const auto now = !core_->closed && core_->node ? core_->node->now().nanoseconds() : 0;
  const int64_t source = static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
  result.source_time_valid = result.received && now > 0 && source > 0 && stamp.sec >= 0 &&
    stamp.nanosec < 1000000000U && result.value.header.frame_id == "odom";
  result.source_age_seconds = static_cast<double>(now - source) / 1e9;
  result.fresh = result.receipt_fresh && result.source_time_valid &&
    result.source_age_seconds >= -.2 &&
    result.source_age_seconds <= core_->options.state_freshness_seconds;
  return result;
}
std::size_t Client::pending_operations() const
{
  std::lock_guard<std::mutex> lock(core_->mutex);
  std::size_t count = 0;
  for (const auto & data : core_->pending) {
    std::lock_guard<std::mutex> op_lock(data->mutex);
    if (data->value.remote_execution_pending) {++count;}
  }
  return count;
}
bool Client::close(double seconds)
{
  if (!std::isfinite(seconds) || seconds < 0) {
    throw std::invalid_argument("close timeout must be finite/nonnegative");
  }
  std::unique_lock<std::mutex> lock(core_->mutex);
  if (!core_->closed) {
    core_->closed = true;
    for (const auto & data : core_->pending) {
      std::lock_guard<std::mutex> op_lock(data->mutex);
      if (!data->value.remote_execution_pending) {continue;}
      data->value.local_status = LocalStatus::Closed;
      data->value.local_message = "client_closed_exact_cancel_pending";
      data->cancel_requested = true;
      if (data->value.cancel_status != CancelStatus::Accepted) {
        if (data->value.cancel_status == CancelStatus::Rejected ||
          data->value.cancel_status == CancelStatus::TransportError)
        {
          data->cancel_sent = false;
        }
        data->value.cancel_status = CancelStatus::Requested;
      }
    }
    const auto weak = std::weak_ptr<Core>(core_);
    core_->inbox.push_back([weak] {if (auto core = weak.lock()) {core->subscription.reset();}});
  }
  if (core_->executor_thread == std::this_thread::get_id()) {return core_->stopped;}
  return core_->stopped_event.wait_for(
    lock, std::chrono::duration<double>(seconds), [&] {
      return core_->stopped;
    });
}
#define EASYFLY_METHOD(method, action_type, name) \
  std::shared_ptr<Operation> Client::method( \
    const easyfly_msgs::action::action_type::Goal & goal) { \
    return core_->submit<easyfly_msgs::action::action_type>(name, goal); \
  }
EASYFLY_METHOD(takeoff, Takeoff, "takeoff")
EASYFLY_METHOD(land, Land, "land")
EASYFLY_METHOD(precision_land, PrecisionLand, "precision_land")
EASYFLY_METHOD(go_to, GoToWaypoint, "goto")
EASYFLY_METHOD(navigate, NavigateTo, "navigate")
EASYFLY_METHOD(follow, FollowTrajectory, "follow")
EASYFLY_METHOD(track, TrackTarget, "track")
EASYFLY_METHOD(execute_mission, ExecuteMission, "execute_mission")
EASYFLY_METHOD(search, SearchArea, "search")
EASYFLY_METHOD(explore, Explore, "explore")
#undef EASYFLY_METHOD
std::shared_ptr<Operation> Client::unsupported(const std::string & capability)
{
  return core_->immediate(Admission::Unsupported, Status::Unsupported, "unsupported:" + capability);
}
}  // namespace easyfly_core
