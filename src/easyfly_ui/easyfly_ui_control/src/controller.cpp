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
#include <unistd.h>
#include <atomic>
#include <cinttypes>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <future>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include "easyfly_ui_control/controller.hpp"
#include <rclcpp/rclcpp.hpp>
#include <easyfly_msgs/srv/manage_reviewed_mission.hpp>
#include <easyfly_mission_format/mission_format.hpp>
#include "journal.hpp"
#include "confirmation_ledger.hpp"
namespace easyfly_ui_control
{
using Api = easyfly_msgs::srv::ManageReviewedMission;
double steady_seconds()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
void audit_event(
  const char * event, const std::string & confirmation, const std::string & instance,
  const std::string & request, const std::string & content_hash)
{
  const auto token = [](const std::string & value, size_t size) {
      return hex_id(value, size) ? value : std::string("none");
    };
  RCLCPP_INFO(
    rclcpp::get_logger("easyfly_managed_audit"),
    "event=%s pid=%d steady=%.6f confirmation=%s instance=%s request=%s content_sha256=%s",
    event, static_cast<int>(getpid()), steady_seconds(), token(confirmation, 32).c_str(),
    token(instance, 32).c_str(), token(request, 32).c_str(), token(content_hash, 64).c_str());
}
namespace
{
bool terminal(const std::string & status)
{
  return status == "succeeded" || status == "failed" || status == "canceled" ||
         status == "rejected";
}
bool noExecution(const std::string & status)
{
  return status == "invalid" || status == "content_changed" ||
         status == "unsupported" || status == "busy";
}
bool fresh(double received)
{
  const double age = steady_seconds() - received;
  return std::isfinite(received) && received > 0 && age >= 0 && age <= 2;
}
}  // namespace
struct MissionControl::Impl
{
  struct Command
  {
    bool cancellation{false};
    easyfly_msgs::srv::GetMissionDetails::Response review;
    double received{0};
    std::string instance, confirmation;
  };
  struct Pending
  {
    std::string operation;
    std::shared_future<Api::Response::SharedPtr> future;
    int64_t transport_id;
    double sent;
  };
  ControlConfig config;
  ControlSnapshot data;
  std::shared_ptr<const ControlSnapshot> published;
  std::mutex mutex;
  std::condition_variable wake;
  std::optional<Command> queued;
  std::optional<Pending> pending;
  std::atomic<bool> closing{false};
  bool fatal{false};
  bool command_reserved{false};
  ConfirmationLedger confirmations;
  std::string current_confirmation;
  double next_read{0};
  std::string scope;
  std::thread worker;
  std::unique_ptr<Journal> journal;
  std::shared_ptr<rclcpp::Context> context;
  rclcpp::Node::SharedPtr node;
  rclcpp::Client<Api>::SharedPtr client;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  explicit Impl(const ControlConfig & options)
  : config(options)
  {
    if (config.journal_directory.empty() || !std::isfinite(config.response_timeout) ||
      config.response_timeout < .05 || config.response_timeout > 30)
    {throw std::invalid_argument("explicit journal directory and timeout .05..30 required");}
    // Acquire once, before any worker/context/remote reads. Keep this lock for restoration.
    journal = std::make_unique<Journal>(config.journal_directory);
    published = std::make_shared<ControlSnapshot>(data);
    worker = std::thread([this] {run();});
  }
  ~Impl()
  {
    stop();
    if (worker.joinable()) {worker.join();}
  }
  void stop() {closing.store(true); wake.notify_all();}
  void publish()
  {
    std::lock_guard<std::mutex> lock(mutex);
    data.request_pending = pending.has_value() || queued.has_value();
    if (!queued) {command_reserved = false;}
    published = std::make_shared<ControlSnapshot>(data);
  }
  bool enqueue(Command command)
  {
    std::lock_guard<std::mutex> lock(mutex);
    const auto & s = *published;
    if (closing.load() || s.stopped || command_reserved || queued || s.request_pending ||
      s.phase == Phase::Blocked)
    {
      return false;
    }
    if (command.cancellation) {
      if (!s.intent || s.intent->resolved || s.intent->execution.empty() ||
        s.intent->cancel_dispatched) {return false;}
    }
    if (!command.cancellation && ((s.phase != Phase::Idle && s.phase != Phase::Completed) ||
      !s.session_ready || !fresh(s.session_received) ||
      command.instance != s.server_instance || !fresh(command.received) ||
      command.review.content.size() > 65536))
    {
      return false;
    }
    if (!command.cancellation && !confirmations.consume(command.confirmation)) {return false;}
    if (!command.cancellation) {
      audit_event(
        "start_enqueued", command.confirmation, command.instance, {},
        command.review.content_sha256);
    }
    command_reserved = true; queued = std::move(command);
    auto visible = *published; visible.request_pending = true;
    published = std::make_shared<ControlSnapshot>(visible);
    wake.notify_all(); return true;
  }
  void block(const std::string & reason)
  {
    fatal = true; data.phase = Phase::Blocked; data.message = reason.substr(0, 2048);
    data.session_ready = false;
    std::lock_guard<std::mutex> lock(mutex); queued.reset();
  }
  bool commit()
  {
    try {journal->save(*data.intent); return true;} catch (const std::exception & exc) {
      block(exc.what()); return false;
    }
  }
  void initialize()
  {
    context = std::make_shared<rclcpp::Context>(); context->init(0, nullptr);
    scope = "domain=" + std::to_string(context->get_domain_id()) + ";localhost=" +
      (std::getenv("ROS_LOCALHOST_ONLY") && std::string(std::getenv("ROS_LOCALHOST_ONLY")) == "1" ?
      "1" : "0");
    data.intent = journal->load();
    if (data.intent) {
      data.phase = data.intent->resolved ? Phase::Completed : Phase::Uncertain;
      if (data.intent->scope != scope) {block("journal ROS scope changed; no remote recovery");}
      audit_event(
        "restored_read_only", {}, data.intent->instance, data.intent->request,
        data.intent->hash);
    }
    rclcpp::NodeOptions options; options.context(context).use_global_arguments(false);
    node = std::make_shared<rclcpp::Node>("easyfly_ui_control_" + new_id(), options);
    rclcpp::ExecutorOptions eo; eo.context = context;
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(eo);
    executor->add_node(node);
    client = node->create_client<Api>("/platform/mission/reviewed");
  }
  void dispatch(const std::string & operation)
  {
    if (closing.load()) {return;}
    const auto confirmation = operation == "start" ? current_confirmation : std::string{};
    auto request = std::make_shared<Api::Request>(); request->operation = operation;
    if (operation != "session") {
      const auto & v = *data.intent;
      request->server_instance_id = v.instance; request->client_request_id = v.request;
      request->mission_name = v.name; request->expected_content_sha256 = v.hash;
      request->execution_id = operation == "cancel" ? v.execution : "";
    }
    if (!client->service_is_ready()) {
      if (operation == "start" || operation == "cancel") {
        audit_event(
          "write_endpoint_unavailable", confirmation, data.intent->instance,
          data.intent->request, data.intent->hash);
      }
      data.message = "managed endpoint unavailable; no fallback or replay";
      if (data.intent && !data.intent->resolved) {data.phase = Phase::Uncertain;}
      data.session_ready = false; return;
    }
    try {
      if (operation == "start" || operation == "cancel") {
        audit_event(
          operation == "start" ? "start_send_attempt" : "cancel_send_attempt",
          confirmation, data.intent->instance, data.intent->request, data.intent->hash);
      }
      auto sent = client->async_send_request(request);
      pending = Pending{operation, sent.future.share(), sent.request_id, steady_seconds()};
      if (operation == "start" || operation == "cancel") {
        audit_event(
          operation == "start" ? "start_transport_queued" : "cancel_transport_queued",
          confirmation, data.intent->instance, data.intent->request, data.intent->hash);
        RCLCPP_INFO(
          node->get_logger(), "managed transport operation=%s sequence=%" PRId64,
          operation.c_str(), sent.request_id);
      }
    } catch (const std::exception & exc) {
      if (operation == "start" || operation == "cancel") {
        audit_event(
          "write_transport_exception", confirmation, data.intent->instance,
          data.intent->request, data.intent->hash);
      }
      data.message = exc.what();
      if (data.intent && !data.intent->resolved) {data.phase = Phase::Uncertain;}
    }
  }
  void command(const Command & cmd)
  {
    if (fatal || closing.load()) {return;}
    if (cmd.cancellation) {
      if (!data.intent || data.intent->resolved || data.intent->execution.empty() ||
        data.intent->cancel_dispatched) {return;}
      data.intent->cancel_dispatched = true;
      if (!commit()) {return;}
      data.phase = Phase::CancelPending; dispatch("cancel"); return;
    }
    const auto & r = cmd.review;
    if ((data.intent && !data.intent->resolved) || !fresh(cmd.received) ||
      !data.session_ready || !fresh(data.session_received) ||
      cmd.instance != data.server_instance || !r.success || r.status != "valid_snapshot" ||
      !catalog_name(r.mission_name) || r.content.size() > 65536 ||
      !easyfly_mission_fsm::missionTextUtf8(r.content) ||
      r.content_sha256 != easyfly_mission_fsm::missionContentSha256(r.content) ||
      !easyfly_mission_fsm::validateMissionText(r.content).valid())
    {
      audit_event("start_worker_refused", cmd.confirmation, cmd.instance, {}, r.content_sha256);
      data.message = "review/confirmation invalid or expired; no request sent"; return;
    }
    data.intent = Intent{scope, cmd.instance, new_id(), r.mission_name, r.content_sha256, "", "",
      false, false};
    if (!commit()) {return;}
    current_confirmation = cmd.confirmation;
    audit_event(
      "start_intent_committed", current_confirmation, cmd.instance, data.intent->request,
      data.intent->hash);
    data.phase = Phase::Sent; dispatch("start");
  }
  void response(const std::string & operation, const Api::Response & r)
  {
    if (r.status.size() > 64 || r.message.size() > 2048 || r.admission_status.size() > 64 ||
      !easyfly_mission_fsm::missionTextUtf8(r.status) ||
      !easyfly_mission_fsm::missionTextUtf8(r.message) ||
      !hex_id(r.server_instance_id, 32) || r.request_capacity < 1 ||
      r.request_capacity > 256 || r.retained_requests > r.request_capacity ||
      (r.admission_success && !r.admission_resolved))
    {block("malformed managed response; unresolved record retained"); return;}
    data.status = r.status; data.message = r.message;
    if (operation == "session") {
      if (!r.success || r.status != "session" || !r.client_request_id.empty() ||
        !r.execution_id.empty() || !r.action_goal_id.empty() || r.execution_pending ||
        r.admission_resolved)
      {block("invalid session response"); return;}
      data.server_instance = r.server_instance_id; data.session_ready = true;
      data.session_received = steady_seconds(); return;
    }
    auto & v = *data.intent;
    if (r.server_instance_id != v.instance || r.client_request_id != v.request ||
      r.mission_name != v.name || r.content_sha256 != v.hash)
    {block("instance/request/content identity unconfirmed; no replay or fault clearing"); return;}
    if ((!v.execution.empty() && r.execution_id != v.execution) ||
      (!v.goal.empty() && r.action_goal_id != v.goal) ||
      (!r.execution_id.empty() && !hex_id(r.execution_id, 32)) ||
      (!r.action_goal_id.empty() && (!hex_id(r.action_goal_id, 32) || r.execution_id.empty())))
    {block("execution/Action identity changed"); return;}
    if (!r.execution_id.empty() && r.status != "pending" && r.status != "pending_timeout" &&
      r.status != "accepted" && r.status != "running" && r.status != "canceling" &&
      r.status != "cancel_requested" && r.status != "cancel_rejected" &&
      r.status != "cancel_timeout" && r.status != "busy" && r.status != "fault" &&
      !terminal(r.status)) {block("unknown execution state"); return;}
    v.execution = r.execution_id; v.goal = r.action_goal_id;
    bool completed = false;
    if (v.execution.empty()) {
      completed = r.admission_resolved && !r.admission_success && !r.execution_pending &&
        noExecution(r.status) && r.admission_status == r.status && r.action_goal_id.empty();
      if (!completed) {block("missing execution is not release evidence"); return;}
    } else if (!r.execution_pending) {
      completed = terminal(r.status) && r.admission_resolved &&
        (r.status == "rejected" ? !r.admission_success && v.goal.empty() : !v.goal.empty());
      if (!completed) {block("terminal/ownership evidence invalid"); return;}
    }
    v.resolved = completed;
    if (!commit()) {v.resolved = false; return;}
    data.phase = completed ? Phase::Completed : terminal(r.status) ? Phase::DrainPending :
      r.status == "fault" || r.status == "cancel_rejected" || r.status == "cancel_timeout" ?
      Phase::Uncertain : v.cancel_dispatched ? Phase::CancelPending :
      r.status == "pending" || r.status == "pending_timeout" ? Phase::Uncertain : Phase::Running;
  }
  void tick()
  {
    executor->spin_some(std::chrono::milliseconds(5));
    const auto now = steady_seconds();
    if (pending && pending->future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      auto completed = std::move(*pending); pending.reset();
      try {
        const auto r = completed.future.get();
        if (!r) {block("missing service response");} else {response(completed.operation, *r);}
      } catch (const std::exception & exc) {block(exc.what());}
      next_read = now + 1;
    } else if (pending && now - pending->sent >= config.response_timeout) {
      client->remove_pending_request(pending->transport_id); pending.reset();
      data.message = "response timeout; only read-only lookup may recover";
      data.session_ready = false;
      if (data.intent && !data.intent->resolved) {data.phase = Phase::Uncertain;}
      next_read = now;
    }
    if (pending || fatal || closing.load()) {return;}
    std::optional<Command> cmd;
    {std::lock_guard<std::mutex> lock(mutex); cmd = std::move(queued); queued.reset();}
    if (cmd) {command(*cmd); next_read = now + 1;} else if (now >= next_read) {
      dispatch(data.intent && !data.intent->resolved ? "lookup" : "session");
      next_read = now + 1;
    }
  }
  void run()
  {
    try {
      initialize(); publish();
      while (!closing.load()) {
        tick(); publish();
        std::unique_lock<std::mutex> lock(mutex);
        wake.wait_for(
          lock, std::chrono::milliseconds(20), [this] {
            return closing.load() || (queued.has_value() && !pending);
          });
      }
    } catch (const std::exception & exc) {block(exc.what());}
    // Pending commands are forgotten locally, never replayed. Durable intent remains.
    if (pending && client) {client->remove_pending_request(pending->transport_id);}
    pending.reset();
    if (executor && node) {executor->remove_node(node);}
    client.reset(); node.reset(); executor.reset();
    if (context && context->is_valid()) {context->shutdown("local control facade stopped");}
    context.reset(); journal.reset();
    {std::lock_guard<std::mutex> lock(mutex); queued.reset();}
    data.stopped = true; data.session_ready = false; publish();
  }
};
MissionControl::MissionControl(const ControlConfig & config)
: impl_(std::make_unique<Impl>(config)) {}
MissionControl::~MissionControl() = default;
std::shared_ptr<const ControlSnapshot> MissionControl::snapshot() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex); return impl_->published;
}
bool MissionControl::start_confirmed(
  const easyfly_msgs::srv::GetMissionDetails::Response & review, double received,
  const std::string & instance)
{
  return start_confirmed(review, received, instance, {});
}
bool MissionControl::start_confirmed(
  const easyfly_msgs::srv::GetMissionDetails::Response & review, double received,
  const std::string & instance, const std::string & confirmation)
{
  if (review.content.size() > 65536 || review.mission_name.size() > 128 ||
    review.content_sha256.size() > 64 || review.message.size() > 2048 || review.status.size() > 64)
  {return false;}
  if (!hex_id(confirmation, 32)) {return false;}
  return impl_->enqueue({false, review, received, instance, confirmation});
}
bool MissionControl::cancel_confirmed()
{
  Impl::Command command; command.cancellation = true;
  return impl_->enqueue(std::move(command));
}
void MissionControl::stop() {impl_->stop();}
}  // namespace easyfly_ui_control
