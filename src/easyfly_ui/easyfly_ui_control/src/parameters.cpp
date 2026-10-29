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
#include <json/json.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <future>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include "easyfly_ui_control/parameters.hpp"
#include <rclcpp/rclcpp.hpp>
#include <easyfly_msgs/srv/query_params.hpp>
#include <easyfly_mission_format/mission_format.hpp>
#include "confirmation_ledger.hpp"
namespace easyfly_ui_control
{
const char * local_parameter_name(LocalParameter p)
{
  if (p == LocalParameter::OperatorNote) {return "operator_note";}
  if (p == LocalParameter::ExportPretty) {return "export_pretty";}
  throw std::invalid_argument("unsupported local parameter");
}
double parameter_steady_seconds()
{return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
std::string parameter_json(LocalParameter p, const ParameterValue & v)
{
  local_parameter_name(p); Json::Value value;
  if (p == LocalParameter::OperatorNote && std::holds_alternative<std::string>(v)) {
    const auto & s = std::get<std::string>(v);
    if (s.size() > 512 || !easyfly_mission_fsm::missionTextUtf8(s)) {
      throw std::invalid_argument("operator_note: UTF-8 text without NUL, <=512 bytes");
    }
    value = s;
  } else if (p == LocalParameter::ExportPretty && std::holds_alternative<bool>(v)) {
    value = std::get<bool>(v);
  } else {throw std::invalid_argument("local parameter type mismatch");}
  Json::StreamWriterBuilder builder; builder["indentation"] = ""; builder["emitUTF8"] = true;
  return Json::writeString(builder, value);
}
std::optional<ParameterValue> parameter_value(LocalParameter p, const std::string & json)
{
  if (json.empty() || json.size() > 4096 || !easyfly_mission_fsm::missionTextUtf8(json)) {
    return {};
  }
  Json::CharReaderBuilder builder; builder["allowComments"] = false;
  builder["allowTrailingCommas"] = false; builder["failIfExtra"] = true;
  builder["allowSingleQuotes"] = false; builder["allowSpecialFloats"] = false;
  std::unique_ptr<Json::CharReader> reader(builder.newCharReader()); Json::Value value;
  std::string error;
  if (!reader->parse(json.data(), json.data() + json.size(), &value, &error)) {return {};}
  std::optional<ParameterValue> result;
  if (p == LocalParameter::OperatorNote && value.isString()) {result = value.asString();}
  if (p == LocalParameter::ExportPretty && value.isBool()) {result = value.asBool();}
  if (!result) {return {};}
  try {
    parameter_json(p, *result);
  } catch (const std::exception &) {
    return {};
  }
  return result;
}
struct ParameterControl::Impl
{
  using Api = easyfly_msgs::srv::QueryParams;
  enum class Stage {Read, Precheck, Write, Check};
  struct Command {LocalParameter parameter; std::optional<ParameterReview> review;
    std::string desired;};
  struct Pending
  {
    Stage stage;
    std::shared_future<Api::Response::SharedPtr> future;
    int64_t transport;
    double sent;
  };
  ParameterConfig config;
  mutable std::mutex mutex;
  std::condition_variable wake;
  ParameterSnapshot data;
  bool closing{false}, inhibited{false}, write_accepted{false};
  uint64_t sequence{0};
  ConfirmationLedger ledger;
  std::optional<Command> queued, command;
  std::optional<Pending> pending;
  std::thread worker;
  std::shared_ptr<rclcpp::Context> context;
  rclcpp::Node::SharedPtr node;
  rclcpp::Client<Api>::SharedPtr client;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  explicit Impl(ParameterConfig options)
  : config(options)
  {
    if (!std::isfinite(config.response_timeout) || config.response_timeout < .05 ||
      config.response_timeout > 30)
    {
      throw std::invalid_argument("parameter timeout must be .05..30s");
    }
    worker = std::thread([this] {run();});
  }
  ~Impl() {stop(); if (worker.joinable()) {worker.join();}}
  void stop()
  {std::lock_guard<std::mutex> lock(mutex); closing = true; wake.notify_all();}
  void finish(ParameterPhase phase, const std::string & message)
  {
    data.phase = phase; data.message = message; data.pending = false;
    if (phase == ParameterPhase::Uncertain) {data.fault = true;}
    if (command && command->review) {
      data.last_write = std::string(local_parameter_name(command->parameter)) + ": " + message;
    }
    command.reset();
  }
  void failure(const std::string & message)
  {
    if (command && !data.dispatched) {data.review.reset();}
    finish(
      data.dispatched && command && command->review ? ParameterPhase::Uncertain :
      data.fault ? ParameterPhase::Uncertain : ParameterPhase::Rejected, message);
  }
  void dispatch(Stage stage)
  {
    if (closing || !command) {return;}
    if (stage == Stage::Write && inhibited) {
      finish(ParameterPhase::Rejected, "precheck completed but write inhibited; not dispatched");
      return;
    }
    if (!client->service_is_ready()) {failure("fixed parameter service unavailable"); return;}
    auto request = std::make_shared<Api::Request>();
    request->param_name = local_parameter_name(command->parameter);
    if (stage == Stage::Write) {request->param_value = command->desired;}
    try {
      if (stage == Stage::Write) {
        data.dispatched = true; data.last_write = "one write dispatched; result not confirmed";
      }
      auto sent = client->async_send_request(request);
      pending = Pending{stage, sent.future.share(), sent.request_id, parameter_steady_seconds()};
      data.phase = stage == Stage::Write ? ParameterPhase::Writing : stage == Stage::Check ?
        ParameterPhase::Checking : stage ==
        Stage::Precheck ? ParameterPhase::Precheck : ParameterPhase::Reading;
    } catch (const std::exception &) {failure("parameter transport failed; no retry/rollback");}
  }
  void reviewed(const std::string & canonical)
  {
    data.review =
      ParameterReview{++sequence, command->parameter, canonical, parameter_steady_seconds()};
  }
  void response(Stage stage, const Api::Response & r)
  {
    if (!command) {return;}
    if (stage == Stage::Write) {
      data.acknowledged = true; write_accepted = r.success;
      // Acknowledgement alone is not readback. Validate the independent next read.
      dispatch(Stage::Check); return;
    }
    const auto value = parameter_value(command->parameter, r.current_value);
    if (!r.success || !value) {failure("parameter read rejected or invalid typed JSON"); return;}
    const auto canonical = parameter_json(command->parameter, *value);
    if (stage == Stage::Precheck) {
      if (canonical != command->review->json) {
        reviewed(canonical); finish(
          ParameterPhase::Rejected,
          "value changed since confirmation; no write dispatched");
      } else {dispatch(Stage::Write);} return;
    }
    reviewed(canonical);
    if (stage == Stage::Read) {
      finish(
        data.fault ? ParameterPhase::Uncertain : ParameterPhase::Reviewed,
        data.fault ? "current value read; previous unconfirmed write remains locked" :
        "typed local value read; not flight configuration");
    } else if (data.fault) {
      finish(ParameterPhase::Uncertain, "reply/readback received after disconnect; lock retained");
    } else if (!write_accepted) {
      finish(ParameterPhase::Rejected, "backend rejected write; current value read independently");
    } else if (canonical == command->desired) {
      finish(
        ParameterPhase::Verified,
        "write reply succeeded; independent readback matched at observation");
    } else {
      finish(
        ParameterPhase::Uncertain,
        "write replied but independent readback differed; no retry/rollback");
    }
  }
  void tick()
  {
    if (pending && pending->future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      auto completed = *pending; pending.reset();
      try {
        const auto r = completed.future.get(); if (r) {
          response(completed.stage, *r);
        } else {
          failure("missing reply");
        }
      } catch (const std::exception &) {failure("parameter response exception");}
    } else if (pending && parameter_steady_seconds() - pending->sent >= config.response_timeout) {
      client->remove_pending_request(pending->transport); pending.reset();
      failure("response timeout; result unconfirmed; no retry/rollback");
    }
    if (!pending && !command && queued) {
      command = std::move(queued); queued.reset();
      dispatch(command->review ? Stage::Precheck : Stage::Read);
    }
  }
  void run()
  {
    try {
      static std::atomic<uint64_t> instances{0};
      context = std::make_shared<rclcpp::Context>(); context->init(0, nullptr);
      rclcpp::NodeOptions options; options.context(context).use_global_arguments(false)
      .start_parameter_services(false).start_parameter_event_publisher(false);
      node = std::make_shared<rclcpp::Node>(
        "easyfly_ui_parameter_" + std::to_string(
          getpid()) + "_" + std::to_string(++instances), options);
      client = node->create_client<Api>("/platform/params/query");
      rclcpp::ExecutorOptions executor_options; executor_options.context = context;
      executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
      executor->add_node(node);
      std::unique_lock<std::mutex> lock(mutex); data.ready = true;
      while (!closing) {
        lock.unlock(); executor->spin_some(std::chrono::milliseconds(5)); lock.lock();
        if (!closing) {tick();}
        wake.wait_for(
          lock, std::chrono::milliseconds(10), [this] {
            return closing || (queued && !pending);
          });
      }
    } catch (const std::exception &) {
      std::lock_guard<std::mutex> lock(mutex); closing = true; data.ready = false;
      failure("local parameter worker failed");
    }
    if (pending && client) {client->remove_pending_request(pending->transport);}
    pending.reset();
    if (executor && node) {executor->remove_node(node);}
    client.reset(); node.reset(); executor.reset();
    if (context && context->is_valid()) {context->shutdown("local parameter editor stopped");}
    context.reset(); std::lock_guard<std::mutex> lock(mutex);
    if (command && command->review && data.dispatched) {
      finish(
        ParameterPhase::Uncertain,
        "local communication stopped; remote write outcome unconfirmed");
    }
    command.reset(); queued.reset(); data.pending = false; data.ready = false; data.stopped = true;
  }
};
ParameterControl::ParameterControl(const ParameterConfig & c)
: impl_(std::make_unique<Impl>(c)) {}
ParameterControl::~ParameterControl() = default;
std::shared_ptr<const ParameterSnapshot> ParameterControl::snapshot() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex); return std::make_shared<ParameterSnapshot>(
    impl_->data);
}
bool ParameterControl::read(LocalParameter p)
{
  try {
    local_parameter_name(p);
  } catch (const std::exception &) {
    return false;
  }
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->closing || impl_->inhibited || !impl_->data.ready || impl_->data.stopped ||
    impl_->data.pending) {return false;}
  impl_->data.pending = true; impl_->data.phase = ParameterPhase::Reading;
  impl_->data.review.reset();
  impl_->queued = Impl::Command{p, {}, {}}; impl_->wake.notify_all(); return true;
}
bool ParameterControl::write_confirmed(
  const ParameterReview & review, const ParameterValue & desired,
  const std::string & confirmation)
{
  std::string json; try {
    json = parameter_json(review.parameter, desired);
  } catch (const std::exception &) {
    return false;
  }
  std::lock_guard<std::mutex> lock(impl_->mutex); const auto & s = impl_->data;
  if (impl_->closing || impl_->inhibited || !s.ready || s.stopped || s.pending || s.fault ||
    !s.review || !review.id || review.id != s.review->id ||
    review.parameter != s.review->parameter ||
    review.json != s.review->json || confirmation.size() != 32 ||
    !impl_->ledger.consume(confirmation)) {return false;}
  ++impl_->data.confirmations; impl_->data.pending = true; impl_->data.dispatched = false;
  impl_->data.acknowledged = false; impl_->data.phase = ParameterPhase::Precheck;
  impl_->queued = Impl::Command{review.parameter, review, std::move(json)};
  impl_->wake.notify_all(); return true;
}
void ParameterControl::inhibit(bool active)
{
  std::lock_guard<std::mutex> lock(impl_->mutex); impl_->inhibited = active;
  if (active && impl_->command && impl_->command->review && impl_->data.dispatched) {
    impl_->data.fault = true;
    impl_->data.phase = ParameterPhase::Uncertain;
    impl_->data.message = "observation disconnected after write; no retry/rollback";
    impl_->data.last_write = impl_->data.message;
  }
}
void ParameterControl::stop() {impl_->stop();}
}  // namespace easyfly_ui_control
