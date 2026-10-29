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
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <future>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include "easyfly_ui_control/importer.hpp"
#include <rclcpp/rclcpp.hpp>
#include <easyfly_msgs/srv/import_mission.hpp>
#include <easyfly_msgs/srv/get_mission_details.hpp>
#include <easyfly_mission_format/mission_format.hpp>
#include "import_journal.hpp"
#include "confirmation_ledger.hpp"
namespace easyfly_ui_control
{
namespace
{
double now()
{return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
bool document_valid(const ImportDocument & d)
{
  const auto ext = std::filesystem::path(d.name).extension();
  return catalog_name(d.name) && (ext == ".yaml" || ext == ".yml") &&
         !d.content.empty() && d.content.size() <= 65536 && hex_id(d.hash, 64) &&
         easyfly_mission_fsm::missionTextUtf8(d.content) &&
         easyfly_mission_fsm::missionContentSha256(d.content) == d.hash;
}
bool same(const ImportDestination & a, const ImportDestination & b)
{return a.instance == b.instance && a.catalog == b.catalog && a.directory == b.directory;}
}  // namespace
struct MissionImporter::Impl
{
  using Api = easyfly_msgs::srv::ImportMission;
  using Details = easyfly_msgs::srv::GetMissionDetails;
  enum class Stage {Session, Precheck, Create, Lookup, Details, FinalSession};
  struct Command {ImportDocument document; ImportDestination destination;};
  struct Pending
  {
    Stage stage;
    int64_t transport;
    double sent;
    std::shared_future<Api::Response::SharedPtr> api;
    std::shared_future<Details::Response::SharedPtr> details;
  };
  ImportConfig config;
  ImportSnapshot data;
  mutable std::mutex mutex;
  std::condition_variable wake;
  bool closing{false}, inhibited{false}, refresh_queued{false};
  std::optional<Command> queued, command;
  std::optional<Pending> pending;
  double next_lookup{0};
  std::string scope;
  ConfirmationLedger confirmations;
  std::unique_ptr<ImportJournal> journal;
  std::thread worker;
  std::shared_ptr<rclcpp::Context> context;
  rclcpp::Node::SharedPtr node;
  rclcpp::Client<Api>::SharedPtr client;
  rclcpp::Client<Details>::SharedPtr details;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  explicit Impl(const ImportConfig & c)
  : config(c)
  {
    const char * local = std::getenv("ROS_LOCALHOST_ONLY");
    if (!local || std::string(local) != "1" || config.journal_directory.empty() ||
      !std::isfinite(config.response_timeout) || config.response_timeout < .05 ||
      config.response_timeout > 30)
    {throw std::invalid_argument("localhost=1, private journal and timeout .05..30 required");}
    journal = std::make_unique<ImportJournal>(config.journal_directory);
    worker = std::thread([this] {run();});
  }
  ~Impl() {stop(); if (worker.joinable()) {worker.join();}}
  void stop() {std::lock_guard<std::mutex> lock(mutex); closing = true; wake.notify_all();}
  bool unresolved() const {return data.intent && !data.intent->resolved;}
  void outside(std::unique_lock<std::mutex> & lock, const std::function<void()> & work)
  {
    // Validation/fsync must not hold the mutex used by GUI polls/inhibit/stop.
    lock.unlock();
    try {work();} catch (...) {lock.lock(); throw;}
    lock.lock();
  }
  void block(const std::string & reason)
  {
    data.fault = true; data.phase = ImportPhase::Blocked; data.message = reason;
    data.active_create = false; data.pending = false; command.reset(); queued.reset();
  }
  void failure(const std::string & reason)
  {
    data.message = reason; data.pending = false; data.active_create = false; command.reset();
    if (unresolved()) {data.phase = ImportPhase::Uncertain; next_lookup = now() + 1;} else {
      data.phase = ImportPhase::Rejected; data.destination.reset();
    }
  }
  void resolve(std::unique_lock<std::mutex> & lock, bool success, const std::string & status)
  {
    auto intent = *data.intent; intent.resolved = true; intent.success = success;
    intent.status = status;
    try {outside(lock, [&] {journal->save(intent);});} catch (const std::exception &) {
      block("cannot durably confirm import result; record retained; no replay"); return;
    }
    data.intent = intent; data.phase = success ? ImportPhase::Completed : ImportPhase::Rejected;
    data.active_create = data.pending = false; command.reset();
    data.message = success ? "import verified by independent readback; NOT executed" :
      "original import rejected; no file published; new intent requires new confirmation";
  }
  void dispatch(Stage stage)
  {
    if (closing || data.fault) {return;}
    try {
      if (stage == Stage::Details) {
        if (!details->service_is_ready()) {failure("readback unavailable; lookup only"); return;}
        auto request = std::make_shared<Details::Request>();
        request->mission_name = data.intent->name;
        auto sent = details->async_send_request(request);
        pending = Pending{stage, sent.request_id, now(), {}, sent.future.share()};
        data.phase = ImportPhase::Readback;
      } else {
        if (!client->service_is_ready()) {
          failure("import endpoint unavailable; no create retry"); return;
        }
        auto request = std::make_shared<Api::Request>();
        request->operation = stage == Stage::Create ? "create" : stage ==
          Stage::Lookup ? "lookup" : "session";
        if (stage == Stage::Create || stage == Stage::Lookup) {
          const auto & i = *data.intent;
          request->server_instance_id = i.instance; request->catalog_id = i.catalog;
          request->client_request_id = i.request;
          if (stage == Stage::Create) {
            request->mission_name = i.name; request->expected_content_sha256 = i.hash;
            request->content = command->document.content;
          }
        }
        auto sent = client->async_send_request(request);
        pending = Pending{stage, sent.request_id, now(), sent.future.share(), {}};
        if (stage == Stage::Create) {data.phase = ImportPhase::Sent;}
      }
      data.pending = true;
    } catch (const std::exception &) {failure("import transport failed; no retry or rollback");}
  }
  std::optional<ImportDestination> destination(const Api::Response & r)
  {
    if (!r.success || r.status != "session" || !hex_id(r.server_instance_id, 32) ||
      r.request_capacity < 1 || r.request_capacity > 256 ||
      r.retained_requests > r.request_capacity ||
      (r.enabled &&
      (!hex_id(r.catalog_id, 32) || !std::filesystem::path(r.catalog_directory).is_absolute())) ||
      r.catalog_directory.size() > 4096 || !easyfly_mission_fsm::missionTextUtf8(
        r.catalog_directory))
    {return {};}
    return ImportDestination{r.server_instance_id, r.catalog_id, r.catalog_directory, r.enabled,
      r.request_capacity, r.retained_requests, now()};
  }
  void response(std::unique_lock<std::mutex> & lock, Stage stage, const Api::Response & r)
  {
    if (r.status == "busy") {
      failure("import worker busy; lookup only for an existing intent"); return;
    }
    if (stage == Stage::Session || stage == Stage::Precheck || stage == Stage::FinalSession) {
      const auto d = destination(r);
      if (!d) {failure("invalid import destination reply"); return;}
      if (stage == Stage::Session) {
        data.destination = *d; data.phase = ImportPhase::Ready;
        data.message = "destination read; not flight admission";
        data.pending = false; return;
      }
      if (stage == Stage::FinalSession) {
        if (d->instance != data.intent->instance || d->catalog != data.intent->catalog ||
          !d->enabled)
        {
          block("instance/catalog changed during readback; record retained"); return;
        }
        data.destination = *d; resolve(lock, true, "imported"); return;
      }
      if (inhibited || !command || !same(*d, command->destination) || !d->enabled ||
        d->retained >= d->capacity)
      {failure("confirmed destination changed/disabled/full or inhibited; no create sent"); return;}
      const auto & doc = command->document;
      easyfly_mission_fsm::MissionValidation validation;
      outside(lock, [&] {validation = easyfly_mission_fsm::validateMissionText(doc.content);});
      if (!validation.valid() || easyfly_mission_fsm::missionFormatName(doc.content) != "yaml_v1") {
        failure("shared v1 validation failed before import; no create sent"); return;
      }
      if (closing || inhibited) {
        failure("import inhibited before journal/create; not sent"); return;
      }
      ImportIntent intent{scope, d->instance, d->catalog, new_id(), doc.name, doc.hash, "submitted",
        false, false};
      try {outside(lock, [&] {journal->save(intent);});} catch (const std::exception &) {
        block("cannot durably prepare import intent; no create sent; inspect local record"); return;
      }
      data.intent = intent; data.destination = *d;
      if (closing || inhibited) {resolve(lock, false, "not_dispatched"); return;}
      dispatch(Stage::Create); return;
    }
    const auto & i = *data.intent;
    if (r.server_instance_id != i.instance || r.catalog_id != i.catalog ||
      r.client_request_id != i.request || r.mission_name != i.name || r.content_sha256 != i.hash)
    {block("original import identity/result unknown or changed; retained fault lock"); return;}
    data.active_create = false;
    if (!r.resolved) {failure("original import unresolved; lookup only, no create retry"); return;}
    if (r.success && r.published && r.status == "imported") {dispatch(Stage::Details); return;}
    if (!r.success && !r.published && (r.status == "exists" || r.status == "rejected" ||
      r.status == "invalid" || r.status == "disabled" || r.status == "request_capacity"))
    {
      resolve(lock, false, r.status); return;
    }
    block("inconsistent original import outcome; retained fault lock");
  }
  void readback(const Details::Response & r)
  {
    const auto & i = *data.intent;
    if (!r.success || r.status != "valid_snapshot" || r.mission_name != i.name ||
      r.content_sha256 != i.hash || r.content.empty() || r.content.size() > 65536 ||
      !easyfly_mission_fsm::missionTextUtf8(r.content) ||
      easyfly_mission_fsm::missionContentSha256(r.content) != i.hash)
    {block("published file readback differed or was invalid; no overwrite/rollback"); return;}
    dispatch(Stage::FinalSession);
  }
  void tick(std::unique_lock<std::mutex> & lock)
  {
    if (pending) {
      auto p = *pending;
      const bool ready = p.stage == Stage::Details ?
        p.details.wait_for(std::chrono::seconds(0)) == std::future_status::ready :
        p.api.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
      if (ready) {
        pending.reset();
        try {
          if (p.stage == Stage::Details) {readback(*p.details.get());} else {
            response(lock, p.stage, *p.api.get());
          }
        } catch (const std::exception &) {failure("import reply failed; no create retry");}
      } else if (now() - p.sent >= config.response_timeout) {
        if (p.stage == Stage::Details) {details->remove_pending_request(p.transport);} else {
          client->remove_pending_request(p.transport);
        }
        pending.reset(); failure("reply timeout; outcome unconfirmed; lookup only");
      }
    }
    if (pending || data.fault) {return;}
    if (queued) {
      command = std::move(queued); queued.reset(); dispatch(Stage::Precheck);
    } else if (refresh_queued) {
      refresh_queued = false; dispatch(Stage::Session);
    } else if (unresolved() && now() >= next_lookup) {
      next_lookup = now() + 1; dispatch(Stage::Lookup);
    }
  }
  void run()
  {
    try {
      static std::atomic<unsigned> sequence{0};
      context = std::make_shared<rclcpp::Context>(); context->init(0, nullptr);
      rclcpp::NodeOptions options; options.context(context).use_global_arguments(false)
      .start_parameter_services(false).start_parameter_event_publisher(false);
      node = std::make_shared<rclcpp::Node>(
        "easyfly_ui_import_" + std::to_string(getpid()) +
        "_" + std::to_string(++sequence), options);
      client = node->create_client<Api>("/platform/mission/import");
      details = node->create_client<Details>("/platform/mission/details");
      rclcpp::ExecutorOptions eo; eo.context = context;
      executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(eo);
      executor->add_node(node);
      std::unique_lock<std::mutex> lock(mutex);
      scope = "domain=" + std::to_string(context->get_domain_id()) + ";localhost=1";
      std::optional<ImportIntent> loaded;
      outside(lock, [&] {loaded = journal->load();}); data.intent = loaded;
      if (data.intent && data.intent->scope != scope) {
        block("import journal ROS scope changed; no remote calls");
      } else if (unresolved()) {
        data.phase = ImportPhase::Uncertain; data.message = "restoring by original lookup only";
      } else if (data.intent) {
        data.phase = data.intent->success ? ImportPhase::Completed : ImportPhase::Rejected;
        data.message = "previous resolved import record; not a fresh catalog observation";
      }
      data.ready = true;
      while (!closing) {
        lock.unlock(); executor->spin_some(std::chrono::milliseconds(5)); lock.lock();
        if (!closing) {tick(lock);}
        wake.wait_for(lock, std::chrono::milliseconds(10), [this] {return closing;});
      }
    } catch (const std::exception &) {
      std::lock_guard<std::mutex> lock(mutex);
      block("local import worker/journal failed; no replay");
    }
    if (pending) {
      if (pending->stage == Stage::Details && details) {
        details->remove_pending_request(pending->transport);
      } else if (client) {client->remove_pending_request(pending->transport);}
    }
    pending.reset(); if (executor && node) {executor->remove_node(node);}
    details.reset(); client.reset(); node.reset(); executor.reset();
    if (context && context->is_valid()) {context->shutdown("local importer stopped");}
    context.reset(); journal.reset();
    std::lock_guard<std::mutex> lock(mutex);
    data.stopped = true; data.ready = false; data.pending = data.active_create = false;
  }
};
MissionImporter::MissionImporter(const ImportConfig & c)
: impl_(std::make_unique<Impl>(c)) {}
MissionImporter::~MissionImporter() = default;
std::shared_ptr<const ImportSnapshot> MissionImporter::snapshot() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex); return std::make_shared<ImportSnapshot>(
    impl_->data);
}
bool MissionImporter::read_destination()
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->closing || impl_->inhibited || !impl_->data.ready || impl_->data.pending ||
    impl_->data.fault || impl_->unresolved()) {return false;}
  impl_->data.pending = true; impl_->data.phase = ImportPhase::Reading;
  impl_->data.destination.reset(); impl_->refresh_queued = true; impl_->wake.notify_all();
  return true;
}
bool MissionImporter::import_confirmed(
  const ImportDocument & doc, const ImportDestination & d,
  const std::string & confirmation)
{
  if (!document_valid(doc) || confirmation.size() != 32) {return false;}
  std::lock_guard<std::mutex> lock(impl_->mutex); const auto & s = impl_->data;
  if (impl_->closing || impl_->inhibited || !s.ready || s.pending || s.fault ||
    impl_->unresolved() ||
    !s.destination || !s.destination->enabled || !same(d, *s.destination) ||
    d.received != s.destination->received || d.retained >= d.capacity ||
    !impl_->confirmations.consume(confirmation)) {return false;}
  impl_->data.pending = impl_->data.active_create = true; ++impl_->data.confirmations;
  impl_->data.phase = ImportPhase::Precheck; impl_->queued = Impl::Command{doc, d};
  impl_->wake.notify_all(); return true;
}
void MissionImporter::inhibit(bool active)
{std::lock_guard<std::mutex> lock(impl_->mutex); impl_->inhibited = active;}
void MissionImporter::stop() {impl_->stop();}
}  // namespace easyfly_ui_control
