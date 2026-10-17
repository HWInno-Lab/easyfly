// Copyright 2026 EasyFly Project
//
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

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <map>
#include <memory>
#include <random>
#include <sstream>
#include <cerrno>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <easyfly_msgs/action/execute_mission.hpp>
#include <easyfly_msgs/action/search_area.hpp>
#include <easyfly_msgs/srv/get_action_ownership.hpp>
#include <easyfly_core/ownership_registry.hpp>
#include <easyfly_msgs/srv/manage_mission.hpp>
#include <easyfly_msgs/srv/manage_reviewed_mission.hpp>
#include <easyfly_msgs/srv/get_mission_details.hpp>
#include <easyfly_msgs/srv/get_capabilities.hpp>
#include <easyfly_mission_fsm/mission_format.hpp>
#include "import_catalog.hpp"
#include "import_worker.hpp"

namespace fs = std::filesystem;
using Api = easyfly_msgs::srv::ManageMission;
using Managed = easyfly_msgs::srv::ManageReviewedMission;
using Action = easyfly_msgs::action::ExecuteMission;
using Child = rclcpp_action::ClientGoalHandle<Action>;
using Clock = std::chrono::steady_clock;
using Ownership = easyfly_msgs::srv::GetActionOwnership;
class MissionServices : public rclcpp::Node
{
  struct Intent;
  struct Execution;
  struct Reply
  {
    std::function<void(const Api::Response &)> send;
    std::weak_ptr<Intent> intent;
    Clock::time_point created{Clock::now()};
    bool sent{false};
  };
  struct Intent
  {
    std::string name, hash;
    bool resolved{false};
    Api::Response admission;
    std::shared_ptr<Execution> execution;
  };
  struct Execution
  {
    std::string id, status{"pending"}, message, snapshot;
    bool terminal{false}, cancel_requested{false}, cancel_sent{false};
    bool result_received{false}, ownership_inflight{false};
    int64_t ownership_request{0};
    Clock::time_point ownership_sent{};
    Child::SharedPtr child;
    easyfly_msgs::msg::MissionStatus feedback;
    Clock::time_point received{};
    std::shared_ptr<Reply> admission, cancellation;
    ~Execution()
    {
      if (terminal && !snapshot.empty()) {
        std::error_code ec; fs::remove(snapshot, ec);
      }
    }
  };

public:
  MissionServices()
  : Node("easyfly_mission_srvs")
  {
    rcl_interfaces::msg::ParameterDescriptor readonly; readonly.read_only = true;
    // ament --symlink-install may create an installed directory containing
    // individual resource symlinks. Resolve our trusted packaged anchor once;
    // explicitly configured catalogs still reject targets outside their root.
    const auto default_directory = fs::canonical(
      ament_index_cpp::get_package_share_directory("easyfly_mission_fsm") +
      "/missions/waypoint_demo.mission").parent_path().string();
    const auto configured_directory = declare_parameter<std::string>(
      "mission_directory", default_directory, readonly);
    root_ = fs::canonical(configured_directory);
    if (!fs::is_directory(root_)) {
      throw std::invalid_argument("mission_directory must be a directory");
    }
    importer_ = std::make_unique<ImportCatalog>(
      declare_parameter<bool>("enable_mission_import", false, readonly), configured_directory,
      instance_id_, declare_parameter<int>("import_request_capacity", 256, readonly),
      std::vector<std::string>{EASYFLY_IMPORT_SOURCE_ROOT, EASYFLY_IMPORT_INSTALL_ROOT,
        default_directory});
    ImportCatalog::Api::Request session_request; session_request.operation = "session";
    import_base_ = importer_->handle(session_request);
    import_ = create_service<ImportCatalog::Api>(
      "/platform/mission/import",
      [this](std::shared_ptr<rclcpp::Service<ImportCatalog::Api>> service,
      std::shared_ptr<rmw_request_id_t> header, ImportCatalog::Api::Request::SharedPtr request) {
        auto reply = [service, header](ImportCatalog::Api::Response response) {
          service->send_response(*header, response);
        };
        if (!import_worker_->submit(
          request,
          reply))
        {
          auto response = import_base_; response.success = false; response.resolved = false;
          response.status = "busy";
          response.message = "bounded import queue full; no retry of create";
          response.client_request_id = hexIdentity(
            request->client_request_id,
            32) ? request->client_request_id : "";
          response.mission_name = request->mission_name;
          response.content_sha256 = request->expected_content_sha256;
          reply(response);
        }
      });
    timeout_ = declare_parameter<double>("response_timeout", 5.0, readonly);
    freshness_ = declare_parameter<double>("feedback_max_age", 1.0, readonly);
    request_capacity_ = declare_parameter<int>("reviewed_request_capacity", 256, readonly);
    if (request_capacity_ < 1 || request_capacity_ > 256) {
      throw std::invalid_argument("reviewed_request_capacity must be 1..256");
    }
    if (!std::isfinite(timeout_) || timeout_ < 0.05 || timeout_ > 30 ||
      !std::isfinite(freshness_) || freshness_ < 0.05 || freshness_ > 10)
    {
      throw std::invalid_argument("invalid service timeouts");
    }
    client_ = rclcpp_action::create_client<Action>(this, "/execute_mission");
    ownership_client_ = create_client<Ownership>("/mission/ownership");
    search_client_ = rclcpp_action::create_client<easyfly_msgs::action::SearchArea>(
      this,
      "/search_area");
    service_ = create_service<Api>(
      "/platform/mission/manage",
      [this](std::shared_ptr<rclcpp::Service<Api>> service,
      std::shared_ptr<rmw_request_id_t> header,
      Api::Request::SharedPtr req) {
        if (req->operation == "start" || req->operation == "cancel") {
          Managed::Request trace; trace.operation = req->operation;
          trace.execution_id = req->execution_id;
          auditReviewed("legacy_write_received", header, trace);
        }
        auto reply = std::make_shared<Reply>();
        reply->send = [service, header](Api::Response response) {
          service->send_response(*header, response);
        };
        handle(reply, *req);
      });
    managed_ = create_service<Managed>(
      "/platform/mission/reviewed",
      [this](std::shared_ptr<rclcpp::Service<Managed>> service,
      std::shared_ptr<rmw_request_id_t> header, Managed::Request::SharedPtr request) {
        handleReviewed(service, header, *request);
      });
    details_ = create_service<easyfly_msgs::srv::GetMissionDetails>(
      "/platform/mission/details",
      [this](easyfly_msgs::srv::GetMissionDetails::Request::SharedPtr request,
      easyfly_msgs::srv::GetMissionDetails::Response::SharedPtr response) {
        const bool name_utf8 = request->mission_name.size() <= 128 &&
        easyfly_mission_fsm::missionTextUtf8(request->mission_name);
        response->mission_name = name_utf8 ? request->mission_name : "";
        try {
          if (!name_utf8) {throw std::invalid_argument("mission name must be valid UTF-8");}
          const auto text = validatedText(request->mission_name);
          if (!easyfly_mission_fsm::missionTextUtf8(text)) {
            throw std::invalid_argument("mission content must be valid UTF-8");
          }
          response->content_sha256 = easyfly_mission_fsm::missionContentSha256(text);
          response->content = text; response->success = true; response->status = "valid_snapshot";
          response->message = "validated catalog snapshot; not execution or flight admission";
        } catch (const std::exception & exc) {
          response->success = false; response->status = "invalid";
          response->content.clear(); response->content_sha256.clear();
          const auto error = std::string(exc.what()).substr(0, 2048);
          response->message = easyfly_mission_fsm::missionTextUtf8(error) ?
          error : "invalid mission content";
        }
      });
    capabilities_ = create_service<easyfly_msgs::srv::GetCapabilities>(
      "/platform/mission/capabilities",
      [this](easyfly_msgs::srv::GetCapabilities::Request::SharedPtr,
      easyfly_msgs::srv::GetCapabilities::Response::SharedPtr response) {
        response->header.stamp = now(); response->header.frame_id = "odom";
        response->names = {"legacy_mission", "versioned_yaml", "search"};
        response->implemented = {true, true, true};
        const auto ready = client_->action_server_is_ready() &&
        ownership_client_->service_is_ready() && !active_;
        response->ready = {ready, ready,
          ready && search_client_->action_server_is_ready()};
        response->details = {"legacy four-step format",
          "schema_version=1, odom m/rad/s, strict sequential eight types",
          "mission step supported; search backend/live flight admission separately required"};
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {tick();});
    // Construct last: later constructor exceptions must not abandon a joinable worker.
    import_worker_ = std::make_unique<ImportWorker>(
      [this](const ImportCatalog::Api::Request & request) {return importer_->handle(request);});
  }

private:
  static double elapsed(Clock::time_point t)
  {
    return std::chrono::duration<double>(Clock::now() - t).count();
  }
  static std::string identity()
  {
    std::random_device random; std::ostringstream out;
    for (int i = 0; i < 16; ++i) {
      out << std::hex << std::setw(2) << std::setfill('0') << (random() & 255);
    }
    return out.str();
  }
  Api::Response executionResponse(
    bool success, const std::string & status,
    const std::string & message, const std::shared_ptr<Execution> & e = {},
    const std::vector<std::string> & names = {})
  {
    Api::Response response; response.success = success; response.status = status;
    response.message = message; response.missions = names;
    if (e) {
      response.execution_id = e->id; response.current_status = e->feedback;
      response.execution_pending = !e->terminal;
      const auto & stamp = e->feedback.header.stamp;
      if (stamp.sec >= 0 && stamp.nanosec < 1000000000u && (stamp.sec || stamp.nanosec)) {
        const auto age = (now() - rclcpp::Time(stamp, get_clock()->get_clock_type())).seconds();
        response.feedback_valid = age >= -0.2 && age <= freshness_ &&
          elapsed(e->received) <= freshness_ &&
          std::isfinite(e->feedback.progress) && e->feedback.progress >= 0 &&
          e->feedback.progress <= 1 &&
          e->feedback.mission_phase <= easyfly_msgs::msg::MissionStatus::EMERGENCY &&
          e->feedback.current_waypoint >= -1 && e->feedback.total_waypoints >= 0;
      }
    }
    return response;
  }
  void respond(
    const std::shared_ptr<Reply> & p, bool success, const std::string & status,
    const std::string & message, const std::shared_ptr<Execution> & e = {},
    const std::vector<std::string> & names = {})
  {
    if (!p || p->sent) {return;}
    p->sent = true;
    const auto intent = p->intent.lock();
    // A busy reply must never associate someone else's execution with this intent.
    const auto response = executionResponse(
      success, status, message, intent && intent->execution != e ? nullptr : e, names);
    if (intent) {intent->resolved = true; intent->admission = response;}
    try {p->send(response);} catch (const std::exception & exc) {
      RCLCPP_WARN(get_logger(), "mission response: %s", exc.what());
    }
  }
  static bool hexIdentity(const std::string & value, size_t length)
  {
    return value.size() == length && std::all_of(
      value.begin(), value.end(), [](char c) {
        return (c >= '0' && c <= '9') ||
        (c >= 'a' && c <= 'f');
      });
  }
  void auditReviewed(
    const char * event, const std::shared_ptr<rmw_request_id_t> & header,
    const Managed::Request & req, const std::string & execution = {},
    const std::string & goal = {})
  {
    const auto token = [](const std::string & value, size_t length) {
        return hexIdentity(value, length) ? value : std::string("none");
      };
    std::ostringstream writer;
    if (header) {
      for (auto byte : header->writer_guid) {
        writer << std::hex << std::setw(2) << std::setfill('0') <<
          static_cast<unsigned>(static_cast<uint8_t>(byte));
      }
    }
    RCLCPP_INFO(
      get_logger(), "managed_audit event=%s pid=%d domain=%zu operation=%s instance=%s "
      "requested_instance=%s request=%s content_sha256=%s writer_guid=%s sequence=%" PRId64 " "
      "execution=%s goal=%s",
      event, static_cast<int>(getpid()), get_node_base_interface()->get_context()->get_domain_id(),
      req.operation == "cancel" ? "cancel" : "start", instance_id_.c_str(),
      token(req.server_instance_id, 32).c_str(), token(req.client_request_id, 32).c_str(),
      token(req.expected_content_sha256, 64).c_str(), writer.str().c_str(),
      header ? header->sequence_number : int64_t{-1},
      token(execution.empty() ? req.execution_id : execution, 32).c_str(), token(goal, 32).c_str());
  }
  void reviewedResponse(
    const std::shared_ptr<rclcpp::Service<Managed>> & service,
    const std::shared_ptr<rmw_request_id_t> & header, const std::string & request_id,
    const Api::Response & source, const std::shared_ptr<Intent> & intent = {})
  {
    Managed::Response response;
    response.success = source.success; response.status = source.status.substr(0, 64);
    const auto message = source.message.substr(0, 2048);
    response.message = easyfly_mission_fsm::missionTextUtf8(message) ? message : "invalid text";
    response.server_instance_id = instance_id_; response.client_request_id = request_id;
    response.request_capacity = request_capacity_; response.retained_requests = intents_.size();
    response.execution_id = source.execution_id; response.current_status = source.current_status;
    response.feedback_valid = source.feedback_valid;
    response.execution_pending = source.execution_pending;
    if (intent) {
      response.mission_name = intent->name; response.content_sha256 = intent->hash;
      response.admission_resolved = intent->resolved;
      response.admission_success = intent->admission.success;
      response.admission_status = intent->admission.status;
      if (intent->execution && intent->execution->child) {
        response.action_goal_id = easyfly_core::goalIdentity(
          intent->execution->child->get_goal_id());
      }
    }
    try {service->send_response(*header, response);} catch (const std::exception & exc) {
      RCLCPP_WARN(get_logger(), "reviewed mission response: %s", exc.what());
    }
  }
  void handleReviewed(
    const std::shared_ptr<rclcpp::Service<Managed>> & service,
    const std::shared_ptr<rmw_request_id_t> & header, const Managed::Request & req)
  {
    if (req.operation == "start" || req.operation == "cancel") {
      auditReviewed("write_received", header, req);
    }
    const auto send = [&](bool ok, const std::string & status, const std::string & message) {
        reviewedResponse(
          service, header, hexIdentity(req.client_request_id, 32) ?
          req.client_request_id : "", executionResponse(ok, status, message));
      };
    if (req.operation == "session") {
      send(true, "session", "instance identity only; not execution or flight admission"); return;
    }
    if (req.operation != "start" && req.operation != "lookup" && req.operation != "cancel") {
      send(false, "unsupported", "supported operations: session, start, lookup, cancel"); return;
    }
    if (req.server_instance_id != instance_id_) {
      send(false, "instance_mismatch", "old or unknown instance; do not replay or infer release");
      return;
    }
    if (!hexIdentity(req.client_request_id, 32)) {
      send(false, "invalid", "client_request_id must be 32 lowercase hex characters"); return;
    }
    const auto found = intents_.find(req.client_request_id);
    if (found != intents_.end()) {
      if (req.operation == "start") {auditReviewed("start_existing_request", header, req);}
      const auto & intent = found->second;
      if ((req.operation == "start" || req.operation == "cancel") &&
        (req.mission_name != intent->name || req.expected_content_sha256 != intent->hash))
      {
        send(false, "request_conflict", "request ID already bound to different content"); return;
      }
      const auto & e = intent->execution;
      if (req.operation == "cancel") {
        if (!e || !hexIdentity(req.execution_id, 32) || req.execution_id != e->id) {
          send(false, "execution_mismatch", "cancel must match this request's execution"); return;
        }
        auto reply = std::make_shared<Reply>();
        // Do not replace the original admission decision with a cancellation reply.
        reply->send = [this, service, header, id = req.client_request_id,
            weak = std::weak_ptr<Intent>(intent)](const Api::Response & response) {
            reviewedResponse(service, header, id, response, weak.lock());
          };
        cancelExecution(reply, e); return;
      }
      const auto source = e ? executionResponse(true, e->status, e->message, e) :
        intent->admission;
      reviewedResponse(service, header, req.client_request_id, source, intent); return;
    }
    if (req.operation != "start") {
      send(false, "not_found", "unknown request in this instance; not release evidence"); return;
    }
    if (req.mission_name.size() > 128 ||
      !easyfly_mission_fsm::missionTextUtf8(req.mission_name) ||
      !hexIdentity(req.expected_content_sha256, 64))
    {
      send(false, "invalid", "valid UTF-8 name and lowercase SHA-256 required"); return;
    }
    if (intents_.size() >= static_cast<size_t>(request_capacity_)) {
      send(false, "request_capacity", "records are never evicted; new managed starts refused");
      return;
    }
    auto intent = std::make_shared<Intent>();
    intent->name = req.mission_name; intent->hash = req.expected_content_sha256;
    intents_.emplace(req.client_request_id, intent);
    auditReviewed("start_new_request", header, req);
    auto reply = std::make_shared<Reply>(); reply->intent = intent;
    reply->send = [this, service, header, request = req, id = req.client_request_id,
        weak = std::weak_ptr<Intent>(intent)](const Api::Response & response) {
        const auto retained = weak.lock();
        const auto goal = retained && retained->execution && retained->execution->child ?
          easyfly_core::goalIdentity(retained->execution->child->get_goal_id()) : "";
        auditReviewed("start_response", header, request, response.execution_id, goal);
        reviewedResponse(service, header, id, response, retained);
      };
    try {
      // One in-memory read is both hashed and snapshotted. Never re-read after comparison.
      const auto text = validatedText(req.mission_name);
      if (!easyfly_mission_fsm::missionTextUtf8(text)) {
        throw std::invalid_argument("mission content must be valid UTF-8");
      }
      if (easyfly_mission_fsm::missionContentSha256(text) != intent->hash) {
        respond(reply, false, "content_changed", "catalog differs from reviewed bytes"); return;
      }
      startExecution(reply, text);
    } catch (const std::exception & exc) {respond(reply, false, "invalid", exc.what());}
  }
  fs::path catalogPath(const std::string & name) const
  {
    if (name.empty() || name.size() > 128 || fs::path(name).filename() != name || name[0] == '.') {
      throw std::invalid_argument("mission_name must be a catalog basename");
    }
    const auto named_extension = fs::path(name).extension().string();
    if (named_extension != ".mission" && named_extension != ".yaml" && named_extension != ".yml") {
      throw std::invalid_argument("unsupported mission name extension");
    }
    auto path = fs::canonical(root_ / name);
    if (path.parent_path() != root_ || !fs::is_regular_file(path) || fs::file_size(path) > 65536) {
      throw std::invalid_argument("mission outside catalog or too large");
    }
    const auto extension = path.extension().string();
    if (extension != ".mission" && extension != ".yaml" && extension != ".yml") {
      throw std::invalid_argument("unsupported mission extension");
    }
    return path;
  }
  std::string validatedText(const std::string & name) const
  {
    const auto path = catalogPath(name);
    const int fd = open(path.c_str(), O_RDONLY | O_NONBLOCK | O_NOFOLLOW | O_CLOEXEC);
    if (fd < 0) {throw std::invalid_argument("cannot open regular catalog file");}
    struct Guard {int fd; ~Guard() {close(fd);}} guard{fd};
    struct stat info {};
    if (fstat(fd, &info) || !S_ISREG(info.st_mode) || info.st_size > 65536) {
      throw std::invalid_argument("mission must be a regular file <=64KiB");
    }
    std::string text(65537, '\0'); size_t size = 0;
    while (size < text.size()) {
      auto n = read(fd, text.data() + size, text.size() - size);
      if (n < 0 && errno == EINTR) {continue;}
      if (n < 0) {throw std::invalid_argument("mission read failed");}
      if (!n) {break;}
      size += static_cast<size_t>(n);
    }
    if (size > 65536) {throw std::invalid_argument("mission too large");}
    text.resize(size);
    const auto check = easyfly_mission_fsm::validateMissionText(text);
    if (!check.valid()) {throw std::invalid_argument(check.error);}
    return text;
  }
  static std::string snapshot(const std::string & text)
  {
    char path[] = "/tmp/easyfly-d2-mission-XXXXXX";
    int fd = mkstemp(path);
    if (fd < 0) {throw std::runtime_error("cannot create private mission snapshot");}
    size_t written = 0;
    while (written < text.size()) {
      const auto n = write(fd, text.data() + written, text.size() - written);
      if (n <= 0) {close(fd); unlink(path); throw std::runtime_error("snapshot write failed");}
      written += static_cast<size_t>(n);
    }
    close(fd); return path;
  }
  void handle(const std::shared_ptr<Reply> & p, const Api::Request & req)
  {
    try {
      if (req.operation == "list") {
        std::vector<std::string> names;
        size_t entries = 0;
        for (const auto & entry : fs::directory_iterator(root_)) {
          if (++entries > 256) {
            throw std::invalid_argument("catalog exceeds 256 directory entries");
          }
          try {
            catalogPath(entry.path().filename().string()); names.push_back(
              entry.path().filename().string());
          } catch (const std::exception &) {}
        }
        std::sort(names.begin(), names.end()); respond(
          p, true, "catalog",
          "schema v1 YAML and legacy flat format", {},
          names);
      } else if (req.operation == "validate" || req.operation == "start") {
        const auto text = validatedText(req.mission_name);
        if (req.operation == "validate") {
          respond(p, true, "valid", "full static validation; live flight admission is separate");
          return;
        }
        startExecution(p, text);
      } else if (req.operation == "query" || req.operation == "cancel") {
        auto found = history_.find(req.execution_id);
        if (req.execution_id.empty() || found == history_.end()) {
          respond(p, false, "not_found", "unknown execution ID"); return;
        }
        auto e = found->second;
        if (req.operation == "query") {respond(p, true, e->status, e->message, e); return;}
        cancelExecution(p, e);
      } else {respond(p, false, "unsupported", "unknown operation");}
    } catch (const std::exception & exc) {respond(p, false, "invalid", exc.what());}
  }
  void startExecution(const std::shared_ptr<Reply> & p, const std::string & text)
  {
    if (active_) {
      respond(p, false, "busy", "previous execution ownership unresolved", active_); return;
    }
    if (!client_->action_server_is_ready() || !ownership_client_->service_is_ready()) {
      respond(p, false, "unsupported", "execute_mission or ownership protocol unavailable");
      return;
    }
    auto e = std::make_shared<Execution>(); e->id = identity(); e->admission = p;
    e->snapshot = snapshot(text);
    if (auto intent = p->intent.lock()) {intent->execution = e;}
    active_ = e; history_[e->id] = e; history_order_.push_back(
      e->id);
    rclcpp_action::Client<Action>::SendGoalOptions callbacks;
    callbacks.goal_response_callback = [this, e](Child::SharedPtr child) {
        e->child = child;
        if (!child) {
          e->terminal = true; e->status = "rejected";
          e->message = "execution rejected by backend";
          respond(e->admission, false, e->status, e->message, e); resolve(e); return;
        }
        e->status = "running"; respond(
          e->admission, true, "accepted",
          "accepted, not completed", e);
        if (e->cancel_requested) {cancel(e);}
      };
    callbacks.feedback_callback =
      [e](Child::SharedPtr, std::shared_ptr<const Action::Feedback> feedback) {
        if (!e->terminal) {e->feedback = feedback->current_status; e->received = Clock::now();}
      };
    callbacks.result_callback = [this, e](const Child::WrappedResult & result) {
        e->result_received = true;
        e->status = result.code == rclcpp_action::ResultCode::CANCELED ? "canceled" :
          result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
          result.result->success ? "succeeded" : "failed";
        e->message = result.result ? result.result->message : "missing backend result";
      };
    Action::Goal goal; goal.mission_file = e->snapshot;
    try {client_->async_send_goal(goal, callbacks);} catch (const std::exception & exc) {
      e->status = "fault"; e->message = exc.what(); e->cancel_requested = true;
      respond(p, false, e->status, "send failed; ownership not confirmed", e);
    }
  }
  void cancelExecution(const std::shared_ptr<Reply> & p, const std::shared_ptr<Execution> & e)
  {
    if (e->terminal || e != active_) {
      respond(p, false, e->status, "execution is already terminal", e); return;
    }
    if (e->cancellation && !e->cancellation->sent) {
      respond(p, false, "busy", "cancellation already pending", e); return;
    }
    e->cancellation = p; e->cancel_requested = true; cancel(e);
  }
  void cancel(const std::shared_ptr<Execution> & e)
  {
    if (e->terminal || e->result_received || !e->child || e->cancel_sent) {return;}
    e->cancel_sent = true; e->status = "canceling";
    try {
      client_->async_cancel_goal(
        e->child, [this, e](auto response) {
          const auto owned = e->child->get_goal_id();
          const bool accepted = response && response->return_code == 0 &&
          std::any_of(
            response->goals_canceling.begin(), response->goals_canceling.end(),
            [&](const auto & info) {return info.goal_id.uuid == owned;});
          respond(
            e->cancellation, accepted, accepted ? "cancel_requested" : "cancel_rejected",
            "cancel response does not confirm terminal", e);
          if (!accepted && !e->terminal && !e->result_received) {
            e->status = "fault"; e->cancel_sent = false; e->cancel_requested = false;
          }
        });
    } catch (const std::exception & exc) {
      respond(e->cancellation, false, "fault", exc.what(), e);
      if (!e->result_received) {e->status = "fault";}
      e->cancel_sent = false; e->cancel_requested = false;
    }
  }
  void resolve(const std::shared_ptr<Execution> & e)
  {
    respond(e->cancellation, e->status == "canceled", e->status, "real terminal observed", e);
    if (!e->snapshot.empty()) {
      std::error_code error; fs::remove(e->snapshot, error); e->snapshot.clear();
    }
    if (active_ == e) {active_.reset();}
    while (history_.size() > 32) {
      auto found = history_.find(history_order_.front());
      if (found == history_.end() || !found->second->terminal) {break;}
      history_.erase(found); history_order_.pop_front();
    }
  }
  void tick()
  {
    auto e = active_; if (!e) {return;}
    if (e->result_received && e->child && !e->terminal) {
      if (e->ownership_inflight && elapsed(e->ownership_sent) >= 1) {
        ownership_client_->remove_pending_request(e->ownership_request);
        e->ownership_inflight = false;
      }
      if (!e->ownership_inflight && elapsed(e->ownership_sent) >= .2 &&
        ownership_client_->service_is_ready())
      {
        auto request = std::make_shared<Ownership::Request>();
        request->goal_id = easyfly_core::goalIdentity(e->child->get_goal_id());
        e->ownership_inflight = true; e->ownership_sent = Clock::now();
        try {
          auto pending = ownership_client_->async_send_request(
            request,
            [this, e, id = request->goal_id](rclcpp::Client<Ownership>::SharedFuture future) {
              e->ownership_inflight = false;
              try {
                auto response = future.get();
                if (response && response->goal_id == id &&
                response->state == Ownership::Response::RELEASED)
                {
                  e->terminal = true; resolve(e);
                }
              } catch (const std::exception &) {}  // A failed query is not release evidence.
            });
          e->ownership_request = pending.request_id;
        } catch (const std::exception &) {e->ownership_inflight = false;}
      }
    }
    if (e->admission && !e->admission->sent && elapsed(e->admission->created) >= timeout_) {
      e->status = "pending_timeout"; e->cancel_requested = true;
      respond(
        e->admission, false, e->status,
        "admission timeout; retained for late exact cancellation", e);
    }
    if (e->cancellation && !e->cancellation->sent &&
      elapsed(e->cancellation->created) >= timeout_)
    {
      respond(
        e->cancellation, false, "cancel_timeout",
        "terminal not confirmed; execution remains owned", e);
    }
    if (e->cancel_requested) {cancel(e);}
  }
  fs::path root_;
  std::unique_ptr<ImportCatalog> importer_;
  std::unique_ptr<ImportWorker> import_worker_;
  ImportCatalog::Api::Response import_base_;
  rclcpp::Service<ImportCatalog::Api>::SharedPtr import_;
  double timeout_, freshness_;
  const std::string instance_id_{identity()};
  int request_capacity_;
  std::map<std::string, std::shared_ptr<Intent>> intents_;
  std::shared_ptr<Execution> active_;
  std::map<std::string, std::shared_ptr<Execution>> history_;
  std::deque<std::string> history_order_;
  rclcpp_action::Client<Action>::SharedPtr client_;
  rclcpp_action::Client<easyfly_msgs::action::SearchArea>::SharedPtr search_client_;
  rclcpp::Client<Ownership>::SharedPtr ownership_client_;
  rclcpp::Service<Api>::SharedPtr service_;
  rclcpp::Service<Managed>::SharedPtr managed_;
  rclcpp::Service<easyfly_msgs::srv::GetMissionDetails>::SharedPtr details_;
  rclcpp::Service<easyfly_msgs::srv::GetCapabilities>::SharedPtr capabilities_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MissionServices>());
  } catch (const std::exception & e) {
    fprintf(stderr, "mission services: %s\n", e.what()); rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
