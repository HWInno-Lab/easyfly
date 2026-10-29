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

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <future>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include "easyfly_ui_bridge/observer.hpp"
#include "easyfly_ui_bridge/camera_observations.hpp"
#include "easyfly_ui_bridge/special_observations.hpp"

namespace easyfly_ui_bridge
{
namespace
{
using Caps = easyfly_msgs::srv::GetCapabilities;
using Mission = easyfly_msgs::srv::ManageMission;
using Details = easyfly_msgs::srv::GetMissionDetails;
using Params = easyfly_msgs::srv::QueryParams;
using Perception = easyfly_msgs::srv::GetPerception;
using Image = sensor_msgs::msg::Image;
std::atomic<uint64_t> next_session{0};

const char * endpoint(Key key)
{
  switch (key) {
    case Key::FlightCapabilities: return "/platform/flight_control/capabilities";
    case Key::MissionCapabilities: return "/platform/mission/capabilities";
    case Key::PerceptionCapabilities: return "/platform/perception/capabilities";
    case Key::LandingCapabilities: return "/platform/precision_landing/capabilities";
    case Key::MissionList: case Key::MissionValidation: case Key::MissionQuery:
      return "/platform/mission/manage";
    case Key::MissionDetails: return "/platform/mission/details";
    case Key::Profile: case Key::UseSimTime: case Key::OperatorNote: case Key::ExportPretty:
      return "/platform/params/query";
    case Key::Targets: case Key::Obstacles: return "/platform/perception/query";
  }
  throw std::invalid_argument("invalid query key");
}
bool is_caps(Key key) {return key <= Key::LandingCapabilities;}
bool is_mission(Key key) {return key >= Key::MissionList && key <= Key::MissionQuery;}
bool is_params(Key key) {return key >= Key::Profile && key <= Key::ExportPretty;}
template<typename T>
void age(Observation<T> & observation, double now)
{
  auto & e = observation.evidence;
  if (e.received_steady > 0) {e.receipt_age = now - e.received_steady;}
  if (e.quality == Quality::ValidAtObservation && e.receipt_age > 2.0) {
    e.quality = Quality::Stale;
    e.reason = "query cache older than 2s; never a current flight permission";
  }
}
}  // namespace

struct Observer::Impl
{
  struct Request {Key key; uint64_t id; uint64_t epoch; std::string argument; double submitted;};
  struct Pending
  {
    Request request;
    std::function<bool()> ready;
    std::function<void()> complete;
    std::function<void()> remove;
  };
  BridgeConfig config;
  ClockWatch clock;
  UiSnapshot data;
  std::shared_ptr<const UiSnapshot> published;
  std::atomic<bool> closing{false};
  std::atomic<uint64_t> current_epoch{0};
  std::mutex mutex;
  std::condition_variable wake;
  std::deque<Request> queue;
  std::map<Key, uint64_t> reserved;
  std::map<std::string, double> last_request;
  uint64_t sequence{0};
  std::atomic<uint64_t> rejected{0};
  std::thread worker;
  std::shared_ptr<rclcpp::Context> context;
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::unique_ptr<easyfly_core::Client> sdk;
  std::map<Key, rclcpp::Client<Caps>::SharedPtr> caps_clients;
  rclcpp::Client<Mission>::SharedPtr mission_client;
  rclcpp::Client<Details>::SharedPtr details_client;
  rclcpp::Client<Params>::SharedPtr params_client;
  rclcpp::Client<Perception>::SharedPtr perception_client;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions;
  std::array<Image::ConstSharedPtr, 2> incoming_images;
  std::array<double, 2> image_received{};
  std::array<std::unique_ptr<CameraObservationBuffer>, 2> camera_buffers;
  std::unique_ptr<SpecialObservationBuffer> special_buffer;
  easyfly_msgs::msg::MissionStatus::ConstSharedPtr incoming_event;
  double event_received{0};
  std::map<Key, Pending> pending;
  std::map<Key, Observation<Perception::Response>> geometry_sources;
  std::optional<easyfly_msgs::msg::UAVState> last_state;
  int64_t blocked_state_stamp{-1};
  double state_first_received{0};
  bool state_conflict{false};

  explicit Impl(const BridgeConfig & options, std::shared_ptr<const UiSnapshot> previous_session)
  : config(options), clock(options.use_sim_time)
  {
    if (config.selected_profile.empty() || config.selected_profile.size() > 256) {
      throw std::invalid_argument("invalid UI profile label");
    }
    for (const auto & topic : config.image_topics) {
      if (topic.size() > 256 || (!topic.empty() && topic.front() != '/')) {
        throw std::invalid_argument("image bindings must be absolute topic names or empty");
      }
    }
    if (!config.image_topics[0].empty() && config.image_topics[0] == config.image_topics[1]) {
      throw std::invalid_argument("camera sources must be distinct");
    }
    for (const auto & binding : config.camera_perception) {
      for (const auto & topic : {binding.image_topic, binding.detections_topic,
          binding.tracks_topic, binding.targets_topic, binding.ellipse_topic,
          binding.color_line_topic})
      {
        if (topic.size() > 256 || (!topic.empty() && topic.front() != '/')) {
          throw std::invalid_argument("camera perception bindings must be absolute topics");
        }
      }
      if (binding.target_frame.empty() || binding.target_frame.size() > 256) {
        throw std::invalid_argument("invalid camera perception target frame");
      }
    }
    data.observation_session_id = ++next_session;
    data.selected_profile = config.selected_profile;
    if (config.selected_localization_backend != "external" &&
      config.selected_localization_backend != "fast_lio" &&
      config.selected_localization_backend != "openvins" &&
      config.selected_localization_backend != "fast_livo2" &&
      config.selected_localization_backend != "orbslam3" &&
      config.selected_localization_backend != "disabled")
    {throw std::invalid_argument("invalid selected localization backend");}
    data.selected_localization_backend = config.selected_localization_backend;
    data.localization_navigation_enabled = config.localization_navigation_enabled;
    data.navigation_localization.evidence.endpoint =
      "/localization/" + data.selected_localization_backend + "/navigation_status";
    data.localization.evidence.endpoint =
      (data.selected_localization_backend == "fast_livo2" ||
      data.selected_localization_backend == "orbslam3") ?
      "/localization/" + data.selected_localization_backend +
      "/status" : "/localization/openvins/status";
    if (previous_session) {
      if (!previous_session->stopped) {
        throw std::invalid_argument("previous observer is not stopped");
      }
      const std::size_t start = previous_session->events.size() > 1000 ?
        previous_session->events.size() - 1000 : 0;
      for (std::size_t i = start; i < previous_session->events.size(); ++i) {
        detail::append_event(data.events, previous_session->events[i]);
      }
      data.command_uncertain_observed = previous_session->command_uncertain_observed;
      if (data.command_uncertain_observed) {
        data.command_uncertain_observed->text =
          data.command_uncertain_observed->text.substr(0, 2048);
      }
    }
    for (Key key : {Key::FlightCapabilities, Key::MissionCapabilities,
        Key::PerceptionCapabilities, Key::LandingCapabilities})
    {data.capabilities[key].evidence.endpoint = endpoint(key);}
    for (Key key : {Key::MissionList, Key::MissionValidation, Key::MissionQuery}) {
      data.missions[key].evidence.endpoint = endpoint(key);
    }
    for (Key key : {Key::Profile, Key::UseSimTime, Key::OperatorNote, Key::ExportPretty}) {
      data.parameters[key].evidence.endpoint = endpoint(key);
    }
    data.mission_details.evidence.endpoint = endpoint(Key::MissionDetails);
    for (Key key : {Key::Targets, Key::Obstacles}) {
      data.perception[key].evidence.endpoint = endpoint(key);
      data.perception_reports[key].evidence.endpoint = endpoint(key);
    }
    publish();
    worker = std::thread([this] {run();});
  }
  ~Impl()
  {
    stop();
    if (worker.joinable()) {worker.join();}
  }
  void stop()
  {
    closing.store(true);
    wake.notify_all();
  }
  RequestReceipt submit(Key key, const std::string & argument = "")
  {
    std::lock_guard<std::mutex> lock(mutex);
    if (closing.load()) {return {0, RequestStatus::Closed};}
    if (reserved.count(key) || reserved.size() >= 8) {
      ++rejected;
      return {0, RequestStatus::Busy};
    }
    const double now = detail::steady_seconds();
    auto & last = last_request[endpoint(key)];
    if (last > 0 && now - last < 1.0) {
      ++rejected;
      return {0, RequestStatus::RateLimited};
    }
    last = now;
    const uint64_t id = ++sequence;
    reserved[key] = id;
    queue.push_back({key, id, current_epoch.load(), argument, now});
    wake.notify_all();
    return {id, RequestStatus::Queued};
  }
  void release(const Request & request)
  {
    std::lock_guard<std::mutex> lock(mutex);
    const auto found = reserved.find(request.key);
    if (found != reserved.end() && found->second == request.id) {reserved.erase(found);}
  }
  Evidence evidence(const Request & request, Quality quality, const std::string & reason)
  {
    Evidence e;
    e.quality = quality; e.reason = reason.substr(0, 2048); e.endpoint = endpoint(request.key);
    e.request_id = request.id; e.time_epoch = request.epoch;
    e.received_steady = detail::steady_seconds();
    e.first_source_steady = e.received_steady;
    return e;
  }
  void failure(const Request & request, Quality quality, const std::string & reason)
  {
    const auto e = evidence(request, quality, reason);
    // A failed attempt supersedes current evidence, not historical fault records.
    if (is_caps(request.key)) {
      data.capabilities[request.key] = {nullptr, e};
    } else if (is_mission(request.key)) {
      data.missions[request.key] = {nullptr, e};
    } else if (request.key == Key::MissionDetails) {
      data.mission_details = {nullptr, e}; data.mission_plan.reset(); data.mission_format.clear();
    } else if (is_params(request.key)) {data.parameters[request.key] = {nullptr, e};} else {
      data.perception[request.key] = {nullptr, e};
      data.perception_reports[request.key] = PerceptionReport();
      data.perception_reports[request.key].evidence = e;
    }
  }
  void event(const std::string & text)
  {
    detail::append_event(
      data.events, {detail::steady_seconds(), current_epoch.load(), text,
        data.observation_session_id});
  }
  void reset_epoch()
  {
    blocked_state_stamp = last_state ? detail::stamp_ns(last_state->header.stamp) : -1;
    last_state.reset(); state_first_received = 0; state_conflict = false;
    for (auto & item : pending) {item.second.remove(); ++data.counters.discarded_replies;}
    pending.clear();
    geometry_sources.clear();
    {
      std::lock_guard<std::mutex> lock(mutex);
      data.counters.discarded_replies += queue.size();
      queue.clear(); reserved.clear();
      current_epoch.store(clock.state(detail::steady_seconds()).epoch);
    }
    for (auto & item : data.capabilities) {
      item.second = {}; item.second.evidence.endpoint = endpoint(item.first);
    }
    for (auto & item : data.missions) {
      item.second = {}; item.second.evidence.endpoint = endpoint(item.first);
    }
    for (auto & item : data.parameters) {
      item.second = {}; item.second.evidence.endpoint = endpoint(item.first);
    }
    data.mission_details = {}; data.mission_plan.reset(); data.mission_format.clear();
    data.mission_details.evidence.endpoint = endpoint(Key::MissionDetails);
    for (auto & item : data.perception) {
      item.second = {}; item.second.evidence.endpoint = endpoint(item.first);
    }
    for (auto & item : data.perception_reports) {
      item.second = PerceptionReport(); item.second.evidence.endpoint = endpoint(item.first);
    }
    for (auto & image : data.images) {image = {};}
    for (auto & buffer : camera_buffers) {if (buffer) {buffer->clear();}}
    data.camera_perception = {};
    if (special_buffer) {special_buffer->clear();} data.special = {};
    data.task_backends.clear(); data.backend_graph_observed = 0;
    incoming_images = {}; incoming_event.reset();
    data.trajectory.clear();
    data.localization = {};
    data.navigation_localization = {};
    data.navigation_localization.evidence.endpoint =
      "/localization/" + data.selected_localization_backend + "/navigation_status";
    data.localization.evidence.endpoint =
      (data.selected_localization_backend == "fast_livo2" ||
      data.selected_localization_backend == "orbslam3") ?
      "/localization/" + data.selected_localization_backend +
      "/status" : "/localization/openvins/status";
    // The event is historical and has no execution identity; keep original epoch.
    data.last_mission_event.evidence.quality = Quality::ClockReset;
    event("ROS clock moved backwards: current layers and read-only requests invalidated");
  }
  void initialize()
  {
    context = std::make_shared<rclcpp::Context>();
    context->init(0, nullptr);
    rclcpp::NodeOptions options;
    options.context(context).use_global_arguments(false);
    options.parameter_overrides({rclcpp::Parameter("use_sim_time", config.use_sim_time)});
    node = std::make_shared<rclcpp::Node>(
      "easyfly_ui_observer_" + std::to_string(data.observation_session_id), options);
    rclcpp::ExecutorOptions eo; eo.context = context;
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(eo);
    executor->add_node(node);
    easyfly_core::ClientOptions sdk_options;
    sdk_options.use_sim_time = config.use_sim_time;
    sdk = std::make_unique<easyfly_core::Client>(sdk_options);
    for (const auto & item : data.capabilities) {
      caps_clients[item.first] = node->create_client<Caps>(endpoint(item.first));
    }
    mission_client = node->create_client<Mission>(endpoint(Key::MissionList));
    details_client = node->create_client<Details>(endpoint(Key::MissionDetails));
    params_client = node->create_client<Params>(endpoint(Key::Profile));
    perception_client = node->create_client<Perception>(endpoint(Key::Targets));
    if (config.use_sim_time) {
      subscriptions.push_back(
        node->create_subscription<rosgraph_msgs::msg::Clock>(
          "/clock", rclcpp::QoS(1).best_effort(),
          [this](rosgraph_msgs::msg::Clock::ConstSharedPtr m) {
            if (clock.observe(detail::stamp_ns(m->clock), detail::steady_seconds())) {
              reset_epoch();
            }
          }));
    }
    subscriptions.push_back(
      node->create_subscription<easyfly_msgs::msg::LocalizationStatus>(
        data.localization.evidence.endpoint, rclcpp::QoS(1).reliable().transient_local(),
        [this](easyfly_msgs::msg::LocalizationStatus::ConstSharedPtr m) {
          const double received = detail::steady_seconds();
          const auto structural = detail::validate_localization(
            *m,
            {Quality::ValidAtObservation, detail::stamp_ns(m->header.stamp), 0}, 0);
          // Bound retained strings and reject malformed identities before storing.
          const std::string expected_backend =
          (data.selected_localization_backend == "fast_livo2" ||
          data.selected_localization_backend == "orbslam3") ?
          data.selected_localization_backend : "openvins";
          if (m->backend_id != expected_backend || (structural == Quality::Invalid && (
            m->session_id.empty() || m->session_id.size() > 128 || m->reason.size() > 2048 ||
            m->last_rejection.size() > 2048 || m->header.frame_id.size() > 256 ||
            m->tracked_frame.size() > 256)))
          {
            data.localization.value.reset();
            data.localization.evidence.quality = Quality::Invalid;
            data.localization.evidence.reason = "invalid localization status envelope";
            return;
          }
          auto & observation = data.localization;
          auto & e = observation.evidence;
          const bool same_source = observation.value &&
          observation.value->session_id == m->session_id &&
          observation.value->generation == m->generation &&
          observation.value->header.stamp == m->header.stamp;
          if (!same_source) {e.first_source_steady = received;}
          e.received_steady = received; e.time_epoch = data.clock.epoch;
          e.reason = "raw localization observation; navigation admission is checked separately";
          observation.value = std::move(m);
        }));
    if (config.localization_navigation_enabled &&
      (config.selected_localization_backend == "openvins" ||
      config.selected_localization_backend == "fast_livo2" ||
      config.selected_localization_backend == "orbslam3"))
    {
      subscriptions.push_back(
        node->create_subscription<easyfly_msgs::msg::LocalizationStatus>(
          data.navigation_localization.evidence.endpoint,
          rclcpp::QoS(1).reliable().transient_local(),
          [this](easyfly_msgs::msg::LocalizationStatus::ConstSharedPtr m) {
            auto & observation = data.navigation_localization;
            auto & e = observation.evidence;
            if (m->backend_id != data.selected_localization_backend + "_navigation" ||
            m->session_id.size() > 128 || m->reason.size() > 2048 ||
            m->last_rejection.size() > 2048 || m->header.frame_id.size() > 256 ||
            m->tracked_frame.size() > 256)
            {
              observation.value.reset(); e.quality = Quality::Invalid;
              e.reason = "invalid navigation localization envelope"; return;
            }
            const double received = detail::steady_seconds();
            if (!observation.value || observation.value->session_id != m->session_id ||
            observation.value->generation != m->generation ||
            observation.value->header.stamp != m->header.stamp)
            {e.first_source_steady = received;}
            e.received_steady = received; e.time_epoch = data.clock.epoch;
            e.reason = "aligned navigation observation; server retains map and flight admission";
            observation.value = std::move(m);
          }));
    }
    special_buffer = std::make_unique<SpecialObservationBuffer>(config.landing_optical_frame);
    const auto special_subscribe = [this](const char * topic, auto sample) {
        using Message = decltype(sample);
        subscriptions.push_back(
          node->create_subscription<Message>(
            topic,
            rclcpp::QoS(1).best_effort(), [this](typename Message::ConstSharedPtr m) {
              special_buffer->receive(std::move(m), detail::steady_seconds(), data.clock.epoch);
            }));
      };
    special_subscribe("/tag_observations", easyfly_msgs::msg::TagObservationArray{});
    special_subscribe("/landing_target", easyfly_msgs::msg::LandingTarget{});
    special_subscribe("/landing/guidance_target", easyfly_msgs::msg::LandingGuidanceTarget{});
    special_subscribe("/mission/step_feedback", easyfly_msgs::msg::MissionStepFeedback{});
    special_subscribe("/target_predictions", easyfly_msgs::msg::TargetPredictionArray{});
    special_subscribe("/obstacles_classified", easyfly_msgs::msg::ClassifiedObstacleArray{});
    subscriptions.push_back(
      node->create_subscription<easyfly_msgs::msg::MissionStatus>(
        "/mission_status", rclcpp::QoS(1),
        [this](easyfly_msgs::msg::MissionStatus::ConstSharedPtr m) {
          incoming_event = std::move(m); event_received = detail::steady_seconds();
        }));
    for (std::size_t i = 0; i < config.image_topics.size(); ++i) {
      if (config.image_topics[i].empty()) {continue;}
      const auto & binding = config.camera_perception[i];
      if (!binding.image_topic.empty() && binding.image_topic == config.image_topics[i]) {
        camera_buffers[i] = std::make_unique<CameraObservationBuffer>(binding);
        const auto subscribe = [this, i](const auto & topic, auto sample, auto callback) {
            using Message = decltype(sample);
            if (topic.empty()) {return;}
            subscriptions.push_back(
              node->create_subscription<Message>(
                topic,
                rclcpp::QoS(1).best_effort(), [this, i, callback](
                  typename Message::ConstSharedPtr message) {
                  (camera_buffers[i].get()->*callback)(
                    message, detail::steady_seconds(),
                    current_epoch.load());
                }));
          };
        subscribe(
          binding.detections_topic, easyfly_msgs::msg::DetectionInfoArray{},
          &CameraObservationBuffer::detections);
        subscribe(
          binding.tracks_topic, easyfly_msgs::msg::TrackInfoArray{},
          &CameraObservationBuffer::tracks);
        subscribe(
          binding.targets_topic, easyfly_msgs::msg::Target3DArray{},
          &CameraObservationBuffer::targets);
        subscribe(
          binding.ellipse_topic, easyfly_msgs::msg::EllipseObservationArray{},
          &CameraObservationBuffer::ellipses);
        subscribe(
          binding.color_line_topic, easyfly_msgs::msg::ColorLineObservationArray{},
          &CameraObservationBuffer::color_lines);
      }
      subscriptions.push_back(
        node->create_subscription<Image>(
          config.image_topics[i],
          rclcpp::QoS(1).best_effort(), [this, i](Image::ConstSharedPtr m) {
            if (camera_buffers[i]) {
              camera_buffers[i]->image(m, detail::steady_seconds(), current_epoch.load());
            }
            const bool same_pending_source = incoming_images[i] &&
            incoming_images[i]->header == m->header;
            if (incoming_images[i]) {++data.counters.dropped_images;}
            incoming_images[i] = std::move(m);
            if (!same_pending_source) {image_received[i] = detail::steady_seconds();}
          }));
    }
    data.running = true;
    event("Read-only observation started; selected profile is not backend confirmation");
  }
  template<typename Service, typename Handler>
  void send(
    const Request & request, const std::shared_ptr<rclcpp::Client<Service>> & client,
    const std::shared_ptr<typename Service::Request> & input, Handler handler)
  {
    if (!client->service_is_ready()) {
      failure(request, Quality::Unavailable, "read-only endpoint not discovered");
      release(request);
      return;
    }
    auto sent = client->async_send_request(input);
    const int64_t transport_id = sent.request_id;
    auto future = sent.future.share();
    pending.emplace(
      request.key, Pending{request,
        [future] {return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;},
        [future, handler] {handler(future.get());},
        [client, transport_id] {client->remove_pending_request(transport_id);}});
  }
  void dispatch(const Request & request)
  {
    if (is_caps(request.key)) {
      send<Caps>(
        request, caps_clients.at(request.key), std::make_shared<Caps::Request>(),
        [this, request](Caps::Response::SharedPtr response) {
          Quality quality = detail::validate_capabilities(*response);
          if (quality == Quality::ValidAtObservation) {
            // Header carries query time, not a heartbeat or authorization.
            quality = detail::source_quality(response->header, data.clock, 0, 2.0, false);
            if (request.key == Key::FlightCapabilities) {
              for (const auto & text : response->details) {
                const auto uncertain = detail::command_uncertain(text);
                if (uncertain && *uncertain) {
                  data.command_uncertain_observed = Event{
                    detail::steady_seconds(),
                    current_epoch.load(), "command_uncertain observed; manual check required",
                    data.observation_session_id};
                }
              }
            }
          }
          const bool retain = detail::validate_capabilities(
            *response) == Quality::ValidAtObservation;
          data.capabilities[request.key] = {retain ? response : nullptr,
            evidence(
              request, quality,
              "per-entry query observation; JSON diagnostic may be unknown")};
        });
    } else if (is_mission(request.key)) {
      auto input = std::make_shared<Mission::Request>();
      if (request.key == Key::MissionList) {
        input->operation = "list";
      } else if (request.key == Key::MissionValidation) {
        input->operation = "validate"; input->mission_name = request.argument;
      } else {input->operation = "query"; input->execution_id = request.argument;}
      send<Mission>(
        request, mission_client, input,
        [this, request](Mission::Response::SharedPtr response) {
          const auto quality = detail::validate_mission(*response, request.key, request.argument);
          const bool retain = quality != Quality::BudgetExceeded && quality != Quality::Invalid;
          data.missions[request.key] = {retain ? response : nullptr,
            evidence(
              request, quality,
              "service success and execution_pending are independent of task status")};
        });
    } else if (request.key == Key::MissionDetails) {
      auto input = std::make_shared<Details::Request>(); input->mission_name = request.argument;
      send<Details>(
        request, details_client, input,
        [this, request](Details::Response::SharedPtr response) {
          easyfly_mission_fsm::MissionValidation plan;
          const auto quality = detail::validate_mission_details(*response, request.argument, &plan);
          const bool retain = quality == Quality::ValidAtObservation || quality == Quality::Unknown;
          data.mission_details = {retain ? response : nullptr,
            evidence(request, quality, "validated content snapshot; not execution or admission")};
          data.mission_plan = quality == Quality::ValidAtObservation ?
          std::make_shared<const easyfly_mission_fsm::MissionValidation>(std::move(plan)) : nullptr;
          data.mission_format = quality == Quality::ValidAtObservation ?
          easyfly_mission_fsm::missionFormatName(response->content) : "";
        });
    } else if (is_params(request.key)) {
      auto input = std::make_shared<Params::Request>();
      switch (request.key) {
        case Key::Profile: input->param_name = "profile"; break;
        case Key::UseSimTime: input->param_name = "use_sim_time"; break;
        case Key::OperatorNote: input->param_name = "operator_note"; break;
        default: input->param_name = "export_pretty"; break;
      }
      // param_value deliberately remains empty, the wire protocol's read operation.
      send<Params>(
        request, params_client, input, [this, request](Params::Response::SharedPtr response) {
          Quality quality = response->success ? Quality::ValidAtObservation : Quality::Unknown;
          const bool json_valid = detail::parameter_json(response->current_value, request.key);
          if (response->message.size() > 2048 || response->current_value.size() > 4096) {
            quality = Quality::BudgetExceeded;
          } else if (response->success && !json_valid) {
            quality = Quality::Invalid;
          }
          data.parameters[request.key] = {
            quality == Quality::BudgetExceeded ? nullptr : response,
            evidence(request, quality, "read-only JSON value from fixed parameter whitelist")};
        });
    } else {
      auto input = std::make_shared<Perception::Request>();
      input->kind = request.key == Key::Targets ? "targets" : "obstacles";
      send<Perception>(
        request, perception_client, input,
        [this, request](Perception::Response::SharedPtr response) {
          const auto quality = detail::validate_perception(*response, request.key, data.clock);
          auto e =
          evidence(
            request, quality,
            "query-time perception status; not a live flight permit");
          auto & previous = geometry_sources[request.key];
          bool retain = quality == Quality::ValidAtObservation;
          const auto source_ns = detail::stamp_ns(response->observed_at);
          const auto previous_ns = previous.value ? detail::stamp_ns(
            previous.value->observed_at) : -1;
          if (retain && source_ns < previous_ns) {
            e.quality = Quality::Invalid; retain = false;
            e.reason = "out-of-order geometry source";
          } else if (retain && previous.value && source_ns == previous_ns) {
            e.first_source_steady = previous.evidence.first_source_steady;
            const bool same = request.key == Key::Targets ?
            previous.value->targets == response->targets :
            previous.value->obstacles == response->obstacles;
            if (!same || previous.evidence.quality == Quality::Invalid) {
              e.quality = Quality::Invalid;
              e.reason = "same geometry source stamp with different payload";
              previous.evidence.quality = Quality::Invalid;
              retain = false;
              ++data.counters.invalid_samples;
            } else {
              ++data.counters.duplicate_samples;
              const auto & header = request.key == Key::Targets ?
              response->targets.header : response->obstacles.header;
              e.quality = detail::source_quality(
                header, data.clock,
                detail::steady_seconds() - e.first_source_steady, 1.0);
            }
          }
          // Do not retain malformed/oversized geometry as a purported valid empty result.
          data.perception[request.key] = {retain ? response : nullptr, e};
          if (retain) {previous = data.perception[request.key];}
          auto & report = data.perception_reports[request.key];
          report.received = true; report.upstream_status = response->status;
          report.source_topic = response->source_topic.substr(0, 256);
          report.message = response->message.substr(0, 2048);
          report.observed_at = response->observed_at; report.reported_age_sec = response->age_sec;
          report.evidence = e;
        });
    }
  }
  void requests(double now)
  {
    std::deque<Request> ready;
    {
      std::lock_guard<std::mutex> lock(mutex);
      ready.swap(queue);
    }
    for (const auto & request : ready) {
      if (request.epoch != current_epoch.load() || closing.load()) {
        ++data.counters.discarded_replies; release(request); continue;
      }
      try {dispatch(request);} catch (const std::exception & e) {
        failure(request, Quality::Unknown, std::string("transport error: ") + e.what());
        release(request);
      }
    }
    for (auto it = pending.begin(); it != pending.end(); ) {
      auto & p = it->second;
      if (now - p.request.submitted >= 1.0) {
        p.remove(); ++data.counters.timed_out;
        failure(
          p.request, Quality::Unknown,
          "read-only request timed out; proves nothing about remote execution");
      } else if (!p.ready()) {++it; continue;} else {
        try {p.complete();} catch (const std::exception & e) {
          failure(p.request, Quality::Unknown, std::string("response error: ") + e.what());
        }
      }
      release(p.request); it = pending.erase(it);
    }
  }
  void observations(double now)
  {
    if (special_buffer) {
      data.special = special_buffer->snapshot(data.clock, now);
      const auto unique_source = [this](auto & observation) {
          if (observation.value && node->count_publishers(observation.evidence.endpoint) != 1) {
            observation.evidence.quality = Quality::Invalid;
            observation.evidence.reason = "specialized publisher missing or ambiguous";
          }
        };
      unique_source(data.special.tags); unique_source(data.special.landing);
      unique_source(data.special.guidance); unique_source(data.special.step_feedback);
      unique_source(data.special.predictions); unique_source(data.special.classified);
    }
    if (data.navigation_localization.value) {
      auto & e = data.navigation_localization.evidence;
      e.receipt_age = now - e.first_source_steady;
      e.source_age_available = data.clock.now_ns > 0;
      e.source_age = e.source_age_available ? static_cast<double>(data.clock.now_ns -
        detail::stamp_ns(data.navigation_localization.value->header.stamp)) / 1e9 : 0;
      e.quality = detail::validate_navigation_localization(
        *data.navigation_localization.value, data.selected_localization_backend,
        data.clock, e.receipt_age);
      if (node->count_publishers(e.endpoint) != 1) {
        e.quality = Quality::Invalid;
        e.reason = "navigation localization publisher missing or ambiguous";
      }
    }
    if (data.localization.value) {
      auto & e = data.localization.evidence;
      e.receipt_age = now - e.first_source_steady;
      e.source_age_available = data.clock.now_ns > 0;
      e.source_age = e.source_age_available ? static_cast<double>(data.clock.now_ns -
        detail::stamp_ns(data.localization.value->header.stamp)) / 1e9 : 0;
      e.quality =
        detail::validate_localization(*data.localization.value, data.clock, e.receipt_age);
      if (node->count_publishers(data.localization.evidence.endpoint) > 1) {
        e.quality = Quality::Invalid;
        e.reason = "multiple localization publishers; ambiguous session";
      }
    }
    if (now - data.state_graph_observed >= 1.0) {
      const auto sources = node->get_publishers_info_by_topic("/uav_state");
      data.state_publishers = sources.size(); data.state_matching_publishers = 0;
      for (const auto & source : sources) {
        if (source.topic_type() == "easyfly_msgs/msg/UAVState") {
          ++data.state_matching_publishers;
        }
      }
      data.state_graph_observed = now;
    }
    const auto state = sdk->state();
    const bool eligible_state = state.received &&
      detail::stamp_ns(state.value.header.stamp) != blocked_state_stamp;
    if (data.state_publishers > 1) {
      // No publisher identity exists in UAVState. Never merge two aircraft into one view.
      blocked_state_stamp = state.received ? detail::stamp_ns(state.value.header.stamp) : -1;
      last_state.reset(); data.trajectory.clear(); data.state = StateView();
      data.state.evidence.quality = Quality::Invalid;
      data.state.evidence.reason = "multiple /uav_state publishers; source identity ambiguous";
    } else if (eligible_state) {
      const int64_t ns = detail::stamp_ns(state.value.header.stamp);
      const int64_t previous = last_state ? detail::stamp_ns(last_state->header.stamp) : -1;
      if (!last_state || ns > previous) {
        last_state = state.value;
        state_first_received = now - state.receipt_age_seconds;
        state_conflict = false;
        auto view = detail::state_view(state, data.clock, state_first_received, now, false);
        if (view.position_valid) {
          detail::append_trajectory(
            data.trajectory,
            {ns, now, state.value.position_x,
              state.value.position_y, state.value.position_z});
        }
      } else if (ns < previous || state.value != *last_state) {
        state_conflict = true;
        ++data.counters.invalid_samples;
      }
      data.state = detail::state_view(state, data.clock, state_first_received, now, state_conflict);
    } else {
      data.state = StateView();
      if (state.received) {data.state.evidence.quality = Quality::ClockReset;}
    }
    detail::prune_trajectory(data.trajectory, now);
    for (size_t i = 0; i < camera_buffers.size(); ++i) {
      if (camera_buffers[i]) {
        data.camera_perception[i] = camera_buffers[i]->snapshot(data.clock, now);
      }
    }
    for (std::size_t i = 0; i < incoming_images.size(); ++i) {
      auto & image = data.images[i];
      if (incoming_images[i]) {
        auto input = std::move(incoming_images[i]);
        const auto ns = detail::stamp_ns(input->header.stamp);
        auto quality = detail::validate_image(*input);
        if (image.value && ns <= detail::stamp_ns(image.value->header.stamp)) {
          if (ns == detail::stamp_ns(image.value->header.stamp) && *input == *image.value) {
            ++data.counters.duplicate_samples;
          } else {
            ++data.counters.dropped_images;
            image.evidence.quality = Quality::Invalid;
            image.evidence.reason = "out-of-order or same-stamp conflicting image";
          }
        } else if (quality != Quality::ValidAtObservation) {
          ++data.counters.dropped_images;
          image = {};
          image.evidence.quality = quality;
          image.evidence.reason = "image encoding, layout or budget rejected";
        } else {
          image.value = input;
          image.evidence = {};
          image.evidence.quality = quality;
          image.evidence.received_steady = image_received[i];
          image.evidence.first_source_steady = image_received[i];
          image.evidence.time_epoch = current_epoch.load();
        }
      }
      image.evidence.endpoint = config.image_topics[i];
      if (image.value) {
        image.evidence.receipt_age = now - image.evidence.first_source_steady;
        image.evidence.source_age = static_cast<double>(
          data.clock.now_ns - detail::stamp_ns(image.value->header.stamp)) / 1e9;
        image.evidence.source_age_available = data.clock.now_ns > 0;
        if (image.evidence.quality != Quality::Invalid) {
          image.evidence.quality = detail::source_quality(
            image.value->header,
            data.clock, image.evidence.receipt_age, 1.0, false);
        }
      }
    }
    if (incoming_event) {
      auto input = std::move(incoming_event);
      const auto quality = detail::validate_event(*input);
      auto & last = data.last_mission_event;
      const bool old_event = last.value && last.evidence.time_epoch == current_epoch.load() &&
        detail::stamp_ns(input->header.stamp) <= detail::stamp_ns(last.value->header.stamp);
      if (last.value && input->header.stamp == last.value->header.stamp && *input == *last.value) {
        ++data.counters.duplicate_samples;
      } else if (old_event) {
        ++data.counters.invalid_samples;
        last.evidence.quality = Quality::Invalid;
        last.evidence.reason = "out-of-order or same-stamp conflicting global event";
      } else {
        last = {};
        last.evidence.quality = quality;
        last.evidence.received_steady = event_received;
        last.evidence.first_source_steady = event_received;
        last.evidence.time_epoch = current_epoch.load();
        last.evidence.reason = "global phase event; no execution ID and not a heartbeat";
        if (quality == Quality::ValidAtObservation) {
          last.value = input; event(input->phase_name + ": " + input->message);
        }
      }
    }
    data.last_mission_event.evidence.endpoint = "/mission_status";
    if (data.last_mission_event.value) {
      data.last_mission_event.evidence.receipt_age =
        now - data.last_mission_event.evidence.first_source_steady;
      data.last_mission_event.evidence.source_age_available = data.clock.now_ns > 0 &&
        data.last_mission_event.evidence.time_epoch == current_epoch.load();
      data.last_mission_event.evidence.source_age = static_cast<double>(data.clock.now_ns -
        detail::stamp_ns(data.last_mission_event.value->header.stamp)) / 1e9;
    }
    for (auto & item : data.capabilities) {
      age(item.second, now);
      if (item.second.value) {
        item.second.evidence.source_age = static_cast<double>(data.clock.now_ns -
          detail::stamp_ns(item.second.value->header.stamp)) / 1e9;
        item.second.evidence.source_age_available = data.clock.now_ns > 0 &&
          detail::stamp_ns(item.second.value->header.stamp) > 0;
        if (item.second.evidence.quality == Quality::ValidAtObservation) {
          item.second.evidence.quality = detail::source_quality(
            item.second.value->header,
            data.clock, item.second.evidence.receipt_age, 2.0, false);
        }
      }
    }
    for (auto & item : data.missions) {age(item.second, now);}
    age(data.mission_details, now);
    for (auto & item : data.parameters) {age(item.second, now);}
    for (auto & item : data.perception) {
      age(item.second, now);
      if (item.second.value) {
        const auto & v = *item.second.value;
        const auto & header = item.first == Key::Targets ? v.targets.header : v.obstacles.header;
        item.second.evidence.source_age = static_cast<double>(
          data.clock.now_ns - detail::stamp_ns(header.stamp)) / 1e9;
        item.second.evidence.source_age_available = data.clock.now_ns > 0;
        if (item.second.evidence.quality == Quality::ValidAtObservation) {
          item.second.evidence.quality = detail::source_quality(
            header, data.clock, now - item.second.evidence.first_source_steady, 1.0);
        }
      }
      data.perception_reports[item.first].evidence = item.second.evidence;
      auto & report = data.perception_reports[item.first];
      if (report.received && detail::stamp_ns(report.observed_at) > 0) {
        report.evidence.source_age = static_cast<double>(data.clock.now_ns -
          detail::stamp_ns(report.observed_at)) / 1e9;
        report.evidence.source_age_available = data.clock.now_ns > 0;
      }
    }
  }
  void publish()
  {
    data.produced_steady = detail::steady_seconds();
    data.time_epoch = current_epoch.load(); ++data.revision;
    data.counters.rejected_requests = rejected.load();
    {
      std::lock_guard<std::mutex> lock(mutex);
      data.pending_queries = reserved.size();
    }
    std::atomic_store(&published, std::make_shared<const UiSnapshot>(data));
  }
  void cleanup()
  {
    for (auto & item : pending) {item.second.remove();}
    pending.clear();
    geometry_sources.clear();
    {
      std::lock_guard<std::mutex> lock(mutex);
      queue.clear(); reserved.clear();
    }
    incoming_images = {}; incoming_event.reset(); subscriptions.clear();
    special_buffer.reset(); data.special = {};
    for (auto & buffer : camera_buffers) {buffer.reset();}
    data.camera_perception = {};
    if (special_buffer) {special_buffer->clear();} data.special = {};
    data.task_backends.clear(); data.backend_graph_observed = 0;
    caps_clients.clear(); mission_client.reset(); params_client.reset(); perception_client.reset();
    details_client.reset();
    if (executor) {executor->cancel(); executor->remove_node(node); executor.reset();}
    node.reset();
    if (context && context->is_valid()) {context->shutdown("read-only observer stopped");}
    context.reset();
    if (sdk) {sdk->close(2.0); sdk.reset();}
    data.running = false; data.stopped = true;
    data.clock.quality = Quality::Unknown;
    data.state.evidence.quality = Quality::Stale;
    data.state.position_valid = data.state.velocity_valid = data.state.attitude_valid = false;
    data.state.battery_valid = data.state.gps_valid = false;
    data.state.speed_mps.reset(); data.state.battery_percent.reset();
    for (auto & image : data.images) {image = {};}
    data.trajectory.clear();
    data.localization.evidence.quality = Quality::Stale;
    data.navigation_localization.evidence.quality = Quality::Stale;
    for (auto & item : data.capabilities) {item.second.evidence.quality = Quality::Stale;}
    for (auto & item : data.missions) {item.second.evidence.quality = Quality::Stale;}
    data.mission_details.evidence.quality = Quality::Stale;
    for (auto & item : data.parameters) {item.second.evidence.quality = Quality::Stale;}
    for (auto & item : data.perception) {item.second.evidence.quality = Quality::Stale;}
    for (auto & item : data.perception_reports) {item.second.evidence.quality = Quality::Stale;}
    event("Read-only observer resources released; other contexts untouched");
    publish();
  }
  void run()
  {
    try {
      if (!closing.load()) {initialize();}
      double last_publish = 0, last_poll = 0;
      bool targets = true;
      while (!closing.load() && context->is_valid()) {
        executor->spin_some(std::chrono::milliseconds(5));
        const double now = detail::steady_seconds();
        data.clock = clock.state(now, node->now().nanoseconds());
        if (config.poll_observations && now - last_poll >= 1.01) {
          data.backend_graph_observed = now;
          for (const auto & name :
            {"takeoff", "goto", "navigate", "track", "search", "explore", "precision_land", "land"})
          {
            data.task_backends[name] = sdk->server_ready(name);
          }
          for (Key key : {Key::FlightCapabilities, Key::MissionCapabilities,
              Key::PerceptionCapabilities, Key::LandingCapabilities}) {submit(key);}
          submit(targets ? Key::Targets : Key::Obstacles); targets = !targets; last_poll = now;
        }
        requests(now);
        if (now - last_publish >= .1) {observations(now); publish(); last_publish = now;}
        std::unique_lock<std::mutex> lock(mutex);
        wake.wait_for(
          lock, std::chrono::milliseconds(10), [this] {
            return closing.load() || !queue.empty();
          });
      }
    } catch (const std::exception & e) {
      data.local_error = std::string(e.what()).substr(0, 2048);
    }
    closing.store(true);
    cleanup();
  }
};

Observer::Observer(const BridgeConfig & config, std::shared_ptr<const UiSnapshot> previous_session)
: impl_(std::make_unique<Impl>(config, std::move(previous_session))) {}
Observer::~Observer() = default;
std::shared_ptr<const UiSnapshot> Observer::snapshot() const
{
  return std::atomic_load(&impl_->published);
}
void Observer::stop() {impl_->stop();}
RequestReceipt Observer::capabilities(CapabilitySource source)
{
  switch (source) {
    case CapabilitySource::Flight: return impl_->submit(Key::FlightCapabilities);
    case CapabilitySource::Mission: return impl_->submit(Key::MissionCapabilities);
    case CapabilitySource::Perception: return impl_->submit(Key::PerceptionCapabilities);
    case CapabilitySource::Landing: return impl_->submit(Key::LandingCapabilities);
  }
  return {0, RequestStatus::Invalid};
}
RequestReceipt Observer::list_missions() {return impl_->submit(Key::MissionList);}
RequestReceipt Observer::validate_mission(const std::string & name)
{
  return detail::basename(name) ? impl_->submit(Key::MissionValidation, name) :
         RequestReceipt{0, RequestStatus::Invalid};
}
RequestReceipt Observer::query_mission(const std::string & id)
{
  return detail::execution_id(id) ? impl_->submit(Key::MissionQuery, id) :
         RequestReceipt{0, RequestStatus::Invalid};
}
RequestReceipt Observer::mission_details(const std::string & name)
{
  return detail::catalog_name(name) ? impl_->submit(Key::MissionDetails, name) :
         RequestReceipt{0, RequestStatus::Invalid};
}
RequestReceipt Observer::read_parameter(Parameter parameter)
{
  switch (parameter) {
    case Parameter::Profile: return impl_->submit(Key::Profile);
    case Parameter::UseSimTime: return impl_->submit(Key::UseSimTime);
    case Parameter::OperatorNote: return impl_->submit(Key::OperatorNote);
    case Parameter::ExportPretty: return impl_->submit(Key::ExportPretty);
  }
  return {0, RequestStatus::Invalid};
}
RequestReceipt Observer::perception(PerceptionKind kind)
{
  switch (kind) {
    case PerceptionKind::Targets: return impl_->submit(Key::Targets);
    case PerceptionKind::Obstacles: return impl_->submit(Key::Obstacles);
  }
  return {0, RequestStatus::Invalid};
}
}  // namespace easyfly_ui_bridge
