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

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <mutex>
#include <thread>
#include <vector>

#include <easyfly_msgs/msg/uav_command.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include "easyfly_ui_bridge/observer.hpp"

using namespace std::chrono_literals;
using namespace easyfly_ui_bridge;  // NOLINT(build/namespaces)
using Mission = easyfly_msgs::srv::ManageMission;
using Details = easyfly_msgs::srv::GetMissionDetails;
using Caps = easyfly_msgs::srv::GetCapabilities;
using Params = easyfly_msgs::srv::QueryParams;
using Perception = easyfly_msgs::srv::GetPerception;
namespace
{
template<typename Predicate>
bool until(Predicate predicate, double seconds = 4)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {return true;}
    std::this_thread::sleep_for(10ms);
  }
  return predicate();
}
}  // namespace

class ObserverTest : public ::testing::Test
{
protected:
  std::shared_ptr<rclcpp::Context> context;
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::thread thread;
  std::vector<std::shared_ptr<void>> resources;
  std::vector<std::shared_ptr<void>> platform_resources;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher;
  rclcpp::Publisher<easyfly_msgs::msg::UAVState>::SharedPtr state_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher;
  rclcpp::Publisher<easyfly_msgs::msg::MissionStatus>::SharedPtr event_publisher;
  std::atomic<int64_t> now_ns{10000000000LL}, fixed_stamp{0};
  std::atomic<int64_t> geometry_stamp{0}, clock_step_ns{20000000LL};
  std::atomic<bool> advance{true}, publish_state{true};
  std::atomic<int> heartbeats{0}, goals{0}, cancels{0}, control_messages{0}, commands{0};
  std::atomic<int> mission_writes{0}, param_writes{0}, mission_calls{0};
  std::atomic<int> response_mode{0}, mission_delay_ms{0};
  std::atomic<int> details_mode{0}, details_delay_ms{0}, details_calls{0};
  std::mutex state_mutex;
  double position_x{0};

  builtin_interfaces::msg::Time stamp(int64_t ns = 0)
  {
    if (!ns) {ns = now_ns.load();}
    builtin_interfaces::msg::Time t;
    t.sec = static_cast<int32_t>(ns / 1000000000LL);
    t.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
    return t;
  }
  template<typename Action>
  void action(const std::string & name)
  {
    resources.push_back(
      rclcpp_action::create_server<Action>(
        node, name,
        [this](const auto &, auto) {++goals; return rclcpp_action::GoalResponse::REJECT;},
        [this](auto) {++cancels; return rclcpp_action::CancelResponse::REJECT;}, [](auto) {}));
  }
  void SetUp() override
  {
    // CTest supplies this isolated domain to both mock and SDK contexts.
    ASSERT_STREQ(std::getenv("ROS_DOMAIN_ID"), "92");
    context = std::make_shared<rclcpp::Context>(); context->init(0, nullptr);
    ASSERT_FALSE(std::getenv("EASYFLY_UI_TEST_FAIL_AFTER_CONTEXT"))
      << "injected setup failure after context";
    rclcpp::NodeOptions options; options.context(context).use_global_arguments(false);
    node = std::make_shared<rclcpp::Node>("ui_readonly_mock", options);
    clock_publisher = node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);
    state_publisher = node->create_publisher<easyfly_msgs::msg::UAVState>("/uav_state", 1);
    image_publisher = node->create_publisher<sensor_msgs::msg::Image>("/camera/color/image_raw", 1);
    event_publisher =
      node->create_publisher<easyfly_msgs::msg::MissionStatus>("/mission_status", 1);
    resources.push_back(
      node->create_subscription<easyfly_msgs::msg::UAVCommand>(
        "/uav_command", 10,
        [this](easyfly_msgs::msg::UAVCommand::ConstSharedPtr) {++control_messages;}));
    resources.push_back(
      node->create_subscription<mavros_msgs::msg::PositionTarget>(
        "/mavros/setpoint_raw/local", 10,
        [this](mavros_msgs::msg::PositionTarget::ConstSharedPtr) {++control_messages;}));
    resources.push_back(
      node->create_service<mavros_msgs::srv::CommandBool>(
        "/mavros/cmd/arming",
        [this](mavros_msgs::srv::CommandBool::Request::SharedPtr,
        mavros_msgs::srv::CommandBool::Response::SharedPtr) {++commands;}));
    resources.push_back(
      node->create_service<mavros_msgs::srv::SetMode>(
        "/mavros/set_mode",
        [this](mavros_msgs::srv::SetMode::Request::SharedPtr,
        mavros_msgs::srv::SetMode::Response::SharedPtr) {++commands;}));
    action<easyfly_msgs::action::Takeoff>("/takeoff");
    action<easyfly_msgs::action::Land>("/land");
    action<easyfly_msgs::action::PrecisionLand>("/precision_land");
    action<easyfly_msgs::action::GoToWaypoint>("/goto_waypoint");
    action<easyfly_msgs::action::NavigateTo>("/navigate");
    action<easyfly_msgs::action::FollowTrajectory>("/follow_trajectory");
    action<easyfly_msgs::action::TrackTarget>("/track_target");
    action<easyfly_msgs::action::ExecuteMission>("/execute_mission");
    action<easyfly_msgs::action::SearchArea>("/search_area");
    platform_resources.push_back(
      node->create_service<Mission>(
        "/platform/mission/manage",
        [this](Mission::Request::SharedPtr request, Mission::Response::SharedPtr response) {
          ++mission_calls;
          if (request->operation != "list" && request->operation != "validate" &&
          request->operation != "query") {++mission_writes;}
          const int delay = mission_delay_ms.load();
          if (delay) {std::this_thread::sleep_for(std::chrono::milliseconds(delay));}
          response->success = true;
          if (request->operation == "list") {
            response->status = "listed"; response->missions = {"demo.yaml"};
          } else if (request->operation == "validate") {response->status = "valid";} else {
            response->execution_id = response_mode ==
            2 ? std::string(32, 'b') : request->execution_id;
            response->status = "failed"; response->execution_pending = true;
            if (response_mode == 3) {response->success = false; response->status = "not_found";}
          }
        }));
    platform_resources.push_back(
      node->create_service<Details>(
        "/platform/mission/details",
        [this](Details::Request::SharedPtr request, Details::Response::SharedPtr response) {
          ++details_calls; const int delay = details_delay_ms.load();
          if (delay) {std::this_thread::sleep_for(std::chrono::milliseconds(delay));}
          response->mission_name = details_mode == 1 ? "wrong.yaml" : request->mission_name;
          response->success = true; response->status = "valid_snapshot";
          response->content = "type=hover\nwait=2\n";
          response->content_sha256 = easyfly_mission_fsm::missionContentSha256(response->content);
          if (details_mode == 2) {response->content_sha256 = std::string(64, '0');}
          if (details_mode == 3) {
            response->success = false; response->status = "invalid"; response->message = "missing";
            response->content.clear(); response->content_sha256.clear();
          }
        }));
    for (const auto * name :
      {"/platform/flight_control/capabilities", "/platform/mission/capabilities",
        "/platform/perception/capabilities", "/platform/precision_landing/capabilities"})
    {
      platform_resources.push_back(
        node->create_service<Caps>(
          name,
          [this](Caps::Request::SharedPtr, Caps::Response::SharedPtr response) {
            response->header.stamp = stamp(); response->header.frame_id = "odom";
            response->names = {"takeoff"}; response->implemented = {true};
            response->ready = {false};
            response->details = {response_mode == 1 ? "{broken" : "{\"command_uncertain\":true}"};
            if (response_mode == 4) {response->ready.clear();}
          }));
    }
    platform_resources.push_back(
      node->create_service<Params>(
        "/platform/params/query",
        [this](Params::Request::SharedPtr request, Params::Response::SharedPtr response) {
          if (!request->param_value.empty()) {++param_writes;}
          response->success = true;
          response->current_value = request->param_name == "profile" ||
          request->param_name == "operator_note" ?
          "\"navigation_dev\"" : "false";
        }));
    platform_resources.push_back(
      node->create_service<Perception>(
        "/platform/perception/query",
        [this](Perception::Request::SharedPtr request, Perception::Response::SharedPtr response) {
          response->status = Perception::Response::VALID; response->age_sec = 0;
          auto & header = request->kind ==
          "targets" ? response->targets.header : response->obstacles.header;
          header.stamp = stamp(geometry_stamp.load()); header.frame_id = "odom";
          response->observed_at = header.stamp;
          response->source_topic = request->kind == "targets" ? "/target_positions" : "/obstacles";
          if (response_mode == 5 && request->kind == "targets") {
            easyfly_msgs::msg::Target3D t; t.header = header; t.position_valid = true;
            response->targets.targets.push_back(t);
          }
        }));
    timer = node->create_wall_timer(
      20ms, [this] {
        ++heartbeats;
        if (advance) {now_ns.fetch_add(clock_step_ns.load());}
        rosgraph_msgs::msg::Clock c; c.clock = stamp(); clock_publisher->publish(c);
        if (publish_state) {
          easyfly_msgs::msg::UAVState s; s.header.stamp = stamp(
            fixed_stamp.load()); s.header.frame_id = "odom";
          s.connected = s.pose_valid = s.velocity_valid = s.battery_valid = true;
          s.battery_voltage = 12; s.battery_remaining = .5;
          {std::lock_guard<std::mutex> lock(state_mutex); s.position_x = position_x;}
          state_publisher->publish(s);
        }
      });
    rclcpp::ExecutorOptions eo; eo.context = context;
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(eo);
    executor->add_node(node); thread = std::thread([this] {executor->spin();});
    ASSERT_TRUE(until([this] {return heartbeats >= 2;}));
  }
  void TearDown() override
  {
    // Also safe when a fatal SetUp assertion returned before any resource existed.
    // Shutdown only this mock context; a late-starting spin observes it as stopped.
    if (context && context->is_valid()) {context->shutdown("mock complete");}
    if (thread.joinable()) {thread.join();}
    timer.reset(); platform_resources.clear(); resources.clear();
    if (executor && node) {executor->remove_node(node);}
    executor.reset();
    image_publisher.reset(); event_publisher.reset(); state_publisher.reset();
    clock_publisher.reset();
    node.reset(); context.reset();
    EXPECT_EQ(goals, 0); EXPECT_EQ(cancels, 0); EXPECT_EQ(control_messages, 0);
    EXPECT_EQ(commands, 0); EXPECT_EQ(mission_writes, 0); EXPECT_EQ(param_writes, 0);
  }
  BridgeConfig configuration()
  {
    BridgeConfig c; c.poll_observations = false; return c;
  }
  void running(Observer & observer)
  {
    ASSERT_TRUE(until([&] {return observer.snapshot()->state.position_valid;}));
    // Discovery readiness for the bridge's own node, without sending extra queries.
    ASSERT_TRUE(
      until(
        [&] {
          for (const auto & name : node->get_node_names()) {
            if (name.find("/easyfly_ui_observer_") == 0) {return true;}
          }
          return false;
        }));
    std::this_thread::sleep_for(200ms);
  }
  template<typename Submit>
  RequestReceipt request(Submit submit)
  {
    RequestReceipt receipt;
    EXPECT_TRUE(until([&] {receipt = submit(); return receipt.status == RequestStatus::Queued;}));
    return receipt;
  }
  void stopped(Observer & observer)
  {
    const int before = heartbeats.load(); observer.stop();
    ASSERT_TRUE(until([&] {return observer.snapshot()->stopped;}));
    ASSERT_TRUE(until([&] {return heartbeats > before + 2;}));
    EXPECT_TRUE(context->is_valid()); EXPECT_EQ(observer.snapshot()->pending_queries, 0U);
    EXPECT_EQ(observer.list_missions().status, RequestStatus::Closed);
  }
};

TEST_F(ObserverTest, CameraPerceptionExactFrameSubscriptionsRemainReadonlyAcrossReset)
{
  auto publisher = node->create_publisher<easyfly_msgs::msg::DetectionInfoArray>("/detections", 1);
  Observer observer(configuration()); running(observer);
  ASSERT_TRUE(until([&] {return publisher->get_subscription_count() == 1;}));
  auto image = sensor_msgs::msg::Image{};
  image.header.frame_id = "camera"; image.header.stamp = stamp();
  image.width = 640; image.height = 480; image.encoding = "rgb8"; image.step = 1920;
  image.data.assign(921600, 5);
  easyfly_msgs::msg::DetectionInfoArray detection; detection.header = image.header;
  easyfly_msgs::msg::DetectionInfo box; box.header = image.header; box.track_id = -1;
  box.class_name = "person"; box.confidence = .9; box.bbox_center_x = box.bbox_center_y = .5;
  box.bbox_width = box.bbox_height = .2; detection.detections.push_back(box);
  image_publisher->publish(image); publisher->publish(detection);
  ASSERT_TRUE(
    until(
      [&] {
        return observer.snapshot()->camera_perception[0].detection_image.value != nullptr;
      }));
  const auto before = observer.snapshot();
  ASSERT_TRUE(before->camera_perception[0].detections.value);
  EXPECT_FALSE(before->camera_perception[1].detection_image.value);
  EXPECT_EQ(before->camera_perception[0].detection_image.value->header, detection.header);
  // DDS is depth1 best-effort; graph depth may be unknown (0) on Fast DDS.
  for (const auto & subscription : node->get_subscriptions_info_by_topic("/detections")) {
    if (subscription.node_name().find("easyfly_ui_observer_") == 0) {
      EXPECT_EQ(subscription.qos_profile().reliability(), rclcpp::ReliabilityPolicy::BestEffort);
    }
  }
  now_ns = 1000000000LL;
  rosgraph_msgs::msg::Clock reset; reset.clock = stamp(); clock_publisher->publish(reset);
  ASSERT_TRUE(until([&] {return observer.snapshot()->time_epoch > before->time_epoch;}));
  EXPECT_FALSE(observer.snapshot()->camera_perception[0].detections.value);
  EXPECT_TRUE(before->camera_perception[0].detections.value);  // immutable old snapshot
  stopped(observer);
  EXPECT_EQ(observer.snapshot()->camera_perception[0].cached_images, 0U);
}
TEST_F(ObserverTest, DetailsMismatchTimeoutLateReplyAndReconnectFailClosed)
{
  Observer observer(configuration()); running(observer);
  EXPECT_EQ(observer.mission_details("../bad.yaml").status, RequestStatus::Invalid);
  for (int mode : {1, 2, 3}) {
    details_mode = mode;
    const auto receipt = request([&] {return observer.mission_details("demo.yaml");});
    ASSERT_TRUE(
      until(
        [&] {
          return observer.snapshot()->mission_details.evidence.request_id == receipt.id;
        }));
    auto snapshot = observer.snapshot(); EXPECT_FALSE(snapshot->mission_plan);
    EXPECT_EQ(
      snapshot->mission_details.evidence.quality,
      mode == 3 ? Quality::Unknown : Quality::Invalid);
    if (mode != 3) {EXPECT_FALSE(snapshot->mission_details.value);}
  }
  details_mode = 0; details_delay_ms = 1400;
  const auto receipt = request([&] {return observer.mission_details("demo.yaml");});
  ASSERT_TRUE(
    until(
      [&] {
        const auto snapshot = observer.snapshot();
        return snapshot->mission_details.evidence.request_id == receipt.id &&
        snapshot->mission_details.evidence.reason.find("timed out") != std::string::npos;
      }));
  std::this_thread::sleep_for(600ms);
  EXPECT_FALSE(observer.snapshot()->mission_details.value); EXPECT_FALSE(
    observer.snapshot()->mission_plan);
  stopped(observer);
  Observer reconnected(configuration(), observer.snapshot()); running(reconnected);
  EXPECT_FALSE(reconnected.snapshot()->mission_details.value);
  EXPECT_FALSE(reconnected.snapshot()->mission_plan); stopped(reconnected);
}
TEST_F(ObserverTest, DetailsAreDiscardedAcrossClockReset)
{
  Observer observer(configuration()); running(observer);
  details_delay_ms = 500;
  const auto receipt = request([&] {return observer.mission_details("demo.yaml");});
  ASSERT_TRUE(until([&] {return details_calls > 0;}));
  const auto before = observer.snapshot()->time_epoch; now_ns = 1000000000LL;
  rosgraph_msgs::msg::Clock reset; reset.clock = stamp(); clock_publisher->publish(reset);
  ASSERT_TRUE(until([&] {return observer.snapshot()->time_epoch > before;}));
  std::this_thread::sleep_for(700ms);
  EXPECT_FALSE(observer.snapshot()->mission_details.value); EXPECT_FALSE(
    observer.snapshot()->mission_plan);
  EXPECT_NE(observer.snapshot()->mission_details.evidence.request_id, receipt.id);
  stopped(observer);
}
TEST_F(ObserverTest, MultipleStatePublishersInvalidateAndRequireANewSample)
{
  Observer observer(configuration()); running(observer);
  ASSERT_TRUE(until([&] {return observer.snapshot()->state.position_valid;}));
  auto other = node->create_publisher<easyfly_msgs::msg::UAVState>("/uav_state", 1);
  ASSERT_TRUE(until([&] {return observer.snapshot()->state_publishers == 2;}));
  const auto conflicted = observer.snapshot();
  EXPECT_FALSE(conflicted->state.position_valid);
  EXPECT_TRUE(conflicted->trajectory.empty());
  EXPECT_EQ(conflicted->state.evidence.quality, Quality::Invalid);
  EXPECT_NE(conflicted->state.evidence.reason.find("multiple"), std::string::npos);
  other.reset();
  ASSERT_TRUE(
    until(
      [&] {
        const auto snapshot = observer.snapshot();
        return snapshot->state_publishers == 1 && snapshot->state.position_valid;
      }));
  EXPECT_EQ(goals.load(), 0); EXPECT_EQ(cancels.load(), 0);
  stopped(observer);
}
TEST_F(ObserverTest, AllReadonlyRequestsPreserveTaskSemanticsAndOtherContext)
{
  Observer observer(configuration()); running(observer);
  auto preview = request([&] {return observer.mission_details("demo.yaml");});
  ASSERT_TRUE(
    until(
      [&] {
        return observer.snapshot()->mission_details.evidence.request_id == preview.id;
      }));
  ASSERT_TRUE(observer.snapshot()->mission_details.value);
  EXPECT_EQ(observer.snapshot()->mission_details.value->mission_name, "demo.yaml");
  ASSERT_TRUE(observer.snapshot()->mission_plan);
  EXPECT_EQ(observer.snapshot()->mission_plan->steps.size(), 1U);
  EXPECT_FALSE(observer.snapshot()->mission_details.evidence.source_age_available);
  auto receipt = request([&] {return observer.list_missions();});
  ASSERT_TRUE(
    until(
      [&] {
        return observer.snapshot()->missions.at(Key::MissionList).evidence.request_id == receipt.id;
      }));
  auto list = observer.snapshot()->missions.at(Key::MissionList);
  ASSERT_TRUE(list.value); ASSERT_EQ(list.value->missions.size(), 1U);
  receipt = request([&] {return observer.validate_mission("demo.yaml");});
  ASSERT_TRUE(
    until(
      [&] {
        return observer.snapshot()->missions.at(Key::MissionValidation).evidence.request_id ==
        receipt.id;
      }));
  receipt = request([&] {return observer.query_mission(std::string(32, 'a'));});
  ASSERT_TRUE(
    until(
      [&] {
        return observer.snapshot()->missions.at(Key::MissionQuery).evidence.request_id ==
        receipt.id;
      }));
  const auto query = observer.snapshot()->missions.at(Key::MissionQuery);
  ASSERT_TRUE(query.value); EXPECT_TRUE(query.value->success);
  EXPECT_EQ(query.value->status, "failed"); EXPECT_TRUE(query.value->execution_pending);
  for (auto source : {CapabilitySource::Flight, CapabilitySource::Mission,
      CapabilitySource::Perception, CapabilitySource::Landing})
  {
    EXPECT_EQ(observer.capabilities(source).status, RequestStatus::Queued);
  }
  receipt = request([&] {return observer.read_parameter(Parameter::Profile);});
  ASSERT_TRUE(
    until(
      [&] {
        return observer.snapshot()->parameters.at(Key::Profile).evidence.request_id == receipt.id;
      }));
  for (auto param : {Parameter::UseSimTime, Parameter::OperatorNote, Parameter::ExportPretty}) {
    request([&] {return observer.read_parameter(param);});
  }
  receipt = request([&] {return observer.perception(PerceptionKind::Targets);});
  ASSERT_TRUE(
    until(
      [&] {
        return observer.snapshot()->perception.at(Key::Targets).evidence.request_id == receipt.id;
      }));
  const auto targets = observer.snapshot()->perception.at(Key::Targets);
  ASSERT_TRUE(targets.value); EXPECT_TRUE(targets.value->targets.targets.empty());
  EXPECT_EQ(targets.evidence.quality, Quality::ValidAtObservation);
  request([&] {return observer.perception(PerceptionKind::Obstacles);});
  EXPECT_EQ(observer.validate_mission("../escape.yaml").status, RequestStatus::Invalid);
  EXPECT_EQ(observer.query_mission("unknown").status, RequestStatus::Invalid);
  stopped(observer);
  Observer reconnected(configuration()); running(reconnected);
  EXPECT_NE(
    reconnected.snapshot()->observation_session_id,
    observer.snapshot()->observation_session_id);
  stopped(reconnected);
}

TEST_F(ObserverTest, DuplicateStatePausedClockAndResetDoNotRenewFreshness)
{
  Observer observer(configuration()); running(observer);
  const int64_t ns = now_ns.load(); fixed_stamp = ns; advance = false;
  ASSERT_TRUE(until([&] {return observer.snapshot()->clock.quality == Quality::ClockPaused;}, 3));
  EXPECT_FALSE(observer.snapshot()->state.position_valid);
  const auto count = observer.snapshot()->trajectory.size();
  std::this_thread::sleep_for(150ms);
  EXPECT_EQ(observer.snapshot()->trajectory.size(), count);
  now_ns = 5000000000LL;
  ASSERT_TRUE(until([&] {return observer.snapshot()->time_epoch == 1;}));
  EXPECT_TRUE(observer.snapshot()->trajectory.empty());
  EXPECT_FALSE(observer.snapshot()->state.position_valid);
  fixed_stamp = 0; advance = true;
  ASSERT_TRUE(until([&] {return observer.snapshot()->state.position_valid;}));
  stopped(observer);
}

TEST_F(ObserverTest, WrongExecutionIdentityAndUnknownIdNeverProveReleased)
{
  Observer observer(configuration()); running(observer); response_mode = 2;
  auto receipt = request([&] {return observer.query_mission(std::string(32, 'a'));});
  ASSERT_TRUE(
    until(
      [&] {
        return observer.snapshot()->missions.at(Key::MissionQuery).evidence.request_id ==
        receipt.id;
      }));
  EXPECT_EQ(observer.snapshot()->missions.at(Key::MissionQuery).evidence.quality, Quality::Invalid);
  EXPECT_FALSE(observer.snapshot()->missions.at(Key::MissionQuery).value);
  response_mode = 3;
  receipt = request([&] {return observer.query_mission(std::string(32, 'a'));});
  ASSERT_TRUE(
    until(
      [&] {
        return observer.snapshot()->missions.at(Key::MissionQuery).evidence.request_id ==
        receipt.id;
      }));
  EXPECT_EQ(observer.snapshot()->missions.at(Key::MissionQuery).evidence.quality, Quality::Unknown);
  stopped(observer);
}

TEST_F(ObserverTest, TimeoutLateResponseAndRepeatedClicksStayBounded)
{
  Observer observer(configuration()); running(observer); mission_delay_ms = 1300;
  const auto receipt = request([&] {return observer.list_missions();});
  for (int i = 0; i < 10000; ++i) {
    EXPECT_NE(observer.list_missions().status, RequestStatus::Queued);
  }
  ASSERT_TRUE(until([&] {return observer.snapshot()->counters.timed_out == 1;}));
  const auto timed_out = observer.snapshot()->missions.at(Key::MissionList);
  EXPECT_EQ(timed_out.evidence.request_id, receipt.id);
  EXPECT_EQ(timed_out.evidence.quality, Quality::Unknown); EXPECT_FALSE(timed_out.value);
  std::this_thread::sleep_for(500ms);
  EXPECT_FALSE(observer.snapshot()->missions.at(Key::MissionList).value);
  EXPECT_LE(observer.snapshot()->pending_queries, 8U); EXPECT_EQ(mission_calls, 1);
  mission_delay_ms = 0;
  const auto next = request([&] {return observer.list_missions();});
  ASSERT_TRUE(
    until(
      [&] {
        return observer.snapshot()->missions.at(Key::MissionList).evidence.request_id == next.id;
      }));
  ASSERT_TRUE(observer.snapshot()->missions.at(Key::MissionList).value);
  stopped(observer);
}

TEST_F(ObserverTest, FaultHistorySurvivesBadDetailsAndStaleCache)
{
  Observer observer(configuration()); running(observer);
  request([&] {return observer.capabilities(CapabilitySource::Flight);});
  ASSERT_TRUE(until([&] {return observer.snapshot()->command_uncertain_observed.has_value();}));
  response_mode = 1;
  auto receipt = request([&] {return observer.capabilities(CapabilitySource::Flight);});
  ASSERT_TRUE(
    until(
      [&] {
        return observer.snapshot()->capabilities.at(Key::FlightCapabilities).evidence.request_id ==
        receipt.id;
      }));
  EXPECT_TRUE(observer.snapshot()->command_uncertain_observed);
  const auto caps = observer.snapshot()->capabilities.at(Key::FlightCapabilities);
  ASSERT_TRUE(caps.value); EXPECT_FALSE(caps.value->ready[0]);
  EXPECT_FALSE(detail::command_uncertain(caps.value->details[0]));
  ASSERT_TRUE(
    until(
      [&] {
        return observer.snapshot()->capabilities.at(Key::FlightCapabilities).evidence.quality ==
        Quality::Stale;
      }));
  response_mode = 4;
  receipt = request([&] {return observer.capabilities(CapabilitySource::Flight);});
  ASSERT_TRUE(
    until(
      [&] {
        return observer.snapshot()->capabilities.at(Key::FlightCapabilities).evidence.request_id ==
        receipt.id;
      }));
  EXPECT_EQ(
    observer.snapshot()->capabilities.at(
      Key::FlightCapabilities).evidence.quality, Quality::Invalid);
  EXPECT_TRUE(observer.snapshot()->command_uncertain_observed);
  stopped(observer);
  const auto history = observer.snapshot();
  Observer reconnected(configuration(), history);
  running(reconnected);
  ASSERT_TRUE(reconnected.snapshot()->command_uncertain_observed);
  EXPECT_EQ(
    reconnected.snapshot()->command_uncertain_observed->observation_session_id,
    history->observation_session_id);
  EXPECT_FALSE(reconnected.snapshot()->capabilities.at(Key::FlightCapabilities).value);
  stopped(reconnected);
}

TEST_F(ObserverTest, ImagesAndGlobalEventsAreBoundedAndImmutable)
{
  Observer observer(configuration()); running(observer);
  sensor_msgs::msg::Image image;
  image.header.stamp = stamp(); image.header.frame_id = "camera_optical_frame";
  image.width = 2; image.height = 2; image.step = 2; image.encoding = "mono8"; image.data.resize(4);
  image_publisher->publish(image);
  // Validate and retain the same immutable snapshot. A later independent load may
  // have been invalidated by a clock epoch change before its pointer is used.
  std::shared_ptr<const UiSnapshot> before;
  ASSERT_TRUE(
    until(
      [&] {
        before = observer.snapshot(); return before->images[0].value != nullptr;
      }));
  EXPECT_FALSE(before->images[1].value);
  image.data[0] = 1; image_publisher->publish(image);
  ASSERT_TRUE(
    until(
      [&] {
        return observer.snapshot()->images[0].evidence.quality == Quality::Invalid;
      }));
  EXPECT_EQ(before->images[0].value->data[0], 0);  // Old snapshots cannot be mutated.
  image.header.stamp = stamp(); image.encoding = "16UC1"; image_publisher->publish(image);
  ASSERT_TRUE(
    until(
      [&] {
        return observer.snapshot()->images[0].evidence.quality == Quality::Unsupported;
      }));
  easyfly_msgs::msg::MissionStatus event;
  event.header.stamp = stamp(); event.current_waypoint = -1; event.phase_name = "HOVER";
  event_publisher->publish(event);
  std::shared_ptr<const UiSnapshot> event_snapshot;
  ASSERT_TRUE(
    until(
      [&] {
        event_snapshot = observer.snapshot();
        return event_snapshot->last_mission_event.value != nullptr;
      }));
  EXPECT_EQ(event_snapshot->last_mission_event.evidence.quality, Quality::ValidAtObservation);
  EXPECT_TRUE(event_snapshot->last_mission_event.value->header.frame_id.empty());
  const auto events_before = event_snapshot->events.size(); event_publisher->publish(event);
  std::this_thread::sleep_for(200ms); EXPECT_EQ(observer.snapshot()->events.size(), events_before);
  stopped(observer);
}

TEST_F(ObserverTest, MissingBindingsAndEndpointsDoNotPreventStartup)
{
  auto config = configuration(); config.use_sim_time = false;
  config.image_topics = {"/missing/front", "/missing/down"};
  // Remove platform query services, but retain every control spy/action server.
  executor->cancel(); thread.join(); platform_resources.clear();
  thread = std::thread([this] {executor->spin();});
  Observer observer(config);
  ASSERT_TRUE(until([&] {return observer.snapshot()->running;}));
  const auto receipt = request([&] {return observer.list_missions();});
  ASSERT_TRUE(
    until(
      [&] {
        return observer.snapshot()->missions.at(Key::MissionList).evidence.request_id == receipt.id;
      }));
  EXPECT_EQ(
    observer.snapshot()->missions.at(
      Key::MissionList).evidence.quality, Quality::Unavailable);
  EXPECT_FALSE(observer.snapshot()->images[0].value);
  stopped(observer);
}

TEST_F(ObserverTest, ResetDiscardsInflightQueryWithoutTreatingItAsStoppedExecution)
{
  Observer observer(configuration()); running(observer);
  mission_delay_ms = 700;
  const auto old = request([&] {return observer.list_missions();});
  ASSERT_TRUE(until([&] {return mission_calls >= 1;}));
  advance = false; now_ns = 5000000000LL;
  rosgraph_msgs::msg::Clock c; c.clock = stamp(); clock_publisher->publish(c);
  ASSERT_TRUE(until([&] {return observer.snapshot()->time_epoch == 1;}));
  ASSERT_TRUE(until([&] {return observer.snapshot()->pending_queries == 0;}));
  std::this_thread::sleep_for(800ms);
  EXPECT_FALSE(observer.snapshot()->missions.at(Key::MissionList).value);
  EXPECT_NE(observer.snapshot()->missions.at(Key::MissionList).evidence.request_id, old.id);
  EXPECT_GE(observer.snapshot()->counters.discarded_replies, 1U);
  mission_delay_ms = 0; advance = true;
  const auto next = request([&] {return observer.list_missions();});
  ASSERT_TRUE(
    until(
      [&] {
        return observer.snapshot()->missions.at(Key::MissionList).evidence.request_id == next.id;
      }));
  ASSERT_TRUE(observer.snapshot()->missions.at(Key::MissionList).value);
  stopped(observer);
}

TEST_F(ObserverTest, SameStampConflictingStateRequiresAnIndependentNewSample)
{
  Observer observer(configuration()); running(observer);
  fixed_stamp = now_ns.load();
  ASSERT_TRUE(
    until(
      [&] {
        return observer.snapshot()->state.sdk.value.header.stamp == stamp(fixed_stamp);
      }));
  {std::lock_guard<std::mutex> lock(state_mutex); position_x = 2;}
  ASSERT_TRUE(until([&] {return observer.snapshot()->state.evidence.quality == Quality::Invalid;}));
  EXPECT_FALSE(observer.snapshot()->state.position_valid);
  fixed_stamp = 0;
  ASSERT_TRUE(until([&] {return observer.snapshot()->state.position_valid;}));
  EXPECT_EQ(observer.snapshot()->state.sdk.value.position_x, 2);
  stopped(observer);
}

TEST_F(ObserverTest, NoTelemetryAndZeroClockWithAutomaticPollRemainUnavailable)
{
  publish_state = false; advance = false; now_ns = 0;
  Observer observer;
  ASSERT_TRUE(until([&] {return observer.snapshot()->running;}));
  ASSERT_TRUE(until([&] {return observer.snapshot()->clock.quality == Quality::ClockNotReady;}));
  EXPECT_FALSE(observer.snapshot()->state.sdk.received);
  EXPECT_FALSE(observer.snapshot()->state.position_valid);
  EXPECT_FALSE(observer.snapshot()->images[0].value);
  EXPECT_FALSE(observer.snapshot()->images[1].value);
  ASSERT_TRUE(
    until(
      [&] {
        return observer.snapshot()->capabilities.at(Key::FlightCapabilities).evidence.request_id >
        0;
      }));
  EXPECT_NE(
    observer.snapshot()->capabilities.at(Key::FlightCapabilities).evidence.quality,
    Quality::ValidAtObservation);
  stopped(observer);
}

TEST_F(ObserverTest, RepeatedGeometryQueryRetainsFirstSourceAgeAndBackendEnvelope)
{
  Observer observer(configuration()); running(observer);
  geometry_stamp = now_ns.load(); clock_step_ns = 2000000LL;
  auto receipt = request([&] {return observer.perception(PerceptionKind::Targets);});
  ASSERT_TRUE(
    until(
      [&] {
        return observer.snapshot()->perception.at(Key::Targets).evidence.request_id == receipt.id;
      }));
  const double first =
    observer.snapshot()->perception.at(Key::Targets).evidence.first_source_steady;
  ASSERT_EQ(
    observer.snapshot()->perception.at(
      Key::Targets).evidence.quality, Quality::ValidAtObservation);
  receipt = request([&] {return observer.perception(PerceptionKind::Targets);});
  ASSERT_TRUE(
    until(
      [&] {
        return observer.snapshot()->perception.at(Key::Targets).evidence.request_id == receipt.id;
      }));
  const auto repeated = observer.snapshot()->perception.at(Key::Targets);
  EXPECT_EQ(repeated.evidence.first_source_steady, first);
  ASSERT_TRUE(
    until(
      [&] {
        return observer.snapshot()->perception.at(Key::Targets).evidence.quality == Quality::Stale;
      }));
  EXPECT_LT(repeated.evidence.source_age, 1.0);
  EXPECT_EQ(observer.snapshot()->clock.quality, Quality::ValidAtObservation);
  response_mode = 5;
  receipt = request([&] {return observer.perception(PerceptionKind::Targets);});
  ASSERT_TRUE(
    until(
      [&] {
        return observer.snapshot()->perception.at(Key::Targets).evidence.request_id == receipt.id;
      }));
  EXPECT_EQ(observer.snapshot()->perception.at(Key::Targets).evidence.quality, Quality::Invalid);
  EXPECT_FALSE(observer.snapshot()->perception.at(Key::Targets).value);
  const auto report = observer.snapshot()->perception_reports.at(Key::Targets);
  EXPECT_TRUE(report.received); EXPECT_EQ(report.upstream_status, Perception::Response::VALID);
  EXPECT_EQ(report.source_topic, "/target_positions"); EXPECT_EQ(report.reported_age_sec, 0);
  response_mode = 0; geometry_stamp = 0;
  receipt = request([&] {return observer.perception(PerceptionKind::Targets);});
  ASSERT_TRUE(
    until(
      [&] {
        return observer.snapshot()->perception.at(Key::Targets).evidence.request_id == receipt.id;
      }));
  EXPECT_EQ(
    observer.snapshot()->perception.at(
      Key::Targets).evidence.quality, Quality::ValidAtObservation);
  stopped(observer);
}

TEST_F(ObserverTest, ClassicalGeometrySubscriptionsInvalidateWithoutReusingOldResults)
{
  auto publisher = node->create_publisher<easyfly_msgs::msg::EllipseObservationArray>(
    "/ellipse_observations", 1);
  auto config = configuration();
  config.camera_perception[0].ellipse_topic = "/ellipse_observations";
  Observer observer(config); running(observer);
  ASSERT_TRUE(until([&] {return publisher->get_subscription_count() == 1;}));
  sensor_msgs::msg::Image image; image.header.stamp = stamp(); image.header.frame_id = "camera";
  image.width = 640; image.height = 480; image.encoding = "rgb8"; image.step = 1920;
  image.data.assign(921600, 5);
  easyfly_msgs::msg::EllipseObservationArray result;
  result.header = image.header; result.image_width = 640; result.image_height = 480;
  result.valid = true;
  image_publisher->publish(image); publisher->publish(result);
  ASSERT_TRUE(
    until(
      [&] {
        return observer.snapshot()->camera_perception[0].ellipse_image.value != nullptr;
      }));
  EXPECT_TRUE(observer.snapshot()->camera_perception[0].ellipses.value->observations.empty());
  result.valid = false; result.reason = "expired"; publisher->publish(result);
  ASSERT_TRUE(until([&] {return !observer.snapshot()->camera_perception[0].ellipse_image.value;}));
  stopped(observer);
}


TEST_F(ObserverTest, LocalizationSessionAndStaleStatusRemainReadOnly)
{
  using Status = easyfly_msgs::msg::LocalizationStatus;
  auto publisher = node->create_publisher<Status>(
    "/localization/openvins/status",
    rclcpp::QoS(1).reliable().transient_local());
  BridgeConfig options; options.poll_observations = false;
  options.selected_localization_backend = "openvins";
  Observer observer(options);
  Status m; m.backend_id = "openvins"; m.session_id = "session-a";
  m.header.frame_id = "openvins_odom"; m.tracked_frame = "livox_imu_frame";
  m.state = m.ESTIMATING; m.pose_valid = true; m.covariance_available = true;
  auto publish_until = [&](Quality quality, bool old_pose = false) {
      return until(
        [&]() {
          m.header.stamp = stamp();
          m.pose_stamp = old_pose ? stamp(now_ns.load() - 2000000000LL) : m.header.stamp;
          publisher->publish(m);
          auto s = observer.snapshot();
          return s->localization.value && s->localization.value->session_id == m.session_id &&
          s->localization.value->state == m.state && s->localization.evidence.quality == quality;
        });
    };
  ASSERT_TRUE(publish_until(Quality::ValidAtObservation));
  EXPECT_EQ(observer.snapshot()->selected_localization_backend, "openvins");
  ASSERT_TRUE(publish_until(Quality::Stale, true));
  m.state = m.RESET_REQUIRED; m.pose_valid = false; ++m.generation;
  ASSERT_TRUE(publish_until(Quality::Invalid));
  m.session_id = "session-b"; m.generation = 0; m.pose_valid = true; m.state = m.ESTIMATING;
  ASSERT_TRUE(publish_until(Quality::ValidAtObservation));
  auto conflicting = node->create_publisher<Status>(
    "/localization/openvins/status",
    rclcpp::QoS(1).reliable().transient_local());
  ASSERT_TRUE(publish_until(Quality::Invalid));
  conflicting.reset();
  ASSERT_TRUE(publish_until(Quality::ValidAtObservation));
  ASSERT_TRUE(
    until(
      [&]() {
        return observer.snapshot()->localization.evidence.quality == Quality::Stale;
      }));
  observer.stop();
  EXPECT_EQ(observer.snapshot()->localization.evidence.quality, Quality::Stale);
  EXPECT_EQ(goals.load(), 0); EXPECT_EQ(commands.load(), 0); EXPECT_EQ(control_messages.load(), 0);
}
