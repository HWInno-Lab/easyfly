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

#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QTimer>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <mutex>
#include <thread>
#include <vector>

#include "easyfly_ui_bridge/observer.hpp"
#include "support.hpp"
#include <easyfly_msgs/msg/uav_command.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

using namespace std::chrono_literals;
using namespace easyfly_ui_bridge;  // NOLINT(build/namespaces)
using Mission = easyfly_msgs::srv::ManageMission;
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
    QCoreApplication::processEvents(); QTest::qWait(5);
  }
  return predicate();
}
}  // namespace

class GuiRos : public ::testing::Test
{
protected:
  std::unique_ptr<easyfly_desktop::MainWindow> window;
  void open() {window = std::make_unique<easyfly_desktop::MainWindow>(); window->show();}
  template<typename T> T * widget(const char * name) {return window->findChild<T *>(name);}
  QString label(const char * name) {return widget<QLabel>(name)->text();}
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
    desktop_test::application();
    ASSERT_STREQ(std::getenv("ROS_DOMAIN_ID"), "93");
    context = std::make_shared<rclcpp::Context>(); context->init(0, nullptr);
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
    if (window) {
      window->close(); EXPECT_TRUE(until([&] {return window->cleanup_complete();})); window.reset();
    }
    executor->cancel(); thread.join(); timer.reset(); platform_resources.clear(); resources.clear();
    executor->remove_node(node); executor.reset();
    image_publisher.reset(); event_publisher.reset(); state_publisher.reset();
    clock_publisher.reset();
    node.reset(); context->shutdown("mock complete");
    EXPECT_EQ(goals, 0); EXPECT_EQ(cancels, 0); EXPECT_EQ(control_messages, 0);
    EXPECT_EQ(commands, 0); EXPECT_EQ(mission_writes, 0); EXPECT_EQ(param_writes, 0);
  }
};
TEST_F(GuiRos, ActualButtonsAreReadonlyAndOtherContextSurvives)
{
  open();
  ASSERT_TRUE(
    until(
      [&] {
        return label("stateConnection").contains(QStringLiteral("本次新鲜"));
      }, 8));
  ASSERT_TRUE(until([&] {return label("connectionSummary").contains("4/4");}, 8));
  widget<QPushButton>("listMissions")->click();
  ASSERT_TRUE(until([&] {return widget<QListWidget>("missionList")->count() == 1;}, 5));
  widget<QListWidget>("missionList")->setCurrentRow(0); QTest::qWait(1050);
  widget<QPushButton>("validateMission")->click();
  ASSERT_TRUE(until([&] {return label("validationResult").contains("valid");}));
  QTest::qWait(1050);
  widget<QLineEdit>("executionId")->setText(QString(32, 'a'));
  widget<QPushButton>("queryMission")->click();
  ASSERT_TRUE(until([&] {return label("queryResult").contains("failed");}));
  EXPECT_TRUE(label("queryResult").contains(QStringLiteral("仍待确认排空")));
  widget<QPushButton>("readParameter")->click();
  ASSERT_TRUE(
    until(
      [&] {
        return label("backendProfile").contains(QStringLiteral("后端 profile（"));
      }));
  const auto before = heartbeats.load();
  window->close(); ASSERT_TRUE(until([&] {return window->cleanup_complete();}));
  ASSERT_TRUE(until([&] {return heartbeats > before + 3;}));
  EXPECT_TRUE(context->is_valid());
}
TEST_F(GuiRos, DelayedServiceDoesNotBlockCloseOrRestart)
{
  open(); ASSERT_TRUE(until([&] {return label("connectionSummary").contains("4/4");}, 8));
  mission_delay_ms = 1300;
  int qt_ticks = 0; QTimer qt_timer;
  QObject::connect(&qt_timer, &QTimer::timeout, [&] {++qt_ticks;}); qt_timer.start(10);
  widget<QPushButton>("listMissions")->click();
  ASSERT_TRUE(until([&] {return mission_calls > 0;}));
  QTest::qWait(200); EXPECT_GT(qt_ticks, 10);
  widget<QPushButton>("observerToggle")->click();
  ASSERT_TRUE(until([&] {return window->cleanup_complete();}));
  mission_delay_ms = 0; widget<QPushButton>("observerToggle")->click();
  ASSERT_TRUE(until([&] {return !window->cleanup_complete();}));
  ASSERT_TRUE(until([&] {return label("connectionSummary").contains("4/4");}, 8));
  EXPECT_EQ(widget<QListWidget>("missionList")->count(), 0);
  ASSERT_TRUE(until([&] {return heartbeats > 10;})); EXPECT_TRUE(context->is_valid());
}
TEST_F(GuiRos, PausedClockInvalidatesActualGeometryAndState)
{
  open();
  ASSERT_TRUE(
    until(
      [&] {
        return label("stateConnection").contains(QStringLiteral("本次新鲜"));
      }, 8));
  advance = false;
  ASSERT_TRUE(until([&] {return label("stateValidity").contains(QStringLiteral("暂停"));}, 5));
  EXPECT_TRUE(label("statePosition").contains(QStringLiteral("不可用")));
  EXPECT_TRUE(label("targetSummary").contains(QStringLiteral("不是有效空")));
}
