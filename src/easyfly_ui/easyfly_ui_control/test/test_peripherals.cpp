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
#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include "easyfly_ui_control/peripherals.hpp"
namespace c = easyfly_ui_control;
namespace
{
template<typename F> bool until(F f, double seconds = 5)
{
  const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
  do {
    if (f()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }while (std::chrono::steady_clock::now() < end); return f();
}
using Single = easyfly_msgs::srv::ManageSingleTrack;
using Angle = easyfly_msgs::srv::SetGimbalAngle;
using Tracking = easyfly_msgs::srv::SetGimbalTracking;
}  // namespace
class Peripherals : public ::testing::Test
{
protected:
  std::shared_ptr<rclcpp::Context> context;
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::thread thread;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock;
  rclcpp::Publisher<easyfly_msgs::msg::SingleTrackState>::SharedPtr single_state;
  rclcpp::Publisher<easyfly_msgs::msg::GimbalState>::SharedPtr gimbal_state;
  rclcpp::Service<Single>::SharedPtr single;
  rclcpp::Service<Angle>::SharedPtr angle;
  rclcpp::Service<Tracking>::SharedPtr tracking;
  rclcpp::TimerBase::SharedPtr timer;
  std::unique_ptr<c::PeripheralControl> control;
  std::mutex mutex;
  std::shared_ptr<rmw_request_id_t> held;
  Single::Request received;
  std::atomic<int> singles{0}, angles{0}, trackings{0};
  std::atomic<bool> hold{false}, reject{false}, publish{true}, frozen{false}, connected{true};
  std::atomic<int64_t> ns{10000000000LL};
  std::atomic<float> actual_pitch{0};
  void SetUp() override
  {
    context = std::make_shared<rclcpp::Context>(); context->init(0, nullptr);
    rclcpp::NodeOptions options; options.context(context).use_global_arguments(false);
    node = std::make_shared<rclcpp::Node>("peripheral_services_fixture", options);
    clock = node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
    single_state = node->create_publisher<easyfly_msgs::msg::SingleTrackState>(
      "/single_track/state", 1);
    gimbal_state = node->create_publisher<easyfly_msgs::msg::GimbalState>("/gimbal/state", 1);
    single = node->create_service<Single>(
      "/single_track/manage",
      [this](std::shared_ptr<rclcpp::Service<Single>>, std::shared_ptr<rmw_request_id_t> header,
      Single::Request::SharedPtr req) {
        ++singles; std::lock_guard<std::mutex> lock(
          mutex); received = *req;
        if (hold) {held = header; return;}
        Single::Response r;
        r.accepted = !reject;
        r.message = "admission only";
        single->send_response(
          *header,
          r);
      });
    angle = node->create_service<Angle>(
      "/gimbal/set_angle",
      [this](Angle::Request::SharedPtr, Angle::Response::SharedPtr r) {
        ++angles; r->success = !reject; r->message = "accepted_not_arrived";
      });
    tracking = node->create_service<Tracking>(
      "/gimbal/set_tracking",
      [this](Tracking::Request::SharedPtr, Tracking::Response::SharedPtr r) {
        ++trackings; r->success = !reject;
      });
    timer = node->create_wall_timer(
      std::chrono::milliseconds(20), [this] {
        if (!frozen) {ns += 20000000LL;} rosgraph_msgs::msg::Clock time;
        time.clock.sec = ns / 1000000000LL; time.clock.nanosec = ns % 1000000000LL;
        clock->publish(time);
        if (!publish) {return;}
        easyfly_msgs::msg::SingleTrackState s; s.header.stamp = time.clock;
        s.header.frame_id = "camera_color_optical_frame"; s.track_id = -1; s.phase = "idle";
        single_state->publish(
          s);
        easyfly_msgs::msg::GimbalState g;
        g.header.stamp = time.clock;
        g.header.frame_id = "base_link";
        g.is_connected = connected; g.gimbal_model = "SIMULATION_ONLY_rate_limited";
        g.tracking_target_id = -1; g.pitch_angle = actual_pitch; gimbal_state->publish(g);
      });
    rclcpp::ExecutorOptions eo; eo.context = context;
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(eo);
    executor->add_node(
      node);
    thread = std::thread([this] {executor->spin();});
    control = std::make_unique<c::PeripheralControl>(c::PeripheralConfig{true, true, .15});
    ASSERT_TRUE(
      until(
        [&] {
          const auto s = control->snapshot();
          return s->single_ready && s->angle_ready && s->tracking_ready && s->gimbal_fresh;
        }));
    control->inhibit(false);
  }
  void TearDown() override
  {
    if (control) {
      control->stop(); EXPECT_TRUE(
        until(
          [&] {
            return control->snapshot()->stopped;
          })); control.reset();
    }
    if (executor) {executor->cancel();}
    if (thread.joinable()) {thread.join();}
    executor.reset(); timer.reset(); single.reset(); angle.reset(); tracking.reset();
    single_state.reset(); gimbal_state.reset(); clock.reset(); node.reset(); context->shutdown(
      "test complete");
  }
};
TEST_F(Peripherals, ExactRoiTimeoutNoRetryAndLateAdmissionDoesNotInventTracking)
{
  hold = true;
  Single::Request r; r.operation = "initialize";
  r.image_header.frame_id = "camera_color_optical_frame";
  r.image_header.stamp.sec = 10; r.roi_x = .2; r.roi_y = .3; r.roi_width = .2; r.roi_height = .4;
  ASSERT_TRUE(control->single_confirmed(r));
  ASSERT_TRUE(
    until(
      [&] {
        return control->snapshot()->uncertain;
      })) << control->snapshot()->message << " requests=" << singles;
  EXPECT_EQ(singles, 1); EXPECT_FALSE(control->single_confirmed(r));
  {std::lock_guard<std::mutex> lock(mutex); ASSERT_TRUE(held); EXPECT_EQ(received, r);
    Single::Response response; response.accepted = true; response.message = "queued only";
    single->send_response(*held, response); held.reset();}
  ASSERT_TRUE(
    until(
      [&] {
        return control->snapshot()->reply_received;
      })) << control->snapshot()->message;
  auto state = control->snapshot(); EXPECT_TRUE(state->accepted); EXPECT_FALSE(state->pending);
  EXPECT_FALSE(state->uncertain); ASSERT_TRUE(state->single); EXPECT_FALSE(state->single->valid);
  EXPECT_TRUE(state->message.find("late") != std::string::npos); EXPECT_EQ(singles, 1);
  hold = false; reject = true; r.operation = "reset"; ASSERT_TRUE(control->single_confirmed(r));
  ASSERT_TRUE(
    until(
      [&] {
        return control->snapshot()->reply_received;
      })) << control->snapshot()->message;
  EXPECT_FALSE(control->snapshot()->accepted); EXPECT_EQ(singles, 2);
}
TEST_F(Peripherals, GimbalAdmissionActualAnglesDisconnectFreshnessAndNoFlightPublishers)
{
  Angle::Request r; r.pitch_angle = 30;
  ASSERT_TRUE(control->angle_confirmed(r)); ASSERT_TRUE(
    until(
      [&] {
        return control->snapshot()->reply_received;
      })) << control->snapshot()->message;
  ASSERT_TRUE(control->snapshot()->accepted); EXPECT_EQ(angles, 1);
  EXPECT_EQ(control->snapshot()->gimbal->pitch_angle, 0);
  actual_pitch = 30;
  ASSERT_TRUE(until([&] {return control->snapshot()->gimbal->pitch_angle == 30;}));
  connected = false;
  ASSERT_TRUE(until([&] {return !control->snapshot()->gimbal->is_connected;}));
  frozen = true; ASSERT_TRUE(until([&] {return !control->snapshot()->gimbal_fresh;}));
  EXPECT_FALSE(control->snapshot()->single_fresh);
  frozen = false; ASSERT_TRUE(until([&] {return control->snapshot()->gimbal_fresh;}));
  publish = false; ASSERT_TRUE(until([&] {return !control->snapshot()->gimbal_fresh;}));
  for (const auto * topic :
    {"/uav_command", "/mavros/setpoint_raw/local", "/target_positions", "/tracks"})
  {
    EXPECT_EQ(node->count_publishers(topic), 0U);
  }
  r.pitch_angle = 91; EXPECT_FALSE(control->angle_confirmed(r));
  Tracking::Request t; t.enabled = true; t.target_id = -1;
  EXPECT_FALSE(control->tracking_confirmed(t));
  control->inhibit(true); r.pitch_angle = 0; EXPECT_FALSE(control->angle_confirmed(r));
}
TEST_F(Peripherals, AmbiguousServiceAndReadonlyConfigurationNeverDispatch)
{
  rclcpp::NodeOptions options; options.context(context).use_global_arguments(false);
  auto other = std::make_shared<rclcpp::Node>("other_single_service", options);
  auto duplicate = other->create_service<Single>(
    "/single_track/manage",
    [](Single::Request::SharedPtr, Single::Response::SharedPtr) {});
  ASSERT_TRUE(until([&] {return node->get_node_names().size() >= 3;}));
  Single::Request r; r.operation = "reset"; ASSERT_TRUE(control->single_confirmed(r));
  ASSERT_TRUE(until([&] {return !control->snapshot()->pending;}));
  EXPECT_EQ(singles, 0); EXPECT_FALSE(control->snapshot()->reply_received);
  c::PeripheralControl readonly({true, false, .2}); readonly.inhibit(false);
  ASSERT_TRUE(until([&] {return readonly.snapshot()->running;}));
  EXPECT_FALSE(readonly.single_confirmed(r)); readonly.stop();
  EXPECT_TRUE(until([&] {return readonly.snapshot()->stopped;}));
}
