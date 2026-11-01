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
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include "easyfly_ui_bridge/observer.hpp"
using namespace std::chrono_literals;
namespace b = easyfly_ui_bridge;
namespace
{
template<typename Predicate>
bool until(Predicate predicate, double seconds = 4.)
{
  const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
  while (std::chrono::steady_clock::now() < end) {
    if (predicate()) {return true;} std::this_thread::sleep_for(10ms);
  }
  return predicate();
}
class CameraObserver : public ::testing::Test
{
protected:
  std::shared_ptr<rclcpp::Context> context;
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock;
  rclcpp::TimerBase::SharedPtr timer;
  std::thread worker;
  std::atomic<int64_t> now{10000000000LL};
  void SetUp() override
  {
    ASSERT_STREQ(std::getenv("ROS_DOMAIN_ID"), "94");
    context = std::make_shared<rclcpp::Context>(); context->init(0, nullptr);
    rclcpp::NodeOptions options; options.context(context).use_global_arguments(false);
    node = std::make_shared<rclcpp::Node>("camera_transport_mock", options);
    clock = node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 4);
    timer = node->create_wall_timer(
      20ms, [this] {
        const auto ns = now.fetch_add(20000000LL); rosgraph_msgs::msg::Clock m;
        m.clock.sec = ns / 1000000000LL; m.clock.nanosec = ns % 1000000000LL; clock->publish(m);
      });
    rclcpp::ExecutorOptions eo; eo.context = context;
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(eo);
    executor->add_node(node); worker = std::thread([this] {executor->spin();});
  }
  void TearDown() override
  {
    if (context && context->is_valid()) {context->shutdown("camera mock complete");}
    if (worker.joinable()) {worker.join();}
    timer.reset(); clock.reset(); executor.reset(); node.reset(); context.reset();
  }
  b::BridgeConfig config()
  {
    b::BridgeConfig c; c.poll_observations = false;
    c.image_topics = {"/adaptive_image", ""}; c.camera_perception = {}; return c;
  }
  sensor_msgs::msg::Image image()
  {
    const auto ns = now.load(); sensor_msgs::msg::Image m;
    m.header.frame_id = "camera"; m.header.stamp.sec = ns / 1000000000LL;
    m.header.stamp.nanosec = ns % 1000000000LL;
    m.width = m.height = 4; m.encoding = "rgb8"; m.step = 12; m.data.assign(48, 10); return m;
  }
  bool policy(rclcpp::ReliabilityPolicy reliability)
  {
    for (const auto & endpoint : node->get_subscriptions_info_by_topic("/adaptive_image")) {
      if (endpoint.node_name().find("easyfly_ui_observer_") == 0) {
        return endpoint.qos_profile().reliability() == reliability;
      }
    }
    return false;
  }
};
TEST_F(CameraObserver, DiscoversLateSourceMatchesReliabilityAndRecoversAfterSourceChanges)
{
  b::Observer observer(config());
  ASSERT_TRUE(until([&] {return observer.snapshot()->running;}));
  EXPECT_EQ(node->count_subscribers("/adaptive_image"), 0U);
  auto reliable = node->create_publisher<sensor_msgs::msg::Image>(
    "/adaptive_image", rclcpp::QoS(4).reliable());
  ASSERT_TRUE(
    until(
      [&] {
        return reliable->get_subscription_count() == 1 &&
        policy(rclcpp::ReliabilityPolicy::Reliable);
      }));
  ASSERT_TRUE(
    until(
      [&] {
        reliable->publish(image());
        return observer.snapshot()->images[0].evidence.quality == b::Quality::ValidAtObservation;
      }));
  const auto immutable = observer.snapshot(); ASSERT_TRUE(immutable->images[0].value);
  reliable.reset(); ASSERT_TRUE(until([&] {return !observer.snapshot()->images[0].value;}));
  auto best_effort = node->create_publisher<sensor_msgs::msg::Image>(
    "/adaptive_image", rclcpp::QoS(4).best_effort());
  ASSERT_TRUE(
    until(
      [&] {
        return best_effort->get_subscription_count() == 1 &&
        policy(rclcpp::ReliabilityPolicy::BestEffort);
      }));
  ASSERT_TRUE(
    until(
      [&] {
        best_effort->publish(image());
        return observer.snapshot()->images[0].evidence.quality == b::Quality::ValidAtObservation;
      }));
  // An ambiguous image origin clears telemetry rather than stitching two sources together.
  auto other = node->create_publisher<sensor_msgs::msg::Image>(
    "/adaptive_image", rclcpp::QoS(4).best_effort());
  ASSERT_TRUE(
    until(
      [&] {
        return !observer.snapshot()->images[0].value &&
        best_effort->get_subscription_count() == 0;
      }));
  other.reset();
  ASSERT_TRUE(until([&] {return best_effort->get_subscription_count() == 1;}));
  ASSERT_TRUE(
    until(
      [&] {
        best_effort->publish(image());
        return observer.snapshot()->images[0].evidence.quality == b::Quality::ValidAtObservation;
      }));
  EXPECT_TRUE(immutable->images[0].value);
  observer.stop(); ASSERT_TRUE(until([&] {return observer.snapshot()->stopped;}));
  EXPECT_FALSE(observer.snapshot()->images[0].value); EXPECT_TRUE(context->is_valid());
}
}  // namespace
