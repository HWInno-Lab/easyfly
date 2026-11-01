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

#include <gtest/gtest.h>
#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <easyfly_msgs/msg/ellipse_observation_array.hpp>
#include "easyfly_image_detection_common/node.hpp"
namespace
{
using Result = easyfly_msgs::msg::EllipseObservationArray;
using Image = sensor_msgs::msg::Image;
class Detector final : public easyfly::image_detection::Node<Result>
{
public:
  explicit Detector(const rclcpp::NodeOptions & options)
  : Node("qos_detector", "ellipse", true, options) {}
  int detections{0};

private:
  void detect(const cv::Mat &, Result &) override {++detections;}
};
class InputReliability : public ::testing::Test
{
protected:
  std::shared_ptr<rclcpp::Context> context;
  std::shared_ptr<rclcpp::Node> probe;
  std::shared_ptr<Detector> detector;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  rclcpp::Publisher<Image>::SharedPtr publisher;
  rclcpp::Subscription<Result>::SharedPtr results;
  std::vector<Result> observed;

  void SetUp() override
  {
    context = std::make_shared<rclcpp::Context>(); context->init(0, nullptr);
    rclcpp::ExecutorOptions options; options.context = context;
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(options);
    probe = std::make_shared<rclcpp::Node>("qos_probe", node_options());
    executor->add_node(probe);
    results = probe->create_subscription<Result>(
      "/qos_test/result", rclcpp::QoS(100),
      [this](Result::ConstSharedPtr message) {
        observed.push_back(*message);
        if (observed.size() > 100) {observed.erase(observed.begin());}
      });
  }
  void TearDown() override
  {
    executor.reset(); detector.reset(); results.reset(); publisher.reset(); probe.reset();
    context->shutdown("test complete"); context.reset();
  }
  rclcpp::NodeOptions node_options() const
  {
    rclcpp::NodeOptions options; options.context(context).use_global_arguments(false);
    return options;
  }
  void start(const std::string & reliability = "auto")
  {
    auto options = node_options();
    options.parameter_overrides(
      {rclcpp::Parameter("input_topic", "/qos_test/image"),
        rclcpp::Parameter("output_topic", "/qos_test/result"),
        rclcpp::Parameter("optical_frame", "camera"),
        rclcpp::Parameter("input_reliability", reliability)});
    detector = std::make_shared<Detector>(options); executor->add_node(detector);
  }
  void offer(bool reliable)
  {
    auto qos = rclcpp::QoS(1);
    if (reliable) {qos.reliable();} else {qos.best_effort();}
    publisher = probe->create_publisher<Image>("/qos_test/image", qos);
  }
  template<typename Predicate> bool until(Predicate predicate, int timeout = 3500)
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout);
    while (std::chrono::steady_clock::now() < end) {
      executor->spin_some(); if (predicate()) {return true;}
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    executor->spin_some(); return predicate();
  }
  bool requested(rclcpp::ReliabilityPolicy reliability)
  {
    const auto endpoints = probe->get_subscriptions_info_by_topic("/qos_test/image");
    return std::any_of(
      endpoints.begin(), endpoints.end(), [&](const auto & endpoint) {
        return endpoint.node_name() == "qos_detector" &&
        endpoint.qos_profile().reliability() == reliability;
      });
  }
  Image image()
  {
    Image frame; frame.header.stamp = probe->now(); frame.header.frame_id = "camera";
    frame.width = frame.height = 8; frame.step = 8; frame.encoding = "mono8";
    frame.data.assign(64, 100); return frame;
  }
  bool processed(const Image & frame)
  {
    return std::any_of(
      observed.begin(), observed.end(), [&](const auto & result) {
        return result.valid && result.header == frame.header;
      });
  }
  bool deliver()
  {
    Image frame;
    auto last = std::chrono::steady_clock::time_point{};
    return until(
      [&] {
        if (processed(frame)) {return true;}
        const auto now = std::chrono::steady_clock::now();
        if (now - last > std::chrono::milliseconds(50)) {
          frame = image(); publisher->publish(frame); last = now;
        }
        return false;
      });
  }
};
TEST_F(InputReliability, AutoUsesReliableTransportForReliableSourceAndExpiresWhenFramesStop)
{
  offer(true); start();
  ASSERT_TRUE(until([&] {return requested(rclcpp::ReliabilityPolicy::Reliable);}));
  EXPECT_TRUE(detector->describe_parameter("input_reliability").read_only);
  EXPECT_FALSE(
    detector->set_parameter(
      rclcpp::Parameter("input_reliability", "best_effort")).successful);
  EXPECT_EQ(detector->get_parameter("input_reliability").as_string(), "auto");
  ASSERT_TRUE(deliver()); ASSERT_FALSE(observed.empty());
  const auto header = observed.back().header;
  ASSERT_TRUE(
    until(
      [&] {
        return !observed.empty() && !observed.back().valid &&
        observed.back().reason == "expired_or_clock_invalid";
      }, 1500));
  EXPECT_EQ(observed.back().header, header);
}
TEST_F(InputReliability, AutoUsesBestEffortForBestEffortSource)
{
  offer(false); start();
  ASSERT_TRUE(until([&] {return requested(rclcpp::ReliabilityPolicy::BestEffort);}));
  ASSERT_TRUE(deliver()); EXPECT_GT(detector->detections, 0);
}
TEST_F(InputReliability, LatePublisherAndSourceSwitchRetireOldResultBeforeNewFrames)
{
  start(); ASSERT_TRUE(until([&] {return requested(rclcpp::ReliabilityPolicy::BestEffort);}));
  offer(true);
  ASSERT_TRUE(until([&] {return requested(rclcpp::ReliabilityPolicy::Reliable);}));
  ASSERT_TRUE(deliver()); observed.clear();
  publisher.reset(); offer(false);
  ASSERT_TRUE(until([&] {return requested(rclcpp::ReliabilityPolicy::BestEffort);}));
  ASSERT_TRUE(
    until(
      [&] {
        return std::any_of(
          observed.begin(), observed.end(), [](const auto & result) {
            return !result.valid && result.reason == "input_source_changed";
          });
      }));
  ASSERT_TRUE(deliver()); observed.clear(); publisher.reset();
  ASSERT_TRUE(
    until(
      [&] {
        return std::any_of(
          observed.begin(), observed.end(), [](const auto & result) {
            return !result.valid && result.reason == "input_source_changed";
          });
      }));
}
TEST_F(InputReliability, SameSourceDuplicatesCannotRenewAcrossDiscoveryRebuild)
{
  start(); offer(true);
  ASSERT_TRUE(
    until(
      [&] {
        return publisher->get_subscription_count() > 0 && results->get_publisher_count() > 0;
      }));
  const auto frame = image(); publisher->publish(frame);
  ASSERT_TRUE(until([&] {return processed(frame);}, 500));
  const int calls = detector->detections;
  // Continue the same source frame through the next graph poll/QoS rebuild.
  const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(1300);
  while (std::chrono::steady_clock::now() < end) {
    publisher->publish(frame); executor->spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
  }
  ASSERT_TRUE(until([&] {return requested(rclcpp::ReliabilityPolicy::Reliable);}));
  EXPECT_EQ(detector->detections, calls); ASSERT_FALSE(observed.empty());
  EXPECT_FALSE(observed.back().valid);
}
TEST_F(InputReliability, ExplicitReliabilityIsStableAndUnknownParameterIsRejected)
{
  offer(true); start("best_effort");
  ASSERT_TRUE(until([&] {return requested(rclcpp::ReliabilityPolicy::BestEffort);}));
  ASSERT_TRUE(deliver());
  auto options = node_options();
  options.parameter_overrides({rclcpp::Parameter("input_reliability", "invalid")});
  EXPECT_THROW(std::make_shared<Detector>(options), std::invalid_argument);
}
TEST_F(InputReliability, ExplicitReliableCanBeSelectedBeforePublisherAppears)
{
  start("reliable");
  ASSERT_TRUE(until([&] {return requested(rclcpp::ReliabilityPolicy::Reliable);}));
  offer(true); ASSERT_TRUE(deliver());
}
}  // namespace
