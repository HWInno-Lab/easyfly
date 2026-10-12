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

#include <limits>

#include "easyfly_ui_bridge/model.hpp"

using easyfly_ui_bridge::ClockWatch;
using easyfly_ui_bridge::ClockState;
using easyfly_ui_bridge::Key;
using easyfly_ui_bridge::Quality;
namespace detail = easyfly_ui_bridge::detail;

TEST(BridgeModel, ClockMissingZeroFrozenAndBackwards)
{
  ClockWatch clock(true);
  EXPECT_EQ(clock.state(10).quality, Quality::ClockNotReady);
  EXPECT_FALSE(clock.observe(0, 10));
  EXPECT_EQ(clock.state(10).quality, Quality::ClockNotReady);
  EXPECT_FALSE(clock.observe(10000000000LL, 11));
  EXPECT_EQ(clock.state(11).quality, Quality::ValidAtObservation);
  EXPECT_FALSE(clock.observe(10000000000LL, 12));
  EXPECT_EQ(clock.state(12.1).quality, Quality::ClockPaused);
  EXPECT_TRUE(clock.observe(5000000000LL, 13));
  EXPECT_EQ(clock.state(13).epoch, 1U);
  EXPECT_EQ(clock.state(13).quality, Quality::ClockReset);
  EXPECT_FALSE(clock.observe(5100000000LL, 13.1));
  EXPECT_EQ(clock.state(13.1).quality, Quality::ValidAtObservation);
  ClockWatch wall(false);
  EXPECT_EQ(wall.state(13, 10000000000LL).now_ns, 10000000000LL);
}

TEST(BridgeModel, SourceAndReceiptAgesAreIndependent)
{
  std_msgs::msg::Header header;
  header.frame_id = "odom"; header.stamp.sec = 10;
  const ClockState clock{Quality::ValidAtObservation, 10000000000LL, 0};
  EXPECT_EQ(detail::source_quality(header, clock, 0, 1), Quality::ValidAtObservation);
  EXPECT_EQ(detail::source_quality(header, clock, 1.1, 1), Quality::Stale);
  header.stamp.sec = 8;
  EXPECT_EQ(detail::source_quality(header, clock, 0, 1), Quality::Stale);
  header.stamp.sec = 11;
  EXPECT_EQ(detail::source_quality(header, clock, 0, 1), Quality::Invalid);
  header.stamp.sec = 10; header.frame_id.clear();
  EXPECT_EQ(detail::source_quality(header, clock, 0, 1), Quality::Invalid);
  EXPECT_EQ(detail::source_quality(header, clock, 0, 1, false), Quality::ValidAtObservation);
}

TEST(BridgeModel, FieldsNeverUseDefaultZeroAsValid)
{
  easyfly_core::StateSnapshot sdk;
  sdk.received = sdk.fresh = sdk.source_time_valid = true;
  sdk.value.header.frame_id = "odom"; sdk.value.header.stamp.sec = 10;
  const ClockState clock{Quality::ValidAtObservation, 10000000000LL, 0};
  auto v = detail::state_view(sdk, clock, 10, 10, false);
  EXPECT_FALSE(v.position_valid); EXPECT_FALSE(v.speed_mps); EXPECT_FALSE(v.battery_percent);
  sdk.value.pose_valid = sdk.value.velocity_valid = sdk.value.battery_valid = true;
  sdk.value.velocity_x = 3; sdk.value.velocity_y = 4; sdk.value.battery_remaining = .5;
  v = detail::state_view(sdk, clock, 10, 10, false);
  ASSERT_TRUE(v.speed_mps); EXPECT_DOUBLE_EQ(*v.speed_mps, 5);
  ASSERT_TRUE(v.battery_percent); EXPECT_DOUBLE_EQ(*v.battery_percent, 50);
  sdk.value.position_x = std::numeric_limits<double>::quiet_NaN();
  sdk.value.velocity_z = std::numeric_limits<double>::infinity();
  sdk.value.battery_remaining = 1.1;
  v = detail::state_view(sdk, clock, 10, 10, false);
  EXPECT_FALSE(v.position_valid); EXPECT_FALSE(v.velocity_valid); EXPECT_FALSE(v.battery_valid);
  EXPECT_TRUE(v.sdk.value.pose_valid);  // Original evidence is not rewritten.
  EXPECT_EQ(detail::state_view(sdk, clock, 10, 10, true).evidence.quality, Quality::Invalid);
}

TEST(BridgeModel, CapabilityArraysBudgetsAndTypedJson)
{
  easyfly_msgs::srv::GetCapabilities::Response v;
  v.names = {"takeoff"}; v.implemented = {true}; v.ready = {false}; v.details = {"{}"};
  EXPECT_EQ(detail::validate_capabilities(v), Quality::ValidAtObservation);
  v.ready.clear(); EXPECT_EQ(detail::validate_capabilities(v), Quality::Invalid);
  v.ready = {false, false}; v.names.push_back("takeoff");
  v.implemented.push_back(true); v.details.push_back("{}");
  EXPECT_EQ(detail::validate_capabilities(v), Quality::Invalid);
  v.names[1] = "land"; v.details[0] = std::string(4097, 'x');
  EXPECT_EQ(detail::validate_capabilities(v), Quality::BudgetExceeded);
  ASSERT_TRUE(detail::command_uncertain("{\"command_uncertain\":true}"));
  EXPECT_TRUE(*detail::command_uncertain("{\"command_uncertain\":true}"));
  EXPECT_FALSE(detail::command_uncertain("{\"command_uncertain\":\"true\"}"));
  EXPECT_FALSE(detail::command_uncertain("{}"));
  for (const auto & text : {"{broken", "{\"a\":1,\"a\":2}", "NaN", "1e999", "{} trailing"}) {
    EXPECT_FALSE(detail::valid_json(text)) << text;
  }
  EXPECT_TRUE(detail::valid_json("\"navigation_dev\""));
  EXPECT_TRUE(detail::valid_json("false"));
  EXPECT_FALSE(detail::parameter_json("\"true\"", Key::UseSimTime));
  EXPECT_TRUE(detail::parameter_json("true", Key::UseSimTime));
}

TEST(BridgeModel, TaskQuerySuccessDoesNotMeanTaskSuccessOrDrained)
{
  easyfly_msgs::srv::ManageMission::Response v;
  v.success = true; v.status = "failed"; v.execution_pending = true;
  v.execution_id = std::string(32, 'a');
  EXPECT_EQ(
    detail::validate_mission(
      v, Key::MissionQuery,
      v.execution_id), Quality::ValidAtObservation);
  EXPECT_EQ(detail::validate_mission(v, Key::MissionQuery, std::string(32, 'b')), Quality::Invalid);
  v.success = false; v.status = "not_found";
  EXPECT_EQ(detail::validate_mission(v, Key::MissionQuery, v.execution_id), Quality::Unknown);
  EXPECT_FALSE(detail::basename("../test.yaml")); EXPECT_FALSE(detail::basename("a\\b"));
  EXPECT_FALSE(detail::execution_id("arbitrary")); EXPECT_TRUE(detail::basename("任务.yaml"));
  EXPECT_FALSE(detail::execution_id(std::string(32, 'A')));
}

TEST(BridgeModel, MissionEventHasNoGeometryFrameOrIdentityRequirement)
{
  easyfly_msgs::msg::MissionStatus event;
  event.header.stamp.sec = 10; event.current_waypoint = -1;
  EXPECT_EQ(detail::validate_event(event), Quality::ValidAtObservation);
  event.progress = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(detail::validate_event(event), Quality::Invalid);
}

TEST(BridgeModel, ImageLayoutEncodingAndBudgets)
{
  sensor_msgs::msg::Image image;
  image.header.stamp.sec = 10; image.header.frame_id = "camera_optical_frame";
  image.encoding = "rgb8"; image.width = 2; image.height = 2; image.step = 6;
  image.data.resize(12);
  EXPECT_EQ(detail::validate_image(image), Quality::ValidAtObservation);
  image.step = 7; EXPECT_EQ(detail::validate_image(image), Quality::Invalid);
  image.step = 6; image.data.pop_back();
  EXPECT_EQ(detail::validate_image(image), Quality::Invalid);
  image.encoding = "16UC1"; EXPECT_EQ(detail::validate_image(image), Quality::Unsupported);
  image.width = 1921; EXPECT_EQ(detail::validate_image(image), Quality::BudgetExceeded);
}

TEST(BridgeModel, PerceptionValidEmptyIsNotInvalidOrUnsupported)
{
  easyfly_msgs::srv::GetPerception::Response v;
  v.status = v.VALID; v.targets.header.frame_id = "odom"; v.targets.header.stamp.sec = 10;
  v.observed_at = v.targets.header.stamp;
  const ClockState clock{Quality::ValidAtObservation, 10000000000LL, 0};
  EXPECT_EQ(detail::validate_perception(v, Key::Targets, clock), Quality::ValidAtObservation);
  easyfly_msgs::msg::Target3D target;
  target.header = v.targets.header; target.confidence = .9; target.position_valid = true;
  v.targets.targets.push_back(target);  // distance=0 is allowed by the backend contract.
  EXPECT_EQ(detail::validate_perception(v, Key::Targets, clock), Quality::ValidAtObservation);
  v.targets.targets[0].position_valid = false;
  EXPECT_EQ(detail::validate_perception(v, Key::Targets, clock), Quality::Invalid);
  v.targets.targets.resize(2001);
  EXPECT_EQ(detail::validate_perception(v, Key::Targets, clock), Quality::BudgetExceeded);
  v.targets.targets.clear(); v.status = v.UNSUPPORTED;
  EXPECT_EQ(detail::validate_perception(v, Key::Targets, clock), Quality::Unsupported);
}

TEST(BridgeModel, EventsAndTrajectoryHitRealLimitsWithoutSleeping)
{
  std::deque<easyfly_ui_bridge::Event> events;
  std::deque<easyfly_ui_bridge::TrajectoryPoint> points;
  for (int i = 0; i < 10000; ++i) {
    detail::append_event(events, {static_cast<double>(i), 0, std::string(3000, 'x')});
    detail::append_trajectory(points, {i, i * .001, 0, 0, 0});
  }
  ASSERT_EQ(events.size(), 1000U); EXPECT_EQ(events.front().steady, 9000);
  EXPECT_EQ(events.back().text.size(), 2048U);
  ASSERT_EQ(points.size(), 4096U); EXPECT_EQ(points.front().source_ns, 5904);
  detail::prune_trajectory(points, 311);
  EXPECT_TRUE(points.empty());
}
