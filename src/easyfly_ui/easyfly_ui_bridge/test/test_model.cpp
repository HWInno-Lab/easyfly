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
#include <utility>
#include <vector>

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

TEST(BridgeModel, NavigationLocalizationRequiresAlignedFreshIdentifiedTracking)
{
  using Status = easyfly_msgs::msg::LocalizationStatus;
  const ClockState clock{Quality::ValidAtObservation, 10000000000LL, 0};
  for (const auto & backend : {"openvins", "fast_livo2", "orbslam3"}) {
    Status m; m.backend_id = std::string(backend) + "_navigation";
    m.session_id = "session"; m.header.frame_id = "odom"; m.tracked_frame = "base_link";
    m.header.stamp.sec = m.pose_stamp.sec = 10;
    m.state = Status::ESTIMATING; m.pose_valid = m.tracking_quality_available = true;
    const auto valid = [&]() {
        return detail::validate_navigation_localization(m, backend, clock, 0);
      };
    EXPECT_EQ(valid(), Quality::ValidAtObservation);
    m.tracking_quality_available = false; EXPECT_EQ(valid(), Quality::Invalid);
    m.tracking_quality_available = true;
    m.pose_stamp.sec = 9; EXPECT_EQ(valid(), Quality::Stale); m.pose_stamp.sec = 10;
    m.header.frame_id = "map"; EXPECT_EQ(valid(), Quality::Invalid); m.header.frame_id = "odom";
    m.session_id.clear(); EXPECT_EQ(valid(), Quality::Invalid); m.session_id = "session";
    m.state = Status::RESET_REQUIRED; m.pose_valid = false; EXPECT_EQ(valid(), Quality::Invalid);
    m.state = Status::ESTIMATING; m.pose_valid = true;
    EXPECT_EQ(detail::validate_navigation_localization(m, backend, clock, .6), Quality::Stale);
    EXPECT_EQ(detail::validate_navigation_localization(m, "external", clock, 0), Quality::Invalid);
    m.backend_id = backend; EXPECT_EQ(valid(), Quality::Invalid);
  }
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
  ASSERT_TRUE(v.horizontal_speed_mps); EXPECT_DOUBLE_EQ(*v.horizontal_speed_mps, 5);
  ASSERT_TRUE(v.vertical_speed_mps); EXPECT_DOUBLE_EQ(*v.vertical_speed_mps, 0);
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


TEST(BridgeModel, LocalizationStatusIsObservationNotNavigationAdmission)
{
  easyfly_msgs::msg::LocalizationStatus m;
  m.backend_id = "openvins"; m.session_id = "test-session";
  m.header.frame_id = "openvins_odom"; m.header.stamp.sec = 10;
  m.tracked_frame = "livox_imu_frame";
  m.pose_stamp.sec = 10; m.state = m.ESTIMATING;
  m.pose_valid = true; m.covariance_available = true;
  ClockState clock{Quality::ValidAtObservation, 10000000000LL, 0};
  EXPECT_EQ(detail::validate_localization(m, clock, 0), Quality::ValidAtObservation);
  m.pose_stamp.sec = 9;
  EXPECT_EQ(detail::validate_localization(m, clock, 0), Quality::Stale);
  m.pose_stamp.sec = 10;
  EXPECT_EQ(detail::validate_localization(m, clock, .6), Quality::Stale);
  clock.quality = Quality::ClockPaused;
  EXPECT_EQ(detail::validate_localization(m, clock, 0), Quality::ClockPaused);
  clock.quality = Quality::ValidAtObservation;
  m.state = m.RESET_REQUIRED; m.pose_valid = false;
  EXPECT_EQ(detail::validate_localization(m, clock, 0), Quality::Invalid);
  m.state = m.INITIALIZING;
  EXPECT_EQ(detail::validate_localization(m, clock, 0), Quality::Unknown);
  m.session_id.clear();
  EXPECT_EQ(detail::validate_localization(m, clock, 0), Quality::Invalid);
  m.session_id = "test"; m.reason = std::string(2049, 'x');
  EXPECT_EQ(detail::validate_localization(m, clock, 0), Quality::Invalid);
}

TEST(BridgeModel, OrbSlamMapPoseRequiresUnavailableCovarianceAndBaseFrame)
{
  easyfly_msgs::msg::LocalizationStatus m;
  m.backend_id = "orbslam3"; m.session_id = "orb-map-session";
  m.header.frame_id = "orbslam3_map"; m.header.stamp.sec = 10;
  m.tracked_frame = "base_link"; m.pose_stamp.sec = 10;
  m.state = m.ESTIMATING; m.pose_valid = true; m.covariance_available = false;
  m.tracking_quality_available = true;
  ClockState clock{Quality::ValidAtObservation, 10000000000LL, 0};
  EXPECT_EQ(detail::validate_localization(m, clock, 0), Quality::ValidAtObservation);
  m.covariance_available = true;
  EXPECT_EQ(detail::validate_localization(m, clock, 0), Quality::Invalid);
  m.covariance_available = false; m.tracked_frame = "livox_imu_frame";
  EXPECT_EQ(detail::validate_localization(m, clock, 0), Quality::Invalid);
  m.tracked_frame = "base_link"; m.state = m.RESET_REQUIRED; m.pose_valid = false;
  EXPECT_EQ(detail::validate_localization(m, clock, 0), Quality::Invalid);
}

TEST(BridgeModel, FastLivoStatusMatchesItsOwnFrameAndFreshness)
{
  easyfly_msgs::msg::LocalizationStatus m;
  m.backend_id = "fast_livo2"; m.session_id = "fastlivo-session";
  m.header.frame_id = "fastlivo_odom"; m.header.stamp.sec = 10;
  m.tracked_frame = "livox_imu_frame"; m.pose_stamp.sec = 10;
  m.state = m.ESTIMATING; m.pose_valid = true; m.covariance_available = true;
  m.tracking_quality_available = true;
  ClockState clock{Quality::ValidAtObservation, 10000000000LL, 0};
  EXPECT_EQ(detail::validate_localization(m, clock, 0), Quality::ValidAtObservation);
  m.header.frame_id = "openvins_odom";
  EXPECT_EQ(detail::validate_localization(m, clock, 0), Quality::Invalid);
  m.header.frame_id = "fastlivo_odom"; m.pose_stamp.sec = 9;
  EXPECT_EQ(detail::validate_localization(m, clock, 0), Quality::Stale);
  m.pose_stamp.sec = 10; m.state = m.RESET_REQUIRED; m.pose_valid = false;
  EXPECT_EQ(detail::validate_localization(m, clock, 0), Quality::Invalid);
}

TEST(BridgeModel, RawGlobalFixQualityDoesNotBorrowAggregateFreshness)
{
  sensor_msgs::msg::NavSatFix fix;
  fix.header.stamp.sec = 10;
  fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  fix.latitude = 31.2; fix.longitude = 121.5; fix.altitude = 10.;
  ClockState clock{Quality::ValidAtObservation, 10000000000LL, 0};
  EXPECT_EQ(detail::validate_global_position(fix, clock, 0), Quality::ValidAtObservation);
  fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  EXPECT_EQ(detail::validate_global_position(fix, clock, 0), Quality::Invalid);
  fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  fix.latitude = 91.; EXPECT_EQ(detail::validate_global_position(fix, clock, 0), Quality::Invalid);
  fix.latitude = 31.2; fix.longitude = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(detail::validate_global_position(fix, clock, 0), Quality::Invalid);
  fix.longitude = 121.5; fix.header.stamp.sec = 7;
  EXPECT_EQ(detail::validate_global_position(fix, clock, 0), Quality::Stale);
  fix.header.stamp.sec = 11;
  EXPECT_EQ(detail::validate_global_position(fix, clock, 0), Quality::Invalid);
  fix.header.stamp.sec = 0;
  EXPECT_EQ(detail::validate_global_position(fix, clock, 0), Quality::Invalid);
}

TEST(BridgeModel, GlobalHistoryBreaksOnDuplicatesAndInvalidityAndClearsAcrossOrigins)
{
  detail::GlobalPositionBuffer buffer;
  auto fix = std::make_shared<sensor_msgs::msg::NavSatFix>();
  fix->header.frame_id = "gps"; fix->header.stamp.sec = 10;
  fix->latitude = 31.2; fix->longitude = 121.5; fix->altitude = 10.;
  ClockState clock{Quality::ValidAtObservation, 10000000000LL, 0};
  buffer.observe(fix, clock, 10., "publisher_one");
  ASSERT_EQ(buffer.trajectory().size(), 1U);
  const auto segment = buffer.trajectory().back().segment;
  buffer.observe(std::make_shared<sensor_msgs::msg::NavSatFix>(*fix), clock, 11., "publisher_one");
  EXPECT_EQ(buffer.trajectory().size(), 1U);
  EXPECT_EQ(buffer.snapshot(clock, 12.1, "publisher_one").evidence.quality, Quality::Stale);
  fix = std::make_shared<sensor_msgs::msg::NavSatFix>(*fix);
  fix->header.stamp.sec = 12; clock.now_ns = 12000000000LL;
  buffer.observe(fix, clock, 12.2, "publisher_one");
  ASSERT_EQ(buffer.trajectory().size(), 2U);
  EXPECT_NE(buffer.trajectory().back().segment, segment);
  auto bad = std::make_shared<sensor_msgs::msg::NavSatFix>(*fix);
  bad->header.stamp.sec = 13; bad->status.status = -1; clock.now_ns = 13000000000LL;
  buffer.observe(bad, clock, 13., "publisher_one");
  EXPECT_EQ(buffer.snapshot(clock, 13., "publisher_one").evidence.quality, Quality::Invalid);
  EXPECT_EQ(buffer.trajectory().size(), 2U);
  fix = std::make_shared<sensor_msgs::msg::NavSatFix>(*fix);
  fix->header.stamp.sec = 14; clock.now_ns = 14000000000LL;
  buffer.observe(fix, clock, 14., "publisher_one");
  ASSERT_EQ(buffer.trajectory().size(), 3U);
  EXPECT_NE(buffer.trajectory()[1].segment, buffer.trajectory()[2].segment);
  buffer.snapshot(clock, 14., "publisher_two");
  EXPECT_TRUE(buffer.trajectory().empty());
  buffer.observe(fix, clock, 14., "publisher_two");
  EXPECT_EQ(buffer.trajectory().size(), 1U);
  ++clock.epoch;
  buffer.snapshot(clock, 14., "publisher_two");
  EXPECT_TRUE(buffer.trajectory().empty());
  buffer.observe(fix, clock, 14., "publisher_two");
  buffer.snapshot(clock, 14., "");
  EXPECT_TRUE(buffer.trajectory().empty());
}

TEST(BridgeModel, GlobalHistoryIsBoundedBySourceProgressCountAndFiveMinutes)
{
  detail::GlobalPositionBuffer buffer;
  ClockState clock{Quality::ValidAtObservation, 1000000000LL, 0};
  for (int i = 0; i < 5000; ++i) {
    auto fix = std::make_shared<sensor_msgs::msg::NavSatFix>();
    const int64_t stamp = 1000000000LL + int64_t(i) * 1000000LL;
    fix->header.stamp.sec = stamp / 1000000000LL;
    fix->header.stamp.nanosec = stamp % 1000000000LL;
    clock.now_ns = stamp;
    buffer.observe(fix, clock, i * .001, "source");
  }
  EXPECT_EQ(buffer.trajectory().size(), 4096U);
  buffer.snapshot(clock, 306., "source");
  EXPECT_TRUE(buffer.trajectory().empty());
}

namespace
{
std::string quoted_json(const std::string & text)
{
  std::string value = "\"";
  for (char c : text) {
    switch (c) {
      case '\n': value += "\\n"; break;
      case '\r': value += "\\r"; break;
      case '\t': value += "\\t"; break;
      case '"': value += "\\\""; break;
      case '\\': value += "\\\\"; break;
      default: value += c;
    }
  }
  return value + "\"";
}
std::string execution_context(
  int64_t stamp = 10000000000LL,
  const std::string & state = "active", const std::string & content = "type=hover\nwait=2\n")
{
  return "{\"schema_version\":1,\"stamp_ns\":" + std::to_string(stamp) +
         ",\"goal_id\":\"0123456789abcdef0123456789abcdef\",\"name\":\"demo.yaml\",\"sha256\":\"" +
         easyfly_mission_fsm::missionContentSha256(content) + "\",\"content\":" +
         quoted_json(content) + ",\"frame_id\":\"odom\",\"state\":" + quoted_json(state) +
         ",\"current_step\":0,\"completed_steps\":0,\"total_steps\":1}";
}
}  // namespace
TEST(BridgeModel, ExecutionOverviewRequiresExactNativeContentHashCountAndIdentity)
{
  easyfly_ui_bridge::MissionExecutionView view;
  int64_t stamp = 0;
  auto payload = execution_context(10000000000LL, "active", "type=hover\r\nwait=2\r\n");
  ASSERT_EQ(
    detail::validate_execution_overview(payload, &view, &stamp),
    Quality::ValidAtObservation);
  EXPECT_EQ(stamp, 10000000000LL); EXPECT_EQ(view.content, "type=hover\r\nwait=2\r\n");
  ASSERT_TRUE(view.plan); EXPECT_EQ(view.plan->steps.size(), 1U);
  for (const auto & replacement : std::vector<std::pair<std::string, std::string>>{
    {"\"schema_version\":1", "\"schema_version\":2"},
    {"\"total_steps\":1", "\"total_steps\":2"},
    {"\"current_step\":0", "\"current_step\":1"},
    {"\"completed_steps\":0", "\"completed_steps\":2"},
    {"\"frame_id\":\"odom\"", "\"frame_id\":\"map\""},
    {"\"name\":\"demo.yaml\"", "\"name\":\"../demo.yaml\""},
    {"0123456789abcdef0123456789abcdef", "not-an-execution"}})
  {
    auto bad = payload;
    bad.replace(bad.find(replacement.first), replacement.first.size(), replacement.second);
    EXPECT_EQ(detail::validate_execution_overview(bad, nullptr, nullptr), Quality::Invalid);
  }
  auto bad = payload;
  const auto digest = easyfly_mission_fsm::missionContentSha256("type=hover\r\nwait=2\r\n");
  bad[bad.find(digest)] = digest[0] == 'a' ? 'b' : 'a';
  EXPECT_EQ(detail::validate_execution_overview(bad, nullptr, nullptr), Quality::Invalid);
  EXPECT_EQ(
    detail::validate_execution_overview(
      "{\"schema_version\":1," + payload.substr(1),
      nullptr, nullptr), Quality::Invalid);
  EXPECT_EQ(
    detail::validate_execution_overview(
      std::string(400 * 1024 + 1, 'x'),
      nullptr, nullptr), Quality::BudgetExceeded);
}

TEST(BridgeModel, ExecutionContextFreshnessProgressTerminalAndSourceRemainIndependent)
{
  detail::MissionExecutionBuffer buffer;
  ClockState clock{Quality::ValidAtObservation, 10000000000LL, 0};
  const auto payload = execution_context();
  buffer.observe(payload, clock, 10., "fsm");
  ASSERT_EQ(buffer.snapshot(clock, 10., "fsm").evidence.quality, Quality::ValidAtObservation);
  buffer.observe(payload, clock, 11.9, "fsm");
  EXPECT_EQ(buffer.snapshot(clock, 12.1, "fsm").evidence.quality, Quality::Stale);
  clock.now_ns = 12000000000LL;
  buffer.observe(execution_context(clock.now_ns), clock, 12.2, "fsm");
  EXPECT_EQ(buffer.snapshot(clock, 12.2, "fsm").evidence.quality, Quality::ValidAtObservation);
  clock.now_ns = 13000000000LL;
  buffer.observe(execution_context(clock.now_ns, "active", "type=land\n"), clock, 13., "fsm");
  EXPECT_EQ(buffer.snapshot(clock, 13., "fsm").evidence.quality, Quality::Invalid);
  buffer.observe(execution_context(clock.now_ns), clock, 13.1, "fsm");
  EXPECT_EQ(buffer.snapshot(clock, 13.1, "fsm").evidence.quality, Quality::Invalid);
  clock.now_ns = 14000000000LL;
  buffer.observe(execution_context(clock.now_ns, "canceled"), clock, 14., "fsm");
  auto terminal = buffer.snapshot(clock, 14., "fsm");
  ASSERT_TRUE(terminal.value); EXPECT_EQ(terminal.value->state, "canceled");
  EXPECT_EQ(terminal.evidence.quality, Quality::ValidAtObservation);
  EXPECT_EQ(buffer.snapshot(clock, 16.1, "fsm").evidence.quality, Quality::Stale);
  EXPECT_TRUE(buffer.snapshot(clock, 16.1, "fsm").value);  // Historical geometry remains available.
  clock.now_ns = 15000000000LL;
  buffer.observe(execution_context(clock.now_ns), clock, 16.2, "fsm");
  EXPECT_EQ(buffer.snapshot(clock, 16.2, "fsm").evidence.quality, Quality::Invalid);
  EXPECT_FALSE(buffer.snapshot(clock, 16.2, "different_gid").value);
  buffer.observe(execution_context(clock.now_ns), clock, 16.2, "different_gid");
  ++clock.epoch;
  EXPECT_FALSE(buffer.snapshot(clock, 16.2, "different_gid").value);
  buffer.observe(execution_context(INT64_MAX), clock, 16.3, "different_gid");
  EXPECT_NE(
    buffer.snapshot(clock, 16.3, "different_gid").evidence.quality,
    Quality::ValidAtObservation);
}

TEST(BridgeModel, ActualExecutionStepMayBeUnknownBetweenTwoCompletedTransitions)
{
  detail::MissionExecutionBuffer buffer;
  ClockState clock{Quality::ValidAtObservation, 10000000000LL, 0};
  const std::string content = "type=hover\nwait=1\n---\ntype=hover\nwait=2\n";
  const auto sample = [&](int current, int completed) {
      auto text = execution_context(clock.now_ns, "active", content);
      const auto replace = [&](const std::string & from, const std::string & to) {
          text.replace(text.find(from), from.size(), to);
        };
      replace("\"total_steps\":1", "\"total_steps\":2");
      replace("\"current_step\":0", "\"current_step\":" + std::to_string(current));
      replace("\"completed_steps\":0", "\"completed_steps\":" + std::to_string(completed));
      return text;
    };
  buffer.observe(sample(0, 0), clock, 10., "fsm");
  ASSERT_EQ(buffer.snapshot(clock, 10., "fsm").evidence.quality, Quality::ValidAtObservation);
  clock.now_ns += 100000000LL;
  buffer.observe(sample(-1, 1), clock, 10.1, "fsm");
  auto waiting = buffer.snapshot(clock, 10.1, "fsm");
  ASSERT_EQ(waiting.evidence.quality, Quality::ValidAtObservation);
  ASSERT_TRUE(waiting.value); EXPECT_EQ(waiting.value->current_step, -1);
  EXPECT_EQ(waiting.value->completed_steps, 1);
  clock.now_ns += 100000000LL;
  buffer.observe(sample(1, 1), clock, 10.2, "fsm");
  auto second = buffer.snapshot(clock, 10.2, "fsm");
  ASSERT_EQ(second.evidence.quality, Quality::ValidAtObservation);
  ASSERT_TRUE(second.value); EXPECT_EQ(second.value->current_step, 1);
  clock.now_ns += 100000000LL;
  buffer.observe(sample(0, 1), clock, 10.3, "fsm");
  EXPECT_EQ(buffer.snapshot(clock, 10.3, "fsm").evidence.quality, Quality::Invalid);
  clock.now_ns += 100000000LL;
  buffer.observe(sample(-1, 0), clock, 10.4, "fsm");
  EXPECT_EQ(buffer.snapshot(clock, 10.4, "fsm").evidence.quality, Quality::Invalid);
}
