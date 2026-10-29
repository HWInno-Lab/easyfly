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
#include "easyfly_ui_bridge/special_observations.hpp"
using easyfly_ui_bridge::SpecialObservationBuffer;
using easyfly_ui_bridge::ClockState;
using easyfly_ui_bridge::Quality;
namespace special = easyfly_ui_bridge::special;
namespace
{
std_msgs::msg::Header header(const std::string & frame = "odom", int sec = 10)
{std_msgs::msg::Header h; h.frame_id = frame; h.stamp.sec = sec; return h;}
easyfly_msgs::msg::LandingGuidanceTarget guidance()
{
  easyfly_msgs::msg::LandingGuidanceTarget m; m.header = header();
  m.source_header = header("camera_down_optical_frame"); m.tag_id = 7; m.valid = true;
  m.tag_size_m = .2; m.position.z = .1; m.plane_normal.z = 1; return m;
}
easyfly_msgs::msg::ClassifiedObstacleArray classified()
{
  easyfly_msgs::msg::ClassifiedObstacleArray m; m.header = header(); m.geometry_valid = true;
  easyfly_msgs::msg::ClassifiedObstacle c; c.obstacle.header = m.header; c.obstacle.obstacle_id = 3;
  c.obstacle.size_x = c.obstacle.size_y = c.obstacle.size_z = 1;
  c.obstacle.obstacle_type = "vehicle"; c.target_id = 4; c.confidence = .8;
  c.observed_at.sec = 10; c.expires_at.sec = 10; c.expires_at.nanosec = 400000000;
  c.association_valid = true; m.obstacles.push_back(c); return m;
}
}  // namespace
TEST(SpecialObservations, TagsAndSelectionDoNotInventLandingPermission)
{
  easyfly_msgs::msg::TagObservationArray m; m.header = header("camera_down_optical_frame");
  m.valid = true; EXPECT_TRUE(special::valid(m, "camera_down_optical_frame"));
  easyfly_msgs::msg::TagObservation tag; tag.header = m.header; tag.tag_id = 7; tag.valid = true;
  tag.pose.position.z = 2; tag.pose.orientation.w = 1; tag.tag_size_m = .2; tag.range_m = 2;
  m.observations.push_back(tag); EXPECT_TRUE(special::valid(m, "camera_down_optical_frame"));
  m.observations.push_back(tag); EXPECT_FALSE(special::valid(m, "camera_down_optical_frame"));
  m.observations.resize(1); m.observations[0].pose.orientation.w = 0;
  EXPECT_FALSE(special::valid(m, "camera_down_optical_frame"));
  easyfly_msgs::msg::LandingTarget selected; selected.header = header("camera_down_optical_frame");
  selected.tag_id = -1; selected.reason = "not_configured";
  EXPECT_TRUE(special::valid(selected, "camera_down_optical_frame"));
  selected.valid = true; EXPECT_FALSE(special::valid(selected, "camera_down_optical_frame"));
  selected.tag_id = 7; selected.tag_size_m = .2; selected.pose = tag.pose;
  selected.optical_depth_m = 2; EXPECT_TRUE(special::valid(selected, "camera_down_optical_frame"));
  selected.optical_depth_m = 1; EXPECT_FALSE(special::valid(selected, "camera_down_optical_frame"));
}
TEST(SpecialObservations, GuidanceRejectsFrameStampAndNormalMismatch)
{
  auto m = guidance(); EXPECT_TRUE(special::valid(m, "camera_down_optical_frame"));
  m.source_header.stamp.sec = 9; EXPECT_FALSE(special::valid(m, "camera_down_optical_frame"));
  m = guidance(); m.plane_normal.z = 2;
  EXPECT_FALSE(special::valid(m, "camera_down_optical_frame"));
  m = guidance(); m.header.frame_id = "map";
  EXPECT_FALSE(special::valid(m, "camera_down_optical_frame"));
  m = guidance(); m.valid = false; m.source_header.frame_id = std::string(257, 'x');
  EXPECT_FALSE(special::valid(m, "camera_down_optical_frame"));
}
TEST(SpecialObservations, DuplicateNeverRenewsAndClockEpochClears)
{
  SpecialObservationBuffer buffer("camera_down_optical_frame");
  auto m = std::make_shared<easyfly_msgs::msg::LandingGuidanceTarget>(guidance());
  const ClockState clock{Quality::ValidAtObservation, 10000000000LL, 2};
  buffer.receive(m, 100, 2);
  EXPECT_EQ(buffer.snapshot(clock, 100.1).guidance.evidence.quality, Quality::ValidAtObservation);
  buffer.receive(m, 100.4, 2);
  EXPECT_EQ(buffer.snapshot(clock, 100.6).guidance.evidence.quality, Quality::Stale);
  EXPECT_EQ(
    buffer.snapshot({Quality::ClockPaused, clock.now_ns, 2}, 100.1).guidance.evidence.quality,
    Quality::ClockPaused);
  EXPECT_EQ(
    buffer.snapshot(
      {Quality::ValidAtObservation, clock.now_ns, 3},
      100.1).guidance.evidence.quality,
    Quality::ClockReset);
  auto conflict = std::make_shared<easyfly_msgs::msg::LandingGuidanceTarget>(*m);
  conflict->position.x = 1; buffer.receive(conflict, 100.2, 2);
  EXPECT_FALSE(buffer.snapshot(clock, 100.2).guidance.value);
  buffer.receive(m, 100.3, 2); EXPECT_FALSE(buffer.snapshot(clock, 100.3).guidance.value);
  buffer.clear(); EXPECT_FALSE(buffer.snapshot(clock, 100.3).guidance.value);
  buffer.receive(m, 100.3, 2); EXPECT_TRUE(buffer.snapshot(clock, 100.3).guidance.value);
}
TEST(SpecialObservations, ClassificationExpiryPreservesGeometry)
{
  auto m = classified(); ASSERT_TRUE(special::valid(m));
  EXPECT_TRUE(special::class_current(m.obstacles[0], 10100000000LL));
  EXPECT_FALSE(special::class_current(m.obstacles[0], 10400000000LL));
  m.obstacles[0].confidence = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(special::class_current(m.obstacles[0], 10100000000LL));
  EXPECT_TRUE(special::valid(m));  // Class failure does not erase valid original geometry.
  m.obstacles.push_back(m.obstacles[0]); EXPECT_FALSE(special::valid(m));
  m.geometry_valid = false; m.obstacles[0].obstacle.obstacle_type = std::string(129, 'x');
  EXPECT_FALSE(special::valid(m));
}
TEST(SpecialObservations, PredictionsRetainExactObservationTimeAndRejectDuplicates)
{
  easyfly_msgs::msg::TargetPredictionArray m; m.header = header();
  easyfly_msgs::msg::TargetPrediction p; p.header = m.header; p.observation_header = header();
  p.track_id = 1; p.valid = true; p.confidence = .8; p.horizon_seconds = .5;
  m.predictions.push_back(p); ASSERT_TRUE(special::valid(m));
  m.predictions[0].observation_age_seconds = 1; EXPECT_FALSE(special::valid(m));
  m.predictions[0] = p; m.predictions.push_back(p); EXPECT_FALSE(special::valid(m));
  m.predictions.resize(1); m.predictions[0].variance_x = -1; EXPECT_FALSE(special::valid(m));
  m.predictions[0] = p; m.predictions[0].valid = false;
  m.predictions[0].observation_header.frame_id = std::string(257, 'x');
  EXPECT_FALSE(special::valid(m));
}
TEST(SpecialObservations, ChildFeedbackRequiresBothActionIdentitiesAndBoundedReceiptAge)
{
  easyfly_msgs::msg::MissionStepFeedback m; m.header = header();
  m.mission_goal_id = std::string(32, 'a'); m.child_goal_id = std::string(32, 'b');
  m.step_type = "precision_land"; m.active = true; m.target_valid = true;
  m.phase = "align"; m.horizontal_error_m = .1; m.height_above_target_m = 1.2;
  EXPECT_TRUE(special::valid(m));
  m.feedback_age_sec = .6; EXPECT_FALSE(special::valid(m)); m.feedback_age_sec = 0;
  m.child_goal_id = m.mission_goal_id; EXPECT_FALSE(special::valid(m));
  m.child_goal_id = std::string(32, '0'); EXPECT_FALSE(special::valid(m));
  m.child_goal_id = std::string(32, 'b'); m.target_valid = false;
  m.height_above_target_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(special::valid(m));
}
