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
#include <QDir>
#include <QPlainTextEdit>
#include <QTableWidget>
#include <QTabWidget>
#include <QTest>
#include <easyfly_ui_bridge/special_observations.hpp>
#include "easyfly_desktop/special_observation_panel.hpp"
namespace ui = easyfly_desktop;
namespace bridge = easyfly_ui_bridge;
namespace control = easyfly_ui_control;
namespace
{
template<typename Message>
void valid(bridge::Observation<Message> & o, const Message & m)
{
  o.value = std::make_shared<const Message>(m);
  o.evidence.quality = bridge::Quality::ValidAtObservation;
  o.evidence.endpoint = "/test/special";
  o.evidence.first_source_steady = o.evidence.received_steady = 100;
  o.evidence.source_age_available = true;
}
bridge::UiSnapshot snapshot()
{
  bridge::UiSnapshot s; s.running = true; s.produced_steady = 100;
  s.clock = {bridge::Quality::ValidAtObservation, 10000000000LL, 0}; return s;
}
std_msgs::msg::Header header()
{std_msgs::msg::Header h; h.frame_id = "odom"; h.stamp.sec = 10; return h;}
void capture(QWidget & panel, const QString & name)
{
  const auto root = qEnvironmentVariable("EASYFLY_UI_CAPTURE_ROOT"); if (root.isEmpty()) {return;}
  QDir dir(root); ASSERT_TRUE(dir.mkpath(".")); panel.show(); QTest::qWait(100);
  EXPECT_TRUE(panel.grab().save(dir.filePath(name)));
}
}  // namespace
TEST(SpecialPanel, ActionPhaseRequiresMatchingUnresolvedMissionAndFreshFeedback)
{
  auto s = snapshot(); easyfly_msgs::msg::MissionStepFeedback m; m.header = header();
  m.mission_goal_id = std::string(32, 'a'); m.child_goal_id = std::string(32, 'b');
  m.step_type = "precision_land"; m.phase = "align"; m.active = m.target_valid = true;
  m.horizontal_error_m = .12; m.height_above_target_m = 1.3; valid(s.special.step_feedback, m);
  ui::SpecialObservationPanel panel(true); panel.resize(1100, 650); panel.observe(s, 100, nullptr);
  auto * text = panel.findChild<QPlainTextEdit *>("ownedStepFeedback"); ASSERT_NE(text, nullptr);
  EXPECT_FALSE(text->toPlainText().contains("horizontal error="));
  auto owned = std::make_shared<control::ControlSnapshot>(); owned->intent = control::Intent{};
  owned->intent->goal = m.mission_goal_id; owned->session_received = 100;
  panel.observe(s, 100, owned); EXPECT_TRUE(text->toPlainText().contains("horizontal error=0.12"));
  panel.findChild<QTabWidget *>()->setCurrentIndex(2);
  capture(panel, "owned-precision-feedback.png");
  owned->intent->goal = std::string(32, 'c'); panel.observe(s, 100, owned);
  EXPECT_FALSE(text->toPlainText().contains("horizontal error="));
  owned->intent->goal = m.mission_goal_id; panel.observe(s, 100.6, owned);
  EXPECT_FALSE(text->toPlainText().contains("horizontal error="));
  owned->intent->resolved = true; panel.observe(s, 100, owned);
  EXPECT_FALSE(text->toPlainText().contains("horizontal error="));
}
TEST(SpecialPanel, ExpiredClassificationKeepsGeometryAndStaleSnapshotClearsRows)
{
  auto s = snapshot(); easyfly_msgs::msg::ClassifiedObstacleArray array; array.header = header();
  array.geometry_valid = true; easyfly_msgs::msg::ClassifiedObstacle m;
  m.obstacle.header = array.header; m.obstacle.obstacle_id = 7;
  m.obstacle.obstacle_type = "vehicle";
  m.obstacle.center_x = 2; m.obstacle.center_z = 1;
  m.obstacle.size_x = m.obstacle.size_y = m.obstacle.size_z = 1;
  m.target_id = 3; m.association_valid = true; m.confidence = .8;
  m.observed_at.sec = 10; m.expires_at.sec = 10; m.expires_at.nanosec = 200000000;
  array.obstacles.push_back(m); valid(s.special.classified, array);
  ui::SpecialObservationPanel panel(false); panel.resize(1100, 650); panel.observe(s, 100, nullptr);
  auto * t = panel.findChild<QTableWidget *>("classifiedObstacleTable"); ASSERT_NE(t, nullptr);
  ASSERT_EQ(t->rowCount(), 1); EXPECT_EQ(t->item(0, 1)->text(), "vehicle");
  const auto geometry = t->item(0, 4)->text(); s.clock.now_ns += 300000000;
  panel.observe(s, 100.3, nullptr); EXPECT_EQ(t->rowCount(), 1);
  EXPECT_EQ(t->item(0, 1)->text(), QStringLiteral("未知／过期")); EXPECT_EQ(
    t->item(
      0,
      4)->text(), geometry);
  panel.findChild<QTabWidget *>()->setCurrentIndex(1); capture(
    panel,
    "classification-expired-geometry-retained.png");
  panel.observe(s, 100.6, nullptr); EXPECT_EQ(t->rowCount(), 0);
  s.stopped = true; panel.observe(s, 100, nullptr); EXPECT_EQ(t->rowCount(), 0);
}
TEST(SpecialPanel, OpticalDepthAndLocalNormalHaveSeparateUnitsAndSources)
{
  auto s = snapshot(); easyfly_msgs::msg::LandingTarget target; target.header = header();
  target.header.frame_id = "camera_down_optical_frame"; target.tag_id = 7; target.valid = true;
  target.tag_size_m = .2; target.optical_depth_m = 1.2; target.yaw_rad = .3;
  valid(s.special.landing, target); easyfly_msgs::msg::LandingGuidanceTarget g;
  g.header = header(); g.source_header = target.header; g.tag_id = 7; g.valid = true;
  g.tag_size_m = .2; g.position.z = .1; g.plane_normal.z = 1; valid(s.special.guidance, g);
  ui::SpecialObservationPanel panel(true); panel.resize(1100, 650); panel.observe(s, 100, nullptr);
  const auto text = panel.findChild<QPlainTextEdit *>("landingObservationDetails")->toPlainText();
  EXPECT_TRUE(text.contains("optical Z=1.20 m"));
  EXPECT_TRUE(text.contains("optical XY yaw=0.30 rad"));
  EXPECT_TRUE(text.contains("camera_down_optical_frame @ 10000000000"));
  EXPECT_TRUE(text.contains("normal=[0.00, 0.00, 1.00]"));
  panel.findChild<QTabWidget *>()->setCurrentIndex(1); capture(
    panel,
    "landing-optical-and-local.png");
}
