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
#include <QPainter>
#include <gtest/gtest.h>
#include <cmath>
#include "../src/obstacle_display.hpp"
#include "../src/widgets.hpp"
#include "support.hpp"
namespace ui = easyfly_desktop;
namespace b = easyfly_ui_bridge;
namespace
{
ui::DisplayObstacle box(double x = 0, bool dynamic = false)
{return {x, 0, x + 1, 1, dynamic, "unknown"};}
std::shared_ptr<nav_msgs::msg::OccupancyGrid> grid()
{
  auto g = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  g->header.frame_id = "odom"; g->header.stamp.sec = 10;
  g->info.width = g->info.height = 2; g->info.resolution = 1;
  g->info.origin.position.x = g->info.origin.position.y = -3;
  g->info.origin.orientation.w = 1; g->data = {100, -1, 0, 0}; return g;
}
std::shared_ptr<b::UiSnapshot> map_snapshot()
{
  auto s = desktop_test::sample(); s->trajectory.clear();
  s->state.sdk.value.pose_valid = false; s->state.position_valid = false;
  return s;
}
QColor at_world(ui::MapCanvas & map, const QImage & image, double x, double y)
{
  const double scale = std::min(map.width(), map.height()) /
    (2.4 * map.property("mapRadius").toDouble());
  return image.pixelColor(
    qRound(map.width() / 2. + x * scale),
    qRound(map.height() / 2. - y * scale));
}
}  // namespace
TEST(ObstacleDisplay, SmallStaticChangesHaveFiniteEasingAndDuplicateDoesNotRestart)
{
  ui::ObstacleDisplay display;
  display.accept({box()}, "source", 1000000000, 1);
  display.accept({box(.08)}, "source", 1500000000, 2);
  ASSERT_EQ(display.at(2).size(), 1u); EXPECT_DOUBLE_EQ(display.at(2)[0].left, 0);
  EXPECT_NEAR(display.at(2.06)[0].left, .04, 1e-10);
  display.accept({box(.08)}, "source", 1500000000, 2.06);
  EXPECT_NEAR(display.at(2.12)[0].left, .08, 1e-10);
  EXPECT_FALSE(display.active(2.13));
}
TEST(ObstacleDisplay, DisappearanceMotionDynamicAndAmbiguityAreImmediate)
{
  ui::ObstacleDisplay display;
  display.accept({box(), box(4)}, "source", 1000000000, 1);
  display.accept({box(.08)}, "source", 1500000000, 2);
  ASSERT_EQ(display.at(2).size(), 1u);  // No historical missing second box.
  display.accept({box(2)}, "source", 1600000000, 3);
  EXPECT_DOUBLE_EQ(display.at(3)[0].left, 2);
  display.accept({box(2.08, true)}, "source", 1700000000, 4);
  EXPECT_DOUBLE_EQ(display.at(4)[0].left, 2.08);
  display.clear(); display.accept({box(), box(.02)}, "source", 1000000000, 1);
  display.accept({box(.04)}, "source", 1500000000, 2);
  EXPECT_DOUBLE_EQ(display.at(2)[0].left, .04);
  display.accept({}, "source", 1600000000, 3); EXPECT_TRUE(display.at(3).empty());
}
TEST(ObstacleDisplay, SplitAndMergeDoNotAnimateEvenWhenMajorFragmentBarelyChanges)
{
  ui::ObstacleDisplay display; display.accept({box()}, "source", 1000000000, 1);
  auto major = box(); major.right = .92; auto fragment = box(.92); fragment.right = 1;
  display.accept({major, fragment}, "source", 1500000000, 2);
  ASSERT_EQ(display.at(2).size(), 2u); EXPECT_DOUBLE_EQ(display.at(2)[0].right, .92);
  EXPECT_FALSE(display.active(2));
  display.accept({box()}, "source", 1600000000, 3);
  EXPECT_DOUBLE_EQ(display.at(3)[0].right, 1); EXPECT_FALSE(display.active(3));
}
TEST(ObstacleDisplay, SourceRollbackGapsAndExplicitResetCannotReuseGeometry)
{
  ui::ObstacleDisplay display;
  display.accept({box()}, "session1:epoch0:topic:odom", 1000000000, 1);
  display.accept({box(.08)}, "session2:epoch0:topic:odom", 1500000000, 2);
  EXPECT_DOUBLE_EQ(display.at(2)[0].left, .08);
  display.accept({box()}, "session2:epoch0:topic:odom", 1400000000, 3);
  EXPECT_DOUBLE_EQ(display.at(3)[0].left, 0);
  display.accept({box(.08)}, "session2:epoch0:topic:odom", 3000000000, 4);
  EXPECT_DOUBLE_EQ(display.at(4)[0].left, .08);
  display.clear(); EXPECT_TRUE(display.at(4).empty());
  display.accept({box(.04)}, "session2:epoch0:topic:odom", 3100000000, 5);
  EXPECT_DOUBLE_EQ(display.at(5)[0].left, .04);
}
TEST(LocalMap, LayerDefaultsAndRotatedCellsPreserveUnknownAndUnmarkedMeaning)
{
  desktop_test::application(); ui::MapCanvas map; map.resize(600, 600); map.set_follow(false);
  EXPECT_FALSE(map.property("grid_enabled").toBool());
  EXPECT_TRUE(map.property("obstacle_overlay").toBool());
  map.set_grid_enabled(true); map.set_obstacle_overlay(false);
  auto s = map_snapshot(); auto g = grid();
  g->info.origin.position.x = 0;  // 90-degree rotation: row +Y becomes world -X.
  g->info.origin.orientation.z = std::sqrt(.5); g->info.origin.orientation.w = std::sqrt(.5);
  s->occupancy_grid = {g, desktop_test::evidence()}; map.snapshot(s, s->produced_steady);
  const auto image = map.grab().toImage(); ASSERT_TRUE(map.property("grid_valid").toBool());
  EXPECT_EQ(map.property("occupied_cells").toInt(), 1);
  const auto occupied = at_world(map, image, -.5, -2.5);
  const auto unknown = at_world(map, image, -.5, -1.5);
  const auto unmarked = at_world(map, image, -1.5, -2.5);
  EXPECT_LT(occupied.red(), 110); EXPECT_GT(unknown.red(), 180);
  EXPECT_LT(unknown.red(), 245); EXPECT_EQ(unmarked, QColor(Qt::white));
  map.snapshot(s, s->produced_steady); map.grab();
  EXPECT_EQ(map.property("grid_image_builds").toInt(), 1);
}
TEST(LocalMap, FitsOccupiedCellsRatherThanFullGridAndClearsInvalidOrStale)
{
  desktop_test::application(); ui::MapCanvas map; map.resize(600, 600); map.set_follow(false);
  map.set_grid_enabled(true); map.set_obstacle_overlay(false);
  auto s = map_snapshot(); auto g = grid();
  g->info.width = g->info.height = 500; g->info.resolution = .2;
  g->info.origin.position.x = g->info.origin.position.y = -50;
  g->data.assign(250000, 0);
  g->data[195 * 500 + 234] = 100; g->data[305 * 500 + 288] = 100;
  s->occupancy_grid = {g, desktop_test::evidence()};
  map.snapshot(s, s->produced_steady); map.grab();
  ASSERT_TRUE(map.property("grid_valid").toBool());
  EXPECT_GT(map.property("mapRadius").toDouble(), 12);
  EXPECT_LT(map.property("mapRadius").toDouble(), 20);
  s->occupancy_grid.evidence.quality = b::Quality::Stale;
  map.snapshot(s, s->produced_steady); map.grab();
  EXPECT_FALSE(map.property("grid_valid").toBool());
  EXPECT_EQ(map.property("occupied_cells").toInt(), 0);
  auto invalid = std::make_shared<nav_msgs::msg::OccupancyGrid>(*g);
  invalid->data.pop_back(); s->occupancy_grid = {invalid, desktop_test::evidence()};
  map.snapshot(s, s->produced_steady); map.grab();
  EXPECT_FALSE(map.property("grid_valid").toBool());
  EXPECT_TRUE(map.property("grid_status").toString().contains(QStringLiteral("不可用")));
}
TEST(LocalMap, DisabledBoxesCannotExpandFitAndExpiredGridIsNotRetained)
{
  desktop_test::application(); ui::MapCanvas map; map.resize(600, 600); map.set_follow(false);
  map.set_grid_enabled(true); map.set_obstacle_overlay(false);
  auto s = map_snapshot();
  auto response = std::make_shared<easyfly_msgs::srv::GetPerception::Response>(
    *s->perception.at(b::Key::Obstacles).value);
  easyfly_msgs::msg::ObstacleInfo obstacle; obstacle.header = response->obstacles.header;
  obstacle.center_x = 100; obstacle.size_x = obstacle.size_y = obstacle.size_z = 1;
  response->obstacles.obstacles.push_back(obstacle);
  s->perception[b::Key::Obstacles].value = response;
  s->occupancy_grid = {grid(), desktop_test::evidence()};
  map.snapshot(s, s->produced_steady); map.grab();
  EXPECT_DOUBLE_EQ(map.property("mapRadius").toDouble(), 5);
  s->occupancy_grid.evidence.receipt_age = 2.01;
  map.snapshot(s, s->produced_steady); map.grab();
  EXPECT_FALSE(map.property("grid_valid").toBool());
  map.set_obstacle_overlay(true); map.grab();
  EXPECT_GT(map.property("mapRadius").toDouble(), 100);
  map.set_grid_enabled(false); EXPECT_FALSE(map.property("grid_valid").toBool());
}
