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
#include <QAction>
#include <QComboBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTabWidget>
#include <QTemporaryDir>
#include <cmath>
#include "easyfly_desktop/geographic_view.hpp"
#include "easyfly_desktop/overview_task.hpp"
#include "support.hpp"
namespace ui = easyfly_desktop;
namespace b = easyfly_ui_bridge;
namespace
{
std::shared_ptr<b::MissionExecutionView> execution()
{
  auto result = std::make_shared<b::MissionExecutionView>();
  result->goal_id = std::string(32, 'a'); result->name = "actual.yaml";
  result->frame = "odom"; result->state = "active"; result->current_step = 1;
  result->completed_steps = 1; result->total_steps = 3;
  result->content = "schema_version: 1\nframe_id: odom\n"
    "units: {position: m, angle: rad, time: s}\nsteps:\n"
    "- {type: waypoint, x: 1, y: 2, z: 1.5}\n"
    "- {type: navigate, x: 3, y: 4, z: 1.5}\n- {type: land}\n";
  result->plan = std::make_shared<easyfly_mission_fsm::MissionValidation>(
    easyfly_mission_fsm::validateMissionText(result->content));
  result->hash = easyfly_mission_fsm::missionContentSha256(result->content); return result;
}
}  // namespace
TEST(OverviewTask, ExecutionContextRatherThanSelectedCatalogDefinesProgress)
{
  auto snapshot = desktop_test::sample(); const auto actual = execution();
  snapshot->mission_execution = {actual, desktop_test::evidence()};
  snapshot->mission_plan = std::make_shared<easyfly_mission_fsm::MissionValidation>(
    easyfly_mission_fsm::validateMissionText("type=hover\nwait=1\n"));
  const auto task = ui::overview_task_overlay(*snapshot, snapshot->produced_steady);
  EXPECT_TRUE(task.bound); EXPECT_TRUE(task.fresh); EXPECT_EQ(task.current_step, 1);
  EXPECT_EQ(task.completed_steps, 1); EXPECT_EQ(task.total_steps, 3);
  ASSERT_EQ(task.geometry.points.size(), 2u); EXPECT_DOUBLE_EQ(task.geometry.points[1].x, 3);
  ASSERT_TRUE(task.current_target); EXPECT_DOUBLE_EQ((*task.current_target)[0], 3);
  EXPECT_TRUE(task.summary.contains("actual.yaml")); EXPECT_TRUE(task.summary.contains("2/3"));
  snapshot->mission_execution.evidence.quality = b::Quality::Stale;
  const auto stale = ui::overview_task_overlay(*snapshot, snapshot->produced_steady);
  EXPECT_TRUE(stale.bound); EXPECT_FALSE(stale.fresh); EXPECT_FALSE(stale.current_target);
  EXPECT_EQ(stale.current_step, -1);
  snapshot->mission_execution.evidence.quality = b::Quality::Invalid;
  EXPECT_FALSE(ui::overview_task_overlay(*snapshot, snapshot->produced_steady).bound);
  snapshot->mission_execution = {}; EXPECT_FALSE(
    ui::overview_task_overlay(*snapshot, snapshot->produced_steady).bound);
}
TEST(OverviewTask, DynamicTargetRequiresSameActualGoalAndFreshFeedback)
{
  auto snapshot = desktop_test::sample(); auto actual = execution();
  actual->content = "schema_version: 1\nframe_id: odom\n"
    "units: {position: m, angle: rad, time: s}\nsteps:\n"
    "- {type: explore, min_bound: [-2,-2,0.5], max_bound: [2,2,2.5], return_home: true}\n";
  actual->plan = std::make_shared<easyfly_mission_fsm::MissionValidation>(
    easyfly_mission_fsm::validateMissionText(actual->content));
  actual->current_step = 0; actual->completed_steps = 0; actual->total_steps = 1;
  snapshot->mission_execution = {actual, desktop_test::evidence()};
  auto feedback = std::make_shared<easyfly_msgs::msg::MissionStepFeedback>();
  feedback->header.frame_id = "odom"; feedback->active = true; feedback->step_type = "explore";
  feedback->mission_goal_id = std::string(32, 'b'); feedback->next_viewpoint.x = 1.2;
  snapshot->special.step_feedback = {feedback, desktop_test::evidence()};
  EXPECT_FALSE(ui::overview_task_overlay(*snapshot, snapshot->produced_steady).current_target);
  feedback->mission_goal_id = actual->goal_id;
  EXPECT_TRUE(ui::overview_task_overlay(*snapshot, snapshot->produced_steady).current_target);
  snapshot->special.step_feedback.evidence.quality = b::Quality::Stale;
  EXPECT_FALSE(ui::overview_task_overlay(*snapshot, snapshot->produced_steady).current_target);
}
TEST(Overview, AutoMapsStateAndCameraOperationsRemainReadOnly)
{
  desktop_test::application(); auto state = std::make_shared<desktop_test::FakeState>();
  state->data->state.horizontal_speed_mps = 2.0; state->data->state.vertical_speed_mps = -.3;
  state->data->mission_execution = {execution(), desktop_test::evidence()};
  auto fix = std::make_shared<sensor_msgs::msg::NavSatFix>(); fix->header.stamp.sec = 10;
  fix->header.frame_id = "gps"; fix->latitude = 31.2; fix->longitude = 121.5;
  state->data->global_position = {fix, desktop_test::evidence()};
  ui::MainWindow window({}, desktop_test::factory(state)); window.show();
  ASSERT_TRUE(window.show_page("overview"));
  auto * tabs = window.findChild<QTabWidget *>("overviewMapTabs");
  auto * summary = window.findChild<QLabel *>("overviewTaskSummary");
  ASSERT_TRUE(
    desktop_test::until(
      [&] {
        return tabs->currentIndex() == 1 &&
        summary->property("executionBound").toBool();
      }));
  EXPECT_EQ(window.findChild<QPushButton *>("overviewOpenSpace"), nullptr);
  EXPECT_EQ(window.findChild<QLabel *>("overviewSpatialHint"), nullptr);
  EXPECT_EQ(window.findChild<QLabel *>("stateSpeed")->text(), "2.00");
  EXPECT_TRUE(window.findChild<QLabel *>("overviewVerticalSpeed")->text().contains("-0.30"));
  auto * mode = window.findChild<QComboBox *>("overviewMapMode"); mode->setCurrentIndex(1);
  ASSERT_TRUE(desktop_test::until([&] {return tabs->currentIndex() == 0;}));
  auto * local_map = window.findChild<QWidget *>("overviewMapCanvas");
  auto * grid_layer = window.findChild<QAction *>("overviewGridLayer");
  auto * obstacle_layer = window.findChild<QAction *>("overviewObstacleLayer");
  ASSERT_NE(grid_layer, nullptr); ASSERT_NE(obstacle_layer, nullptr);
  EXPECT_TRUE(grid_layer->isChecked()); EXPECT_FALSE(obstacle_layer->isChecked());
  EXPECT_TRUE(local_map->property("grid_enabled").toBool());
  EXPECT_FALSE(local_map->property("obstacle_overlay").toBool());
  obstacle_layer->trigger();
  EXPECT_TRUE(local_map->property("obstacle_overlay").toBool());
  obstacle_layer->trigger(); grid_layer->trigger();
  EXPECT_FALSE(local_map->property("grid_enabled").toBool()); grid_layer->trigger();
  state->data->trajectory.push_back({10000000000LL, b::detail::steady_seconds(), 100, 100, 1});
  ASSERT_TRUE(desktop_test::until([&] {return local_map->property("mapRadius").toDouble() > 100;}));
  const auto radius = local_map->property("mapRadius").toDouble();
  window.findChild<QPushButton *>("overviewMapZoomIn")->click();
  ASSERT_TRUE(
    desktop_test::until(
      [&] {
        return std::abs(local_map->property("mapRadius").toDouble() - radius / 1.4) < .01;
      }));
  window.findChild<QPushButton *>("overviewMapFit")->click();

  window.findChild<QPushButton *>("overviewMapFocus")->click();
  EXPECT_EQ(window.property("overviewFocus").toInt(), 1);
  EXPECT_FALSE(window.findChild<QWidget *>("overviewCamera")->isVisible());
  window.findChild<QPushButton *>("overviewMapFocus")->click();
  EXPECT_EQ(window.property("overviewFocus").toInt(), 0);
  window.findChild<QPushButton *>("overviewCameraFocus")->click();
  EXPECT_EQ(window.property("overviewFocus").toInt(), 2); EXPECT_FALSE(tabs->isVisible());
  window.findChild<QPushButton *>("overviewCameraFocus")->click();
  window.findChild<QComboBox *>("overviewCameraSelector")->setCurrentIndex(1);
  state->data->state.sdk.value.connected = false;
  ASSERT_TRUE(
    desktop_test::until(
      [&] {
        return window.findChild<QLabel *>(
          "overviewArmed")->text().contains(QStringLiteral("未知"));
      }));
  EXPECT_TRUE(window.findChild<QLabel *>("stateMode")->text().contains(QStringLiteral("最后")));
  EXPECT_TRUE(window.findChild<QLabel *>("overviewAlert")->isVisible());
  EXPECT_EQ(state->list_calls, 0); EXPECT_EQ(state->validate_calls, 0); EXPECT_EQ(
    state->query_calls, 0);
  window.close(); ASSERT_TRUE(desktop_test::until([&] {return window.cleanup_complete();}));
}
TEST(Overview, LocalMapDoesNotShrinkWhenPerceptionExpiresAndExplicitFitStillResets)
{
  desktop_test::application(); auto state = std::make_shared<desktop_test::FakeState>();
  auto response = std::make_shared<easyfly_msgs::srv::GetPerception::Response>(
    *state->data->perception.at(b::Key::Obstacles).value);
  easyfly_msgs::msg::ObstacleInfo box; box.header = response->obstacles.header;
  box.center_x = 20; box.center_z = 1; box.size_x = box.size_y = box.size_z = 1;
  response->obstacles.obstacles.push_back(box);
  auto & observation = state->data->perception.at(b::Key::Obstacles);
  observation.value = response;
  ui::MainWindow window({}, desktop_test::factory(state)); window.show();
  ASSERT_TRUE(window.show_page("overview"));
  window.findChild<QComboBox *>("overviewMapMode")->setCurrentIndex(1);
  window.findChild<QAction *>("overviewObstacleLayer")->setChecked(true);
  auto * map = window.findChild<QWidget *>("overviewMapCanvas");
  ASSERT_TRUE(desktop_test::until([&] {return map->property("mapRadius").toDouble() > 15;}));
  const double wide = map->property("mapRadius").toDouble();
  for (int i = 0; i < 3; ++i) {
    observation.evidence.quality = b::Quality::Stale;
    ASSERT_TRUE(desktop_test::until([&] {return map->property("obstacle_count").toInt() == 0;}));
    map->grab(); EXPECT_DOUBLE_EQ(map->property("mapRadius").toDouble(), wide);
    observation.evidence = desktop_test::evidence();
    ASSERT_TRUE(desktop_test::until([&] {return map->property("obstacle_count").toInt() == 1;}));
    map->grab(); EXPECT_DOUBLE_EQ(map->property("mapRadius").toDouble(), wide);
  }
  observation.evidence.quality = b::Quality::Stale;
  ASSERT_TRUE(desktop_test::until([&] {return map->property("obstacle_count").toInt() == 0;}));
  window.findChild<QPushButton *>("overviewMapFit")->click();
  ASSERT_TRUE(desktop_test::until([&] {return map->property("mapRadius").toDouble() == 5;}));
  observation.evidence = desktop_test::evidence();
  ASSERT_TRUE(desktop_test::until([&] {return map->property("mapRadius").toDouble() > 15;}));
  observation.evidence.quality = b::Quality::Stale; ++state->data->observation_session_id;
  ASSERT_TRUE(desktop_test::until([&] {return map->property("mapRadius").toDouble() == 5;}));
  window.close(); ASSERT_TRUE(desktop_test::until([&] {return window.cleanup_complete();}));
}
TEST(SpatialMap, GridAndOptionalBoxesAreIndependentReadonlyLayersWithoutStaleRetention)
{
  desktop_test::application(); auto state = std::make_shared<desktop_test::FakeState>();
  auto grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  grid->header.frame_id = "odom"; grid->header.stamp.sec = 10;
  grid->info.width = grid->info.height = 2; grid->info.resolution = .2F;
  grid->info.origin.orientation.w = 1; grid->data = {0, 100, -1, 0};
  state->data->occupancy_grid = {grid, desktop_test::evidence()};
  int control_constructions = 0;
  const ui::ControlFactory controls = [&](const ui::control::ControlConfig &) {
      ++control_constructions; return std::unique_ptr<ui::ControlPort>{};
    };
  ui::MainWindow window(
    {}, desktop_test::factory(state), nullptr, ui::default_tool_options(), {}, controls);
  window.show(); ASSERT_TRUE(window.show_page("space"));
  auto * map = window.findChild<QWidget *>("spatialCanvas");
  auto * overview = window.findChild<QWidget *>("overviewMapCanvas");
  auto * grid_layer = window.findChild<QAction *>("spatialGridLayer");
  auto * boxes = window.findChild<QAction *>("spatialObstacleLayer");
  auto * overview_grid = window.findChild<QAction *>("overviewGridLayer");
  auto * overview_boxes = window.findChild<QAction *>("overviewObstacleLayer");
  ASSERT_NE(map, nullptr); ASSERT_NE(overview, nullptr); ASSERT_NE(grid_layer, nullptr);
  ASSERT_NE(boxes, nullptr); ASSERT_NE(overview_grid, nullptr); ASSERT_NE(overview_boxes, nullptr);
  EXPECT_TRUE(grid_layer->isChecked()); EXPECT_FALSE(boxes->isChecked());
  EXPECT_TRUE(map->property("grid_enabled").toBool());
  EXPECT_FALSE(map->property("obstacle_overlay").toBool());
  ASSERT_TRUE(desktop_test::until([&] {return map->property("grid_valid").toBool();}));
  EXPECT_EQ(map->property("occupied_cells").toInt(), 1);

  boxes->trigger(); EXPECT_TRUE(map->property("obstacle_overlay").toBool());
  EXPECT_FALSE(overview_boxes->isChecked());
  EXPECT_FALSE(overview->property("obstacle_overlay").toBool());
  boxes->trigger(); EXPECT_FALSE(map->property("obstacle_overlay").toBool());
  grid_layer->trigger(); EXPECT_FALSE(map->property("grid_enabled").toBool());
  EXPECT_FALSE(map->property("grid_valid").toBool());
  EXPECT_EQ(map->property("occupied_cells").toInt(), 0);
  EXPECT_TRUE(overview_grid->isChecked()); EXPECT_TRUE(overview->property("grid_enabled").toBool());
  grid_layer->trigger();
  ASSERT_TRUE(desktop_test::until([&] {return map->property("grid_valid").toBool();}));
  overview_grid->trigger(); overview_boxes->trigger();
  EXPECT_TRUE(map->property("grid_enabled").toBool());
  EXPECT_FALSE(map->property("obstacle_overlay").toBool());

  // Keep the observation marked valid but expire its original receipt lifetime.
  state->data->occupancy_grid.evidence.receipt_age = 2.01;
  ASSERT_TRUE(desktop_test::until([&] {return !map->property("grid_valid").toBool();}));
  EXPECT_EQ(map->property("occupied_cells").toInt(), 0);
  state->data->occupancy_grid.evidence = desktop_test::evidence();
  ASSERT_TRUE(desktop_test::until([&] {return map->property("grid_valid").toBool();}));
  EXPECT_EQ(map->property("occupied_cells").toInt(), 1);
  EXPECT_EQ(control_constructions, 0); EXPECT_EQ(state->list_calls, 0);
  EXPECT_EQ(state->validate_calls, 0); EXPECT_EQ(state->query_calls, 0);
  window.close(); ASSERT_TRUE(desktop_test::until([&] {return window.cleanup_complete();}));
}
TEST(Overview, ShortCameraGapKeepsLabeledHistoryWithoutFreshFrameOrCrossSourceReuse)
{
  desktop_test::application(); auto state = std::make_shared<desktop_test::FakeState>();
  const auto publish = [&](int second) {
      auto image = std::make_shared<sensor_msgs::msg::Image>();
      image->header.stamp.sec = second; image->header.frame_id = "camera";
      image->width = image->height = 2; image->step = 6; image->encoding = "rgb8";
      image->data.assign(12, 100);
      auto evidence = desktop_test::evidence(); evidence.time_epoch = state->data->time_epoch;
      state->data->images[0] = {image, evidence};
      state->data->clock.now_ns = int64_t(second) * 1000000000;
    };
  publish(10); QTemporaryDir directory; ASSERT_TRUE(directory.isValid());
  ui::MainWindow window(
    {}, desktop_test::factory(state), nullptr, ui::default_tool_options(),
    ui::MissionOptions{}, ui::control_factory(), ui::ParameterOptions{}, ui::parameter_factory(),
    ui::ImportOptions{}, ui::import_factory(), ui::RvizOptions{}, ui::RecordingOptions{true});
  window.show(); ASSERT_TRUE(window.show_page("overview"));
  auto * camera = window.findChild<QWidget *>("overviewCamera");
  auto * raw = window.findChild<QWidget *>("cameraCanvas");
  auto * record = window.findChild<QPushButton *>("recordingStart");
  if (auto * path = window.findChild<QLineEdit *>("recordingDirectory")) {
    path->setText(directory.filePath("record"));
  }
  const auto fresh = [&](int second) {
      return desktop_test::until(
        [&] {
          return camera->property("display_stamp_ns").toLongLong() ==
          int64_t(second) * 1000000000 && !camera->property("frame_frozen").toBool();
        });
    };
  const auto stale = [&] {
      state->data->images[0].evidence.quality = b::Quality::Stale;
      state->data->clock.now_ns += 1200000000;
      return desktop_test::until([&] {return camera->property("frame_frozen").toBool();});
    };
  ASSERT_TRUE(fresh(10));
#ifdef EASYFLY_ENABLE_MEDIA
  ASSERT_NE(record, nullptr);
  ASSERT_TRUE(desktop_test::until([&] {return record->isEnabled();}));
#endif
  ASSERT_TRUE(stale()); EXPECT_TRUE(camera->property("has_frame").toBool());
  EXPECT_FALSE(camera->property("display_stamp_ns").isValid());
  EXPECT_FALSE(raw->property("display_stamp_ns").isValid());
  EXPECT_EQ(raw->property("overlay_count").toInt(), 0);
  EXPECT_TRUE(camera->accessibleDescription().contains(QStringLiteral("最近画面（已过期）")));
  EXPECT_TRUE(camera->accessibleDescription().contains("stamp=10000000000"));
  if (record) {EXPECT_FALSE(record->isEnabled());}

  publish(12); ASSERT_TRUE(fresh(12)); ASSERT_TRUE(stale());
  window.findChild<QComboBox *>("overviewCameraSelector")->setCurrentIndex(1);
  ASSERT_TRUE(desktop_test::until([&] {return !camera->property("has_frame").toBool();}));
  EXPECT_FALSE(camera->property("frame_frozen").toBool());
  window.findChild<QComboBox *>("overviewCameraSelector")->setCurrentIndex(0);
  QTest::qWait(150); EXPECT_FALSE(camera->property("has_frame").toBool());

  publish(14); ASSERT_TRUE(fresh(14)); ASSERT_TRUE(stale());
  state->data->time_epoch = state->data->clock.epoch = 1;
  ASSERT_TRUE(desktop_test::until([&] {return !camera->property("has_frame").toBool();}));
  EXPECT_FALSE(camera->property("frame_frozen").toBool());

  publish(16); ASSERT_TRUE(fresh(16)); ASSERT_TRUE(stale());
  ASSERT_TRUE(
    desktop_test::until([&] {return !camera->property("has_frame").toBool();}, 6000));
  EXPECT_FALSE(camera->property("frame_frozen").toBool());
  publish(18); ASSERT_TRUE(fresh(18));
  state->data->images[0].evidence.quality = b::Quality::Invalid;
  ASSERT_TRUE(desktop_test::until([&] {return !camera->property("has_frame").toBool();}));
  EXPECT_FALSE(camera->property("frame_frozen").toBool());
  window.close(); ASSERT_TRUE(desktop_test::until([&] {return window.cleanup_complete();}));
}
