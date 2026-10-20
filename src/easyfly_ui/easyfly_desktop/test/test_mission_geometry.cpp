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
#include <QComboBox>
#include <QDir>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QTabWidget>
#include <QTest>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include "easyfly_desktop/mission_draft.hpp"
#include "easyfly_desktop/mission_editor.hpp"
#include "easyfly_desktop/mission_geometry.hpp"
#include "support.hpp"
namespace ui = easyfly_desktop;
namespace mission = easyfly_mission_fsm;
namespace
{
std::vector<mission::MissionStep> steps()
{
  std::vector<mission::MissionStep> s;
  for (const auto * t :
    {"takeoff", "waypoint", "hover", "navigate", "search", "navigate", "land"})
  {
    s.push_back(ui::draft_default_step(t));
  }
  s[1].x = -6; s[1].y = -4;
  s[3].x = 4; s[3].y = 6; s[3].z = 2;
  s[4].center_x = 12; s[4].center_y = -6; s[4].area_width = 8; s[4].area_height = 6;
  s[4].altitude = 3;
  s[5].x = 15; s[5].y = 10; s[5].z = 3; return s;
}
mission::MissionValidation plan() {return mission::validateMissionText(ui::draft_yaml(steps()));}
bool ready(ui::MissionEditor & editor)
{return editor.findChild<ui::MissionGeometryCanvas *>()->property("geometry_ready").toBool();}
void close(ui::MainWindow & window)
{
  window.close(); auto * prompt = window.findChild<QMessageBox *>("draftExitPrompt");
  if (prompt) {prompt->done(QMessageBox::Yes);}
  EXPECT_TRUE(desktop_test::until([&] {return window.cleanup_complete();}));
}
}  // namespace
TEST(MissionGeometry, ExplicitTargetsAndSearchBoundsOnly)
{
  const auto g = ui::draft_geometry(plan()); ASSERT_TRUE(g.accepted);
  ASSERT_EQ(g.points.size(), 3u); ASSERT_EQ(g.areas.size(), 1u); ASSERT_EQ(g.links.size(), 1u);
  EXPECT_EQ(g.points[0].step, 1u); EXPECT_DOUBLE_EQ(g.points[0].x, -6);
  EXPECT_DOUBLE_EQ(g.points[1].y, 6); EXPECT_DOUBLE_EQ(g.points[1].z, 2);
  EXPECT_EQ(g.links[0].from, 0u); EXPECT_EQ(g.links[0].to, 1u);  // Hover can preserve point order.
  EXPECT_EQ(g.areas[0].step, 4u); EXPECT_DOUBLE_EQ(g.areas[0].x, 8);
  EXPECT_DOUBLE_EQ(g.areas[0].y, -9); EXPECT_DOUBLE_EQ(g.areas[0].width, 8);
  EXPECT_DOUBLE_EQ(g.areas[0].height, 6); EXPECT_DOUBLE_EQ(g.areas[0].z, 3);
  EXPECT_EQ(g.unlocated, (std::vector<size_t>{0, 2, 6}));
}
TEST(MissionGeometry, DynamicAndRestartStepsBreakOrderLines)
{
  for (const auto * t : {"search", "track", "precision_land", "land", "takeoff"}) {
    auto a = ui::draft_default_step("waypoint"); auto b = ui::draft_default_step("navigate");
    b.x = 3;
    const auto validated =
      mission::validateMissionText(ui::draft_yaml({a, ui::draft_default_step(t), b}));
    const auto g = ui::draft_geometry(validated); ASSERT_TRUE(g.accepted); EXPECT_EQ(
      g.points.size(), 2u);
    EXPECT_TRUE(g.links.empty()) << t;
  }
}
TEST(MissionGeometry, NoInventedStartOrLandingAndDefensiveLimits)
{
  auto g = ui::draft_geometry(
    mission::validateMissionText(
      ui::draft_yaml(
  {
    ui::draft_default_step("takeoff"), ui::draft_default_step("hover"),
    ui::draft_default_step("land")})));
  EXPECT_TRUE(g.accepted); EXPECT_TRUE(g.points.empty()); EXPECT_TRUE(g.areas.empty()); EXPECT_TRUE(
    g.links.empty());
  EXPECT_EQ(g.unlocated.size(), 3u);
  EXPECT_FALSE(ui::draft_geometry({}).accepted);
  auto p = plan(); p.error = "rejected"; EXPECT_FALSE(ui::draft_geometry(p).accepted);
  p = plan(); p.steps[1].x = std::numeric_limits<double>::quiet_NaN();
  g = ui::draft_geometry(p); EXPECT_FALSE(g.accepted); EXPECT_TRUE(g.points.empty()); EXPECT_TRUE(
    g.areas.empty());
  p = plan(); p.steps[4].area_width = -1; EXPECT_FALSE(ui::draft_geometry(p).accepted);
  p = plan(); p.steps[1].type = "unknown"; EXPECT_FALSE(ui::draft_geometry(p).accepted);
  p.steps.assign(257, ui::draft_default_step("waypoint")); EXPECT_FALSE(
    ui::draft_geometry(
      p).accepted);
}
TEST(MissionGeometryCanvas, OrientationAspectFitZoomAndReadOnlySelection)
{
  ui::MissionGeometryCanvas canvas; canvas.resize(440, 320); canvas.show(); canvas.plan(plan());
  const auto origin = canvas.world_to_screen(0, 0), x = canvas.world_to_screen(1, 0),
    y = canvas.world_to_screen(0, 1);
  EXPECT_GT(x.x(), origin.x()); EXPECT_LT(y.y(), origin.y());
  EXPECT_NEAR(x.x() - origin.x(), origin.y() - y.y(), 1e-10);
  for (const auto & p : canvas.geometry().points) {
    const auto screen = canvas.world_to_screen(p.x, p.y);
    EXPECT_TRUE(canvas.rect().contains(screen.toPoint()));
    const auto restored = canvas.screen_to_world(screen); EXPECT_NEAR(restored.x(), p.x, 1e-9);
    EXPECT_NEAR(restored.y(), p.y, 1e-9);
  }
  int selected = -1;
  canvas.selection_handler([&](size_t step) {selected = static_cast<int>(step);});
  QTest::mouseClick(
    &canvas, Qt::LeftButton, Qt::NoModifier,
    canvas.world_to_screen(4, 6).toPoint()); EXPECT_EQ(selected, 3);
  QTest::mouseClick(
    &canvas, Qt::LeftButton, Qt::NoModifier, canvas.world_to_screen(
      12,
      -6).toPoint()); EXPECT_EQ(selected, 4);
  EXPECT_EQ(canvas.geometry().points.size(), 3u); EXPECT_EQ(canvas.geometry().links.size(), 1u);
  canvas.zoom(100000); EXPECT_DOUBLE_EQ(canvas.zoom_level(), 16); canvas.zoom(.000001);
  EXPECT_DOUBLE_EQ(canvas.zoom_level(), .25);
  canvas.zoom(std::numeric_limits<double>::quiet_NaN()); EXPECT_DOUBLE_EQ(canvas.zoom_level(), .25);
  canvas.fit(); EXPECT_DOUBLE_EQ(canvas.zoom_level(), 1);
  canvas.invalidate("pending"); selected = -1;
  QTest::mouseClick(&canvas, Qt::LeftButton, Qt::NoModifier, canvas.rect().center()); EXPECT_EQ(
    selected, -1);
  EXPECT_FALSE(canvas.property("geometry_ready").toBool()); EXPECT_TRUE(
    canvas.geometry().points.empty());
  EXPECT_EQ(canvas.property("selected_step").toInt(), -1);
}
TEST(MissionGeometryCanvas, WheelAndMiddleDragOnlyChangeView)
{
  ui::MissionGeometryCanvas canvas; canvas.resize(440, 320); canvas.show(); canvas.plan(plan());
  const auto center = QPointF(canvas.rect().center());
  QWheelEvent wheel(center, center, QPoint(), QPoint(0, 120), Qt::NoButton,
    Qt::NoModifier, Qt::NoScrollPhase, false);
  QCoreApplication::sendEvent(&canvas, &wheel); EXPECT_DOUBLE_EQ(canvas.zoom_level(), 1.25);
  const auto before = canvas.world_to_screen(0, 0);
  QTest::mousePress(&canvas, Qt::MiddleButton, Qt::NoModifier, center.toPoint());
  QMouseEvent move(QEvent::MouseMove, center + QPointF(20, 10), Qt::NoButton,
    Qt::MiddleButton, Qt::NoModifier);
  QCoreApplication::sendEvent(&canvas, &move);
  QTest::mouseRelease(
    &canvas, Qt::MiddleButton, Qt::NoModifier, (center + QPointF(
      20,
      10)).toPoint());
  const auto after = canvas.world_to_screen(0, 0);
  EXPECT_NEAR(after.x() - before.x(), 20, 1e-9); EXPECT_NEAR(after.y() - before.y(), 10, 1e-9);
  EXPECT_EQ(canvas.geometry().points.size(), 3u); EXPECT_EQ(canvas.geometry().areas.size(), 1u);
  EXPECT_EQ(canvas.geometry().links.size(), 1u); canvas.fit(); EXPECT_DOUBLE_EQ(
    canvas.zoom_level(), 1);
}
TEST(MissionGeometryCanvas, FarCoincidentAndMaximumSizeDoNotOverflow)
{
  std::vector<mission::MissionStep> s(256, ui::draft_default_step("waypoint"));
  for (auto & point : s) {
    point.x = 1000; point.y = -1000; point.timeout = 1;
  }
  ui::MissionGeometryCanvas canvas; canvas.resize(220, 180); canvas.show();
  canvas.plan(mission::validateMissionText(ui::draft_yaml(s))); ASSERT_TRUE(
    canvas.geometry().accepted);
  EXPECT_EQ(canvas.geometry().points.size(), 256u); EXPECT_EQ(canvas.geometry().links.size(), 255u);
  EXPECT_TRUE(canvas.rect().contains(canvas.world_to_screen(1000, -1000).toPoint()));
  canvas.select_step(255); EXPECT_FALSE(canvas.grab().isNull());
}
TEST(MissionGeometryEditor, InvalidationAndFormResetDoNotShowOldGeometry)
{
  ui::MissionEditor editor; editor.resize(1120, 840); editor.show();
  auto * body = editor.findChild<QPlainTextEdit *>("draftBody");
  body->setPlainText(
    QString::fromStdString(
      ui::draft_yaml(
        steps())));
  ASSERT_TRUE(desktop_test::until([&] {return ready(editor);}));
  auto * canvas = editor.findChild<ui::MissionGeometryCanvas *>();
  auto * list = editor.findChild<QListWidget *>("draftSteps"); list->setCurrentRow(1);
  editor.findChild<QTabWidget *>("draftTabs")->setCurrentIndex(0);
  auto * field = editor.findChild<QLineEdit *>("draftField_x"); ASSERT_NE(field, nullptr);
  field->setFocus(); field->selectAll(); QTest::keyClicks(field, "8");
  EXPECT_FALSE(ready(editor)); EXPECT_TRUE(canvas->geometry().points.empty());
  EXPECT_FALSE(editor.findChild<QPushButton *>("draftExport")->isEnabled());
  editor.findChild<QPushButton *>("draftResetForm")->click(); ASSERT_TRUE(ready(editor));
  EXPECT_DOUBLE_EQ(canvas->geometry().points[0].x, -6);
  body->setPlainText("schema_version: 9\n"); EXPECT_FALSE(ready(editor)); EXPECT_TRUE(
    canvas->geometry().points.empty());
  ASSERT_TRUE(
    desktop_test::until(
      [&] {
        return editor.findChild<QLabel *>("draftStatus")->text().contains(QStringLiteral("未通过"));
      }));
  body->setPlainText("type=takeoff\naltitude=1.5\n---\ntype=land\n");
  ASSERT_TRUE(
    desktop_test::until(
      [&] {
        return editor.findChild<QLabel *>("draftStatus")->text().contains("legacy_flat");
      }));
  EXPECT_FALSE(ready(editor)); editor.stop(); EXPECT_TRUE(
    desktop_test::until(
      [&] {
        return editor.stopped();
      }));
}
TEST(MissionGeometryEditor, LatestRevisionOnlyAndZoomSurvivesStepSelection)
{
  ui::MissionEditor editor; auto * body = editor.findChild<QPlainTextEdit *>("draftBody");
  auto s = steps();
  for (int i = 0; i < 40; ++i) {
    s[1].x = i; body->setPlainText(QString::fromStdString(ui::draft_yaml(s)));
  }
  ASSERT_TRUE(desktop_test::until([&] {return ready(editor);}));
  auto * canvas = editor.findChild<ui::MissionGeometryCanvas *>(); EXPECT_DOUBLE_EQ(
    canvas->geometry().points[0].x, 39);
  canvas->zoom(2); editor.findChild<QListWidget *>("draftSteps")->setCurrentRow(3);
  EXPECT_DOUBLE_EQ(canvas->zoom_level(), 2);
  EXPECT_EQ(canvas->property("selected_step").toInt(), 3);
  EXPECT_TRUE(editor.findChild<QLabel *>("draftGeometrySummary")->text().contains("#4 navigate"));
  editor.stop(); EXPECT_TRUE(desktop_test::until([&] {return editor.stopped();}));
}
TEST(MissionGeometryEditor, ActualScreenshotsAndClicksNeverUseTaskRpc)
{
  auto state = std::make_shared<desktop_test::FakeState>();
  state->data = std::make_shared<easyfly_ui_bridge::UiSnapshot>(); state->data->running = true;
  ui::MainWindow window(easyfly_ui_bridge::BridgeConfig{}, desktop_test::factory(state));
  window.show();
  window.show_page("tasks"); window.findChild<QTabWidget *>("taskCenterTabs")->setCurrentIndex(1);
  auto * editor = window.findChild<ui::MissionEditor *>();
  auto * body = window.findChild<QPlainTextEdit *>("draftBody");
  body->setPlainText(
    QString::fromStdString(
      ui::draft_yaml(
        steps())));
  ASSERT_TRUE(desktop_test::until([&] {return ready(*editor);}));
  window.findChild<QTabWidget *>("draftTabs")->setCurrentIndex(1);
  window.findChild<QLabel *>("requestNote")->setText(
    QStringLiteral("离线构造几何测试 · 非真实飞行 · 不发送导航"));
  auto * canvas = window.findChild<ui::MissionGeometryCanvas *>(); QTest::qWait(100);
  const auto before = body->toPlainText();
  QTest::mouseClick(
    canvas, Qt::LeftButton, Qt::NoModifier,
    canvas->world_to_screen(4, 6).toPoint());
  EXPECT_EQ(window.findChild<QListWidget *>("draftSteps")->currentRow(), 3);
  EXPECT_EQ(body->toPlainText(), before);
  QDir().mkpath(EASYFLY_GEOMETRY_SCREENSHOT_DIRECTORY);
  EXPECT_TRUE(
    window.grab().save(
      QString(EASYFLY_GEOMETRY_SCREENSHOT_DIRECTORY) +
      "/geometry-offline.png"));
  window.resize(1024, 720); QTest::qWait(100);
  const QRect visible(canvas->mapTo(&window, QPoint()), canvas->size());
  EXPECT_TRUE(window.rect().contains(visible));
  EXPECT_TRUE(
    window.grab().save(
      QString(EASYFLY_GEOMETRY_SCREENSHOT_DIRECTORY) +
      "/geometry-small-offline.png"));
  EXPECT_EQ(
    state->list_calls + state->validate_calls + state->query_calls + state->details_calls,
    0);
  close(window);
}
