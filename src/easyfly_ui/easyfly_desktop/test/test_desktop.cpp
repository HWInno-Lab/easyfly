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
#include <QColor>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMessageBox>
#include <QTemporaryDir>
#include <QResource>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPalette>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QScrollBar>
#include <QSplitter>
#include <QToolButton>
#include <QTableView>
#include <QTabWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>
#include <limits>
#include <iostream>
#include <stdexcept>
#include "easyfly_desktop/presentation.hpp"
#include "easyfly_desktop/mission_editor.hpp"
#include "easyfly_ui_bridge/camera_observations.hpp"
#ifdef EASYFLY_ENABLE_MEDIA
#include <opencv2/videoio.hpp>
#endif
#include "support.hpp"
using namespace desktop_test;  // NOLINT(build/namespaces)
class Desktop : public ::testing::Test
{
protected:
  std::shared_ptr<FakeState> state;
  std::unique_ptr<ui::MainWindow> window;
  void SetUp() override {application(); state = std::make_shared<FakeState>();}
  void open()
  {
    window = std::make_unique<ui::MainWindow>(b::BridgeConfig{}, factory(state)); window->show();
  }
  void TearDown() override
  {
    if (window) {
      window->close(); EXPECT_TRUE(
        until(
          [&] {
            return window->cleanup_complete();
          })); window.reset();
    }
  }
  template<typename T> T * widget(const char * name) {return window->findChild<T *>(name);}
  QString content(const char * name) {return widget<QLabel>(name)->text();}
  void click(const char * name) {widget<QPushButton>(name)->click();}
};
TEST_F(Desktop, ConnectionLobbyCreatesNoSourceAndWaitsForAnIntegratedFreshWorkspace)
{
  auto creations = std::make_shared<int>(0);
  auto original = factory(state);
  ui::SourceFactory guarded = [original, creations](
    const b::BridgeConfig & config, std::shared_ptr<const b::UiSnapshot> history)
    {
      ++*creations;
      return original(config, std::move(history));
    };
  window = std::make_unique<ui::MainWindow>(
    b::BridgeConfig{}, guarded, nullptr, ui::default_tool_options(), ui::MissionOptions{},
    ui::control_factory(), ui::ParameterOptions{}, ui::parameter_factory(), ui::ImportOptions{},
    ui::import_factory(), ui::RvizOptions{}, ui::RecordingOptions{}, false);
  auto * page = new QWidget(window.get());
  window->set_connection_page(page, true); window->show();
  auto * navigation = widget<QListWidget>("navigation");
  ASSERT_NE(navigation, nullptr);
  EXPECT_EQ(navigation->currentRow(), 0);
  EXPECT_EQ(navigation->currentItem()->text(), QStringLiteral("连接与仿真"));
  EXPECT_EQ(navigation->count(), 7);
  const QStringList keys{"connection", "overview", "tasks", "perception", "space", "records",
    "settings"};
  for (int i = 0; i < navigation->count(); ++i) {
    EXPECT_EQ(navigation->item(i)->data(Qt::UserRole).toString(), keys[i]);
  }
  for (const int i : {1, 3, 4}) {
    EXPECT_FALSE(navigation->item(i)->flags() & Qt::ItemIsEnabled);
  }
  for (const int i : {2, 5, 6}) {
    EXPECT_TRUE(navigation->item(i)->flags() & Qt::ItemIsEnabled);
  }
  EXPECT_FALSE(widget<QPushButton>("observerToggle")->isEnabled());
  QJsonObject status{{"process_running", true}, {"workspace_fresh", true},
    {"workspace", QJsonObject{{"connection_page", false}}}};
  page->setProperty("connectionState", status);
  QApplication::processEvents();
  EXPECT_TRUE(window->isVisible());  // A legacy window is not the integrated handoff target.
  status["workspace"] = QJsonObject{{"connection_page", true}, {"closing", true}};
  page->setProperty("connectionState", status);
  QApplication::processEvents(); EXPECT_TRUE(window->isVisible());
  status["workspace"] = QJsonObject{{"connection_page", true}, {"closing", false}};
  status["workspace_fresh"] = false;
  page->setProperty("connectionState", status);
  QApplication::processEvents(); EXPECT_TRUE(window->isVisible());
  EXPECT_EQ(*creations, 0);
  window->show_page("tasks"); widget<QTabWidget>("taskCenterTabs")->setCurrentIndex(1);
  click("draftNew"); auto * editor = window->findChild<ui::MissionEditor *>();
  ASSERT_TRUE(
    until(
      [&] {
        return editor->dirty() && !editor->busy() && widget<QPushButton>("draftAdd")->isEnabled();
      }));
  status["workspace_fresh"] = true;
  page->setProperty("connectionState", status);
  QApplication::processEvents(); EXPECT_TRUE(window->isVisible());
  EXPECT_TRUE(editor->dirty()); EXPECT_EQ(*creations, 0);
  QTemporaryDir drafts; editor->prepare_directory(drafts.path() + "/drafts");
  ASSERT_TRUE(until([&] {return !editor->busy();})); editor->export_file("offline.yaml");
  ASSERT_TRUE(until([&] {return !editor->dirty();}));
  EXPECT_TRUE(QFile::exists(drafts.path() + "/drafts/offline.yaml"));
  status["workspace_fresh"] = false; page->setProperty("connectionState", status);
  status["workspace_fresh"] = true; page->setProperty("connectionState", status);
  ASSERT_TRUE(until([&] {return window->cleanup_complete() && !window->isVisible();}));
  EXPECT_EQ(*creations, 0); EXPECT_EQ(state->stop_calls, 0);
}
TEST_F(Desktop, ConnectionPageSharesNavigationWithoutRestartingObservation)
{
  open();
  auto * page = new QWidget(window.get());
  window->set_connection_page(page);
  auto * navigation = widget<QListWidget>("navigation");
  ASSERT_NE(navigation, nullptr);
  EXPECT_EQ(navigation->currentRow(), 1);
  EXPECT_TRUE(window->show_connection_page());
  EXPECT_EQ(navigation->currentRow(), 0);
  navigation->setCurrentRow(0);
  page->setProperty("connectionRaiseRequested", 1);
  EXPECT_EQ(navigation->currentRow(), 0);
  EXPECT_TRUE(window->connection_status()["connection_page"].toBool());
  EXPECT_EQ(state->stop_calls, 0);
  EXPECT_TRUE(widget<QWidget>("configurationPanel")->isHidden());
  EXPECT_EQ(window->findChildren<ui::RvizView *>().size(), ui::RvizView::available() ? 1 : 0);
  EXPECT_EQ(widget<QTabWidget>("taskCenterTabs")->count(), 3);
  EXPECT_EQ(widget<QTabWidget>("spaceViewTabs")->count(), 3);
  EXPECT_EQ(widget<QTabWidget>("spaceViewTabs")->widget(2)->objectName(), "geographicView");
}
TEST_F(Desktop, TaskCatalogAndExecutionStayTogetherWithoutWastingNavigationSpace)
{
  open(); window->show_page("tasks");
  auto * tabs = widget<QTabWidget>("taskCenterTabs"); tabs->setCurrentIndex(0);
  auto * catalog = widget<QWidget>("missionCatalogPage");
  auto * query = widget<QPushButton>("queryMission");
  auto * execution = widget<QToolButton>("toggleTaskExecution");
  EXPECT_FALSE(query->isVisible()); execution->click();
  auto * navigation = widget<QListWidget>("navigation");
  for (const auto & size : {QSize(1366, 768), QSize(1024, 720)}) {
    window->resize(size); QTest::qWait(60);
    EXPECT_TRUE(catalog->isVisible()); EXPECT_TRUE(query->isVisible());
    EXPECT_GE(catalog->width(), 350);
    const auto start = catalog->mapTo(window.get(), QPoint());
    const auto nav_start = navigation->mapTo(window.get(), QPoint());
    EXPECT_LE(start.x() - nav_start.x() - navigation->width(), 24);
    EXPECT_LT(start.x() + catalog->width(), query->mapTo(window.get(), QPoint()).x());
    EXPECT_TRUE(
      window->rect().contains(
        QRect(
          query->mapTo(window.get(), QPoint()),
          query->size())));
  }
}
TEST_F(Desktop, OfflineNeverUsesDefaultZeros)
{
  state->data = std::make_shared<b::UiSnapshot>(); state->data->running = true; open();
  EXPECT_TRUE(content("statePosition").contains(QStringLiteral("不可用")));
  EXPECT_TRUE(content("stateConnection").contains(QStringLiteral("未收到")));
  EXPECT_TRUE(content("backendProfile").contains(QStringLiteral("未确认")));
  EXPECT_FALSE(widget<QPushButton>("queryMission")->isEnabled());
  EXPECT_FALSE(widget<QWidget>("cameraCanvas")->property("has_frame").toBool());
  EXPECT_FALSE(widget<QWidget>("overviewInstruments")->property("attitude_valid").toBool());
  EXPECT_FALSE(widget<QWidget>("overviewMapCanvas")->property("position_valid").toBool());
  EXPECT_EQ(content("overviewAltitude"), QStringLiteral("—"));
  for (auto * button : window->findChildren<QPushButton *>()) {
    EXPECT_FALSE(button->objectName().contains("takeoff")); EXPECT_FALSE(
      button->objectName().contains("cancel"));
  }
}
TEST_F(Desktop, CameraCanvasKeepsSpaceAndTablesExpandBelowInsteadOfInsideSidebar)
{
  open(); window->show_page("perception");
  auto * scroll = widget<QScrollArea>("cameraControlsScroll");
  auto * canvas = widget<QWidget>("cameraCanvas");
  auto * results = widget<QTabWidget>("perceptionObjectTabs");
  auto * focus = widget<QToolButton>("cameraFocusMode");
  auto * expand = widget<QToolButton>("togglePerceptionResults");
  ASSERT_NE(scroll, nullptr); ASSERT_NE(canvas, nullptr); ASSERT_NE(results, nullptr);
  for (const auto & size : {QSize(1440, 900), QSize(1366, 768), QSize(1024, 720)}) {
    window->resize(size); QTest::qWait(80);
    const QRect image(canvas->mapToGlobal(QPoint()), canvas->size());
    const QRect side(scroll->mapToGlobal(QPoint()), scroll->size());
    EXPECT_LT(image.right(), side.left()); EXPECT_GT(image.width(), side.width());
    EXPECT_GE(image.height(), 240); EXPECT_FALSE(results->isVisible());
    EXPECT_EQ(scroll->horizontalScrollBar()->maximum(), 0);
    const auto original_width = canvas->width();
    expand->click(); QTest::qWait(30);
    EXPECT_TRUE(results->isVisible()); EXPECT_GT(results->width(), scroll->width());
    EXPECT_GT(results->mapToGlobal(QPoint()).y(), image.top());
    for (const auto * name : {"targetTable", "obstacleTable", "algorithmStatusTable"}) {
      EXPECT_TRUE(results->isAncestorOf(widget<QWidget>(name)));
    }
    focus->click(); QTest::qWait(30);
    EXPECT_FALSE(scroll->isVisible()); EXPECT_FALSE(results->isVisible());
    EXPECT_GT(canvas->width(), original_width); EXPECT_FALSE(expand->isEnabled());
    focus->click(); QTest::qWait(30);
    EXPECT_TRUE(scroll->isVisible()); EXPECT_TRUE(results->isVisible());
    expand->click();
  }
}
TEST_F(Desktop, BlueGrayWorkspaceKeepsWhiteCardsAndClearActionHierarchy)
{
  open(); QTest::qWait(50);
  auto * nav = widget<QListWidget>("navigation");
  const auto nav_pixels = nav->viewport()->grab().toImage();
  EXPECT_EQ(nav_pixels.pixelColor(10, nav_pixels.height() - 8), QColor("#eaf0f8"));
  EXPECT_EQ(nav_pixels.pixelColor(10, 20), QColor("#eaf0f8"));
  const auto content_pixels = widget<QWidget>("pageContent")->grab().toImage();
  EXPECT_EQ(content_pixels.pixelColor(content_pixels.width() - 8, 8), QColor("#f3f6fb"));
  EXPECT_EQ(widget<QWidget>("topBar")->grab().toImage().pixelColor(2, 2), QColor("#ffffff"));
  EXPECT_EQ(
    widget<QTableView>("parameterTable")->palette().color(QPalette::Base),
    QColor("#ffffff"));
  auto * primary = widget<QPushButton>("queryMission");
  EXPECT_FALSE(primary->isEnabled());
  widget<QLineEdit>("executionId")->setText(QString(32, 'a')); EXPECT_TRUE(primary->isEnabled());
  QCoreApplication::processEvents();
  EXPECT_TRUE(primary->isEnabled());
  EXPECT_NE(primary->palette().color(QPalette::ButtonText), QColor("#ffffff"));
}
TEST_F(Desktop, QueryReceiptTracksMatchingReplyAndNeverLabelsMissionExecutionSuccessful)
{
  open();
  auto * input = widget<QLineEdit>("executionId");
  input->setText(QString(32, 'a'));
  state->answer = false;
  click("queryMission");
  ASSERT_TRUE(content("requestNote").contains(QStringLiteral("排队")));
  const auto id = state->sequence;
  auto & evidence = state->data->missions[b::Key::MissionQuery].evidence;
  evidence.request_id = id - 1; evidence.quality = b::Quality::ValidAtObservation;
  QTest::qWait(150); EXPECT_TRUE(content("requestNote").contains(QStringLiteral("排队")));
  evidence.request_id = id; evidence.quality = b::Quality::Unavailable;
  evidence.reason = "service response timed out";
  ASSERT_TRUE(until([&] {return content("requestNote").contains("service response timed out");}));
  EXPECT_TRUE(content("requestNote").contains(QStringLiteral("已结束")));
  EXPECT_FALSE(content("requestNote").contains(QStringLiteral("执行成功")));
}
TEST_F(Desktop, ReferenceThemeTextContrastIsReadable)
{
  open(); QTest::qWait(50);
  const auto luminance = [](const QColor & color) {
      const auto channel = [](double value) {
          return value <= .04045 ? value / 12.92 : std::pow((value + .055) / 1.055, 2.4);
        };
      return .2126 * channel(color.redF()) + .7152 * channel(color.greenF()) +
             .0722 * channel(color.blueF());
    };
  const auto contrast = [&](const QColor & a, const QColor & bcolor) {
      const double first = luminance(a), second = luminance(bcolor);
      return (std::max(first, second) + .05) / (std::min(first, second) + .05);
    };
  const auto * summary = widget<QLabel>("connectionSummary");
  EXPECT_GE(contrast(summary->palette().color(QPalette::WindowText), QColor("#ffffff")), 4.5);
  const auto * value = widget<QLabel>("statePosition");
  EXPECT_GE(contrast(value->palette().color(QPalette::WindowText), QColor("#ffffff")), 4.5);
  EXPECT_GE(contrast(QColor("#ffffff"), QColor("#1769df")), 4.5);
  state->data->command_uncertain_observed = b::Event{1, 0, "fault", 101};
  ASSERT_TRUE(until([&] {return widget<QLabel>("warningBanner")->isVisible();}));
  const auto * warning = widget<QLabel>("warningBanner");
  EXPECT_GE(contrast(warning->palette().color(QPalette::WindowText), QColor("#ffffff")), 4.5);
}
TEST_F(Desktop, LogoIsBundledAndOverviewContainsIndependentObservationPanels)
{
  open(); ASSERT_TRUE(QResource(":/easyfly/logo.png").isValid());
  ASSERT_NE(widget<QLabel>("brand"), nullptr);
  EXPECT_FALSE(widget<QLabel>("brand")->pixmap(Qt::ReturnByValue).isNull());
  EXPECT_TRUE(widget<QLabel>("brand")->text().isEmpty());
  for (const auto & size : window->windowIcon().availableSizes()) {
    EXPECT_LE(size.width(), 64); EXPECT_LE(size.height(), 64);
  }
  auto * map = widget<QWidget>("overviewMapCanvas");
  auto * split = widget<QSplitter>("overviewSplitter");
  ASSERT_NE(map, nullptr); ASSERT_NE(split, nullptr);
  EXPECT_EQ(split->orientation(), Qt::Horizontal); EXPECT_FALSE(split->childrenCollapsible());
  EXPECT_TRUE(map->property("position_valid").toBool());
  EXPECT_FALSE(map->property("heading_valid").toBool());
  EXPECT_EQ(content("overviewAltitude"), "1.50");
  EXPECT_EQ(widget<QTabWidget>("overviewMapTabs")->count(), 2);
  EXPECT_NE(widget<QLabel>("overviewSpatialHint"), nullptr);
  for (int i = 0; i < widget<QListWidget>("navigation")->count(); ++i) {
    EXPECT_FALSE(widget<QListWidget>("navigation")->item(i)->icon().isNull());
  }
}
TEST_F(Desktop, EnuCompassCardinalDirectionsAndWrapAreCorrect)
{
  constexpr double pi = 3.14159265358979323846;
  const double yaw[] = {0, pi / 2, pi, -pi / 2, 2 * pi, 5 * pi / 2};
  const double expected[] = {90, 0, 270, 180, 90, 0};
  for (std::size_t i = 0; i < 6; ++i) {
    ASSERT_TRUE(ui::enu_heading_degrees(yaw[i]));
    EXPECT_NEAR(*ui::enu_heading_degrees(yaw[i]), expected[i], 1e-10);
  }
  EXPECT_FALSE(ui::enu_heading_degrees(std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(ui::enu_heading_degrees(std::numeric_limits<double>::quiet_NaN()));
}
TEST_F(Desktop, InstrumentsRequireIndependentValidityFreshClockAndOdomFrame)
{
  auto s = sample(); auto & v = s->state.sdk.value;
  s->state.attitude_valid = v.attitude_valid = true;
  v.roll = .2; v.pitch = -.1; v.yaw = .5;
  const double now = s->produced_steady;
  ASSERT_TRUE(ui::flight_instruments(*s, now).attitude_valid);
  EXPECT_NEAR(ui::flight_instruments(*s, now).roll_degrees, 11.4591559, 1e-6);
  s->state.position_valid = false;
  EXPECT_TRUE(ui::flight_instruments(*s, now).attitude_valid);
  EXPECT_FALSE(ui::flight_instruments(*s, now).position_valid);
  for (const auto quality : {b::Quality::Invalid, b::Quality::Stale, b::Quality::ClockPaused,
      b::Quality::ClockReset, b::Quality::ClockNotReady})
  {
    s->clock.quality = quality; EXPECT_FALSE(ui::flight_instruments(*s, now).attitude_valid);
  }
  s->clock.quality = b::Quality::ValidAtObservation;
  s->state.evidence.time_epoch = 1; EXPECT_FALSE(ui::flight_instruments(*s, now).attitude_valid);
  s->state.evidence.time_epoch = 0;
  EXPECT_FALSE(ui::flight_instruments(*s, now + 1.1).attitude_valid);
  s->state.sdk.fresh = false; EXPECT_FALSE(ui::flight_instruments(*s, now).attitude_valid);
  s->state.sdk.fresh = true; s->running = false;
  EXPECT_FALSE(ui::flight_instruments(*s, now).attitude_valid);
  s->running = true; v.header.frame_id = "map";
  EXPECT_FALSE(ui::flight_instruments(*s, now).attitude_valid);
  v.header.frame_id = "odom"; v.yaw = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(ui::flight_instruments(*s, now).attitude_valid);
  v.yaw = 0; v.pitch = 2; EXPECT_FALSE(ui::flight_instruments(*s, now).attitude_valid);
  v.pitch = 0; s->state.attitude_valid = false;
  EXPECT_FALSE(ui::flight_instruments(*s, now).attitude_valid);
}
TEST_F(Desktop, OverviewNeedlesClearOnInvalidAttitudePositionPauseAndDisconnect)
{
  state->data->state.attitude_valid = state->data->state.sdk.value.attitude_valid = true;
  open(); auto * dial = widget<QWidget>("overviewInstruments");
  auto * map = widget<QWidget>("overviewMapCanvas");
  ASSERT_TRUE(dial->property("attitude_valid").toBool());
  EXPECT_DOUBLE_EQ(dial->property("heading_degrees").toDouble(), 90);
  EXPECT_TRUE(map->property("heading_valid").toBool());
  state->data->state.attitude_valid = false;
  ASSERT_TRUE(until([&] {return !dial->property("attitude_valid").toBool();}));
  EXPECT_FALSE(dial->property("heading_degrees").isValid());
  EXPECT_TRUE(map->property("position_valid").toBool());
  EXPECT_FALSE(map->property("heading_valid").toBool());
  state->data->state.attitude_valid = true; state->data->state.position_valid = false;
  ASSERT_TRUE(until([&] {return !map->property("position_valid").toBool();}));
  EXPECT_TRUE(dial->property("attitude_valid").toBool());
  state->data->clock.quality = b::Quality::ClockPaused;
  ASSERT_TRUE(until([&] {return !dial->property("attitude_valid").toBool();}));
  click("observerToggle"); ASSERT_TRUE(until([&] {return window->cleanup_complete();}));
  EXPECT_FALSE(map->property("position_valid").toBool());
}
TEST_F(Desktop, OverviewDetailsAreExpandableAndMapCameraDoNotOverlap)
{
  open(); EXPECT_FALSE(widget<QWidget>("overviewEvidencePanel")->isVisible());
  click("overviewEvidenceToggle");
  EXPECT_TRUE(widget<QWidget>("overviewEvidencePanel")->isVisible());
  click("overviewEvidenceToggle"); EXPECT_FALSE(
    widget<QWidget>(
      "overviewEvidencePanel")->isVisible());
  for (const auto & size : {QSize(1440, 900), QSize(1024, 720)}) {
    window->resize(size); QTest::qWait(100);
    auto * map = widget<QWidget>("overviewMapCanvas");
    auto * camera = widget<QWidget>("overviewCamera");
    const QRect a(map->mapToGlobal(QPoint()), map->size());
    const QRect b(camera->mapToGlobal(QPoint()), camera->size());
    EXPECT_LT(a.right(), b.left()); EXPECT_GE(map->width(), 320); EXPECT_GE(camera->width(), 180);
    if (size.width() == 1440) {
      EXPECT_GE(map->visibleRegion().boundingRect().height() + 2, map->height());
      EXPECT_GE(camera->visibleRegion().boundingRect().height() + 2, camera->height());
    }
  }
}
TEST_F(Desktop, FreshnessPauseAndInvalidFields)
{
  open(); EXPECT_TRUE(content("statePosition").contains("1.25"));
  state->data->clock.quality = b::Quality::ClockPaused;
  ASSERT_TRUE(until([&] {return content("statePosition").contains(QStringLiteral("不可用"));}));
  EXPECT_TRUE(content("stateValidity").contains(QStringLiteral("暂停")));
  state->data = sample(); state->data->state.position_valid = false;
  state->data->state.sdk.value.position_x = std::numeric_limits<double>::quiet_NaN();
  ASSERT_TRUE(until([&] {return content("stateValidity").contains(QStringLiteral("位置 否"));}));
  state->data = sample(); state->data->produced_steady -= 2; state->frozen_snapshot = true;
  ASSERT_TRUE(until([&] {return content("stateValidity").contains(QStringLiteral("过期"));}));
}
TEST_F(Desktop, SourceAgeNotAdvancedWithSteadyTime)
{
  auto s = sample(); auto e = evidence(); e.source_age = .2;
  const auto display = ui::ages(e, *s, s->produced_steady + .5);
  EXPECT_TRUE(display.contains("0.20 s")); EXPECT_FALSE(display.contains("0.70 s"));
  e.source_age_available = false;
  EXPECT_TRUE(ui::ages(e, *s, s->produced_steady).endsWith(QStringLiteral("— s")));
  e.time_epoch = 1; EXPECT_EQ(
    ui::display_quality(
      e, *s, s->produced_steady, 1,
      true), b::Quality::ClockReset);
}
TEST_F(Desktop, ListValidateAndQueryHaveDistinctSemantics)
{
  open(); click("listMissions");
  ASSERT_TRUE(until([&] {return widget<QListWidget>("missionList")->count() == 1;}));
  widget<QListWidget>("missionList")->setCurrentRow(0); click("validateMission");
  ASSERT_TRUE(until([&] {return content("validationResult").contains("demo.yaml");}));
  EXPECT_EQ(state->last_validation, "demo.yaml");
  widget<QLineEdit>("executionId")->setText(QString(32, 'a')); click("queryMission");
  ASSERT_TRUE(until([&] {return content("queryResult").contains("failed");}));
  EXPECT_TRUE(content("queryResult").contains(QStringLiteral("不是任务成功")));
  EXPECT_TRUE(content("queryResult").contains(QStringLiteral("仍待确认排空")));
  EXPECT_TRUE(content("queryResult").contains("<b>"));
  EXPECT_EQ(widget<QLabel>("queryResult")->textFormat(), Qt::PlainText);
  widget<QLineEdit>("executionId")->setText(QString(32, 'c'));
  QTest::qWait(120); EXPECT_TRUE(content("queryResult").contains(QString(32, 'a')));
}
TEST_F(Desktop, WrongIdentityAndNotFoundDoNotProveRelease)
{
  state->wrong_id = true; open(); widget<QLineEdit>("executionId")->setText(QString(32, 'a'));
  click("queryMission");
  ASSERT_TRUE(until([&] {return content("queryResult").contains(QStringLiteral("错配"));}));
  state->wrong_id = false; state->unknown_id = true; click("queryMission");
  ASSERT_TRUE(until([&] {return content("queryResult").contains("not_found");}));
  EXPECT_TRUE(content("queryResult").contains(QStringLiteral("不证明已停止")));
}
TEST_F(Desktop, OlderQueryReplyHiddenWhileNewRequestPending)
{
  open(); widget<QLineEdit>("executionId")->setText(QString(32, 'a')); click("queryMission");
  ASSERT_TRUE(until([&] {return content("queryResult").contains("failed");}));
  state->answer = false; widget<QLineEdit>("executionId")->setText(QString(32, 'c')); click(
    "queryMission");
  ASSERT_TRUE(until([&] {return content("queryResult").contains(QStringLiteral("等待只读查询"));}));
  EXPECT_FALSE(content("queryResult").contains("failed"));
}
TEST_F(Desktop, WhitelistedParameterAndNoRichTooltip)
{
  open(); widget<QComboBox>("parameterSelector")->setCurrentIndex(2); click("readParameter");
  auto * table = widget<QTableView>("parameterTable");
  ASSERT_TRUE(
    until(
      [&] {
        return table->model()->index(2, 1).data().toString().contains(
          "<script>");
      }));
  EXPECT_FALSE(table->model()->index(2, 1).data(Qt::ToolTipRole).isValid());
  EXPECT_FALSE(table->model()->flags(table->model()->index(2, 1)) & Qt::ItemIsEditable);
  EXPECT_EQ(table->editTriggers(), QAbstractItemView::NoEditTriggers);
}
TEST_F(Desktop, EmptyPerceptionDistinctFromInvalidAndCameraSelectionClears)
{
  auto frame = std::make_shared<sensor_msgs::msg::Image>();
  frame->header.stamp.sec = 10; frame->header.frame_id = "camera";
  frame->encoding = "rgb8"; frame->width = 2; frame->height = 2; frame->step = 6;
  frame->data.assign(12, 140);
  state->data->images[0] = {frame, evidence()}; open();
  ASSERT_TRUE(until([&] {return widget<QWidget>("cameraCanvas")->property("has_frame").toBool();}));
  EXPECT_TRUE(content("targetSummary").contains(QStringLiteral("有效查询：0")));
  widget<QComboBox>("cameraSelector")->setCurrentIndex(1);
  ASSERT_TRUE(
    until(
      [&] {
        return !widget<QWidget>("cameraCanvas")->property("has_frame").toBool();
      }));
  state->data->perception[b::Key::Targets].evidence.quality = b::Quality::Invalid;
  ASSERT_TRUE(until([&] {return content("targetSummary").contains(QStringLiteral("不是有效空"));}));
  EXPECT_EQ(widget<QWidget>("spatialCanvas")->property("target_count").toInt(), 0);
}
TEST_F(Desktop, SameFrameCameraDisplayKeepsOverviewRawAndClearsOnSourceLoss)
{
  auto raw = std::make_shared<sensor_msgs::msg::Image>();
  raw->header.stamp.sec = 10; raw->header.stamp.nanosec = 200000000;
  raw->header.frame_id = "camera"; raw->encoding = "rgb8";
  raw->width = 640; raw->height = 480; raw->step = 1920; raw->data.assign(921600, 80);
  auto measured = std::make_shared<sensor_msgs::msg::Image>(*raw);
  measured->header.stamp.nanosec = 0; measured->data.assign(921600, 120);
  const auto now = b::detail::steady_seconds();
  b::CameraObservationBuffer buffer(
    {"/camera/color/image_raw", "/detections", "/tracks_raw", "/target_positions", "odom"});
  buffer.image(measured, now, 0);
  auto tracks = std::make_shared<easyfly_msgs::msg::TrackInfoArray>();
  tracks->header = measured->header;
  easyfly_msgs::msg::TrackInfo box; box.header = tracks->header;
  box.track_id = 7; box.class_name = "<b>person</b>"; box.confidence = .9;
  box.bbox_center_x = box.bbox_center_y = .5; box.bbox_width = box.bbox_height = .2;
  tracks->tracks.push_back(box); buffer.tracks(tracks, now, 0);
  state->data->clock.now_ns = 10200000000LL;
  state->data->images[0] = {raw, evidence()};
  state->data->camera_perception[0] = buffer.snapshot(state->data->clock, now);
  open();
  auto * camera = widget<QWidget>("cameraCanvas");
  ASSERT_TRUE(until([&] {return camera->property("has_frame").toBool();}));
  EXPECT_EQ(widget<QComboBox>("cameraOverlayMode")->currentIndex(), 0);
  widget<QComboBox>("cameraOverlayMode")->setCurrentIndex(2);
  ASSERT_TRUE(until([&] {return camera->property("overlay_count").toInt() == 1;}));
  EXPECT_EQ(camera->property("display_stamp_ns").toLongLong(), 10000000000LL);
  EXPECT_EQ(
    widget<QWidget>("overviewCamera")->property("display_stamp_ns").toLongLong(),
    10200000000LL);
  EXPECT_EQ(state->data->images[0].value->data[0], 80);
  EXPECT_EQ(measured->data[0], 120);
  auto * table = widget<QTableView>("cameraOverlayTable");
  EXPECT_EQ(table->model()->index(0, 0).data().toString(), "SORT #7");
  EXPECT_EQ(table->model()->index(0, 3).data().toString(), QStringLiteral("—"));
  EXPECT_EQ(table->model()->index(0, 1).data().toString(), "<b>person</b>");
  state->data->clock.quality = b::Quality::ClockPaused;
  ASSERT_TRUE(until([&] {return !camera->property("has_frame").toBool();}));
  EXPECT_EQ(camera->property("overlay_count").toInt(), 0);
  widget<QComboBox>("cameraSelector")->setCurrentIndex(1);
  EXPECT_FALSE(camera->property("overlay_matched").toBool());
}
TEST_F(Desktop, OverlayDisplayNeverReplacesRawRecordingPixels)
{
#ifndef EASYFLY_ENABLE_MEDIA
  GTEST_SKIP() << "Requires the opt-in media build";
#else
  QTemporaryDir directory; ASSERT_TRUE(directory.isValid()); directory.setAutoRemove(false);
  int64_t source_ns = 10200000000LL;
  b::CameraObservationBuffer buffer(
    {"/camera/color/image_raw", "/detections", "/tracks_raw", "/target_positions", "odom"});
  const auto publish = [&] {
      const auto now = b::detail::steady_seconds(); source_ns += 50000000;
      state->data->produced_steady = now; state->data->clock.now_ns = source_ns;
      auto raw = std::make_shared<sensor_msgs::msg::Image>(); raw->header.frame_id = "camera";
      raw->header.stamp.sec = source_ns / 1000000000LL;
      raw->header.stamp.nanosec = source_ns % 1000000000LL;
      raw->encoding = "rgb8"; raw->width = 640; raw->height = 480; raw->step = 1920;
      raw->data.assign(921600, 80); state->data->images[0] = {raw, evidence()};
      auto measured = std::make_shared<sensor_msgs::msg::Image>(*raw);
      const auto old_ns = source_ns - 100000000;
      measured->header.stamp.sec = old_ns / 1000000000LL;
      measured->header.stamp.nanosec = old_ns % 1000000000LL;
      measured->data.assign(921600, 120); buffer.image(measured, now, 0);
      auto tracks = std::make_shared<easyfly_msgs::msg::TrackInfoArray>();
      tracks->header = measured->header;
      easyfly_msgs::msg::TrackInfo box; box.header = tracks->header; box.track_id = 7;
      box.class_name = "person"; box.confidence = .9;
      box.bbox_center_x = box.bbox_center_y = .5; box.bbox_width = box.bbox_height = .2;
      tracks->tracks.push_back(box); buffer.tracks(tracks, now, 0);
      state->data->camera_perception[0] = buffer.snapshot(state->data->clock, now);
    };
  publish(); QTimer producer; producer.setInterval(50);
  QObject::connect(&producer, &QTimer::timeout, publish); producer.start();
  window = std::make_unique<ui::MainWindow>(
    b::BridgeConfig{}, factory(state), nullptr,
    ui::default_tool_options(), ui::MissionOptions{}, ui::control_factory(),
    ui::ParameterOptions{}, ui::parameter_factory(), ui::ImportOptions{}, ui::import_factory(),
    ui::RvizOptions{}, ui::RecordingOptions{true}); window->show();
  widget<QComboBox>("cameraOverlayMode")->setCurrentIndex(2);
  ASSERT_TRUE(
    until(
      [&] {
        return widget<QWidget>("cameraCanvas")->property("overlay_count").toInt() == 1;
      }));
  widget<QLineEdit>("recordingDirectory")->setText(directory.path() + "/records");
  ASSERT_TRUE(until([&] {return widget<QPushButton>("recordingStart")->isEnabled();}));
  click("recordingStart"); auto * prompt = widget<QMessageBox>("recordingConfirmation");
  ASSERT_NE(prompt, nullptr); prompt->done(QMessageBox::Ok); QTest::qWait(1000);
  click("recordingFinish");
  QString record;
  ASSERT_TRUE(
    until(
      [&] {
        const auto dirs =
        QDir(directory.path() + "/records").entryList(QDir::Dirs | QDir::NoDotAndDotDot);
        if (dirs.size() != 1) {return false;} record = directory.path() + "/records/" + dirs[0];
        QFile manifest(record + "/manifest.json"); if (!manifest.open(QIODevice::ReadOnly)) {
          return false;
        }
        return QJsonDocument::fromJson(manifest.readAll()).object()["status"].toString() ==
        "verified";
      }, 5000));
  producer.stop(); window->close(); ASSERT_TRUE(until([&] {return window->cleanup_complete();}));
  QFile manifest(record + "/manifest.json"); ASSERT_TRUE(manifest.open(QIODevice::ReadOnly));
  const auto data = QJsonDocument::fromJson(manifest.readAll()).object();
  ASSERT_GE(data["frames"].toString().toInt(), 3);
  for (const auto & segment : data["segments"].toArray()) {
    cv::VideoCapture video((record + "/" + segment.toObject()["name"].toString()).toStdString());
    ASSERT_TRUE(video.isOpened()); cv::Mat frame; int frames = 0;
    while (video.read(frame)) {
      ++frames; ASSERT_FALSE(frame.empty());
      for (const auto point : {cv::Point(0, 0), cv::Point(frame.cols / 2, frame.rows / 2)}) {
        const auto pixel = frame.at<cv::Vec3b>(point);
        for (int channel = 0; channel < 3; ++channel) {
          EXPECT_NEAR(pixel[channel], 80, 5);
        }
      }
    }
    EXPECT_GT(frames, 0);
  }
  std::cout << "Raw-recording pixel evidence retained: " << record.toStdString() << std::endl;
#endif
}
TEST_F(Desktop, DisconnectAndCloseWaitWithoutBlockingGui)
{
  state->stop_delay_ms = 300;
  state->data->command_uncertain_observed = b::Event{1, 0, "fault", 101}; open();
  int ticks = 0; QTimer timer; QObject::connect(&timer, &QTimer::timeout, [&] {++ticks;});
  timer.start(10);
  click("observerToggle"); EXPECT_FALSE(window->cleanup_complete());
  EXPECT_FALSE(widget<QGroupBox>("configurationPanel")->isEnabled());
  ASSERT_TRUE(until([&] {return window->cleanup_complete();})); EXPECT_GT(ticks, 10);
  EXPECT_TRUE(widget<QGroupBox>("configurationPanel")->isEnabled()); click("observerToggle");
  ASSERT_TRUE(until([&] {return !window->cleanup_complete();}));
  EXPECT_TRUE(widget<QLabel>("warningBanner")->isVisible());
  window->close(); EXPECT_TRUE(window->isVisible());
  ASSERT_TRUE(until([&] {return window->cleanup_complete() && !window->isVisible();}));
  EXPECT_EQ(state->stop_calls, 2);
}
TEST_F(Desktop, NonemptyGeometryClearsWhenInvalidOrTimeEpochChanges)
{
  auto response = std::make_shared<easyfly_msgs::srv::GetPerception::Response>(
    *state->data->perception[b::Key::Targets].value);
  easyfly_msgs::msg::Target3D target; target.header = response->targets.header;
  target.track_id = 7; target.class_name = "<b>observation</b>";
  target.position_valid = true; target.position_x = 2; target.position_y = 1;
  target.distance = 2.5; target.confidence = .8; response->targets.targets = {target};
  state->data->perception[b::Key::Targets].value = response; open();
  EXPECT_EQ(widget<QTableView>("targetTable")->model()->rowCount(), 1);
  EXPECT_EQ(widget<QWidget>("spatialCanvas")->property("target_count").toInt(), 1);
  state->data->time_epoch = 1; state->data->clock.epoch = 1;
  ASSERT_TRUE(until([&] {return widget<QTableView>("targetTable")->model()->rowCount() == 0;}));
  EXPECT_EQ(widget<QWidget>("spatialCanvas")->property("target_count").toInt(), 0);
}
TEST_F(Desktop, NewEpochNeverKeepsPreviouslyVisibleImage)
{
  auto frame = std::make_shared<sensor_msgs::msg::Image>();
  frame->header.stamp.sec = 10; frame->header.frame_id = "camera";
  frame->encoding = "rgb8"; frame->width = 2; frame->height = 2; frame->step = 6;
  frame->data.assign(12, 100); state->data->images[0] = {frame, evidence()}; open();
  ASSERT_TRUE(until([&] {return widget<QWidget>("cameraCanvas")->property("has_frame").toBool();}));
  state->data->time_epoch = 1; state->data->clock.epoch = 1;
  ASSERT_TRUE(
    until(
      [&] {
        return !widget<QWidget>("cameraCanvas")->property("has_frame").toBool();
      }));
  EXPECT_TRUE(content("imageEvidence").contains(QStringLiteral("旧时间域")));
}
TEST_F(Desktop, FactoryFailureRemainsInspectableAndClosable)
{
  window = std::make_unique<ui::MainWindow>(
    b::BridgeConfig{}, [](const auto &, auto) -> std::unique_ptr<ui::Source> {
      throw std::runtime_error("isolated construction failure");
    }); window->show(); EXPECT_TRUE(window->cleanup_complete());
  EXPECT_TRUE(content("requestNote").contains("isolated construction failure"));
  EXPECT_TRUE(widget<QGroupBox>("configurationPanel")->isEnabled());
}
TEST_F(Desktop, BadCapabilitiesAndUnknownJsonNeverImplyHealth)
{
  auto caps = std::make_shared<easyfly_msgs::srv::GetCapabilities::Response>(
    *state->data->capabilities[b::Key::FlightCapabilities].value);
  caps->ready.clear(); state->data->capabilities[b::Key::FlightCapabilities].value = caps;
  open();
  auto * model = widget<QTableView>("capabilityTable")->model();
  EXPECT_EQ(model->rowCount(), 4);
  EXPECT_EQ(model->index(0, 2).data().toString(), QStringLiteral("—"));
  caps->ready = {false}; caps->details = {"{broken"};
  QTest::qWait(120);
  EXPECT_TRUE(
    window->findChild<QPlainTextEdit *>("diagnosticText")->toPlainText().contains(
      QStringLiteral("不能默认健康")));
}
TEST_F(Desktop, GlobalEventNeverBindsToSelectedFileOrBecomesHeartbeat)
{
  auto event = std::make_shared<easyfly_msgs::msg::MissionStatus>();
  event->header.stamp.sec = 10; event->phase_name = "isolated phase";
  event->current_waypoint = 2; event->total_waypoints = 3; event->progress = .5;
  state->data->last_mission_event = {event, evidence()}; open();
  EXPECT_TRUE(content("globalPhaseMission").contains("2 / 3"));
  EXPECT_TRUE(content("globalPhaseMission").contains(QStringLiteral("无执行身份")));
  EXPECT_TRUE(content("globalPhaseMission").contains(QStringLiteral("不等于任务失败")));
  EXPECT_FALSE(content("globalPhaseMission").contains("demo.yaml"));
}
TEST_F(Desktop, MissionDetailsShowValidatedDefaultsPlainTextAndUnknownProfile)
{
  open(); click("listMissions");
  ASSERT_TRUE(until([&] {return widget<QListWidget>("missionList")->count() == 1;}));
  widget<QListWidget>("missionList")->setCurrentRow(0); click("inspectMission");
  ASSERT_TRUE(until([&] {return widget<QTableView>("missionSteps")->model()->rowCount() == 3;}));
  EXPECT_EQ(state->last_details, "demo.yaml"); EXPECT_EQ(state->details_calls, 1);
  auto * steps = widget<QTableView>("missionSteps")->model();
  EXPECT_TRUE(steps->index(0, 2).data().toString().contains("climb_speed=1.000000 m/s"));
  EXPECT_EQ(steps->index(0, 3).data().toString(), "120.000000");
  EXPECT_TRUE(widget<QPlainTextEdit>("missionContent")->isReadOnly());
  EXPECT_EQ(
    widget<QPlainTextEdit>(
      "missionContent")->toPlainText().toStdString(), state->details_content);
  EXPECT_TRUE(content("missionDetailsSummary").contains("SHA-256"));
  EXPECT_TRUE(content("missionDetailsSummary").contains(QStringLiteral("非任务文件字段")));
  EXPECT_TRUE(content("backendProfile").contains(QStringLiteral("未确认")));
  EXPECT_TRUE(content("missionLimit").contains(QStringLiteral("没有启动/取消")));
}
TEST_F(Desktop, SelectionPendingFailureAndReconnectNeverReuseOldMissionDetails)
{
  open(); click("listMissions");
  ASSERT_TRUE(until([&] {return widget<QListWidget>("missionList")->count() == 1;}));
  auto * list = widget<QListWidget>("missionList"); list->setCurrentRow(0); click("inspectMission");
  ASSERT_TRUE(
    until(
      [&] {
        return !widget<QPlainTextEdit>(
          "missionContent")->toPlainText().isEmpty();
      }));
  list->addItem("second.yaml"); list->setCurrentRow(1);
  EXPECT_TRUE(widget<QPlainTextEdit>("missionContent")->toPlainText().isEmpty());
  state->answer = false; click("inspectMission");
  ASSERT_TRUE(
    until([&] {return content("missionDetailsSummary").contains(QStringLiteral("等待"));}));
  EXPECT_EQ(widget<QTableView>("missionSteps")->model()->rowCount(), 0);
  state->answer = true; state->wrong_id = true; click("inspectMission");
  ASSERT_TRUE(
    until(
      [&] {
        return content("missionDetailsSummary").contains(QStringLiteral("不可用"));
      }));
  EXPECT_TRUE(widget<QPlainTextEdit>("missionContent")->toPlainText().isEmpty());
  click("observerToggle"); ASSERT_TRUE(until([&] {return window->cleanup_complete();}));
  click("observerToggle"); QTest::qWait(120);
  EXPECT_TRUE(widget<QPlainTextEdit>("missionContent")->toPlainText().isEmpty());
  EXPECT_EQ(widget<QTableView>("missionSteps")->model()->rowCount(), 0);
}
TEST_F(Desktop, CatalogRefreshPreservesDetailsRequestedForTheSameSelection)
{
  open(); click("listMissions");
  ASSERT_TRUE(until([&] {return widget<QListWidget>("missionList")->count() == 1;}));
  auto * list = widget<QListWidget>("missionList"); list->setCurrentRow(0);
  state->answer = false; click("listMissions");
  const auto refresh_id = state->sequence;
  ASSERT_TRUE(until([&] {return content("catalogQuality").contains(QStringLiteral("等待"));}));
  state->answer = true; click("inspectMission");
  ASSERT_TRUE(until([&] {return content("missionDetailsSummary").contains("SHA-256");}));
  const auto detail_calls = state->details_calls;
  auto response = std::make_shared<easyfly_msgs::srv::ManageMission::Response>();
  response->success = true; response->missions = {"demo.yaml"};
  auto received = evidence(); received.request_id = refresh_id;
  state->data->missions[b::Key::MissionList] = {response, received};
  ASSERT_TRUE(until([&] {return !content("catalogQuality").contains(QStringLiteral("等待"));}));
  ASSERT_NE(list->currentItem(), nullptr);
  EXPECT_EQ(list->currentItem()->text(), "demo.yaml");
  EXPECT_TRUE(content("missionDetailsSummary").contains("SHA-256"));
  EXPECT_EQ(widget<QTableView>("missionSteps")->model()->rowCount(), 3);
  EXPECT_EQ(state->details_calls, detail_calls);
}
TEST_F(Desktop, CatalogRefreshRemovingSelectionClearsItsDetails)
{
  open(); click("listMissions");
  ASSERT_TRUE(until([&] {return widget<QListWidget>("missionList")->count() == 1;}));
  auto * list = widget<QListWidget>("missionList"); list->setCurrentRow(0); click("inspectMission");
  ASSERT_TRUE(until([&] {return content("missionDetailsSummary").contains("SHA-256");}));
  state->answer = false; click("listMissions");
  const auto refresh_id = state->sequence;
  ASSERT_TRUE(until([&] {return content("catalogQuality").contains(QStringLiteral("等待"));}));
  auto response = std::make_shared<easyfly_msgs::srv::ManageMission::Response>();
  response->success = true; response->missions = {"second.yaml"};
  auto received = evidence(); received.request_id = refresh_id;
  state->data->missions[b::Key::MissionList] = {response, received};
  ASSERT_TRUE(until([&] {return !content("catalogQuality").contains(QStringLiteral("等待"));}));
  EXPECT_EQ(list->currentItem(), nullptr);
  EXPECT_FALSE(content("missionDetailsSummary").contains("SHA-256"));
  EXPECT_EQ(widget<QTableView>("missionSteps")->model()->rowCount(), 0);
  EXPECT_FALSE(widget<QPushButton>("inspectMission")->isEnabled());
}
TEST_F(Desktop, AllStepTypesAndHistoricalPreviewStayReadonly)
{
  state->details_content =
    "schema_version: 1\nframe_id: odom\nunits: {position: m, angle: rad, time: s}\nsteps:\n"
    "- {type: takeoff, altitude: 1.5}\n- {type: hover, duration: 1}\n"
    "- {type: waypoint, x: 1, y: 0, z: 1.5}\n- {type: navigate, x: 0, y: 0, z: 1.5}\n"
    "- {type: track, target_id: -1, duration: 1}\n"
    "- {type: search, center_x: 0, center_y: 0, area_width: 4, area_height: 3, "
    "altitude: 1.5, pattern: boustrophedon}\n"
    "- {type: land}\n- {type: precision_land, tag_id: 4}\n";
  open(); click("listMissions");
  ASSERT_TRUE(until([&] {return widget<QListWidget>("missionList")->count() == 1;}));
  widget<QListWidget>("missionList")->setCurrentRow(0); click("inspectMission");
  ASSERT_TRUE(until([&] {return widget<QTableView>("missionSteps")->model()->rowCount() == 8;}));
  auto * model = widget<QTableView>("missionSteps")->model();
  EXPECT_TRUE(model->index(2, 2).data().toString().contains("yaw=0.000000 rad"));
  EXPECT_TRUE(model->index(4, 2).data().toString().contains("target_id=-1"));
  EXPECT_TRUE(model->index(5, 2).data().toString().contains("geometric coverage"));
  EXPECT_TRUE(model->index(7, 2).data().toString().contains("tag_id=4"));
  state->data->mission_details.evidence.received_steady -= 3;
  ASSERT_TRUE(
    until(
      [&] {
        return content("missionDetailsSummary").contains(QStringLiteral("最后"));
      }));
  EXPECT_EQ(widget<QTableView>("missionSteps")->model()->rowCount(), 8);
  EXPECT_EQ(
    widget<QPlainTextEdit>("missionContent")->toPlainText().toStdString(),
    state->details_content);
}
TEST_F(Desktop, BoundedEventsAndSmallWindowPages)
{
  state->data->state.sdk.value.flight_mode_name = "AUTO.LOITER";
  for (int i = 0; i < 1005; ++i) {
    state->data->events.push_back({static_cast<double>(i), 0, "event", 101});
  }
  open(); window->resize(1024, 720); QTest::qWait(50);
  auto * mode_label = widget<QLabel>("stateMode");
  ASSERT_TRUE(
    until(
      [&] {
        return mode_label->fontMetrics().horizontalAdvance(mode_label->text()) <=
        mode_label->contentsRect().width();
      }));
  EXPECT_EQ(mode_label->text(), "AUTO.LOITER");
  EXPECT_EQ(widget<QTableView>("eventTable")->model()->rowCount(), 1000);
  auto * nav = widget<QListWidget>("navigation"); ASSERT_EQ(nav->count(), 7);
  for (int i = 0; i < nav->count(); ++i) {
    nav->setCurrentRow(i); QCoreApplication::processEvents(); EXPECT_TRUE(window->isVisible());
  }
}
TEST_F(Desktop, ActualWidgetScreenshotsOfIsolatedFixture)
{
  open();
  auto * watermark = new QLabel(
    QStringLiteral(
      "隔离 mock / 构造数据，非"
      "真实飞行；无飞行控制"), window.get());
  watermark->setTextFormat(Qt::PlainText); watermark->setStyleSheet(
    "color:#8a4b08; background:#fff5df; padding:6px;");
  static_cast<QVBoxLayout *>(window->centralWidget()->layout())->insertWidget(0, watermark);
  const auto output = QDir(QStringLiteral(EASYFLY_SCREENSHOT_DIRECTORY));
  ASSERT_TRUE(output.mkpath("."));
  const QStringList files{"overview", "missions", "perception", "map", "diagnostics",
    "tools-boundary"};
  const QStringList keys{"overview", "tasks", "perception", "space", "settings", "settings"};
  for (int i = 0; i < files.size(); ++i) {
    state->data = sample(); window->show_page(keys[i]);
    if (keys[i] == "settings") {
      widget<QTabWidget>("settingsTabs")->setCurrentIndex(i == 5 ? 2 : 0);
    }
    QTest::qWait(120);
    ASSERT_TRUE(window->grab().save(output.filePath(files[i] + ".png")));
  }
  const auto detail_directory = QDir(output.filePath("../d6-4"));
  ASSERT_TRUE(detail_directory.mkpath("."));
  window->show_page("tasks"); click("listMissions");
  ASSERT_TRUE(until([&] {return widget<QListWidget>("missionList")->count() == 1;}));
  widget<QListWidget>("missionList")->setCurrentRow(0); click("inspectMission"); QTest::qWait(250);
  ASSERT_TRUE(window->grab().save(detail_directory.filePath("mission-details.png")));
  widget<QTabWidget>("missionDocumentTabs")->setCurrentIndex(1); QTest::qWait(50);
  ASSERT_TRUE(window->grab().save(detail_directory.filePath("mission-content.png")));
  window->resize(1024, 720); state->data->clock.quality = b::Quality::ClockPaused;
  state->data->command_uncertain_observed = b::Event{1, 0, "fault", 101};
  window->show_page("overview"); QTest::qWait(120);
  ASSERT_TRUE(window->grab().save(output.filePath("small-warning.png")));
}
TEST_F(Desktop, OverviewFreshAttitudeAndInvalidationScreenshots)
{
  state->data->state.attitude_valid = state->data->state.sdk.value.attitude_valid = true;
  state->data->state.sdk.value.roll = .25; state->data->state.sdk.value.pitch = -.1;
  state->data->state.sdk.value.yaw = .65; open();
  auto * mark = new QLabel(
    QStringLiteral("布局验收：构造观测，无 PX4 / 飞机 / 飞行控制"), window.get());
  mark->setStyleSheet("background:#ffffff; color:#8a4b08; padding:3px;");
  static_cast<QVBoxLayout *>(window->centralWidget()->layout())->insertWidget(0, mark);
  const QDir output(QStringLiteral(EASYFLY_SCREENSHOT_DIRECTORY) + "/../overview-refresh");
  ASSERT_TRUE(output.mkpath(".")); QTest::qWait(120);
  ASSERT_TRUE(window->grab().save(output.filePath("overview-fresh.png")));
  window->resize(1024, 720); QTest::qWait(100);
  ASSERT_TRUE(window->grab().save(output.filePath("overview-small.png")));
  state->data->clock.quality = b::Quality::ClockPaused;
  ASSERT_TRUE(
    until(
      [&] {
        return !widget<QWidget>("overviewInstruments")->property("attitude_valid").toBool();
      }));
  ASSERT_TRUE(window->grab().save(output.filePath("overview-paused.png")));
}
TEST_F(Desktop, ClassicalLineViewShowsSameFrameGeometryAndClearsOnInvalidation)
{
  const auto now = b::detail::steady_seconds();
  auto image = std::make_shared<sensor_msgs::msg::Image>();
  image->header.stamp.sec = 10; image->header.frame_id = "camera";
  image->encoding = "rgb8"; image->width = 640; image->height = 480; image->step = 1920;
  image->data.assign(921600, 80);
  b::CameraPerceptionBinding binding;
  binding.image_topic = "/camera/color/image_raw";
  binding.color_line_topic = "/color_line_observations";
  b::CameraObservationBuffer buffer(binding); buffer.image(image, now, 0);
  auto lines = std::make_shared<easyfly_msgs::msg::ColorLineObservationArray>();
  lines->header = image->header; lines->valid = true; lines->image_width = 640;
  lines->image_height = 480;
  easyfly_msgs::msg::ColorLineObservation line;
  line.center_x = 320; line.center_y = 240; line.start_x = 160; line.start_y = 240;
  line.end_x = 480; line.end_y = 240; line.length_px = 320; line.width_px = 12;
  line.area_px = 3840; line.score = .9; lines->observations.push_back(line);
  buffer.color_lines(lines, now, 0);
  state->data->images[0] = {image, evidence()};
  state->data->camera_perception[0] = buffer.snapshot(state->data->clock, now);
  open(); window->show_page("perception");
  widget<QComboBox>("cameraOverlayMode")->setCurrentIndex(4);
  auto * canvas = widget<QWidget>("cameraCanvas");
  ASSERT_TRUE(until([&] {return canvas->property("overlay_count").toInt() == 1;}));
  auto * table = widget<QTableView>("cameraOverlayTable");
  EXPECT_TRUE(table->model()->index(0, 4).data().toString().contains("320"));
  EXPECT_EQ(table->model()->index(0, 3).data().toString(), QStringLiteral("—"));
  if (qEnvironmentVariableIsSet("EASYFLY_GEOMETRY_SCREENSHOT")) {
    ASSERT_TRUE(window->grab().save(qEnvironmentVariable("EASYFLY_GEOMETRY_SCREENSHOT")));
  }
  lines->valid = false; buffer.color_lines(lines, b::detail::steady_seconds(), 0);
  state->data->camera_perception[0] = buffer.snapshot(
    state->data->clock,
    b::detail::steady_seconds());
  ASSERT_TRUE(until([&] {return !canvas->property("has_frame").toBool();}));
  EXPECT_EQ(canvas->property("overlay_count").toInt(), 0);
}
TEST_F(Desktop, OpenvinsStatusShowsSessionAndDoesNotClaimNavigationReadiness)
{
  state->data->selected_localization_backend = "openvins";
  auto m = std::make_shared<easyfly_msgs::msg::LocalizationStatus>();
  m->backend_id = "openvins"; m->session_id = "visible-session";
  m->state = m->ESTIMATING; m->pose_valid = true;
  m->tracked_frame = "livox_imu_frame"; m->reason = "fresh_output_tracking_quality_unavailable";
  state->data->localization = {m, evidence()};
  open();
  auto * table = widget<QTableView>("algorithmStatusTable"); ASSERT_NE(table, nullptr);
  auto contents = [&]() {
      QString result;
      for (int row = 0; row < table->model()->rowCount(); ++row) {
        for (int column = 0; column < table->model()->columnCount(); ++column) {
          result += table->model()->data(table->model()->index(row, column)).toString();
        }
      }
      return result;
    };
  ASSERT_TRUE(until([&]() {return contents().contains("visible-session");}));
  EXPECT_TRUE(contents().contains(QStringLiteral("导航已禁用")));
  EXPECT_TRUE(contents().contains(QStringLiteral("仅表示输出新鲜")));
  m->state = m->RESET_REQUIRED; m->pose_valid = false;
  state->data->localization.evidence.quality = b::Quality::Invalid;
  ASSERT_TRUE(until([&]() {return contents().contains(QStringLiteral("需要重启整个定位会话"));}));
}
TEST_F(Desktop, VisualNavigationNeedsExplicitModeAndFreshMatchingSession)
{
  auto plan = std::make_shared<easyfly_mission_fsm::MissionValidation>();
  easyfly_mission_fsm::MissionStep step; step.type = "navigate";
  plan->steps.push_back(step); state->data->mission_plan = plan;
  state->data->selected_localization_backend = "openvins";
  const auto now = b::detail::steady_seconds();
  EXPECT_TRUE(ui::missing_mission_backends(*state->data, now).contains("OpenVINS"));
  state->data->selected_localization_backend = "fast_livo2";
  EXPECT_TRUE(ui::missing_mission_backends(*state->data, now).contains("FAST-LIVO2"));
  state->data->selected_localization_backend = "orbslam3";
  EXPECT_TRUE(ui::missing_mission_backends(*state->data, now).contains("ORB-SLAM3"));
  state->data->selected_localization_backend = "fast_lio";
  EXPECT_TRUE(ui::missing_mission_backends(*state->data, now).isEmpty());
  for (const auto & backend : {"openvins", "fast_livo2", "orbslam3"}) {
    state->data->selected_localization_backend = backend;
    state->data->selected_profile = "navigation_dev";
    state->data->localization_navigation_enabled = true;
    auto raw = std::make_shared<easyfly_msgs::msg::LocalizationStatus>();
    raw->session_id = "live"; raw->generation = 7;
    auto nav = std::make_shared<easyfly_msgs::msg::LocalizationStatus>();
    nav->session_id = raw->session_id; nav->generation = raw->generation;
    nav->backend_id = std::string(backend) + "_navigation";
    nav->header.frame_id = "odom"; nav->tracked_frame = "base_link";
    nav->header.stamp.sec = 10; nav->pose_stamp.sec = 10;
    nav->state = nav->ESTIMATING; nav->pose_valid = nav->tracking_quality_available = true;
    state->data->clock = {b::Quality::ValidAtObservation, 10000000000LL, 0};
    state->data->localization = {raw, evidence()};
    state->data->navigation_localization = {nav, evidence()};
    EXPECT_TRUE(ui::missing_mission_backends(*state->data, now).isEmpty());
    nav->generation = 8;
    EXPECT_FALSE(ui::missing_mission_backends(*state->data, now).isEmpty());
    nav->generation = 7; nav->pose_stamp.sec = 9;
    EXPECT_FALSE(ui::missing_mission_backends(*state->data, now).isEmpty());
    nav->pose_stamp.sec = 10;
    state->data->navigation_localization.evidence.quality = b::Quality::Stale;
    EXPECT_FALSE(ui::missing_mission_backends(*state->data, now).isEmpty());
  }
}
