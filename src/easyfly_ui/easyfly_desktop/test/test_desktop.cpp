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
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPalette>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QTableView>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "easyfly_desktop/presentation.hpp"
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
TEST_F(Desktop, OfflineNeverUsesDefaultZeros)
{
  state->data = std::make_shared<b::UiSnapshot>(); state->data->running = true; open();
  EXPECT_TRUE(content("statePosition").contains(QStringLiteral("不可用")));
  EXPECT_TRUE(content("stateConnection").contains(QStringLiteral("未收到")));
  EXPECT_TRUE(content("backendProfile").contains(QStringLiteral("未确认")));
  EXPECT_FALSE(widget<QPushButton>("queryMission")->isEnabled());
  EXPECT_FALSE(widget<QWidget>("cameraCanvas")->property("has_frame").toBool());
  for (auto * button : window->findChildren<QPushButton *>()) {
    EXPECT_FALSE(button->objectName().contains("takeoff")); EXPECT_FALSE(
      button->objectName().contains("cancel"));
  }
}
TEST_F(Desktop, ReferenceThemeHasLightContentDarkNavigationAndBlueSelection)
{
  open(); QTest::qWait(50);
  auto * nav = widget<QListWidget>("navigation");
  const auto nav_pixels = nav->viewport()->grab().toImage();
  EXPECT_EQ(nav_pixels.pixelColor(10, nav_pixels.height() - 8), QColor("#233142"));
  EXPECT_EQ(nav_pixels.pixelColor(10, 20), QColor("#1769df"));
  const auto content_pixels = widget<QWidget>("pageContent")->grab().toImage();
  EXPECT_EQ(content_pixels.pixelColor(content_pixels.width() - 8, 8), QColor("#f4f6f9"));
  EXPECT_EQ(
    widget<QTableView>("parameterTable")->palette().color(QPalette::Base),
    QColor("#ffffff"));
  auto * primary = widget<QPushButton>("queryMission");
  EXPECT_FALSE(primary->isEnabled());
  widget<QLineEdit>("executionId")->setText(QString(32, 'a')); EXPECT_TRUE(primary->isEnabled());
  QCoreApplication::processEvents();
  const auto button_pixels = primary->grab().toImage();
  EXPECT_EQ(button_pixels.pixelColor(8, 8), QColor("#1769df"));
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
  EXPECT_GE(contrast(summary->palette().color(QPalette::WindowText), QColor("#202d3d")), 4.5);
  const auto * value = widget<QLabel>("statePosition");
  EXPECT_GE(contrast(value->palette().color(QPalette::WindowText), QColor("#ffffff")), 4.5);
  EXPECT_GE(contrast(QColor("#ffffff"), QColor("#1769df")), 4.5);
  state->data->command_uncertain_observed = b::Event{1, 0, "fault", 101};
  ASSERT_TRUE(until([&] {return widget<QLabel>("warningBanner")->isVisible();}));
  const auto * warning = widget<QLabel>("warningBanner");
  EXPECT_GE(contrast(warning->palette().color(QPalette::WindowText), QColor("#fff5df")), 4.5);
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
TEST_F(Desktop, BoundedEventsAndSmallWindowPages)
{
  for (int i = 0; i < 1005; ++i) {
    state->data->events.push_back({static_cast<double>(i), 0, "event", 101});
  }
  open(); window->resize(1024, 720); QTest::qWait(50);
  EXPECT_EQ(widget<QTableView>("eventTable")->model()->rowCount(), 1000);
  auto * nav = widget<QListWidget>("navigation"); ASSERT_EQ(nav->count(), 6);
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
  auto * nav = widget<QListWidget>("navigation");
  const QStringList files{"overview", "missions", "perception", "map", "diagnostics",
    "tools-boundary"};
  for (int i = 0; i < files.size(); ++i) {
    state->data = sample(); nav->setCurrentRow(i); QTest::qWait(120);
    ASSERT_TRUE(window->grab().save(output.filePath(files[i] + ".png")));
  }
  window->resize(1024, 720); state->data->clock.quality = b::Quality::ClockPaused;
  state->data->command_uncertain_observed = b::Event{1, 0, "fault", 101};
  nav->setCurrentRow(0); QTest::qWait(120);
  ASSERT_TRUE(window->grab().save(output.filePath("small-warning.png")));
}
