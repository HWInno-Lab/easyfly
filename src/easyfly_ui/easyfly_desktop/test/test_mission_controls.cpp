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
#include <sys/stat.h>
#include <unistd.h>
#include <QCheckBox>
#include <QDir>
#include <QFile>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QProcess>
#include <QPushButton>
#include <QScrollArea>
#include <QStatusBar>
#include <QTemporaryDir>
#include <QTabWidget>
#include <QVBoxLayout>
#include <functional>
#include <memory>
#include <vector>
#include "support.hpp"
using namespace desktop_test;  // NOLINT(build/namespaces)
namespace c = easyfly_ui_control;
struct ControlData
{
  c::ControlSnapshot value;
  int starts{0}, cancels{0}, stops{0}, constructions{0};
  std::string started_name, started_hash, canceled_execution, confirmation;
  bool renew_session{true};
  ControlData()
  {
    value.server_instance = std::string(32, 'a'); value.session_ready = true;
    value.session_received = c::steady_seconds();
  }
};
class FakeControl final : public ui::ControlPort
{
public:
  explicit FakeControl(std::shared_ptr<ControlData> data)
  : data_(std::move(data)) {}
  std::shared_ptr<const c::ControlSnapshot> snapshot() const override
  {
    auto copy = std::make_shared<c::ControlSnapshot>(data_->value);
    // Simulate a worker reply already received, not a timestamp newer than the GUI check.
    if (data_->renew_session) {copy->session_received = c::steady_seconds() - 0.01;}
    return copy;
  }
  bool start_confirmed(
    const easyfly_msgs::srv::GetMissionDetails::Response & r, double,
    const std::string & instance, const std::string & confirmation) override
  {
    data_->confirmation = confirmation;
    ++data_->starts; data_->started_name = r.mission_name; data_->started_hash = r.content_sha256;
    data_->value.intent = c::Intent{"domain=93;localhost=1", instance, std::string(32, 'b'),
      r.mission_name, r.content_sha256, std::string(32, 'c'), std::string(32, 'd'), false, false};
    data_->value.phase = c::Phase::Running; return true;
  }
  bool cancel_confirmed() override
  {
    if (!data_->value.intent) {return false;}
    ++data_->cancels; data_->canceled_execution = data_->value.intent->execution;
    data_->value.intent->cancel_dispatched = true; data_->value.phase = c::Phase::CancelPending;
    return true;
  }
  void stop() override {++data_->stops; data_->value.stopped = true;}

private:
  std::shared_ptr<ControlData> data_;
};
class MissionGui : public ::testing::Test
{
protected:
  QTemporaryDir directory;
  std::shared_ptr<FakeState> source;
  std::shared_ptr<ControlData> control;
  std::unique_ptr<ui::MainWindow> window;
  void SetUp() override
  {
    application(); ASSERT_TRUE(directory.isValid());
    source = std::make_shared<FakeState>(); control = std::make_shared<ControlData>();
    source->data->state.attitude_valid = source->data->state.sdk.value.attitude_valid = true;
    auto caps = std::make_shared<easyfly_msgs::srv::GetCapabilities::Response>();
    caps->names = {"versioned_yaml", "legacy_mission"};
    caps->implemented = {true, true}; caps->ready = {true, true};
    source->data->capabilities[b::Key::MissionCapabilities] = {caps, evidence()};
    parameter(b::Key::Profile, "\"navigation_dev\""); parameter(b::Key::UseSimTime, "true");
  }
  void parameter(b::Key key, const std::string & json)
  {
    auto response = std::make_shared<easyfly_msgs::srv::QueryParams::Response>();
    response->success = true; response->current_value = json;
    source->data->parameters[key] = {response, evidence()};
    source->parameter_values[key] = json;
  }
  void open(bool enabled = true, bool simulation = true, ui::ToolOptions tools = {})
  {
    auto build = [data = control](const c::ControlConfig &) {
        ++data->constructions; return std::make_unique<FakeControl>(data);
      };
    b::BridgeConfig config; config.use_sim_time = simulation;
    window = std::make_unique<ui::MainWindow>(
      config, factory(source), nullptr,
      tools, ui::MissionOptions{enabled, directory.path().toStdString()}, build);
    window->show();
  }
  template<typename T> T * widget(const char * name) {return window->findChild<T *>(name);}
  void answer(const char * name, bool yes)
  {
    auto * box = widget<QMessageBox>(name);
    ASSERT_NE(box, nullptr);
    box->done(yes ? QMessageBox::Yes : QMessageBox::Cancel);
    QCoreApplication::processEvents();
  }
  bool prepare()
  {
    widget<QPushButton>("listMissions")->click();
    if (!until([&] {return widget<QListWidget>("missionList")->count() == 1;})) {return false;}
    widget<QListWidget>("missionList")->setCurrentRow(0);
    widget<QPushButton>("inspectMission")->click();
    if (!until([&] {return widget<QLabel>("missionDetailsSummary")->text().contains("SHA-256");})) {
      return false;
    }
    widget<QCheckBox>("managedSimulationPermit")->setChecked(true);
    const bool ready = until([&] {return widget<QPushButton>("startMission")->isEnabled();});
    if (!ready) {
      ADD_FAILURE() << widget<QLabel>("missionPreflight")->text().toStdString();
    }
    return ready;
  }
  bool begin()
  {
    if (!prepare()) {return false;}
    widget<QPushButton>("startMission")->click();
    const auto exists = widget<QMessageBox>("missionStartPrompt") != nullptr;
    if (exists) {answer("missionStartPrompt", true);}
    return exists && until([&] {return control->starts == 1;});
  }
  void TearDown() override
  {
    if (!window) {return;}
    window->close();
    if (widget<QMessageBox>("missionExitPrompt")) {answer("missionExitPrompt", true);}
    EXPECT_TRUE(until([&] {return window->cleanup_complete();})); window.reset();
  }
};
TEST_F(MissionGui, DefaultWindowDoesNotCreateControllerOrControlButtons)
{
  open(false); EXPECT_EQ(control->constructions, 0);
  EXPECT_EQ(widget<QPushButton>("startMission"), nullptr);
  EXPECT_EQ(widget<QPushButton>("cancelOwnedMission"), nullptr);
}
TEST_F(MissionGui, OptInStillNeedsManualPermitAndConfirmationDefaultsToCancel)
{
  open(); EXPECT_EQ(control->constructions, 1);
  EXPECT_TRUE(widget<QLabel>("connectionSummary")->text().contains(QStringLiteral("受管仿真")));
  EXPECT_TRUE(window->statusBar()->currentMessage().contains(QStringLiteral("仿真操作逐项确认")));
  EXPECT_FALSE(widget<QCheckBox>("managedSimulationPermit")->isChecked());
  EXPECT_FALSE(widget<QPushButton>("startMission")->isEnabled()); EXPECT_EQ(control->starts, 0);
  ASSERT_TRUE(prepare()); widget<QPushButton>("startMission")->click();
  auto * box = widget<QMessageBox>("missionStartPrompt"); ASSERT_NE(box, nullptr);
  EXPECT_EQ(box->defaultButton(), box->button(QMessageBox::Cancel));
  EXPECT_EQ(box->textFormat(), Qt::PlainText); EXPECT_TRUE(box->text().contains("SHA-256"));
  answer("missionStartPrompt", false); EXPECT_EQ(control->starts, 0);
}
TEST_F(MissionGui, ExplicitStartBindsReviewedHashAndDoesNotRepeat)
{
  open(); ASSERT_TRUE(begin());
  EXPECT_EQ(control->started_name, "demo.yaml");
  EXPECT_EQ(control->started_hash, source->data->mission_details.value->content_sha256);
  EXPECT_EQ(control->confirmation.size(), 32U);
  EXPECT_FALSE(widget<QPushButton>("startMission")->isEnabled());
  widget<QPushButton>("startMission")->click(); EXPECT_EQ(control->starts, 1);
  EXPECT_TRUE(
    until(
      [&] {
        return widget<QLabel>("managedMissionStatus")->text().contains(QString(32, 'b'));
      }));
}
TEST_F(MissionGui, RetiredPromptCannotConfirmOrDismissANewerPrompt)
{
  open(); ASSERT_TRUE(prepare()); widget<QPushButton>("startMission")->click();
  auto * retired = widget<QMessageBox>("missionStartPrompt"); ASSERT_NE(retired, nullptr);
  // Exercise the deferred-deletion window without inventing an actual operator action.
  retired->done(QMessageBox::Cancel);
  widget<QPushButton>("startMission")->click();
  auto * current = widget<QMessageBox>("missionStartPrompt"); ASSERT_NE(current, nullptr);
  ASSERT_NE(current, retired);
  retired->done(QMessageBox::Yes);
  EXPECT_EQ(widget<QMessageBox>("missionStartPrompt"), current);
  QTest::qWait(200); EXPECT_EQ(control->starts, 0);
  ASSERT_EQ(widget<QMessageBox>("missionStartPrompt"), current);
  answer("missionStartPrompt", true);
  ASSERT_TRUE(until([&] {return control->starts == 1;}));
}
TEST_F(MissionGui, HumanReviewCanOutlastBudgetButUsesNewExactReads)
{
  open(); ASSERT_TRUE(prepare()); widget<QPushButton>("startMission")->click();
  source->data->mission_details.evidence.received_steady -= 3;
  for (auto & p : source->data->parameters) {
    p.second.evidence.received_steady -= 3;
  }
  for (auto & p : source->data->capabilities) {
    p.second.evidence.received_steady -= 3;
  }
  QTest::qWait(2200);
  source->data->state.evidence = evidence();
  answer("missionStartPrompt", true); EXPECT_EQ(control->starts, 0);
  ASSERT_TRUE(until([&] {return control->starts == 1;}));
  EXPECT_EQ(source->details_calls, 2); EXPECT_EQ(source->parameter_calls, 2);
  EXPECT_EQ(control->started_hash, source->data->mission_details.value->content_sha256);
}
TEST_F(MissionGui, SelectionChangesInvalidatePromptImmediately)
{
  open(); ASSERT_TRUE(prepare()); widget<QPushButton>("startMission")->click();
  auto * list = widget<QListWidget>("missionList"); list->addItem("other.yaml");
  list->setCurrentRow(1); answer("missionStartPrompt", true); EXPECT_EQ(control->starts, 0);
  EXPECT_FALSE(widget<QPushButton>("startMission")->isEnabled());
}
TEST_F(MissionGui, PendingNewDetailsAndClockResetInvalidateConfirmation)
{
  open(); ASSERT_TRUE(prepare()); widget<QPushButton>("startMission")->click();
  source->answer = false; widget<QPushButton>("inspectMission")->click();
  answer("missionStartPrompt", true); EXPECT_EQ(control->starts, 0);
  source->answer = true;
  widget<QPushButton>("inspectMission")->click();
  ASSERT_TRUE(until([&] {return widget<QPushButton>("startMission")->isEnabled();}));
  widget<QPushButton>("startMission")->click();
  ++source->data->time_epoch; source->data->clock.quality = b::Quality::ClockReset;
  QTest::qWait(120); EXPECT_FALSE(widget<QCheckBox>("managedSimulationPermit")->isChecked());
  EXPECT_EQ(control->starts, 0); EXPECT_EQ(widget<QMessageBox>("missionStartPrompt"), nullptr);
}
TEST_F(MissionGui, RecheckRejectsBadProfileSimStateFaultWithoutFlight)
{
  open(); ASSERT_TRUE(prepare()); const auto baseline = *source->data;
  const std::vector<std::function<void()>> corruptions{
    [&] {parameter(b::Key::Profile, "\"hardware_unverified\"");},
    [&] {parameter(b::Key::Profile, "\"base_sim\"");},
    [&] {parameter(b::Key::UseSimTime, "\"true\"");},
    [&] {source->data->state.sdk.value.connected = false;},
    [&] {source->data->state.attitude_valid = false;},
    [&] {source->data->state.sdk.fresh = false;},
    [&] {source->data->state.evidence.received_steady -= 3;},
    [&] {source->data->clock.quality = b::Quality::ClockPaused;},
    [&] {source->data->command_uncertain_observed = b::Event{};},
    [&] {control->renew_session = false; control->value.session_received -= 3;}
  };
  for (const auto & corrupt : corruptions) {
    *source->data = baseline; control->renew_session = true;
    parameter(b::Key::Profile, "\"navigation_dev\""); parameter(b::Key::UseSimTime, "true");
    control->value.session_received = c::steady_seconds();
    const auto refresh = [](b::Evidence & e) {
        e.received_steady = e.first_source_steady = c::steady_seconds(); e.receipt_age = 0;
      };
    refresh(source->data->state.evidence); refresh(source->data->mission_details.evidence);
    for (auto & value : source->data->parameters) {
      refresh(value.second.evidence);
    }
    for (auto & value : source->data->capabilities) {
      refresh(value.second.evidence);
    }
    // Each case is a new explicit review; do not restore a now-superseded request ID.
    widget<QPushButton>("inspectMission")->click();
    ASSERT_TRUE(until([&] {return widget<QPushButton>("startMission")->isEnabled();}));
    corrupt();
    QTest::qWait(120);
    widget<QPushButton>("startMission")->click();
    if (widget<QMessageBox>("missionStartPrompt")) {answer("missionStartPrompt", true);}
    ASSERT_TRUE(
      until(
        [&] {
          return !widget<QPushButton>("stopMissionRecheck")->isEnabled();
        }));
    EXPECT_EQ(control->starts, 0);
  }
}
TEST_F(MissionGui, ContentChangeAfterReviewRequiresNewExplicitReview)
{
  open(); ASSERT_TRUE(prepare()); widget<QPushButton>("startMission")->click();
  source->details_content += "# mutated catalog\n";
  answer("missionStartPrompt", true);
  ASSERT_TRUE(
    until(
      [&] {
        return widget<QLabel>("missionPreflight")->text().contains(QStringLiteral("必须重新审阅"));
      }));
  EXPECT_EQ(control->starts, 0); QTest::qWait(200); EXPECT_EQ(control->starts, 0);
}
TEST_F(MissionGui, ReadOnlyRecheckCanBeCanceledAndCloseNeverDispatches)
{
  open(); ASSERT_TRUE(prepare()); source->answer = false;
  widget<QPushButton>("startMission")->click(); answer("missionStartPrompt", true);
  ASSERT_TRUE(until([&] {return widget<QPushButton>("stopMissionRecheck")->isEnabled();}));
  EXPECT_EQ(control->starts, 0); widget<QPushButton>("stopMissionRecheck")->click();
  source->answer = true; QTest::qWait(150); EXPECT_EQ(control->starts, 0);
}
TEST_F(MissionGui, LateReadRepliesAfterClosingCannotStart)
{
  open(); ASSERT_TRUE(prepare()); source->answer = false;
  widget<QPushButton>("startMission")->click(); answer("missionStartPrompt", true);
  window->close(); source->answer = true;
  ASSERT_TRUE(until([&] {return window->cleanup_complete();}));
  EXPECT_EQ(control->starts, 0); EXPECT_EQ(control->cancels, 0);
}
TEST_F(MissionGui, ReadSchedulingTimeoutDoesNotRetryFlight)
{
  open(); ASSERT_TRUE(prepare()); source->recheck_rate_limited = true;
  widget<QPushButton>("startMission")->click(); answer("missionStartPrompt", true);
  ASSERT_TRUE(
    until(
      [&] {
        return widget<QLabel>("missionPreflight")->text().contains(QStringLiteral("只读复核超时"));
      }, 6500));
  EXPECT_EQ(control->starts, 0); EXPECT_EQ(source->details_calls, 1);
}
TEST_F(MissionGui, OwnedToolExitChoicesNeverCancelTaskOrStopUserTool)
{
  auto options = ui::default_tool_options(); options.rviz_path = EASYFLY_TOOL_FIXTURE;
  open(true, true, options); ASSERT_TRUE(begin());
  QProcess external; external.start(QStringLiteral(EASYFLY_TOOL_FIXTURE), QStringList{});
  ASSERT_TRUE(until([&] {return external.state() == QProcess::Running;}));
  widget<QPushButton>("startRviz")->click();
  ASSERT_TRUE(
    until(
      [&] {
        return widget<QLabel>("rvizState")->text().contains(QStringLiteral("包装进程运行中"));
      }));
  window->close(); answer("missionExitPrompt", false); EXPECT_EQ(control->stops, 0);
  window->close(); answer("missionExitPrompt", true);
  auto * prompt = widget<QMessageBox>("toolExitPrompt"); ASSERT_NE(prompt, nullptr);
  EXPECT_EQ(prompt->defaultButton(), prompt->button(QMessageBox::Cancel));
  answer("toolExitPrompt", false); EXPECT_EQ(control->stops, 0); EXPECT_EQ(control->cancels, 0);
  window->close(); ASSERT_NE(widget<QMessageBox>("missionExitPrompt"), nullptr);
  answer("missionExitPrompt", true); answer("toolExitPrompt", true);
  ASSERT_TRUE(until([&] {return window->cleanup_complete();}, 6000));
  EXPECT_EQ(control->cancels, 0); EXPECT_FALSE(control->value.intent->resolved);
  EXPECT_EQ(external.state(), QProcess::Running);
  external.terminate(); ASSERT_TRUE(external.waitForFinished(3000));
}
TEST_F(MissionGui, CancelUsesOwnedIdentityDespiteTypedIdSelectionAndStaleTelemetry)
{
  open(); ASSERT_TRUE(begin());
  widget<QLineEdit>("executionId")->setText(QString(32, 'f'));
  widget<QListWidget>("missionList")->addItem("other.yaml");
  widget<QListWidget>("missionList")->setCurrentRow(1);
  widget<QCheckBox>("managedSimulationPermit")->setChecked(false);
  source->data->clock.quality = b::Quality::ClockPaused;
  source->data->state.evidence.received_steady -= 3;
  ASSERT_TRUE(until([&] {return widget<QPushButton>("cancelOwnedMission")->isEnabled();}));
  widget<QPushButton>("cancelOwnedMission")->click();
  auto * box = widget<QMessageBox>("missionCancelPrompt"); ASSERT_NE(box, nullptr);
  EXPECT_TRUE(box->text().contains(QString(32, 'c')));
  EXPECT_FALSE(box->text().contains(QString(32, 'f')));
  answer("missionCancelPrompt", true); EXPECT_EQ(control->cancels, 1);
  EXPECT_EQ(control->canceled_execution, std::string(32, 'c'));
  EXPECT_FALSE(control->value.intent->resolved);
  EXPECT_FALSE(widget<QPushButton>("cancelOwnedMission")->isEnabled());
}
TEST_F(MissionGui, ChangedOwnedIdentityWhileCancelPromptIsOpenCannotCancelAnotherExecution)
{
  open(); ASSERT_TRUE(begin());
  ASSERT_TRUE(until([&] {return widget<QPushButton>("cancelOwnedMission")->isEnabled();}));
  widget<QPushButton>("cancelOwnedMission")->click();
  control->value.intent->execution = std::string(32, 'e');
  answer("missionCancelPrompt", true); EXPECT_EQ(control->cancels, 0);
}
TEST_F(MissionGui, PublicTerminalAndFaultRemainDistinctFromResolved)
{
  open(); ASSERT_TRUE(begin()); control->value.phase = c::Phase::DrainPending;
  ASSERT_TRUE(
    until(
      [&] {
        return widget<QLabel>("managedMissionStatus")->text().contains(QStringLiteral("排空"));
      }));
  EXPECT_FALSE(widget<QPushButton>("startMission")->isEnabled());
  control->value.phase = c::Phase::Blocked;
  ASSERT_TRUE(until([&] {return !widget<QPushButton>("cancelOwnedMission")->isEnabled();}));
  EXPECT_FALSE(control->value.intent->resolved);
}
TEST_F(MissionGui, CloseDefaultsToStayThenStopsLocallyWithoutCancelOrClearing)
{
  open(); ASSERT_TRUE(begin()); window->close();
  auto * box = widget<QMessageBox>("missionExitPrompt"); ASSERT_NE(box, nullptr);
  EXPECT_EQ(box->defaultButton(), box->button(QMessageBox::Cancel));
  EXPECT_EQ(control->stops, 0); EXPECT_EQ(control->cancels, 0);
  EXPECT_FALSE(widget<QPushButton>("cancelOwnedMission")->isEnabled());
  EXPECT_FALSE(widget<QCheckBox>("managedSimulationPermit")->isEnabled());
  answer("missionExitPrompt", false); EXPECT_FALSE(window->cleanup_complete());
  EXPECT_EQ(control->stops, 0);
  ASSERT_TRUE(until([&] {return widget<QPushButton>("cancelOwnedMission")->isEnabled();}));
  window->close(); answer("missionExitPrompt", true);
  ASSERT_TRUE(until([&] {return window->cleanup_complete();}));
  EXPECT_EQ(control->stops, 1); EXPECT_EQ(control->cancels, 0);
  EXPECT_FALSE(control->value.intent->resolved);
}
TEST_F(MissionGui, FaultWithoutDecodedIntentStillRequiresUnresolvedExitChoice)
{
  open(); control->value.phase = c::Phase::Blocked;
  ASSERT_TRUE(
    until(
      [&] {
        return widget<QLabel>("managedMissionStatus")->text().contains(QStringLiteral("故障锁"));
      }));
  window->close(); ASSERT_NE(widget<QMessageBox>("missionExitPrompt"), nullptr);
  EXPECT_EQ(control->stops, 0); answer("missionExitPrompt", false);
  EXPECT_EQ(control->stops, 0); EXPECT_EQ(control->cancels, 0); EXPECT_EQ(control->starts, 0);
}
TEST_F(MissionGui, ManagedWidgetScreenshotsUseExplicitMockWatermark)
{
  open();
  auto * watermark = new QLabel(
    QStringLiteral("隔离 mock / 构造观测；受管任务测试，无 PX4/Gazebo/实机"), window.get());
  watermark->setTextFormat(Qt::PlainText);
  watermark->setStyleSheet("color:#8a4b08; background:#fff5df; padding:6px;");
  static_cast<QVBoxLayout *>(window->centralWidget()->layout())->insertWidget(0, watermark);
  window->show_page("tasks");
  widget<QTabWidget>("taskCenterTabs")->setCurrentIndex(3);
  ASSERT_TRUE(prepare());
  const QDir output(QStringLiteral(EASYFLY_MANAGED_SCREENSHOT_DIRECTORY));
  ASSERT_TRUE(output.mkpath("."));
  auto * panel = widget<QWidget>("managedMissionPanel"); ASSERT_NE(panel, nullptr);
  auto * ancestor = panel->parentWidget();
  while (ancestor && !qobject_cast<QScrollArea *>(ancestor)) {ancestor = ancestor->parentWidget();}
  ASSERT_NE(ancestor, nullptr);
  auto * scroll = qobject_cast<QScrollArea *>(ancestor);
  scroll->ensureWidgetVisible(panel); QTest::qWait(30);
  ASSERT_TRUE(window->grab().save(output.filePath("managed-preflight.png")));
  widget<QPushButton>("startMission")->click();
  auto * prompt = widget<QMessageBox>("missionStartPrompt"); ASSERT_NE(prompt, nullptr);
  prompt->setInformativeText(QStringLiteral("隔离 mock 界面截图，非飞行验收"));
  QTest::qWait(30);
  ASSERT_TRUE(prompt->grab().save(output.filePath("managed-start-confirmation.png")));
  answer("missionStartPrompt", true);
  ASSERT_TRUE(until([&] {return control->starts == 1;}));
  QTest::qWait(120); scroll->ensureWidgetVisible(panel); QTest::qWait(30);
  ASSERT_TRUE(window->grab().save(output.filePath("managed-running.png")));
  window->resize(1024, 720); scroll->ensureWidgetVisible(panel); QTest::qWait(30);
  ASSERT_TRUE(window->grab().save(output.filePath("managed-small-window.png")));
}
TEST_F(MissionGui, StartupFlagsRejectImplicitEphemeralAndSystemTimeControl)
{
  const std::vector<QStringList> invalid{
    {"--enable-mission-control"},
    {"--mission-journal", directory.path()},
    {"--enable-mission-control", "--mission-journal", "relative"},
    {"--enable-mission-control", "--mission-journal", directory.path() + "/"},
    {"--enable-mission-control", "--mission-journal", directory.path(), "--system-time"}
  };
  for (const auto & arguments : invalid) {
    QProcess child; child.start(QStringLiteral(EASYFLY_DESKTOP_BINARY), arguments);
    ASSERT_TRUE(child.waitForFinished(3000)); EXPECT_EQ(child.exitCode(), 2);
  }
  QProcess child;
  child.start(
    QStringLiteral(EASYFLY_DESKTOP_BINARY),
    {"--enable-mission-control", "--mission-journal", directory.path() + "/journal",
      "--quit-after-ms", "200"});
  ASSERT_TRUE(child.waitForFinished(5000)); EXPECT_EQ(child.exitCode(), 0);
  EXPECT_FALSE(QFile::exists(directory.path() + "/journal/intent.json"));
}
TEST_F(MissionGui, SystemTimeAndNonLocalhostNeverPermitNewStart)
{
  open(true, false);
  widget<QCheckBox>("managedSimulationPermit")->setChecked(true); QTest::qWait(120);
  EXPECT_TRUE(widget<QLabel>("missionPreflight")->text().contains("ROS_LOCALHOST_ONLY"));
  EXPECT_FALSE(widget<QPushButton>("startMission")->isEnabled());
  ASSERT_TRUE(until([&] {window->close(); return window->cleanup_complete();})); window.reset();
  source = std::make_shared<FakeState>(); control = std::make_shared<ControlData>(); open();
  const QByteArray previous = qgetenv("ROS_LOCALHOST_ONLY");
  qputenv("ROS_LOCALHOST_ONLY", "0");
  widget<QCheckBox>("managedSimulationPermit")->setChecked(true); QTest::qWait(120);
  EXPECT_TRUE(widget<QLabel>("missionPreflight")->text().contains("ROS_LOCALHOST_ONLY"));
  EXPECT_FALSE(widget<QPushButton>("startMission")->isEnabled());
  qputenv("ROS_LOCALHOST_ONLY", previous); EXPECT_EQ(control->starts, 0);
}
TEST_F(MissionGui, UnsafeJournalStartupExitsWithoutPromptOrChangingExistingBytes)
{
  const auto unsafe = directory.path() + "/unsafe";
  ASSERT_TRUE(QDir().mkdir(unsafe));
  QFile record(unsafe + "/intent.json"); ASSERT_TRUE(record.open(QIODevice::WriteOnly));
  const QByteArray original("preserve existing unknown intent\n");
  ASSERT_EQ(record.write(original), original.size()); record.close();
  ASSERT_EQ(chmod(unsafe.toUtf8().constData(), 0755), 0);
  for (const QStringList arguments : {
    QStringList{"--enable-mission-control", "--mission-journal", unsafe},
    QStringList{"--enable-mission-control", "--mission-journal", unsafe,
      "--quit-after-ms", "200"}})
  {
    QProcess child; child.start(QStringLiteral(EASYFLY_DESKTOP_BINARY), arguments);
    ASSERT_TRUE(child.waitForFinished(4000)); EXPECT_EQ(child.exitStatus(), QProcess::NormalExit);
    EXPECT_EQ(child.exitCode(), 2);
    EXPECT_TRUE(child.readAllStandardError().contains("mission journal startup failed"));
    EXPECT_FALSE(QFile::exists(unsafe + "/lock"));
    ASSERT_TRUE(record.open(QIODevice::ReadOnly)); EXPECT_EQ(record.readAll(), original);
    record.close();
    struct stat info {}; ASSERT_EQ(stat(unsafe.toUtf8().constData(), &info), 0);
    EXPECT_EQ(info.st_mode & 0777, 0755);
  }
}
TEST_F(MissionGui, OccupiedAndLinkedJournalFailBeforeOpeningWindow)
{
  const auto owned = directory.path() + "/owned";
  c::MissionControl owner({owned.toStdString(), 2.0});
  const auto linked = directory.path() + "/linked";
  ASSERT_EQ(symlink(owned.toUtf8().constData(), linked.toUtf8().constData()), 0);
  for (const auto & path : {owned, linked}) {
    QProcess child;
    child.start(
      QStringLiteral(EASYFLY_DESKTOP_BINARY),
      {"--enable-mission-control", "--mission-journal", path});
    ASSERT_TRUE(child.waitForFinished(4000)); EXPECT_EQ(child.exitCode(), 2);
    EXPECT_TRUE(child.readAllStandardError().contains("mission journal startup failed"));
  }
  EXPECT_FALSE(QFile::exists(owned + "/intent.json")); owner.stop();
}
