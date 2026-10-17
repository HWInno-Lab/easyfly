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
#include <QCheckBox>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QProcess>
#include <QPushButton>
#include <QTemporaryDir>
#include <QTimer>
#include <QRegularExpression>
#include <QCryptographicHash>
#include <atomic>
#include <cstdlib>
#include <fstream>
#include <mutex>
#include <set>
#include <thread>
#include <vector>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <easyfly_msgs/action/execute_mission.hpp>
#include <easyfly_msgs/srv/get_action_ownership.hpp>
#include <easyfly_msgs/srv/manage_reviewed_mission.hpp>
#include <easyfly_core/ownership_registry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include "easyfly_desktop/playback.hpp"
// Platform integration fixture; kept outside the desktop's minimal dependency closure.
#include "support.hpp"
using namespace desktop_test;  // NOLINT(build/namespaces)
using Action = easyfly_msgs::action::ExecuteMission;
using Handle = rclcpp_action::ServerGoalHandle<Action>;
using Ownership = easyfly_msgs::srv::GetActionOwnership;
using Reviewed = easyfly_msgs::srv::ManageReviewedMission;
class ManagedStack : public ::testing::Test
{
protected:
  QTemporaryDir directory;
  QProcess process;
  std::shared_ptr<FakeState> observation;
  std::unique_ptr<ui::MainWindow> window;
  std::shared_ptr<rclcpp::Context> context;
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  rclcpp_action::Server<Action>::SharedPtr backend;
  rclcpp::Service<Ownership>::SharedPtr ownership;
  rclcpp::TimerBase::SharedPtr timer;
  std::thread thread;
  std::mutex mutex;
  std::shared_ptr<Handle> handle;
  std::set<std::string> terminal_ids;
  std::atomic<int> goals{0}, cancels{0};
  std::atomic<bool> released{false}, allow_admission{true};
  std::string accepted_bytes, canceled_id;
  std::string accepted_file;
  rclcpp::Service<Reviewed>::SharedPtr reviewed_tap;
  rclcpp::Client<Reviewed>::SharedPtr reviewed_forward;
  std::vector<Reviewed::Request> reviewed_requests;
  virtual bool tap_reviewed() const {return false;}
  virtual QStringList extra_service_arguments() const {return {};}
  virtual bool cancel_terminal_allowed() const {return true;}
  virtual void reviewed_reply(
    const std::shared_ptr<rclcpp::Service<Reviewed>> & service,
    const std::shared_ptr<rmw_request_id_t> & header, const Reviewed::Request &,
    const Reviewed::Response::SharedPtr & response)
  {service->send_response(*header, *response);}

  void SetUp() override
  {
    application(); ASSERT_TRUE(directory.isValid());
    ASSERT_STREQ(std::getenv("ROS_DOMAIN_ID"), "115");
    context = std::make_shared<rclcpp::Context>(); context->init(0, nullptr);
    rclcpp::NodeOptions options; options.context(context).use_global_arguments(false);
    node = std::make_shared<rclcpp::Node>("managed_stack_mock_actions", options);
    if (tap_reviewed()) {
      // Test-only transparent asynchronous relay. Production service still handles each request.
      reviewed_forward = node->create_client<Reviewed>("/c03/mission/reviewed");
      reviewed_tap = node->create_service<Reviewed>(
        "/platform/mission/reviewed",
        [this](const std::shared_ptr<rclcpp::Service<Reviewed>> & service,
        const std::shared_ptr<rmw_request_id_t> & header, Reviewed::Request::SharedPtr request) {
          {std::lock_guard<std::mutex> lock(mutex); reviewed_requests.push_back(*request);}
          reviewed_forward->async_send_request(
            request,
            [this, service, header, request](rclcpp::Client<Reviewed>::SharedFuture reply) {
              reviewed_reply(service, header, *request, reply.get());
            });
        });
    }
    backend = rclcpp_action::create_server<Action>(
      node, "/execute_mission",
      [this](const auto &, auto) {
        ++goals;
        while (!allow_admission) {std::this_thread::sleep_for(std::chrono::milliseconds(10));}
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](auto goal) {
        ++cancels; std::lock_guard<std::mutex> lock(mutex);
        canceled_id = easyfly_core::goalIdentity(goal->get_goal_id());
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](auto goal) {
        std::lock_guard<std::mutex> lock(mutex); handle = goal;
        accepted_file = goal->get_goal()->mission_file;
        std::ifstream file(goal->get_goal()->mission_file, std::ios::binary);
        accepted_bytes = std::string((std::istreambuf_iterator<char>(file)), {});
      });
    ownership = node->create_service<Ownership>(
      "/mission/ownership",
      [this](Ownership::Request::SharedPtr request, Ownership::Response::SharedPtr response) {
        response->goal_id = request->goal_id; std::lock_guard<std::mutex> lock(mutex);
        response->state = released && terminal_ids.count(request->goal_id) ?
        Ownership::Response::RELEASED : Ownership::Response::OWNED;
      });
    timer = node->create_wall_timer(
      std::chrono::milliseconds(20), [this] {
        std::lock_guard<std::mutex> lock(mutex);
        if (handle && handle->is_canceling() && cancel_terminal_allowed()) {
          terminal_ids.insert(easyfly_core::goalIdentity(handle->get_goal_id()));
          auto result = std::make_shared<Action::Result>();
          result->success = false; result->message = "mock canceled; no flight";
          handle->canceled(result); handle.reset();
        }
      });
    rclcpp::ExecutorOptions eo; eo.context = context;
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(eo);
    executor->add_node(node); thread = std::thread([this] {executor->spin();});
    fresh_observation();
    QFile catalog(directory.path() + "/demo.yaml");
    ASSERT_TRUE(catalog.open(QIODevice::WriteOnly));
    const auto body = QByteArray::fromStdString(observation->details_content);
    ASSERT_EQ(catalog.write(body), body.size()); catalog.close();
    process.setProcessChannelMode(QProcess::MergedChannels);
    QStringList arguments{"--ros-args", "-p", "mission_directory:=" + directory.path()};
    if (tap_reviewed()) {
      arguments << "-r" << "/platform/mission/reviewed:=/c03/mission/reviewed";
    }
    arguments.append(extra_service_arguments());
    process.start(
      QString::fromStdString(ament_index_cpp::get_package_prefix("easyfly_mission_srvs")) +
      "/lib/easyfly_mission_srvs/mission_services",
      arguments);
    ASSERT_TRUE(until([&] {return process.state() == QProcess::Running;}));
  }
  void fresh_observation()
  {
    observation = std::make_shared<FakeState>();
    observation->data->state.attitude_valid = observation->data->state.sdk.value.attitude_valid =
      true;
    auto caps = std::make_shared<easyfly_msgs::srv::GetCapabilities::Response>();
    caps->names = {"versioned_yaml"}; caps->implemented = {true}; caps->ready = {true};
    observation->data->capabilities[b::Key::MissionCapabilities] = {caps, evidence()};
    for (auto key : {b::Key::Profile, b::Key::UseSimTime}) {
      auto p = std::make_shared<easyfly_msgs::srv::QueryParams::Response>(); p->success = true;
      p->current_value = key == b::Key::Profile ? "\"navigation_dev\"" : "true";
      observation->data->parameters[key] = {p, evidence()};
      observation->parameter_values[key] = p->current_value;
    }
  }
  void renew_evidence()
  {
    observation->data->state.evidence = evidence();
    for (auto & value : observation->data->parameters) {
      if (value.second.value) {value.second.evidence = evidence();}
    }
    for (auto & value : observation->data->capabilities) {value.second.evidence = evidence();}
  }
  void open(const ui::ToolOptions & tools = ui::ToolOptions{}, bool media = false)
  {
    window = std::make_unique<ui::MainWindow>(
      b::BridgeConfig{}, factory(observation), nullptr,
      tools, ui::MissionOptions{true, (directory.path() + "/journal").toStdString()},
      ui::control_factory(), ui::ParameterOptions{}, ui::parameter_factory(), ui::ImportOptions{},
      ui::import_factory(), ui::RvizOptions{}, ui::RecordingOptions{media});
    window->show();
  }
  template<typename T> T * widget(const char * name) {return window->findChild<T *>(name);}
  bool wait_status(const QString & value)
  {
    return until(
      [&] {
        return widget<QLabel>("managedMissionStatus")->text().contains(value);
      }, 8000);
  }
  bool start()
  {
    if (!until(
        [&] {
          renew_evidence();
          return !widget<QLabel>("missionPreflight")->text().contains(QStringLiteral("实例未取得"));
        }, 8000)) {return false;}
    widget<QPushButton>("listMissions")->click();
    if (!until([&] {return widget<QListWidget>("missionList")->count() == 1;})) {return false;}
    widget<QListWidget>("missionList")->setCurrentRow(0);
    widget<QPushButton>("inspectMission")->click();
    if (!until(
        [&] {
          renew_evidence();
          return widget<QLabel>("missionDetailsSummary")->text().contains("SHA-256");
        })) {return false;}
    widget<QCheckBox>("managedSimulationPermit")->setChecked(true);
    if (!until(
        [&] {
          renew_evidence(); return widget<QPushButton>("startMission")->isEnabled();
        }, 8000)) {return false;}
    widget<QPushButton>("startMission")->click();
    const auto prompt = widget<QMessageBox>("missionStartPrompt");
    if (!prompt) {return false;}
    prompt->done(QMessageBox::Yes);
    return until([&] {return goals == 1;}, 8000);
  }
  bool close()
  {
    if (!window) {return true;}
    window->close();
    if (auto * prompt = widget<QMessageBox>("missionExitPrompt")) {prompt->done(QMessageBox::Yes);}
    if (auto * prompt = widget<QMessageBox>("toolExitPrompt")) {prompt->done(QMessageBox::Yes);}
    return until([&] {return window->cleanup_complete();}, 8000);
  }
  void TearDown() override
  {
    allow_admission = true;
    if (window) {EXPECT_TRUE(close()); window.reset();}
    // Finish only our mock backend after the UI assertions, not as a UI close action.
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (handle && handle->is_active() && !handle->is_canceling()) {
        terminal_ids.insert(easyfly_core::goalIdentity(handle->get_goal_id()));
        auto result = std::make_shared<Action::Result>();
        result->success = true; result->message = "fixture teardown only";
        handle->succeed(result); handle.reset();
      }
    }
    released = true;
    std::string private_file;
    {std::lock_guard<std::mutex> lock(mutex); private_file = accepted_file;}
    if (!private_file.empty() && process.state() == QProcess::Running) {
      EXPECT_TRUE(until([&] {return !QFile::exists(QString::fromStdString(private_file));}, 4000));
    }
    if (process.state() != QProcess::NotRunning) {
      process.terminate();
      if (!process.waitForFinished(3000)) {process.kill(); process.waitForFinished(3000);}
    }
    if (context && context->is_valid()) {context->shutdown("mock complete");}
    if (thread.joinable()) {thread.join();}
    timer.reset(); backend.reset(); ownership.reset(); handle.reset();
    reviewed_tap.reset(); reviewed_forward.reset();
    executor.reset(); node.reset(); context.reset();
  }
};
TEST_F(ManagedStack, QtControllerServiceExactCancelDrainCloseAndRestore)
{
  open(); ASSERT_TRUE(start()); ASSERT_TRUE(wait_status(QStringLiteral("执行中")));
  {std::lock_guard<std::mutex> lock(mutex); EXPECT_EQ(accepted_bytes, observation->details_content);
  }
  widget<QPushButton>("cancelOwnedMission")->click();
  auto * prompt = widget<QMessageBox>("missionCancelPrompt"); ASSERT_NE(prompt, nullptr);
  prompt->done(QMessageBox::Yes);
  ASSERT_TRUE(wait_status(QStringLiteral("仍等待所有权排空")));
  EXPECT_EQ(goals, 1); EXPECT_EQ(cancels, 1);
  {std::lock_guard<std::mutex> lock(mutex);
    EXPECT_TRUE(
      widget<QLabel>("managedMissionStatus")->text().contains(
        QString::fromStdString(
          canceled_id)));}
  ASSERT_TRUE(close()); window.reset(); EXPECT_EQ(process.state(), QProcess::Running);
  fresh_observation(); open(); ASSERT_TRUE(wait_status(QStringLiteral("仍等待所有权排空")));
  EXPECT_EQ(goals, 1); EXPECT_EQ(cancels, 1);
  EXPECT_FALSE(widget<QCheckBox>("managedSimulationPermit")->isChecked());
  EXPECT_FALSE(widget<QPushButton>("startMission")->isEnabled());
  released = true; ASSERT_TRUE(wait_status(QStringLiteral("该意图已确认解决（不等于任务成功）")));
  EXPECT_TRUE(QFile::exists(directory.path() + "/journal/intent.json"));
  EXPECT_EQ(goals, 1); EXPECT_EQ(cancels, 1);
}
TEST_F(ManagedStack, ClosingBeforeAdmissionReplyRestoresOriginalRequestWithoutResubmission)
{
  allow_admission = false; open(); ASSERT_TRUE(start());
  ASSERT_TRUE(close()); window.reset(); EXPECT_EQ(cancels, 0);
  allow_admission = true;
  fresh_observation(); open(); ASSERT_TRUE(wait_status(QStringLiteral("执行中")));
  EXPECT_EQ(goals, 1); EXPECT_EQ(cancels, 0);
  ASSERT_TRUE(close()); EXPECT_EQ(goals, 1); EXPECT_EQ(cancels, 0);
}
TEST_F(ManagedStack, CompletedJournalReopenAndPollingNeverCreateAnotherIntent)
{
  open(); ASSERT_TRUE(start()); ASSERT_TRUE(wait_status(QStringLiteral("执行中")));
  {
    std::lock_guard<std::mutex> lock(mutex);
    ASSERT_TRUE(handle && handle->is_active());
    terminal_ids.insert(easyfly_core::goalIdentity(handle->get_goal_id()));
    auto result = std::make_shared<Action::Result>(); result->success = true;
    result->message = "explicit mock completion; no flight";
    handle->succeed(result); handle.reset();
  }
  released = true; ASSERT_TRUE(wait_status(QStringLiteral("该意图已确认解决（不等于任务成功）")));
  const auto journal_path = directory.path() + "/journal/intent.json";
  QFile before_file(journal_path); ASSERT_TRUE(before_file.open(QIODevice::ReadOnly));
  const auto before = before_file.readAll(); before_file.close();
  ASSERT_TRUE(QJsonDocument::fromJson(before).object().value("resolved").toBool());
  ASSERT_TRUE(close()); window.reset();
  fresh_observation(); open();
  ASSERT_TRUE(wait_status(QStringLiteral("该意图已确认解决（不等于任务成功）")));
  EXPECT_FALSE(widget<QCheckBox>("managedSimulationPermit")->isChecked());
  // Even enabling the gate and renewing observations is not a start confirmation.
  widget<QCheckBox>("managedSimulationPermit")->setChecked(true);
  for (int i = 0; i < 30; ++i) {
    renew_evidence(); QTest::qWait(100);
  }
  EXPECT_EQ(goals, 1); EXPECT_EQ(cancels, 0);
  EXPECT_EQ(widget<QMessageBox>("missionStartPrompt"), nullptr);
  QFile after_file(journal_path); ASSERT_TRUE(after_file.open(QIODevice::ReadOnly));
  EXPECT_EQ(after_file.readAll().toStdString(), before.toStdString());
  const auto intent = QJsonDocument::fromJson(before).object();
  const auto backend_audit = process.readAll();
  EXPECT_EQ(backend_audit.count("event=start_new_request "), 1);
  EXPECT_EQ(backend_audit.count("event=write_received "), 1);
  EXPECT_TRUE(backend_audit.contains("request=" + intent.value("request").toString().toUtf8()));
  EXPECT_TRUE(backend_audit.contains("execution=" + intent.value("execution").toString().toUtf8()));
  EXPECT_TRUE(backend_audit.contains("goal=" + intent.value("goal").toString().toUtf8()));
}
// Explicit GUI opt-in. Real tool executables, but fabricated observation and mock Action only.
// No PX4, Gazebo, hardware, real flight goal, or remote cancellation is launched by this test.
TEST_F(ManagedStack, DISABLED_RealToolsUnresolvedExitAndLookupOnlyReopen)
{
  auto tools = ui::default_tool_options();
  tools.qgc_path = qEnvironmentVariable("EASYFLY_QGC_TEST_IMAGE");
  ASSERT_FALSE(tools.rviz_path.isEmpty()); ASSERT_FALSE(tools.qgc_path.isEmpty());
  open(tools); ASSERT_TRUE(start()); ASSERT_TRUE(wait_status(QStringLiteral("执行中")));
  widget<QPushButton>("startRviz")->click(); widget<QPushButton>("startQgc")->click();
  ASSERT_TRUE(
    until(
      [&] {
        return widget<QLabel>("rvizState")->text().contains(QStringLiteral("包装进程运行中")) &&
        widget<QLabel>("qgcState")->text().contains(QStringLiteral("包装进程运行中"));
      }, 15000));
  ASSERT_TRUE(
    until(
      [&] {
        const auto log = widget<QPlainTextEdit>("toolLog")->toPlainText();
        return log.contains("OpenGl") && log.contains("/tmp/easyfly-qgc-config/");
      }, 15000));
  QTest::qWait(3500);
  EXPECT_TRUE(widget<QLabel>("rvizState")->text().contains(QStringLiteral("包装进程运行中")));
  EXPECT_TRUE(widget<QLabel>("qgcState")->text().contains(QStringLiteral("包装进程运行中")));
  const auto journal_path = directory.path() + "/journal/intent.json";
  QFile journal(journal_path); ASSERT_TRUE(journal.open(QIODevice::ReadOnly));
  const auto original = journal.readAll(); journal.close();
  ASSERT_TRUE(original.contains("\"resolved\":false"));
  window->close(); auto * prompt = widget<QMessageBox>("missionExitPrompt");
  ASSERT_NE(prompt, nullptr);
  EXPECT_EQ(prompt->defaultButton(), prompt->button(QMessageBox::Cancel));
  prompt->done(QMessageBox::Cancel); QCoreApplication::processEvents();
  EXPECT_FALSE(window->cleanup_complete()); EXPECT_EQ(cancels, 0);
  window->close(); prompt = widget<QMessageBox>("missionExitPrompt"); ASSERT_NE(prompt, nullptr);
  prompt->done(QMessageBox::Yes); QCoreApplication::processEvents();
  prompt = widget<QMessageBox>("toolExitPrompt"); ASSERT_NE(prompt, nullptr);
  EXPECT_EQ(prompt->defaultButton(), prompt->button(QMessageBox::Cancel));
  prompt->done(QMessageBox::Cancel); QCoreApplication::processEvents();
  EXPECT_FALSE(window->cleanup_complete()); EXPECT_EQ(cancels, 0);
  window->close(); prompt = widget<QMessageBox>("missionExitPrompt"); ASSERT_NE(prompt, nullptr);
  prompt->done(QMessageBox::Yes); QCoreApplication::processEvents();
  prompt = widget<QMessageBox>("toolExitPrompt"); ASSERT_NE(prompt, nullptr);
  prompt->done(QMessageBox::Yes);
  ASSERT_TRUE(until([&] {return window->cleanup_complete();}, 8000)); window.reset();
  EXPECT_EQ(process.state(), QProcess::Running); EXPECT_EQ(goals, 1); EXPECT_EQ(cancels, 0);
  ASSERT_TRUE(journal.open(QIODevice::ReadOnly)); EXPECT_EQ(journal.readAll(), original);
  journal.close();
  {std::lock_guard<std::mutex> lock(mutex); ASSERT_TRUE(handle); EXPECT_TRUE(handle->is_active());}
  fresh_observation(); open(tools); ASSERT_TRUE(wait_status(QStringLiteral("执行中")));
  EXPECT_EQ(goals, 1); EXPECT_EQ(cancels, 0);
  EXPECT_FALSE(widget<QCheckBox>("managedSimulationPermit")->isChecked());
  EXPECT_TRUE(widget<QLabel>("rvizState")->text().contains(QStringLiteral("未启动")));
  EXPECT_TRUE(widget<QLabel>("qgcState")->text().contains(QStringLiteral("未启动")));
  ASSERT_TRUE(close()); EXPECT_EQ(goals, 1); EXPECT_EQ(cancels, 0);
}

// Real Qt, controller, ROS relay and production mission service. Pixels/state/action are MOCK.
// Only our local tool wrappers are stopped; the mock goal is completed later by fixture teardown.
class ManagedMediaStack : public ManagedStack
{
protected:
  QTimer images;
  QProcess unrelated;
  double began{0};
  bool tap_reviewed() const override {return true;}
  void SetUp() override
  {
    if (!ui::RecordingWorker::available()) {GTEST_SKIP() << "Requires opt-in media build";}
    ManagedStack::SetUp(); if (::testing::Test::HasFatalFailure()) {return;}
    qunsetenv("EASYFLY_TOOL_TEST_MODE"); began = b::detail::steady_seconds();
    QObject::connect(&images, &QTimer::timeout, &images, [this] {image();});
    image(); images.start(100);
  }
  void TearDown() override
  {
    images.stop();
    if (unrelated.state() != QProcess::NotRunning) {
      unrelated.terminate(); EXPECT_TRUE(unrelated.waitForFinished(3000));
    }
    ManagedStack::TearDown();
  }
  void image()
  {
    if (!observation) {return;}
    // Camera cadence must not overwrite correlated parameter/capability request IDs.
    observation->data->state.evidence = evidence();
    auto value = std::make_shared<sensor_msgs::msg::Image>();
    const auto ns = static_cast<int64_t>((10 + b::detail::steady_seconds() - began) * 1e9);
    value->header.stamp.sec = ns / 1000000000LL;
    value->header.stamp.nanosec = ns % 1000000000LL;
    value->header.frame_id = "c03_synthetic_pixels_not_camera_or_aircraft";
    value->width = 96; value->height = 64; value->step = 288; value->encoding = "rgb8";
    value->data.assign(96 * 64 * 3, static_cast<uint8_t>(ns / 100000000 % 255));
    observation->data->clock.now_ns = ns; observation->data->images[0] = {value, evidence()};
  }
  static QByteArray bytes(const QString & path)
  {
    QFile file(path); return file.open(QIODevice::ReadOnly) ? file.readAll() : QByteArray{};
  }
  uint64_t frames()
  {
    const auto match = QRegularExpression(QStringLiteral("已写 (\\d+) 帧")).match(
      widget<QLabel>("recordingStatus")->text());
    return match.hasMatch() ? match.captured(1).toULongLong() : 0;
  }
  static std::set<qint64> tool_pids()
  {
    std::set<qint64> result;
    for (const auto & entry : QDir("/proc").entryList(QDir::Dirs | QDir::NoDotAndDotDot)) {
      bool ok = false; const auto pid = entry.toLongLong(&ok); if (!ok) {continue;}
      const auto name = QFileInfo(QFileInfo("/proc/" + entry + "/exe").symLinkTarget()).fileName();
      if (name == "tool_fixture" || name == "rviz2" || name.startsWith("QGroundControl") ||
        name.contains("easyfly-qgc.AppImage") || name.contains("c03 offline stub.AppImage"))
      {result.insert(pid);}
    }
    return result;
  }
  void exercise(bool pending, bool real_tools)
  {
    auto tools = ui::default_tool_options();
    if (real_tools) {
      tools.qgc_path = qEnvironmentVariable("EASYFLY_QGC_TEST_IMAGE");
      ASSERT_FALSE(tools.rviz_path.isEmpty()); ASSERT_FALSE(tools.qgc_path.isEmpty());
    } else {
      tools.rviz_path = EASYFLY_TOOL_FIXTURE;
      tools.qgc_path = directory.path() + "/c03 offline stub.AppImage";
      ASSERT_TRUE(QFile::copy(EASYFLY_TOOL_FIXTURE, tools.qgc_path));
      QFile stub(tools.qgc_path); ASSERT_TRUE(stub.open(QIODevice::ReadWrite));
      ASSERT_TRUE(stub.seek(8)); ASSERT_EQ(stub.write(QByteArray("AI\x02", 3)), 3); stub.close();
    }
    unrelated.start(EASYFLY_TOOL_FIXTURE, QStringList{});
    ASSERT_TRUE(until([&] {return unrelated.state() == QProcess::Running;}));
    const auto original_tool_pids = tool_pids();
    allow_admission = !pending; open(tools, true); ASSERT_TRUE(start());
    window->setWindowTitle(QStringLiteral("C03 测试：mock任务/观测/图像，不连接飞机"));
    if (!pending) {ASSERT_TRUE(wait_status(QStringLiteral("执行中")));}
    const auto records = directory.path() + "/records";
    auto * nav = widget<QListWidget>("navigation"); nav->setCurrentRow(nav->count() - 1);
    widget<QLineEdit>("recordingDirectory")->setText(records);
    ASSERT_TRUE(until([&] {return widget<QPushButton>("recordingStart")->isEnabled();}));
    widget<QPushButton>("recordingStart")->click();
    auto * record_prompt = widget<QMessageBox>("recordingConfirmation");
    ASSERT_NE(record_prompt, nullptr); record_prompt->done(QMessageBox::Ok);
    ASSERT_TRUE(until([&] {return frames() >= 2;}));
    widget<QPushButton>("startRviz")->click(); widget<QPushButton>("startQgc")->click();
    const auto tools_running = [&] {
        return widget<QLabel>("rvizState")->text().contains(QStringLiteral("包装进程运行中")) &&
               widget<QLabel>("qgcState")->text().contains(QStringLiteral("包装进程运行中"));
      };
    ASSERT_TRUE(until(tools_running, 15000));
    if (real_tools) {
      ASSERT_TRUE(
        until(
          [&] {
            const auto log = widget<QPlainTextEdit>("toolLog")->toPlainText();
            return log.contains("OpenGl") && log.contains("/tmp/easyfly-qgc-config/");
          }, 15000));
    } else {
      ASSERT_TRUE(
        until(
          [&] {
            return widget<QPlainTextEdit>("toolLog")->toPlainText().count("fixture started") == 2;
          }));
    }
    auto owned_tool_pids = tool_pids();
    for (const auto pid : original_tool_pids) {owned_tool_pids.erase(pid);}
    ASSERT_GE(owned_tool_pids.size(), 2u);
    const auto journal = directory.path() + "/journal/intent.json";
    const auto original = bytes(journal); ASSERT_TRUE(original.contains("\"resolved\":false"));
    const auto identity = QJsonDocument::fromJson(original).object();
    if (pending) {
      ASSERT_TRUE(identity["execution"].toString().isEmpty());
      ASSERT_TRUE(identity["goal"].toString().isEmpty());
    } else {
      ASSERT_EQ(identity["execution"].toString().size(), 32);
      ASSERT_EQ(identity["goal"].toString().size(), 32);
    }
    uint64_t previous = frames();
    window->close(); auto * prompt = widget<QMessageBox>("missionExitPrompt");
    ASSERT_NE(prompt, nullptr);
    EXPECT_EQ(prompt->defaultButton(), prompt->button(QMessageBox::Cancel));
    prompt->done(QMessageBox::Cancel);
    ASSERT_TRUE(until([&] {return frames() > previous;}));
    EXPECT_TRUE(window->isVisible()); EXPECT_TRUE(tools_running());
    EXPECT_EQ(observation->stop_calls, 0); EXPECT_EQ(cancels, 0);
    previous = frames(); window->close(); prompt = widget<QMessageBox>("missionExitPrompt");
    ASSERT_NE(prompt, nullptr); prompt->done(QMessageBox::Yes);
    prompt = widget<QMessageBox>("toolExitPrompt"); ASSERT_NE(prompt, nullptr);
    EXPECT_EQ(prompt->defaultButton(), prompt->button(QMessageBox::Cancel));
    prompt->done(QMessageBox::Cancel);
    ASSERT_TRUE(until([&] {return frames() > previous;}));
    EXPECT_FALSE(window->cleanup_complete()); EXPECT_TRUE(tools_running());
    EXPECT_EQ(observation->stop_calls, 0); EXPECT_EQ(cancels, 0);
    // In the pending case, do not exceed the unchanged service's 5s admission timeout.
    ASSERT_TRUE(close());
    EXPECT_TRUE(
      until(
        [&] {
          const auto live = tool_pids();
          return std::none_of(
            owned_tool_pids.begin(), owned_tool_pids.end(),
            [&](auto pid) {return live.count(pid);});
        }, 4000));
    EXPECT_EQ(unrelated.state(), QProcess::Running); EXPECT_EQ(process.state(), QProcess::Running);
    EXPECT_EQ(goals, 1); EXPECT_EQ(cancels, 0); EXPECT_EQ(bytes(journal), original);
    auto * panel = widget<ui::RecordingPanel>("recordingPage"); ASSERT_TRUE(panel->stopped());
    const auto dirs = QDir(records).entryList(QDir::Dirs | QDir::NoDotAndDotDot);
    ASSERT_EQ(dirs.size(), 1); const auto path = records + "/" + dirs[0];
    const auto manifest_bytes = bytes(path + "/manifest.json");
    const auto manifest = QJsonDocument::fromJson(manifest_bytes).object();
    EXPECT_EQ(manifest["status"].toString(), "verified");
    EXPECT_EQ(manifest["reason"].toString(), "window_closed");
    EXPECT_GT(manifest["frames"].toString().toULongLong(), 2u);
    const auto events = bytes(path + "/events.jsonl");
    EXPECT_TRUE(events.contains(identity["request"].toString().toUtf8()));
    if (!pending) {
      EXPECT_TRUE(events.contains(identity["execution"].toString().toUtf8()));
      EXPECT_TRUE(events.contains(identity["goal"].toString().toUtf8()));
    }
    window.reset(); std::size_t restored_index;
    {std::lock_guard<std::mutex> lock(mutex); restored_index = reviewed_requests.size();}
    allow_admission = true; fresh_observation();
    observation->data->observation_session_id = 102; image(); open(tools, true);
    ASSERT_TRUE(wait_status(QStringLiteral("执行中")));
    EXPECT_FALSE(widget<QCheckBox>("managedSimulationPermit")->isChecked());
    EXPECT_FALSE(widget<QPushButton>("startMission")->isEnabled());
    EXPECT_TRUE(widget<QLabel>("rvizState")->text().contains(QStringLiteral("未启动")));
    EXPECT_TRUE(widget<QLabel>("qgcState")->text().contains(QStringLiteral("未启动")));
    EXPECT_TRUE(widget<QLabel>("recordingStatus")->text().contains(QStringLiteral("未录像")));
    EXPECT_EQ(QDir(records).entryList(QDir::Dirs | QDir::NoDotAndDotDot).size(), 1);
    EXPECT_EQ(bytes(path + "/manifest.json"), manifest_bytes);
    const auto restored_identity = QJsonDocument::fromJson(bytes(journal)).object();
    for (const auto * key : {"scope", "instance", "request", "name", "hash"}) {
      EXPECT_EQ(restored_identity[key], identity[key]);
    }
    if (!pending) {EXPECT_EQ(bytes(journal), original);}
    QJsonArray transport;
    {
      std::lock_guard<std::mutex> lock(mutex); unsigned starts = 0, cancels_seen = 0, restores = 0;
      for (std::size_t i = 0; i < reviewed_requests.size(); ++i) {
        const auto & r = reviewed_requests[i];
        starts += r.operation == "start"; cancels_seen += r.operation == "cancel";
        if (i >= restored_index) {
          ++restores; EXPECT_EQ(r.operation, "lookup");
          EXPECT_EQ(r.server_instance_id, identity["instance"].toString().toStdString());
          EXPECT_EQ(r.client_request_id, identity["request"].toString().toStdString());
        }
        transport.append(
          QJsonObject{{"operation", QString::fromStdString(r.operation)},
            {"instance", QString::fromStdString(r.server_instance_id)},
            {"request", QString::fromStdString(
                r.client_request_id)}, {"after_reopen", i >= restored_index}});
      }
      EXPECT_EQ(starts, 1u); EXPECT_EQ(cancels_seen, 0u); EXPECT_GT(restores, 0u);
      ASSERT_TRUE(handle && handle->is_active());
    }
    ASSERT_TRUE(close()); window.reset(); EXPECT_EQ(goals, 1); EXPECT_EQ(cancels, 0);
    EXPECT_EQ(unrelated.state(), QProcess::Running);
    const auto audit = process.readAll();
    EXPECT_EQ(audit.count("event=write_received "), 1);
    EXPECT_EQ(audit.count("event=start_new_request "), 1);
    EXPECT_EQ(audit.count("event=start_existing_request "), 0);
    const auto evidence_root = qEnvironmentVariable("EASYFLY_C03_EVIDENCE_ROOT");
    if (!evidence_root.isEmpty()) {
      QJsonArray initial_pids, managed_pids;
      for (const auto pid : original_tool_pids) {initial_pids.append(QString::number(pid));}
      for (const auto pid : owned_tool_pids) {managed_pids.append(QString::number(pid));}
      directory.setAutoRemove(false); const auto name = QString::fromLatin1(
        ::testing::UnitTest::GetInstance()->current_test_info()->name());
      QFile report(evidence_root + "/" + name + ".json");
      ASSERT_TRUE(report.open(QIODevice::WriteOnly | QIODevice::NewOnly));
      report.write(
        QJsonDocument(
          QJsonObject{{"kind", "C03 MOCK observations/action/pixels; no flight"},
            {"real_tools", real_tools}, {"pending_admission", pending},
            {"artifact_directory", directory.path()},
            {"transport", transport}, {"original_journal", identity},
            {"restored_journal", restored_identity},
            {"record", manifest}, {"action_goals", goals.load()},
            {"action_cancels", cancels.load()},
            {"unrelated_process_survived", unrelated.state() == QProcess::Running},
            {"baseline_tool_pids", initial_pids}, {"managed_tool_pids", managed_pids},
            {"assertions_passed_before_teardown", !::testing::Test::HasFailure()}}).toJson());
      QFile log(evidence_root + "/" + name + ".backend.log");
      ASSERT_TRUE(log.open(QIODevice::WriteOnly | QIODevice::NewOnly)); log.write(audit);
      QFile raw(evidence_root + "/" + name + ".original-intent.json");
      ASSERT_TRUE(raw.open(QIODevice::WriteOnly | QIODevice::NewOnly)); raw.write(original);
    }
  }
};
TEST_F(ManagedMediaStack, RunningMediaToolsExitAndLookupOnlyReopen) {
  exercise(false, false);
}
TEST_F(ManagedMediaStack, PendingAdmissionMediaToolsExitAndLookupOnlyReopen) {
  exercise(true, false);
}
TEST_F(ManagedMediaStack, DISABLED_RealToolsMediaExitAndLookupOnlyReopen) {
  exercise(false, true);
}

#ifdef EASYFLY_ENABLE_MEDIA
// Real ROS observation/service/control/recording chain; publishers and Action are MOCK.
class TimelineStack : public ManagedStack
{
protected:
  QProcess parameters;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub;
  rclcpp::Publisher<easyfly_msgs::msg::UAVState>::SharedPtr state_pub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
  rclcpp::Publisher<easyfly_msgs::msg::MissionStatus>::SharedPtr event_pub;
  rclcpp::Service<easyfly_msgs::srv::GetCapabilities>::SharedPtr flight_caps;
  rclcpp::TimerBase::SharedPtr telemetry;
  std::atomic<bool> hold_start{true}, finish_cancel{false};
  std::atomic<int> stage{0};
  std::atomic<int64_t> simulation_ns{10000000000LL};
  std::vector<std::function<void()>> held;
  QJsonArray wire, events;
  QJsonObject manifest, identity;
  QString record_path;
  ui::Source * observed{nullptr};
  double began{0};
  bool tap_reviewed() const override {return true;}
  bool cancel_terminal_allowed() const override {return finish_cancel;}
  QStringList extra_service_arguments() const override {return {"-p", "use_sim_time:=true"};}
  void reviewed_reply(
    const std::shared_ptr<rclcpp::Service<Reviewed>> & service,
    const std::shared_ptr<rmw_request_id_t> & header, const Reviewed::Request & request,
    const Reviewed::Response::SharedPtr & reply) override
  {
    std::lock_guard<std::mutex> lock(mutex);
    wire.append(
      QJsonObject{{"operation", QString::fromStdString(request.operation)},
        {"request", QString::fromStdString(reply->client_request_id)},
        {"instance", QString::fromStdString(reply->server_instance_id)},
        {"execution", QString::fromStdString(reply->execution_id)},
        {"goal", QString::fromStdString(reply->action_goal_id)},
        {"status", QString::fromStdString(reply->status)},
        {"admission_resolved", reply->admission_resolved},
        {"admission_success", reply->admission_success},
        {"execution_pending", reply->execution_pending}, {"steady", b::detail::steady_seconds()}});
    if (request.operation == "start" && hold_start) {
      held.push_back([service, header, reply] {service->send_response(*header, *reply);});
    } else {service->send_response(*header, *reply);}
  }
  void SetUp() override
  {
    ManagedStack::SetUp(); if (::testing::Test::HasFatalFailure()) {return;}
    began = b::detail::steady_seconds();
    flight_caps = node->create_service<easyfly_msgs::srv::GetCapabilities>(
      "/platform/flight_control/capabilities",
      [this](easyfly_msgs::srv::GetCapabilities::Request::SharedPtr,
      easyfly_msgs::srv::GetCapabilities::Response::SharedPtr response) {
        response->header.stamp = rclcpp::Time(simulation_ns.load(), RCL_ROS_TIME);
        response->header.frame_id = "odom";
        response->names = {"C06_MOCK_observation_only"}; response->implemented = {true};
        response->ready = {false}; response->details = {"{\"command_uncertain\":false}"};
      });
    clock_pub = node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);
    state_pub = node->create_publisher<easyfly_msgs::msg::UAVState>("/uav_state", 1);
    image_pub = node->create_publisher<sensor_msgs::msg::Image>(
      "/camera/color/image_raw", rclcpp::QoS(1).best_effort());
    event_pub = node->create_publisher<easyfly_msgs::msg::MissionStatus>("/mission_status", 1);
    telemetry = node->create_wall_timer(
      std::chrono::milliseconds(50), [this] {
        const auto ns = static_cast<int64_t>((10 + b::detail::steady_seconds() - began) * 1e9);
        simulation_ns = ns;
        rosgraph_msgs::msg::Clock clock; clock.clock.sec = ns / 1000000000LL;
        clock.clock.nanosec = ns % 1000000000LL; clock_pub->publish(clock);
        auto state = observation->data->state.sdk.value; state.header.stamp = clock.clock;
        state_pub->publish(state);
        sensor_msgs::msg::Image image; image.header = state.header;
        image.header.frame_id = "C06_MOCK_pixels_not_camera_or_aircraft";
        image.width = 96; image.height = 64; image.step = 288; image.encoding = "rgb8";
        image.data.assign(96 * 64 * 3, static_cast<uint8_t>(ns / 100000000 % 255));
        image_pub->publish(image);
        easyfly_msgs::msg::MissionStatus global; global.header = state.header;
        global.phase_name = "C06 MOCK global stage " + std::to_string(stage.load());
        global.message = "not an execution identity; no flight"; event_pub->publish(global);
        std::vector<std::function<void()>> ready;
        {std::lock_guard<std::mutex> lock(mutex); if (!hold_start) {ready.swap(held);}}
        for (const auto & send : ready) {send();}
      });
    const auto binary = ament_index_cpp::get_package_prefix("easyfly_params") +
      "/lib/easyfly_params/parameter_manager";
    parameters.setProcessChannelMode(QProcess::MergedChannels);
    parameters.start(
      QString::fromStdString(binary), {"--ros-args", "-p", "profile:=navigation_dev",
        "-p", "use_sim_time:=true"}); ASSERT_TRUE(parameters.waitForStarted(2000));
    ui::SourceFactory source = [this](const b::BridgeConfig & config,
        std::shared_ptr<const b::UiSnapshot> history) {
        auto result = ui::bridge_factory()(config, history); observed = result.get(); return result;
      };
    b::BridgeConfig config; config.image_topics[1].clear();
    window = std::make_unique<ui::MainWindow>(
      config, source, nullptr, ui::default_tool_options(),
      ui::MissionOptions{true, (directory.path() + "/journal").toStdString()},
      ui::control_factory(),
      ui::ParameterOptions{}, ui::parameter_factory(), ui::ImportOptions{}, ui::import_factory(),
      ui::RvizOptions{}, ui::RecordingOptions{true});
    window->setWindowTitle(QStringLiteral("C06 自动化：MOCK任务/像素/状态，实际ROS链，无飞控"));
    window->show();
    ASSERT_TRUE(until([this] {return observed->snapshot()->state.sdk.fresh;}, 6000));
  }
  void TearDown() override
  {
    hold_start = false; finish_cancel = true;
    ManagedStack::TearDown(); telemetry.reset(); clock_pub.reset(); state_pub.reset();
    image_pub.reset(); event_pub.reset();
    flight_caps.reset();
    if (parameters.state() != QProcess::NotRunning) {
      parameters.terminate(); EXPECT_TRUE(parameters.waitForFinished(3000));
      if (parameters.state() != QProcess::NotRunning) {
        parameters.kill(); parameters.waitForFinished(2000);
      }
    }
    const auto root = qEnvironmentVariable("EASYFLY_C06_EVIDENCE_ROOT");
    if (!root.isEmpty()) {
      directory.setAutoRemove(false);
      const auto name = QString::fromLatin1(
        ::testing::UnitTest::GetInstance()->current_test_info()->name());
      QFile report(root + "/" + name + ".json"); ASSERT_TRUE(
        report.open(QIODevice::WriteOnly | QIODevice::NewOnly));
      QJsonArray requests;
      for (const auto & request : reviewed_requests) {
        requests.append(
          QJsonObject{{"operation", QString::fromStdString(request.operation)},
            {"instance", QString::fromStdString(request.server_instance_id)},
            {"request", QString::fromStdString(request.client_request_id)},
            {"execution", QString::fromStdString(request.execution_id)}});
      }
      report.write(
        QJsonDocument(
          QJsonObject{{"kind", "C06 real ROS chain, MOCK Action/pixels/state"},
            {"artifact_directory", directory.path()}, {"record_path", record_path},
            {"wire", wire}, {"requests", requests}, {"identity", identity}, {"events", events},
            {"manifest", manifest}, {"action_goals", goals.load()},
            {"action_cancels", cancels.load()},
            {"assertions_passed", !::testing::Test::HasFailure()}}).toJson());
      QFile log(root + "/" + name + ".backend.log"); ASSERT_TRUE(
        log.open(QIODevice::WriteOnly | QIODevice::NewOnly)); log.write(process.readAll());
    }
  }
  static QByteArray bytes(const QString & path)
  {QFile file(path); return file.open(QIODevice::ReadOnly) ? file.readAll() : QByteArray{};}
  QJsonArray current_events()
  {
    QJsonArray result;
    for (const auto & line : bytes(record_path + "/events.jsonl").split('\n')) {
      const auto event = QJsonDocument::fromJson(line).object(); if (!event.isEmpty()) {
        result.append(event);
      }
    }
    return result;
  }
  bool recorded(easyfly_ui_control::Phase phase)
  {
    for (const auto & value : current_events()) {
      const auto event = value.toObject();
      if (event["kind"] == "control_snapshot" &&
        event["phase"].toInt() == static_cast<int>(phase)) {return true;}
    }
    return false;
  }
  void exercise(bool cancellation)
  {
    using Phase = easyfly_ui_control::Phase;
    const auto records = directory.path() + "/records";
    auto * nav = widget<QListWidget>("navigation"); nav->setCurrentRow(nav->count() - 1);
    widget<QLineEdit>("recordingDirectory")->setText(records);
    ASSERT_TRUE(until([&] {return widget<QPushButton>("recordingStart")->isEnabled();}, 6000));
    widget<QPushButton>("recordingStart")->click();
    auto * prompt = widget<QMessageBox>("recordingConfirmation"); ASSERT_NE(prompt, nullptr);
    prompt->done(QMessageBox::Ok);
    ASSERT_TRUE(
      until(
        [&] {
          const auto dirs = QDir(records).entryList(QDir::Dirs | QDir::NoDotAndDotDot);
          if (dirs.size() != 1) {return false;} record_path = records + "/" + dirs[0];
          return bytes(record_path + "/frames.jsonl").count('\n') >= 2;
        }));
    nav->setCurrentRow(1); ASSERT_TRUE(start());
    ASSERT_TRUE(until([&] {return recorded(Phase::Sent);}, 1200));
    hold_start = false; ASSERT_TRUE(wait_status(QStringLiteral("执行中")));
    ASSERT_TRUE(until([&] {return recorded(Phase::Running);}));
    identity = QJsonDocument::fromJson(bytes(directory.path() + "/journal/intent.json")).object();
    ASSERT_EQ(identity["request"].toString().size(), 32);
    ASSERT_EQ(identity["execution"].toString().size(), 32);
    ASSERT_EQ(identity["goal"].toString().size(), 32);
    ASSERT_EQ(identity["instance"].toString().size(), 32);
    {std::lock_guard<std::mutex> lock(mutex); ASSERT_TRUE(handle);
      EXPECT_EQ(
        QString::fromStdString(easyfly_core::goalIdentity(handle->get_goal_id())),
        identity["goal"].toString());}
    // This read is an observer request, not a managed intent or Action goal ID.
    widget<QLineEdit>("executionId")->setText(identity["execution"].toString());
    widget<QPushButton>("queryMission")->click();
    ASSERT_TRUE(
      until(
        [&] {
          const auto snapshot = observed->snapshot();
          const auto query = snapshot->missions.find(b::Key::MissionQuery);
          return query != snapshot->missions.end() && query->second.value &&
          query->second.value->execution_id == identity["execution"].toString().toStdString();
        }));
    ASSERT_TRUE(
      until(
        [&] {
          for (const auto & event : current_events()) {
            if (event.toObject()["kind"] == "mission_query_result") {return true;}
          }
          return false;
        }));
    stage = 1;
    if (cancellation) {
      widget<QPushButton>("cancelOwnedMission")->click();
      prompt = widget<QMessageBox>("missionCancelPrompt"); ASSERT_NE(prompt, nullptr);
      prompt->done(QMessageBox::Yes);
      ASSERT_TRUE(until([&] {return cancels == 1 && recorded(Phase::CancelPending);}));
      {std::lock_guard<std::mutex> lock(mutex); ASSERT_TRUE(handle && handle->is_canceling());
        EXPECT_EQ(QString::fromStdString(canceled_id), identity["goal"].toString());}
      ASSERT_TRUE(
        until(
          [&] {
            for (const auto & value : current_events()) {
              const auto event = value.toObject();
              if (event["kind"] == "control_snapshot" && event["status"] == "cancel_requested") {
                return true;
              }
            }
            return false;
          }));
      EXPECT_FALSE(
        QJsonDocument::fromJson(bytes(directory.path() + "/journal/intent.json"))
        .object()["resolved"].toBool()); finish_cancel = true;
    } else {
      std::lock_guard<std::mutex> lock(mutex); ASSERT_TRUE(handle && handle->is_active());
      terminal_ids.insert(easyfly_core::goalIdentity(handle->get_goal_id()));
      auto result = std::make_shared<Action::Result>(); result->success = true;
      result->message = "C06 explicit MOCK completion; ownership still held";
      handle->succeed(result); handle.reset();
    }
    ASSERT_TRUE(wait_status(QStringLiteral("仍等待所有权排空")));
    ASSERT_TRUE(until([&] {return recorded(Phase::DrainPending);}));
    EXPECT_FALSE(widget<QPushButton>("startMission")->isEnabled());
    EXPECT_FALSE(
      QJsonDocument::fromJson(bytes(directory.path() + "/journal/intent.json"))
      .object()["resolved"].toBool());
    stage = 2; released = true;
    ASSERT_TRUE(wait_status(QStringLiteral("该意图已确认解决（不等于任务成功）")));
    ASSERT_TRUE(until([&] {return recorded(Phase::Completed);}));
    QTest::qWait(150); widget<QPushButton>("recordingFinish")->click();
    ASSERT_TRUE(
      until(
        [&] {
          manifest = QJsonDocument::fromJson(bytes(record_path + "/manifest.json")).object();
          return manifest["status"] == "verified";
        }, 5000)); events = current_events();
    bool global = false, query = false; std::vector<int> progression;
    for (const auto & value : events) {
      const auto event = value.toObject(); const auto kind = event["kind"].toString();
      if (kind == "control_snapshot") {
        const auto phase = event["phase"].toInt();
        if (event["request_id"].toString().isEmpty()) {continue;}
        progression.push_back(phase);
        EXPECT_EQ(event["instance_id"], identity["instance"]); EXPECT_EQ(
          event["request_id"],
          identity["request"]);
        EXPECT_EQ(event["content_hash"], identity["hash"]);
        if (phase == static_cast<int>(Phase::Sent)) {
          EXPECT_TRUE(event["execution_id"].toString().isEmpty());
          EXPECT_TRUE(event["action_uuid"].toString().isEmpty()); EXPECT_FALSE(
            event["resolved"].toBool());
        } else {
          EXPECT_EQ(event["execution_id"], identity["execution"]); EXPECT_EQ(
            event["action_uuid"],
            identity["goal"]);
          EXPECT_EQ(event["resolved"].toBool(), phase == static_cast<int>(Phase::Completed));
        }
      } else if (kind == "mission_query_result") {
        query = true; EXPECT_EQ(event["attribution"], "queried_execution_only");
        EXPECT_EQ(event["execution_id"], identity["execution"]); EXPECT_TRUE(
          event["request_id"].isString());
        EXPECT_NE(event["request_id"], identity["request"]);
        EXPECT_FALSE(event.contains("action_uuid"));
      } else if (kind == "global_mission_status" || kind == "observer_event") {
        global = true; EXPECT_EQ(event["attribution"], "global_unattributed");
        EXPECT_FALSE(event.contains("execution_id")); EXPECT_FALSE(event.contains("action_uuid"));
        EXPECT_FALSE(event.contains("request_id"));
      }
    }
    EXPECT_TRUE(global); EXPECT_TRUE(query);
    EXPECT_TRUE(std::is_sorted(progression.begin(), progression.end()));
    EXPECT_EQ(goals, 1); EXPECT_EQ(cancels, cancellation ? 1 : 0);
    {
      std::lock_guard<std::mutex> lock(mutex); unsigned starts = 0, cancel_requests = 0;
      for (const auto & request : reviewed_requests) {
        starts += request.operation == "start"; cancel_requests += request.operation == "cancel";
        if (request.operation != "session") {
          EXPECT_EQ(
            QString::fromStdString(request.client_request_id),
            identity["request"].toString());
          EXPECT_EQ(
            QString::fromStdString(
              request.server_instance_id), identity["instance"].toString());
        }
      }
      EXPECT_EQ(starts, 1u); EXPECT_EQ(cancel_requests, cancellation ? 1u : 0u);
    }
    const auto manifest_before = bytes(record_path + "/manifest.json");
    const auto files = QDir(record_path).entryList(QDir::Files | QDir::Hidden);
    std::vector<QByteArray> before;
    for (const auto & file : files) {
      before.push_back(
        QCryptographicHash::hash(
          bytes(record_path + "/" + file),
          QCryptographicHash::Sha256).toHex());
    }
    ui::PlaybackWorker reader; ASSERT_TRUE(reader.open(records, QFileInfo(record_path).fileName()));
    ASSERT_TRUE(until([&] {return !reader.state().loading;}, 5000));
    ASSERT_NE(reader.state().record, nullptr); EXPECT_TRUE(reader.state().error.isEmpty());
    EXPECT_EQ(reader.state().record->events.size(), static_cast<std::size_t>(events.size()));
    for (std::size_t i = 0; i < reader.state().record->frames.size(); ++i) {
      ASSERT_TRUE(reader.seek(i));
      ASSERT_TRUE(
        until(
          [&] {
            return !reader.state().pending && reader.state().frame == static_cast<int>(i);
          }));
      EXPECT_EQ(reader.state().image.size(), QSize(96, 64));
    }
    reader.stop(); ASSERT_TRUE(until([&] {return reader.state().stopped;}));
    for (int i = 0; i < files.size(); ++i) {
      EXPECT_EQ(
        QCryptographicHash::hash(
          bytes(record_path + "/" + files[i]),
          QCryptographicHash::Sha256).toHex(), before[i]);
    }
    EXPECT_EQ(bytes(record_path + "/manifest.json"), manifest_before); ASSERT_TRUE(close());
  }
};
TEST_F(TimelineStack, SuccessTerminalAndOwnershipReleaseAreSeparateRecordedStages)
{
  exercise(false);
}
TEST_F(TimelineStack, CancelAcknowledgmentTerminalAndReleaseKeepExactIdentity)
{
  exercise(true);
}
#else
TEST(TimelineStack, RequiresExplicitMediaBuild)
{
  GTEST_SKIP() << "C06 task timeline requires opt-in media backend";
}
#endif
