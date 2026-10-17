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
#include <QComboBox>
#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QProcess>
#include <QProcessEnvironment>
#include <QPushButton>
#include <QRegularExpression>
#include <QTemporaryDir>
#include <QTimer>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "easyfly_desktop/playback.hpp"
// Platform integration fixture; kept outside the desktop's minimal dependency closure.
#include "support.hpp"

namespace ui = easyfly_desktop;
namespace b = easyfly_ui_bridge;
namespace p = easyfly_ui_control;
namespace
{
#ifdef EASYFLY_ENABLE_MEDIA
// Real Qt/control/parameter manager/recorder. Only observation pixels/clock/status are MOCK.
class ParameterMedia : public ::testing::Test
{
protected:
  using Query = easyfly_msgs::srv::QueryParams;
  QTemporaryDir directory;
  QProcess backend;
  QTimer images;
  std::shared_ptr<desktop_test::FakeState> observation;
  std::unique_ptr<ui::MainWindow> window;
  ui::ParameterPanel * panel{nullptr};
  rclcpp::Context::SharedPtr context;
  rclcpp::Node::SharedPtr node;
  rclcpp::Client<Query>::SharedPtr forward;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr exporter;
  rclcpp::Service<Query>::SharedPtr relay;
  rclcpp::TimerBase::SharedPtr release_timer;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::thread thread;
  std::mutex mutex;
  QJsonArray trace;
  std::vector<std::function<void()>> delayed;
  std::atomic<bool> hold_write_reply{false};
  std::atomic<int> writes{0}, completed_writes{0}, delivered_writes{0};
  QString record_path;
  QJsonObject manifest;
  double began{0};

  template<typename Widget>
  Widget * widget(const char * name) {return window->findChild<Widget *>(name);}
  static QByteArray bytes(const QString & path)
  {QFile file(path); return file.open(QIODevice::ReadOnly) ? file.readAll() : QByteArray{};}
  static QByteArray digest(const QString & path)
  {return QCryptographicHash::hash(bytes(path), QCryptographicHash::Sha256).toHex();}
  void SetUp() override
  {
    desktop_test::application(); ASSERT_TRUE(directory.isValid());
    ASSERT_STREQ(std::getenv("ROS_DOMAIN_ID"), "124");
    ASSERT_STREQ(std::getenv("ROS_LOCALHOST_ONLY"), "1");
    context = std::make_shared<rclcpp::Context>(); context->init(0, nullptr);
    rclcpp::NodeOptions options; options.context(context).use_global_arguments(false);
    node = std::make_shared<rclcpp::Node>("c05_parameter_relay_no_flight", options);
    forward = node->create_client<Query>("/c05/backend/query");
    exporter = node->create_client<std_srvs::srv::Trigger>("/platform/params/export");
    relay = node->create_service<Query>(
      "/platform/params/query",
      [this](const std::shared_ptr<rclcpp::Service<Query>> & service,
      const std::shared_ptr<rmw_request_id_t> & header, Query::Request::SharedPtr request) {
        int index;
        const bool write = !request->param_value.empty(); if (write) {++writes;}
        {
          std::lock_guard<std::mutex> lock(mutex); index = trace.size();
          trace.append(
            QJsonObject{{"parameter", QString::fromStdString(request->param_name)},
              {"desired_json", QString::fromStdString(request->param_value)},
              {"received_steady", b::detail::steady_seconds()}});
        }
        // Asynchronous transparent forwarding; production still applies every request.
        forward->async_send_request(
          request, [this, service, header, index, write](
            rclcpp::Client<Query>::SharedFuture reply) {
            const auto response = reply.get();
            if (write) {++completed_writes;}
            auto deliver = [this, service, header, response, index, write] {
              service->send_response(*header, *response);
              if (write) {++delivered_writes;}
              std::lock_guard<std::mutex> lock(mutex);
              auto entry = trace[index].toObject(); entry["delivered"] = true;
              entry["delivered_steady"] = b::detail::steady_seconds(); trace[index] = entry;
            };
            {
              std::lock_guard<std::mutex> lock(mutex);
              auto entry = trace[index].toObject(); entry["success"] = response->success;
              entry["current_json"] = QString::fromStdString(response->current_value);
              entry["completed_steady"] = b::detail::steady_seconds(); trace[index] = entry;
              if (write && hold_write_reply) {delayed.push_back(std::move(deliver)); return;}
            }
            deliver();
          });
      });
    release_timer = node->create_wall_timer(
      std::chrono::milliseconds(20), [this] {
        std::vector<std::function<void()>> ready;
        {std::lock_guard<std::mutex> lock(mutex);
          if (!hold_write_reply) {ready.swap(delayed);}}
        for (const auto & deliver : ready) {deliver();}
      });
    rclcpp::ExecutorOptions eo; eo.context = context;
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(eo);
    executor->add_node(node); thread = std::thread([this] {executor->spin();});
    auto environment = QProcessEnvironment::systemEnvironment();
    environment.insert("ROS_LOG_DIR", directory.path()); backend.setProcessEnvironment(environment);
    backend.setProcessChannelMode(QProcess::MergedChannels);
    const auto binary = ament_index_cpp::get_package_prefix("easyfly_params") +
      "/lib/easyfly_params/parameter_manager";
    backend.start(
      QString::fromStdString(binary), {"--ros-args", "-p", "profile:=base_sim",
        "-p", "use_sim_time:=false", "-r", "/platform/params/query:=/c05/backend/query"});
    ASSERT_TRUE(backend.waitForStarted(2000));
    ASSERT_TRUE(desktop_test::until([this] {return forward->service_is_ready();}, 6000));
    observation = std::make_shared<desktop_test::FakeState>();
    observation->data = std::make_shared<b::UiSnapshot>(); observation->data->running = true;
    observation->data->observation_session_id = 105;
    observation->data->clock.quality = b::Quality::ValidAtObservation;
    began = b::detail::steady_seconds();
    QObject::connect(&images, &QTimer::timeout, &images, [this] {image();});
    image(); images.start(50);
    b::BridgeConfig config; config.use_sim_time = false;
    config.selected_profile = "c05_MOCK_no_flight";
    config.image_topics = {"/c05/synthetic_not_camera", ""};
    window = std::make_unique<ui::MainWindow>(
      config, desktop_test::factory(observation), nullptr,
      ui::default_tool_options(), ui::MissionOptions{}, ui::control_factory(), ui::ParameterOptions{
        true}, ui::parameter_factory(), ui::ImportOptions{}, ui::import_factory(),
      ui::RvizOptions{},
      ui::RecordingOptions{true});
    window->show(); panel = window->findChild<ui::ParameterPanel *>(); ASSERT_NE(panel, nullptr);
    EXPECT_EQ(widget<QPushButton>("startMission"), nullptr);
    EXPECT_EQ(widget<QPushButton>("cancelOwnedMission"), nullptr);
  }
  void image()
  {
    const auto ns = static_cast<int64_t>((10 + b::detail::steady_seconds() - began) * 1e9);
    auto value = std::make_shared<sensor_msgs::msg::Image>();
    value->header.stamp.sec = ns / 1000000000LL; value->header.stamp.nanosec = ns % 1000000000LL;
    value->header.frame_id = "c05_MOCK_pixels_not_camera_or_aircraft";
    value->width = 96; value->height = 64; value->step = 288; value->encoding = "rgb8";
    value->data.assign(96 * 64 * 3, static_cast<uint8_t>(ns / 100000000 % 255));
    observation->data->clock.now_ns = ns;
    observation->data->images[0] = {value, desktop_test::evidence()};
    auto global = std::make_shared<easyfly_msgs::msg::MissionStatus>();
    global->header = value->header; global->phase_name = "c05_MOCK_global_not_owned";
    observation->data->last_mission_event = {global, desktop_test::evidence()};
  }
  void TearDown() override
  {
    hold_write_reply = false;
    if (window) {
      desktop_test::until([this] {return !panel || !panel->busy();}, 3500);
      window->close();
      if (auto * prompt = widget<QMessageBox>("parameterExitPrompt")) {
        prompt->done(QMessageBox::Yes);
      }
      EXPECT_TRUE(desktop_test::until([this] {return window->cleanup_complete();}, 5000));
      window.reset();
    }
    images.stop();
    if (backend.state() != QProcess::NotRunning) {
      backend.terminate(); EXPECT_TRUE(backend.waitForFinished(3000));
      if (backend.state() != QProcess::NotRunning) {backend.kill(); backend.waitForFinished(2000);}
    }
    if (executor) {executor->cancel();}
    if (thread.joinable()) {thread.join();}
    if (executor && node) {executor->remove_node(node);}
    release_timer.reset(); relay.reset(); forward.reset(); exporter.reset(); node.reset();
    executor.reset(); if (context && context->is_valid()) {context->shutdown("C05 test end");}
    const auto root = qEnvironmentVariable("EASYFLY_C05_EVIDENCE_ROOT");
    if (!root.isEmpty()) {
      directory.setAutoRemove(false);
      const auto name = QString::fromLatin1(
        ::testing::UnitTest::GetInstance()->current_test_info()
        ->name());
      QFile report(root + "/" + name + ".json"); ASSERT_TRUE(
        report.open(QIODevice::WriteOnly | QIODevice::NewOnly));
      report.write(
        QJsonDocument(
          QJsonObject{{"kind", "C05 real parameter service; MOCK pixels"},
            {"artifact_directory", directory.path()}, {"record_path", record_path},
            {"trace", trace}, {"writes", writes.load()},
            {"completed_writes", completed_writes.load()},
            {"delivered_writes", delivered_writes.load()}, {"manifest", manifest},
            {"assertions_passed", !::testing::Test::HasFailure()}}).toJson());
      QFile log(root + "/" + name + ".backend.log"); ASSERT_TRUE(
        log.open(QIODevice::WriteOnly | QIODevice::NewOnly)); log.write(backend.readAll());
    }
  }
  uint64_t frames()
  {
    const auto match = QRegularExpression(QStringLiteral("已写 (\\d+) 帧")).match(
      widget<QLabel>("recordingStatus")->text());
    return match.hasMatch() ? match.captured(1).toULongLong() : 0;
  }
  p::ParameterPhase phase()
  {
    return static_cast<p::ParameterPhase>(widget<QLabel>("parameterEditStatus")->property(
             "parameterPhase").toInt());
  }
  void read_key(int index)
  {
    widget<QComboBox>("parameterEditKey")->setCurrentIndex(index);
    ASSERT_TRUE(
      desktop_test::until(
        [this] {
          return widget<QPushButton>("parameterEditRead")
          ->isEnabled();
        }));
    widget<QPushButton>("parameterEditRead")->click();
    ASSERT_TRUE(
      desktop_test::until(
        [this] {
          return phase() == p::ParameterPhase::Reviewed &&
          !panel->busy();
        }, 4000));
  }
  void confirm(int answer = QMessageBox::Yes)
  {
    widget<QPushButton>("parameterEditWrite")->click();
    auto * prompt = widget<QMessageBox>("parameterEditPrompt"); ASSERT_NE(prompt, nullptr);
    EXPECT_EQ(prompt->defaultButton(), prompt->button(QMessageBox::Cancel)); prompt->done(answer);
  }
  void start_recording()
  {
    auto * navigation = widget<QListWidget>("navigation");
    navigation->setCurrentRow(navigation->count() - 1);
    widget<QLineEdit>("recordingDirectory")->setText(directory.path() + "/records");
    ASSERT_TRUE(
      desktop_test::until(
        [this] {
          return widget<QPushButton>("recordingStart")
          ->isEnabled();
        }, 5000));
    widget<QPushButton>("recordingStart")->click();
    auto * prompt = widget<QMessageBox>("recordingConfirmation"); ASSERT_NE(prompt, nullptr);
    prompt->done(QMessageBox::Ok);
    ASSERT_TRUE(desktop_test::until([this] {return frames() >= 2;}, 5000));
    navigation->setCurrentRow(4);
  }
  Query::Response::SharedPtr actual(const std::string & name)
  {
    auto request = std::make_shared<Query::Request>(); request->param_name = name;
    auto future = forward->async_send_request(request);
    if (!desktop_test::until(
        [&future] {
          return future.wait_for(std::chrono::seconds(0)) ==
          std::future_status::ready;
        }, 4000)) {return {};}
    return future.get();
  }
  void verify_record(const QString & reason = "operator_stop")
  {
    if (reason == "operator_stop") {widget<QPushButton>("recordingFinish")->click();}
    const auto records = directory.path() + "/records";
    ASSERT_TRUE(
      desktop_test::until(
        [&] {
          const auto dirs = QDir(records).entryList(QDir::Dirs | QDir::NoDotAndDotDot);
          if (dirs.size() != 1) {return false;} record_path = records + "/" + dirs[0];
          const auto status = QJsonDocument::fromJson(bytes(record_path + "/manifest.json"))
          .object()["status"].toString();
          return status == "verified" || status == "failed";
        }, 5000));
    manifest = QJsonDocument::fromJson(bytes(record_path + "/manifest.json")).object();
    ASSERT_EQ(manifest["status"].toString(), "verified");
    EXPECT_EQ(manifest["reason"].toString(), reason);
    EXPECT_GE(manifest["frames"].toString().toULongLong(), 2u);
    ASSERT_TRUE(
      desktop_test::until(
        [&] {
          return frames() ==
          manifest["frames"].toString().toULongLong() &&
          widget<QLabel>("recordingStatus")->text().contains(QStringLiteral("已收尾"));
        }));
    bool global = false;
    for (const auto & line : bytes(record_path + "/events.jsonl").split('\n')) {
      if (line.isEmpty()) {continue;} const auto event = QJsonDocument::fromJson(line).object();
      EXPECT_FALSE(event.contains("execution_id")); EXPECT_FALSE(event.contains("action_uuid"));
      EXPECT_FALSE(event.contains("request_id"));
      if (event["kind"].toString() == "global_mission_status") {
        global = true; EXPECT_EQ(event["attribution"].toString(), "global_unattributed");
      }
    }
    EXPECT_TRUE(global); EXPECT_EQ(
      observation->list_calls + observation->validate_calls +
      observation->query_calls, 0);
    // Independent local reader: every sampled frame is decodable; files stay byte-identical.
    const auto files = QDir(record_path).entryList(QDir::Files | QDir::Hidden);
    std::vector<QByteArray> before; for (const auto & file : files) {
      before.push_back(digest(record_path + "/" + file));
    }
    ui::PlaybackWorker reader; ASSERT_TRUE(reader.open(records, QFileInfo(record_path).fileName()));
    ASSERT_TRUE(desktop_test::until([&] {return !reader.state().loading;}, 5000));
    ASSERT_TRUE(reader.state().error.isEmpty()) << reader.state().error.toStdString();
    ASSERT_NE(reader.state().record, nullptr);
    const auto count = reader.state().record->frames.size();
    EXPECT_EQ(count, manifest["frames"].toString().toULongLong());
    for (std::size_t i = 0; i < count; ++i) {
      ASSERT_TRUE(reader.seek(i));
      ASSERT_TRUE(
        desktop_test::until(
          [&] {
            return !reader.state().pending &&
            reader.state().frame == static_cast<int>(i);
          }, 3000));
      EXPECT_TRUE(reader.state().error.isEmpty());
      EXPECT_EQ(reader.state().image.size(), QSize(96, 64));
    }
    reader.stop(); ASSERT_TRUE(desktop_test::until([&] {return reader.state().stopped;}));
    for (int i = 0; i < files.size(); ++i) {
      EXPECT_EQ(digest(record_path + "/" + files[i]), before[i]);
    }
  }
};
TEST_F(ParameterMedia, TwoMetadataWritesReadbackAndRecordingRemainIndependent)
{
  start_recording(); ASSERT_FALSE(::testing::Test::HasFatalFailure()); read_key(0);
  widget<QPlainTextEdit>("parameterEditNote")->setPlainText(QStringLiteral("C05 隔离备注"));
  confirm(QMessageBox::Cancel); EXPECT_EQ(writes, 0);
  const auto previous = frames(); confirm();
  ASSERT_TRUE(desktop_test::until([this] {return phase() == p::ParameterPhase::Verified;}, 4000));
  EXPECT_EQ(writes, 1); EXPECT_FALSE(panel->fault());
  EXPECT_EQ(widget<QPlainTextEdit>("parameterEditCurrent")->toPlainText(), "\"C05 隔离备注\"");
  ASSERT_TRUE(desktop_test::until([&] {return frames() > previous;}));
  read_key(1); widget<QCheckBox>("parameterEditPretty")->setChecked(true); confirm();
  ASSERT_TRUE(desktop_test::until([this] {return phase() == p::ParameterPhase::Verified;}, 4000));
  EXPECT_EQ(writes, 2);
  EXPECT_EQ(widget<QPlainTextEdit>("parameterEditCurrent")->toPlainText(), "true");
  ASSERT_TRUE(exporter->service_is_ready());
  auto exported = exporter->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_TRUE(
    desktop_test::until(
      [&] {
        return exported.wait_for(std::chrono::seconds(0)) ==
        std::future_status::ready;
      }, 4000));
  const auto result = exported.get(); EXPECT_TRUE(result->success);
  EXPECT_NE(result->message.find('\n'), std::string::npos);
  EXPECT_EQ(
    QJsonDocument::fromJson(QByteArray::fromStdString(result->message)).object()
    ["operator_note"].toString(), QStringLiteral("C05 隔离备注"));
  ASSERT_TRUE(desktop_test::until([&] {return frames() >= previous + 3;}));
  EXPECT_EQ(delivered_writes, 2); verify_record();
  std::lock_guard<std::mutex> lock(mutex); std::vector<QString> operations;
  for (const auto & value : trace) {
    const auto entry = value.toObject(); const auto key = entry["parameter"].toString();
    EXPECT_TRUE(key == "operator_note" || key == "export_pretty");
    operations.push_back(entry["desired_json"].toString().isEmpty() ? "read" : "write");
  }
  EXPECT_EQ(
    operations, (std::vector<QString>{"read", "read", "write", "read",
      "read", "read", "write", "read"}));
}
TEST_F(ParameterMedia, AppliedWriteReplyTimeoutKeepsLockWithoutRetryOrRollback)
{
  start_recording(); ASSERT_FALSE(::testing::Test::HasFatalFailure()); read_key(0);
  widget<QPlainTextEdit>("parameterEditNote")->setPlainText("C05 delayed ACK");
  hold_write_reply = true; const auto previous = frames(); confirm();
  ASSERT_TRUE(desktop_test::until([this] {return completed_writes == 1;}, 4000));
  const auto remote = actual("operator_note"); ASSERT_NE(remote, nullptr);
  EXPECT_TRUE(remote->success); EXPECT_EQ(remote->current_value, "\"C05 delayed ACK\"");
  ASSERT_TRUE(
    desktop_test::until(
      [this] {
        return panel->fault() && !panel->busy() &&
        phase() == p::ParameterPhase::Uncertain &&
        widget<QPushButton>("parameterEditRead")->isEnabled();
      }, 4000));
  EXPECT_EQ(phase(), p::ParameterPhase::Uncertain); EXPECT_EQ(delivered_writes, 0);
  EXPECT_FALSE(widget<QPushButton>("parameterEditWrite")->isEnabled());
  ASSERT_TRUE(desktop_test::until([&] {return frames() > previous + 2;}));
  hold_write_reply = false;
  ASSERT_TRUE(desktop_test::until([this] {return delivered_writes == 1;}));
  widget<QPushButton>("parameterEditRead")->click();
  ASSERT_TRUE(
    desktop_test::until(
      [this] {
        return !panel->busy() &&
        widget<QPlainTextEdit>("parameterEditCurrent")->toPlainText() == "\"C05 delayed ACK\"";
      }));
  EXPECT_TRUE(panel->fault()); EXPECT_EQ(phase(), p::ParameterPhase::Uncertain);
  widget<QPushButton>("parameterEditWrite")->click(); QTest::qWait(150);
  EXPECT_EQ(writes, 1); EXPECT_EQ(widget<QMessageBox>("parameterEditPrompt"), nullptr);
  const auto after = actual("operator_note"); ASSERT_NE(after, nullptr);
  EXPECT_EQ(after->current_value, "\"C05 delayed ACK\""); verify_record();
}
TEST_F(ParameterMedia, ObservationDisconnectFinalizesRecordingAndRetainsWriteFault)
{
  start_recording(); ASSERT_FALSE(::testing::Test::HasFatalFailure()); read_key(0);
  widget<QPlainTextEdit>("parameterEditNote")->setPlainText("C05 disconnect");
  hold_write_reply = true; confirm();
  ASSERT_TRUE(desktop_test::until([this] {return completed_writes == 1;}, 4000));
  widget<QPushButton>("observerToggle")->click();
  ASSERT_TRUE(desktop_test::until([this] {return panel->fault();}));
  hold_write_reply = false;
  ASSERT_TRUE(desktop_test::until([this] {return !panel->busy() && delivered_writes == 1;}));
  EXPECT_FALSE(widget<QPushButton>("parameterEditWrite")->isEnabled()); EXPECT_EQ(writes, 1);
  verify_record("observation_disconnected"); const auto before = frames(); QTest::qWait(200);
  EXPECT_EQ(frames(), before); EXPECT_TRUE(panel->fault());
  const auto remote = actual("operator_note"); ASSERT_NE(remote, nullptr);
  EXPECT_EQ(remote->current_value, "\"C05 disconnect\"");
}
#else
TEST(ParameterMedia, RequiresExplicitMediaBuild)
{
  GTEST_SKIP() << "C05 parameter/recording combination requires opt-in media backend";
}
#endif
}  // namespace
