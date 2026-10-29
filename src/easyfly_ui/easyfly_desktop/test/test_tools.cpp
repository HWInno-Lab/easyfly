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
#include <unistd.h>
#include <gtest/gtest.h>
#include <QFile>
#include <QLabel>
#include <QDir>
#include <QComboBox>
#include <QMessageBox>
#include <QPushButton>
#include <QStandardPaths>
#include <QTemporaryDir>
#include <QUdpSocket>
#include <QHostAddress>
#include <iostream>
#include "easyfly_desktop/tools.hpp"
#include "support.hpp"
using namespace desktop_test;  // NOLINT(build/namespaces)
namespace
{
ui::ToolOptions options()
{
  auto result = ui::default_tool_options(); result.rviz_path = EASYFLY_TOOL_FIXTURE; return result;
}
}
class Tools : public ::testing::Test
{
protected:
  void SetUp() override {application(); qunsetenv("EASYFLY_TOOL_TEST_MODE");}
  void TearDown() override {qunsetenv("EASYFLY_TOOL_TEST_MODE");}
  void stop(ui::ToolProcess & process)
  {
    process.stop(); EXPECT_TRUE(
      until(
        [&] {
          return !process.active();
        }, 6000));
  }
};
TEST_F(Tools, MissingInvalidPathAndSandboxFailClosed)
{
  ui::ToolProcess process; auto config = options(); config.rviz_path = "/does/not/exist";
  EXPECT_FALSE(process.start(ui::ToolKind::Rviz, config, {}));
  EXPECT_EQ(process.state(), ui::ToolState::Failed);
  config = options(); config.sandbox_path.clear();
  EXPECT_FALSE(process.start(ui::ToolKind::Rviz, config, {}));
  EXPECT_TRUE(process.status().contains("bwrap"));
  config = options(); config.qgc_path = config.rviz_path;
  EXPECT_FALSE(process.start(ui::ToolKind::Qgc, config, {}));
  EXPECT_TRUE(process.status().contains("AppImage"));
  config = options(); config.rviz_path = "relative"; EXPECT_FALSE(
    process.start(ui::ToolKind::Rviz, config, {}));
}
TEST_F(Tools, CommandsUseArgumentsAndQgcHasNoNetworkFallback)
{
  QTemporaryDir directory; ASSERT_TRUE(directory.isValid());
  const auto path = directory.path() + "/test file.AppImage";
  QFile file(path); ASSERT_TRUE(file.open(QIODevice::WriteOnly));
  file.write(QByteArray("\x7f" "ELF\0\0\0\0" "AI\x02", 11)); file.close();
  ASSERT_TRUE(file.setPermissions(QFile::ReadOwner | QFile::WriteOwner | QFile::ExeOwner));
  auto config = options(); config.qgc_path = path;
  const auto command = ui::tool_command(ui::ToolKind::Qgc, config, directory.path(), {}, 0);
  ASSERT_TRUE(command.error.isEmpty()) << command.error.toStdString();
  EXPECT_TRUE(command.arguments.contains("--unshare-net"));
  EXPECT_TRUE(command.arguments.contains("--unshare-pid"));
  EXPECT_TRUE(command.arguments.contains("--unshare-ipc"));
  EXPECT_TRUE(command.arguments.contains("--die-with-parent"));
  EXPECT_TRUE(command.arguments.contains(path));
  EXPECT_EQ(command.arguments.last(), "--appimage-extract-and-run");
  EXPECT_FALSE(command.arguments.contains("--dev-bind"));
  EXPECT_FALSE(command.arguments.contains("/dev/shm"));
  EXPECT_FALSE(command.environment.contains("LD_LIBRARY_PATH"));
  EXPECT_EQ(command.environment.value("XDG_CONFIG_HOME"), "/tmp/easyfly-qgc-config");
}
TEST_F(Tools, PresetsContainOnlyViewingToolsAndExplicitEvidenceLabels)
{
  b::BridgeConfig config; config.image_topics = {"/camera/with_'quotes", "/down/image"};
  for (int preset = 0; preset < 4; ++preset) {
    const auto yaml = ui::rviz_preview(config, preset);
    EXPECT_TRUE(yaml.contains(preset == 3 ? "Fixed Frame: map" : "Fixed Frame: odom"));
    EXPECT_TRUE(yaml.contains("MoveCamera")); EXPECT_TRUE(yaml.contains("Select"));
    EXPECT_FALSE(yaml.contains("SetGoal")); EXPECT_FALSE(yaml.contains("SetInitialPose"));
    EXPECT_FALSE(yaml.contains("PublishPoint")); EXPECT_FALSE(yaml.contains("RobotModel"));
    EXPECT_TRUE(yaml.contains("not freshness checked") || yaml.contains("not flight admission"));
    EXPECT_TRUE(yaml.contains("Reliability Policy: Best Effort"));
    EXPECT_TRUE(yaml.contains("Depth: 1"));
  }
  EXPECT_TRUE(ui::rviz_preview(config, 2).contains("/down/image"));
  EXPECT_FALSE(ui::rviz_preview(config, 2).contains("/camera/with"));
}
TEST_F(Tools, BasicFlightUsesBoundedRealOdometryWithoutSlamOrSyntheticTransforms)
{
  const auto yaml = ui::rviz_preview({}, 3);
  EXPECT_TRUE(yaml.contains("easyfly_rviz_plugins/Odometry"));
  EXPECT_TRUE(yaml.contains("/mavros/local_position/odom"));
  EXPECT_TRUE(yaml.contains("Fixed Frame: map"));
  EXPECT_TRUE(yaml.contains("Keep: 1"));
  EXPECT_TRUE(yaml.contains("Position Tolerance: 0"));
  EXPECT_TRUE(yaml.contains("Angle Tolerance: 0"));
  EXPECT_TRUE(yaml.contains("Value: Axes"));
  EXPECT_FALSE(yaml.contains("/path"));
  EXPECT_FALSE(yaml.contains("/cloud_registered"));
  EXPECT_FALSE(yaml.contains("/optimal_list"));
  EXPECT_FALSE(yaml.contains("rviz_default_plugins/TF"));
  auto state = std::make_shared<FakeState>();
  ui::MainWindow window({}, factory(state), nullptr, options());
  const auto * preset = window.findChild<QComboBox *>("rvizPreset");
  ASSERT_NE(preset, nullptr);
  EXPECT_EQ(preset->currentIndex(), 3);
  EXPECT_EQ(preset->count(), 4);
}
TEST_F(Tools, BasicFlightCommandPreservesRosScopeAndRejectsInvalidPreset)
{
  QTemporaryDir directory; ASSERT_TRUE(directory.isValid());
  b::BridgeConfig bridge_config; bridge_config.use_sim_time = true;
  const auto command = ui::tool_command(
    ui::ToolKind::Rviz, options(), directory.path(), bridge_config, 3);
  ASSERT_TRUE(command.error.isEmpty()) << command.error.toStdString();
  EXPECT_FALSE(command.arguments.contains("--unshare-net"));
  EXPECT_TRUE(command.arguments.contains("--unshare-pid"));
  EXPECT_TRUE(command.arguments.contains("--unshare-ipc"));
  EXPECT_EQ(command.arguments.count("/dev/shm"), 2);
  EXPECT_TRUE(command.arguments.contains("--dev"));
  EXPECT_FALSE(command.arguments.contains("--dev-bind"));
  EXPECT_TRUE(command.arguments.contains("use_sim_time:=true"));
  EXPECT_EQ(command.environment.value("ROS_DOMAIN_ID"), qEnvironmentVariable("ROS_DOMAIN_ID"));
  EXPECT_EQ(
    command.environment.value("ROS_LOCALHOST_ONLY"), qEnvironmentVariable("ROS_LOCALHOST_ONLY"));
  QFile preview(directory.path() + "/preview.rviz");
  ASSERT_TRUE(preview.open(QIODevice::ReadOnly));
  EXPECT_EQ(QString::fromUtf8(preview.readAll()), ui::rviz_preview(bridge_config, 3));
  EXPECT_FALSE(
    ui::tool_command(
      ui::ToolKind::Rviz, options(), directory.path(), bridge_config, 4).error.isEmpty());
}
TEST_F(Tools, OfflineQgcWrapperCannotReachHostLoopback)
{
  QUdpSocket host; ASSERT_TRUE(host.bind(QHostAddress(QHostAddress::LocalHost), 0));
  QTemporaryDir directory; const auto path = directory.path() + "/offline stub.AppImage";
  ASSERT_TRUE(QFile::copy(EASYFLY_TOOL_FIXTURE, path));
  // ELF padding carries the AppImage Type-2 identification in this test stub.
  QFile file(path); ASSERT_TRUE(file.open(QIODevice::ReadWrite));
  ASSERT_TRUE(file.seek(8)); ASSERT_EQ(file.write(QByteArray("AI\x02", 3)), 3); file.close();
  auto config = options(); config.qgc_path = path;
  qputenv("EASYFLY_TOOL_TEST_MODE", "isolation");
  qputenv("EASYFLY_TOOL_TEST_PORT", QByteArray::number(host.localPort()));
  ui::ToolProcess process; ASSERT_TRUE(process.start(ui::ToolKind::Qgc, config, {}));
  ASSERT_TRUE(until([&] {return !process.active();}));
  EXPECT_EQ(process.state(), ui::ToolState::Exited) << process.log().toStdString();
  char network[256] = {};
  ASSERT_GT(readlink("/proc/self/ns/net", network, sizeof(network) - 1), 0);
  EXPECT_TRUE(process.log().contains("network=net:["));
  EXPECT_FALSE(process.log().contains(QString("network=") + network));
  EXPECT_FALSE(host.hasPendingDatagrams()); qunsetenv("EASYFLY_TOOL_TEST_PORT");
}
TEST_F(Tools, ExplicitLaunchDuplicateStopAndUnrelatedProcess)
{
  auto config = options();
  QProcess external; external.start(EASYFLY_TOOL_FIXTURE, QStringList{});
  ASSERT_TRUE(until([&] {return external.state() == QProcess::Running;}));
  ui::ToolProcess process;
  const bool started = process.start(ui::ToolKind::Rviz, config, {});
  EXPECT_TRUE(started);
  EXPECT_TRUE(until([&] {return process.state() != ui::ToolState::Starting;}));
  EXPECT_EQ(process.state(), ui::ToolState::Running) << process.log().toStdString();
  const auto pid = process.pid(); EXPECT_GT(pid, 0);
  EXPECT_FALSE(process.start(ui::ToolKind::Rviz, config, {})); EXPECT_EQ(process.pid(), pid);
  stop(process);
  EXPECT_EQ(external.state(), QProcess::Running);
  external.terminate(); EXPECT_TRUE(until([&] {return external.state() == QProcess::NotRunning;}));
}
TEST_F(Tools, PathsWithSpacesSelfExitFailureAndBoundedLog)
{
  QTemporaryDir directory; const auto path = directory.path() + "/fixture with spaces";
  ASSERT_TRUE(QFile::link(EASYFLY_TOOL_FIXTURE, path));
  auto config = options(); config.rviz_path = path;
  ui::ToolProcess process;
  qputenv("EASYFLY_TOOL_TEST_MODE", "exit");
  ASSERT_TRUE(process.start(ui::ToolKind::Rviz, config, {}));
  ASSERT_TRUE(until([&] {return !process.active();})); EXPECT_EQ(
    process.state(), ui::ToolState::Exited);
  qputenv("EASYFLY_TOOL_TEST_MODE", "fail");
  ASSERT_TRUE(process.start(ui::ToolKind::Rviz, config, {}));
  ASSERT_TRUE(until([&] {return !process.active();})); EXPECT_EQ(
    process.state(), ui::ToolState::Failed);
  qputenv("EASYFLY_TOOL_TEST_MODE", "flood");
  ASSERT_TRUE(process.start(ui::ToolKind::Rviz, config, {}));
  ASSERT_TRUE(until([&] {return process.log().contains("END");}));
  EXPECT_LE(process.log().size(), 32768); stop(process);
}
TEST_F(Tools, StopDuringStartupAndEmergencyDestructionAreContained)
{
  ui::ToolProcess process; ASSERT_TRUE(process.start(ui::ToolKind::Rviz, options(), {}));
  process.stop(); EXPECT_TRUE(until([&] {return !process.active();}, 6000));
  auto emergency = std::make_unique<ui::ToolProcess>();
  ASSERT_TRUE(emergency->start(ui::ToolKind::Rviz, options(), {}));
  ASSERT_TRUE(until([&] {return emergency->state() != ui::ToolState::Starting;}));
  ASSERT_EQ(emergency->state(), ui::ToolState::Running);
  emergency.reset();  // No dangling callback from QProcess destruction.
}
TEST_F(Tools, WindowCloseWaitsForExplicitChoice)
{
  auto state = std::make_shared<FakeState>();
  ui::MainWindow window({}, factory(state), nullptr, options()); window.show();
  auto * start = window.findChild<QPushButton *>("startRviz"); ASSERT_NE(start, nullptr);
  EXPECT_TRUE(
    window.findChild<QLabel *>("rvizState")->text().contains(QStringLiteral("未启动")));
  start->click();
  ASSERT_TRUE(until([&] {return !start->isEnabled();}));
  window.close();
  auto * prompt = window.findChild<QMessageBox *>("toolExitPrompt"); ASSERT_NE(prompt, nullptr);
  EXPECT_FALSE(window.cleanup_complete()); EXPECT_EQ(state->stop_calls, 0);
  prompt->done(QMessageBox::Cancel); QCoreApplication::processEvents();
  EXPECT_EQ(state->stop_calls, 0); EXPECT_TRUE(window.isVisible());
  window.close(); prompt = window.findChild<QMessageBox *>("toolExitPrompt");
  ASSERT_NE(prompt, nullptr); prompt->done(QMessageBox::Yes);
  EXPECT_TRUE(until([&] {return window.cleanup_complete();}, 6000));
  EXPECT_GT(state->stop_calls, 0); EXPECT_EQ(state->query_calls, 0);
}
// Opt-in GUI smoke checks, excluded from unattended tests. No backend is started.
TEST_F(Tools, DISABLED_RealRvizPresets)
{
  auto config = ui::default_tool_options(); b::BridgeConfig bridge_config;
  bridge_config.use_sim_time = false;
  for (int preset = 0; preset < 4; ++preset) {
    ui::ToolProcess process; ASSERT_TRUE(
      process.start(ui::ToolKind::Rviz, config, bridge_config, preset));
    QTest::qWait(5000);
    EXPECT_EQ(process.state(), ui::ToolState::Running) << process.log().toStdString();
    EXPECT_TRUE(process.log().contains("OpenGl")) << process.log().toStdString();
    EXPECT_FALSE(process.log().contains("Failed to load library"));
    std::cout << "RViz preset " << preset << "\n" << process.log().toStdString() << std::endl;
    stop(process);
  }
}
TEST_F(Tools, DISABLED_RealQgcOfflinePreview)
{
  auto config = ui::default_tool_options();
  config.qgc_path = qEnvironmentVariable("EASYFLY_QGC_TEST_IMAGE");
  ASSERT_FALSE(config.qgc_path.isEmpty());
  ui::ToolProcess process; ASSERT_TRUE(process.start(ui::ToolKind::Qgc, config, {}));
  QTest::qWait(8000);
  EXPECT_EQ(process.state(), ui::ToolState::Running) << process.log().toStdString();
  EXPECT_TRUE(process.log().contains("/tmp/easyfly-qgc-config/"));
  std::cout << "QGC offline preview\n" << process.log().toStdString() << std::endl;
  stop(process);
}
