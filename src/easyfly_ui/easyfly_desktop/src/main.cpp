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
#include <QApplication>
#include <QAction>
#include <QCommandLineParser>
#include <QFileInfo>
#include <QFont>
#include <QJsonDocument>
#include <QLocalSocket>
#include <QMenuBar>
#include <QMessageBox>
#include <QProcess>
#include <QTimer>
#include <QUuid>
#include <csignal>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <utility>
#include "easyfly_desktop/main_window.hpp"
#include "connection_page.hpp"
namespace
{
volatile std::sig_atomic_t exit_requested = 0;
void request_exit(int) {exit_requested = 1;}
// The management channel carries bounded status only. Images remain in the existing desktop.
class ManagedSession : public QObject
{
public:
  explicit ManagedSession(easyfly_desktop::MainWindow * window, bool lobby = false)
  : QObject(window), window_(window)
  {
    if (lobby) {return;}
    path_ = qEnvironmentVariable("EASYFLY_CONNECTION_SOCKET");
    session_ = qEnvironmentVariable("EASYFLY_CONNECTION_SESSION");
    token_ = qEnvironmentVariable("EASYFLY_CONNECTION_TOKEN");
    if (path_.isEmpty() || session_.isEmpty() || token_.isEmpty()) {return;}
    window_->setWindowTitle(
      QStringLiteral("EasyFly · %1 · domain %2").arg(
        qEnvironmentVariable("EASYFLY_CONNECTION_NAME"), qEnvironmentVariable("ROS_DOMAIN_ID")));
    if (!window_->property("connectionPageAvailable").toBool()) {
      auto * action = window_->menuBar()->addAction(QStringLiteral("连接管理"));
      QObject::connect(action, &QAction::triggered, this, [this] {send("show_manager");});
    }
    QObject::connect(
      &socket_, &QLocalSocket::readyRead, this, [this] {
        input_.append(socket_.readAll());
        if (input_.size() > 65536) {socket_.abort(); return;}
        while (input_.contains('\n')) {
          const auto end = input_.indexOf('\n');
          const auto value = QJsonDocument::fromJson(input_.left(end)).object();
          input_.remove(0, end + 1);
          if (value["event"] == "close" && value["session"] == session_ &&
          !window_->connection_status()["unresolved"].toBool(true))
          {
            window_->close();
          }
        }
      });
    QObject::connect(&socket_, &QLocalSocket::disconnected, this, [this] {input_.clear();});
    QObject::connect(
      &timer_, &QTimer::timeout, this, [this] {
        if (socket_.state() == QLocalSocket::UnconnectedState) {socket_.connectToServer(path_);}
        send("workspace");
      });
    timer_.start(500); socket_.connectToServer(path_);
  }
  ~ManagedSession() override
  {
    timer_.stop();
    QObject::disconnect(&socket_, nullptr, this, nullptr);
    socket_.abort();
  }
  bool return_to_connections() const
  {
    return window_->property("connectionCloseReturnsToLobby").toBool();
  }

private:
  void send(const QString & operation)
  {
    if (socket_.state() != QLocalSocket::ConnectedState || socket_.bytesToWrite() > 32768) {return;}
    const QJsonObject message{{"v", 1}, {"id", QUuid::createUuid().toString(QUuid::WithoutBraces)},
      {"op", operation}, {"session", session_}, {"token", token_},
      {"status", window_->connection_status()}};
    socket_.write(QJsonDocument(message).toJson(QJsonDocument::Compact) + '\n');
  }
  easyfly_desktop::MainWindow * window_;
  QLocalSocket socket_;
  QTimer timer_;
  QByteArray input_;
  QString path_, session_, token_;
};
}  // namespace
int main(int argc, char ** argv)
{
  QApplication app(argc, argv); app.setApplicationName("EasyFly");
  app.setApplicationVersion("0.1.0");
  app.setFont(QFont("Noto Sans CJK SC", 10));
  QCommandLineParser parser;
  parser.setApplicationDescription(
    QStringLiteral(
      "EasyFly 默认只读观测；受管仿真任务须显式启用及人工确认。不启动后端。"));
  parser.addHelpOption(); parser.addVersionOption();
  parser.addOption({"connection-lobby", QStringLiteral("打开整合连接页，连接前不创建 ROS 观测器")});
  parser.addOption({"connection-config", QStringLiteral("整合连接配置文件"), "path"});
  parser.addOption({"connection-runtime", QStringLiteral("整合连接管理运行目录"), "path"});
  parser.addOption({"system-time", QStringLiteral("显式使用系统时间，默认模拟时钟")});
  parser.addOption(
    {"enable-recording", QStringLiteral(
        "兼容旧配置的录像标志（需媒体构建）；页面默认可见，仍须人工开始")});
  parser.addOption(
    {"enable-rviz-embed", QStringLiteral(
        "兼容旧配置的三维标志（需内嵌构建）；空间页默认可见，仍须手动打开")});
  parser.addOption(
    {"enable-peripheral-control", QStringLiteral(
        "显式提供 ROI／仿真云台操作入口，页面中仍须允许本次操作")});
  parser.addOption(
    {"enable-mission-control", QStringLiteral("显式提供受管仿真任务入口，仍需界面确认")});
  parser.addOption(
    {"enable-parameter-edit", QStringLiteral(
        "显式启用两项平台元数据编辑，不开放飞行参数")});
  parser.addOption({"mission-journal", QStringLiteral("持久未决记录的独立私有绝对目录"), "path"});
  parser.addOption(
    {"enable-mission-import", QStringLiteral(
        "显式启用受控文件导入，不执行任务")});
  parser.addOption(
    {"import-journal", QStringLiteral(
        "导入未决记录的独立持久私有绝对目录"), "path"});
  parser.addOption(
    {"profile", QStringLiteral("UI 选择标签（不启动后端）"), "name", "navigation_dev"});
  parser.addOption({"qgc-path", QStringLiteral("QGC AppImage 绝对路径，仅供手动离线预览"), "path"});
  parser.addOption({"rviz-path", QStringLiteral("覆盖 RViz2 可执行绝对路径"), "path"});
  parser.addOption(
    {"front-topic", QStringLiteral("前视图像 topic"), "topic",
      "/camera/color/image_raw"});
  parser.addOption(
    {"down-topic", QStringLiteral(
        "下视图像 topic（空字符串禁用）"), "topic", "/camera_down/color/image_raw"});
  parser.addOption(
    {"quit-after-ms", QStringLiteral(
        "隔离 smoke 检查：N ms 后"
        "请求正常异步关闭；默认不"
        "自动退出"), "N"});
  parser.process(app);
  const bool lobby = parser.isSet("connection-lobby");
  if (lobby && (parser.isSet("enable-mission-control") || parser.isSet("enable-mission-import") ||
    parser.isSet("enable-parameter-edit") || parser.isSet("enable-peripheral-control") ||
    parser.isSet("enable-recording") ||
    parser.isSet("enable-rviz-embed")))
  {
    std::fprintf(stderr, "The connection lobby does not create mutable or observation ports.\n");
    return 2;
  }
  if (qEnvironmentVariableIsSet("EASYFLY_CONNECTION_SESSION") &&
    qEnvironmentVariable("EASYFLY_CONNECTION_OPERATIONS") != "configured" &&
    (parser.isSet("enable-mission-control") || parser.isSet("enable-mission-import") ||
    parser.isSet("enable-parameter-edit") || parser.isSet("enable-peripheral-control")))
  {
    std::fprintf(stderr, "Managed maintenance/observation session rejects mutable flags.\n");
    return 2;
  }
  if (parser.isSet("enable-peripheral-control") && parser.isSet("system-time")) {
    std::fprintf(stderr, "Peripheral commands are limited to explicit simulation sessions.\n");
    return 2;
  }
  easyfly_desktop::RecordingOptions recording;
  recording.enabled = parser.isSet("enable-recording");
  if (recording.enabled && !easyfly_desktop::RecordingWorker::available()) {
    std::fprintf(stderr, "Local recording is not enabled in this build.\n"); return 2;
  }
  easyfly_desktop::RvizOptions spatial;
  spatial.enabled = parser.isSet("enable-rviz-embed");
  if (spatial.enabled && !easyfly_desktop::RvizView::available()) {
    std::fprintf(stderr, "RViz embedding is not enabled in this build; use standalone RViz.\n");
    return 2;
  }
  easyfly_desktop::MissionOptions missions;
  easyfly_desktop::ParameterOptions parameters;
  easyfly_desktop::ImportOptions imports;
  imports.enabled = parser.isSet("enable-mission-import");
  imports.journal_directory = parser.value("import-journal").toStdString();
  const auto import_path = std::filesystem::path(imports.journal_directory);
  if ((imports.enabled && (!parser.isSet("import-journal") || !import_path.is_absolute() ||
    import_path.filename().empty() || import_path.filename() == "." ||
    import_path.filename() == "..")) ||
    (!imports.enabled && parser.isSet("import-journal"))) {parser.showHelp(2);}
  parameters.enabled = parser.isSet("enable-parameter-edit");
  missions.enabled = parser.isSet("enable-mission-control");
  missions.journal_directory = parser.value("mission-journal").toStdString();
  const auto journal_path = std::filesystem::path(missions.journal_directory);
  if ((missions.enabled && (!parser.isSet("mission-journal") || !journal_path.is_absolute() ||
    journal_path.filename().empty() || journal_path.filename() == "." ||
    journal_path.filename() == ".." || parser.isSet("system-time"))) ||
    (!missions.enabled && parser.isSet("mission-journal"))) {parser.showHelp(2);}
  easyfly_ui_bridge::BridgeConfig config;
  config.use_sim_time = !parser.isSet("system-time"); config.selected_profile = parser.value(
    "profile").toStdString();
  const auto localization = qEnvironmentVariable("EASYFLY_LOCALIZATION_BACKEND");
  config.selected_localization_backend = localization.isEmpty() ? "external" :
    localization.toStdString();
  config.localization_navigation_enabled =
    qEnvironmentVariable("EASYFLY_LOCALIZATION_NAVIGATION") == "true";
  config.image_topics =
  {parser.value("front-topic").toStdString(), parser.value("down-topic").toStdString()};
  const int geometry_camera = qEnvironmentVariable("EASYFLY_CLASSICAL_CAMERA") == "down" ? 1 : 0;
  auto & geometry_binding = config.camera_perception[geometry_camera];
  if (geometry_binding.image_topic != config.image_topics[geometry_camera]) {
    geometry_binding.detections_topic.clear(); geometry_binding.tracks_topic.clear();
    geometry_binding.targets_topic.clear();
  }
  geometry_binding.image_topic = config.image_topics[geometry_camera];
  geometry_binding.ellipse_topic = "/ellipse_observations";
  geometry_binding.color_line_topic = "/color_line_observations";
  int duration = 0;
  if (parser.isSet("quit-after-ms")) {
    bool valid = false; duration = parser.value("quit-after-ms").toInt(&valid);
    if (!valid || duration < 100 || duration > 60000) {parser.showHelp(2);}
  }
  auto tools = easyfly_desktop::default_tool_options();
  tools.qgc_path = parser.value("qgc-path");
  if (parser.isSet("rviz-path")) {tools.rviz_path = parser.value("rviz-path");}
  auto controls = easyfly_desktop::control_factory();
  if (missions.enabled) {
    try {
      // Transfer the same prepared controller/held journal lock; no check/reopen race.
      auto prepared = std::make_shared<std::unique_ptr<easyfly_desktop::ControlPort>>(
        controls({missions.journal_directory, 2.0}));
      controls = [prepared](const easyfly_ui_control::ControlConfig &) {
          return std::move(*prepared);
        };
    } catch (const std::exception & exc) {
      std::fprintf(stderr, "EasyFly mission journal startup failed: %s\n", exc.what());
      return 2;  // No window, control operation, permission repair or journal clearing.
    }
  }
  auto import_controls = easyfly_desktop::import_factory();
  if (imports.enabled) {
    try {
      auto prepared = std::make_shared<std::unique_ptr<easyfly_desktop::ImportPort>>(
        import_controls({imports.journal_directory, 2}));
      import_controls = [prepared](const easyfly_ui_control::ImportConfig &) {
          return std::move(*prepared);
        };
    } catch (const std::exception & e) {
      std::fprintf(stderr, "EasyFly import journal startup failed: %s\n", e.what()); return 2;
    }
  }
  QString return_program;
  QStringList return_arguments;
  int result = 0;
  {
    easyfly_desktop::MainWindow window(config, easyfly_desktop::bridge_factory(), nullptr, tools,
      missions, controls, parameters, easyfly_desktop::parameter_factory(), imports,
      import_controls,
      spatial, recording, !lobby, parser.isSet("enable-peripheral-control"));
    easyfly_desktop::ConnectionPageHandle connection_page;
    if (lobby || qEnvironmentVariableIsSet("EASYFLY_CONNECTION_SOCKET")) {
      const auto runtime = lobby ? parser.value("connection-runtime") :
        QFileInfo(qEnvironmentVariable("EASYFLY_CONNECTION_SOCKET")).absolutePath();
      const auto file = lobby ? parser.value("connection-config") :
        qEnvironmentVariable("EASYFLY_CONNECTION_CONFIG");
      connection_page = easyfly_desktop::load_connection_page(&window, file, runtime, lobby);
      if (!connection_page.widget && lobby) {
        QMessageBox::critical(
          nullptr, QStringLiteral("EasyFly 连接页不可用"), connection_page.error);
        return 2;
      }
      if (!connection_page.error.isEmpty()) {
        std::fprintf(stderr, "Connection page: %s\n", connection_page.error.toUtf8().constData());
      }
    }
    window.show();
    ManagedSession managed_session(&window, lobby);
    std::signal(SIGINT, request_exit); std::signal(SIGTERM, request_exit);
    QTimer signal_timer;
    QObject::connect(
      &signal_timer, &QTimer::timeout, &window, [&window] {
        if (exit_requested) {
          exit_requested = 0;
          window.close();
        }
      });
    signal_timer.start(100);
    if (duration) {
      QTimer::singleShot(duration, &window, [&window] {window.close();});
    }
    result = app.exec();
    if (!lobby && managed_session.return_to_connections() && connection_page.widget &&
      window.cleanup_complete())
    {
      return_program = connection_page.launcher;
      return_arguments = QStringList{"--config-file", connection_page.config_file,
        "--runtime-directory", connection_page.runtime};
    }
  }
  if (!return_program.isEmpty() && !QProcess::startDetached(return_program, return_arguments)) {
    std::fprintf(stderr, "Cannot return to the EasyFly connection page.\n");
  }
  return result;
}
