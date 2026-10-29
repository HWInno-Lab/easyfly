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
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QStandardPaths>
#include <utility>
#include "easyfly_desktop/tools.hpp"
namespace easyfly_desktop
{
namespace
{
QString quoted(const QString & value)
{
  auto result = QJsonDocument(QJsonArray{value}).toJson(QJsonDocument::Compact);
  return QString::fromUtf8(result.mid(1, result.size() - 2));
}
bool executable(const QString & path)
{
  const QFileInfo file(path);
  return file.isAbsolute() && file.isFile() && file.isExecutable();
}
QString topic(const QString & value)
{
  return QString(
    "      Topic:\n        Depth: 1\n        Durability Policy: Volatile\n"
    "        History Policy: Keep Last\n        Reliability Policy: Best Effort\n"
    "        Value: %1\n").arg(quoted(value));
}
}  // namespace
ToolOptions default_tool_options()
{
  ToolOptions result;
  result.rviz_path = QStandardPaths::findExecutable("rviz2");
  result.sandbox_path = QStandardPaths::findExecutable("bwrap");
  return result;  // QGC path must be explicitly selected; no implicit download or start.
}
QString rviz_preview(const bridge::BridgeConfig & config, int preset)
{
  QString result =
    "Panels:\n  - Class: rviz_common/Displays\n    Name: Displays\n"
    "  - Class: rviz_common/Views\n    Name: Views\n"
    "Visualization Manager:\n  Class: \"\"\n  Global Options:\n"
    "    Fixed Frame: %1\n    Background Color: 244; 246; 249\n    Frame Rate: 10\n"
    "  Displays:\n    - Class: rviz_default_plugins/Grid\n      Name: Local %1 grid\n"
    "      Enabled: true\n      Plane: XY\n      Cell Size: 1\n      Color: 190; 199; 212\n";
  result = result.arg(preset == 3 ? "map" : "odom");
  const auto display = [&](const QString & type, const QString & name,
      const QString & endpoint, bool enabled) {
      const QString class_id = type == "Odometry" ? "easyfly_rviz_plugins/Odometry" :
        "rviz_default_plugins/" + type;
      result += QString(
        "    - Class: %1\n      Name: %2\n      Enabled: %3\n").arg(
        class_id, quoted(name), enabled ? "true" : "false");
      result += topic(endpoint);
    };
  if (preset == 3) {
    // MAVROS's local odometry is already in map; no synthetic map/odom TF,
    // SLAM publisher or vehicle-control plugin is required by this view.
    display(
      "Odometry", "MAVROS vehicle pose / last received, not freshness checked",
      "/mavros/local_position/odom", true);
    result +=
      "      Keep: 1\n      Position Tolerance: 0\n      Angle Tolerance: 0\n"
      "      Shape:\n        Value: Axes\n        Axes Length: 0.8\n        Axes Radius: 0.04\n"
      "      Covariance:\n        Value: false\n";
  } else if (preset == 0) {
    display("Path", "Actual path / last known, not freshness checked", "/path", true);
    display(
      "Marker", "Planned trajectory / last known, not flight admission",
      "/optimal_list", false);
    display(
      "PointCloud2", "SLAM geometry / last known, not current obstacle evidence",
      "/cloud_registered", false);
  } else if (preset == 1) {
    display(
      "MarkerArray", "Measured targets / publisher lifetime, not freshness checked",
      "/target_markers", true);
    display(
      "MarkerArray", "Geometry obstacles / publisher lifetime, not freshness checked",
      "/obstacle_markers", true);
    display("PointCloud2", "Filtered geometry / last known", "/pointcloud_filtered", false);
  }
  if ((preset == 1 || preset == 2) && !config.image_topics.empty()) {
    const auto index = preset == 2 ? 1U : 0U;
    if (config.image_topics.size() > index && !config.image_topics[index].empty()) {
      display(
        "Image", preset == 2 ? "Down camera / last received, not freshness checked" :
        "Front camera / last received, not freshness checked",
        QString::fromStdString(config.image_topics[index]), true);
    }
  }
  result += "  Tools:\n    - Class: rviz_default_plugins/MoveCamera\n"
    "    - Class: rviz_default_plugins/Select\n"
    "  Views:\n    Current:\n      Class: rviz_default_plugins/Orbit\n"
    "      Distance: 12\n      Pitch: 0.7\n      Yaw: 0.7\n"
    "Window Geometry:\n  Width: 1100\n  Height: 760\n";
  return result;
}
ToolCommand tool_command(
  ToolKind kind, const ToolOptions & options, const QString & directory,
  const bridge::BridgeConfig & config, int preset)
{
  ToolCommand result; result.environment = QProcessEnvironment::systemEnvironment();
  const QString path = kind == ToolKind::Rviz ? options.rviz_path : options.qgc_path;
  if (!executable(path)) {
    result.error = QStringLiteral("工具缺失或不是可执行的绝对文件路径");
    return result;
  }
  if (!executable(options.sandbox_path)) {
    result.error = QStringLiteral("缺少 bwrap：拒绝启动，不降级为普通联网启动"); return result;
  }
  if (!QFileInfo(directory).isDir()) {
    result.error = QStringLiteral("临时配置目录不可用"); return result;
  }
  result.program = options.sandbox_path;
  result.arguments =
    QStringList{"--unshare-pid", "--as-pid-1", "--unshare-ipc", "--die-with-parent",
    "--ro-bind", "/", "/", "--dev", "/dev", "--proc", "/proc",
    "--bind", directory, directory};
  result.environment.remove("QT_PLUGIN_PATH");
  result.environment.remove("QT_QPA_PLATFORM_PLUGIN_PATH");
  result.environment.insert("QT_QPA_PLATFORM", "xcb");
  result.environment.insert("LIBGL_ALWAYS_SOFTWARE", "1");
  result.environment.insert("XDG_CONFIG_HOME", directory + "/config");
  result.environment.insert("XDG_DATA_HOME", directory + "/data");
  result.environment.insert("XDG_CACHE_HOME", directory + "/cache");
  if (kind == ToolKind::Qgc) {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {result.error = "Cannot read AppImage"; return result;}
    const auto magic = file.read(11);
    if (magic.size() != 11 || !magic.startsWith(QByteArray("\x7f" "ELF", 4)) ||
      magic.mid(8, 3) != QByteArray("AI\x02", 3))
    {
      result.error = QStringLiteral("需要 Type-2 AppImage；未启动未知格式程序"); return result;
    }
    result.arguments += QStringList{"--unshare-net", "--tmpfs", "/tmp", "--tmpfs", "/run"};
    result.arguments += QStringList{"--ro-bind", path, "/tmp/easyfly-qgc.AppImage"};
    // Re-expose only the GUI transport, never host runtime IPC/device directories.
    if (QFileInfo("/tmp/.X11-unix").isDir()) {
      result.arguments += QStringList{"--ro-bind", "/tmp/.X11-unix", "/tmp/.X11-unix"};
    }
    const auto authority = result.environment.value("XAUTHORITY");
    if (!authority.isEmpty()) {
      if (!QFileInfo(authority).isFile()) {
        result.error = QStringLiteral("X11 认证文件不可读，拒绝启动"); return result;
      }
      result.arguments += QStringList{"--ro-bind", authority, "/tmp/easyfly-qgc-authority"};
      result.environment.insert("XAUTHORITY", "/tmp/easyfly-qgc-authority");
    }
    result.environment.insert("XDG_CONFIG_HOME", "/tmp/easyfly-qgc-config");
    result.environment.insert("XDG_DATA_HOME", "/tmp/easyfly-qgc-data");
    result.environment.insert("XDG_CACHE_HOME", "/tmp/easyfly-qgc-cache");
    result.environment.insert("QT_QUICK_BACKEND", "software");
    result.environment.remove("LD_LIBRARY_PATH");
    result.arguments +=
      QStringList{"--", "/tmp/easyfly-qgc.AppImage", "--appimage-extract-and-run"};
  } else {
    // Fast DDS localhost participants still use POSIX shared memory. Hiding
    // /dev/shm lets discovery/process startup appear healthy but drops data.
    // RViz is a trusted ROS viewer, not a hostile-code sandbox; QGC never gets
    // this binding. Keep PID ownership, device restrictions and ROS scope.
    const QFileInfo shared_memory("/dev/shm");
    if (!shared_memory.isDir() || !shared_memory.isReadable() || !shared_memory.isWritable()) {
      result.error = QStringLiteral("RViz 所需 DDS 共享内存目录不可用；未启动"); return result;
    }
    result.arguments += QStringList{"--bind", "/dev/shm", "/dev/shm"};
    result.environment.insert("ROS_HOME", directory + "/ros");
    result.environment.insert("ROS_LOG_DIR", directory + "/ros/log");
    result.environment.insert("__GLX_VENDOR_LIBRARY_NAME", "mesa");
    if (preset < 0 || preset > 3) {result.error = "Invalid RViz preset"; return result;}
    QFile file(directory + "/preview.rviz");
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
      result.error = "Cannot write RViz preset"; return result;
    }
    const auto encoded = rviz_preview(config, preset).toUtf8();
    if (file.write(encoded) != encoded.size()) {
      result.error = "Cannot write RViz preset"; return result;
    }
    file.close();
    result.arguments += QStringList{"--", path, "-d", file.fileName(), "--ros-args",
      "-p", config.use_sim_time ? "use_sim_time:=true" : "use_sim_time:=false",
      "-r", "__node:=easyfly_rviz_preview"};
  }
  return result;
}
ToolProcess::ToolProcess()
{
  process_.setProcessChannelMode(QProcess::MergedChannels);
  deadline_.setSingleShot(true); escalation_.setSingleShot(true);
  QObject::connect(&process_, &QProcess::readyReadStandardOutput, &process_, [this] {drain();});
  QObject::connect(
    &process_, &QProcess::started, &process_, [this] {
      deadline_.stop();
      if (state_ == ToolState::Stopping) {process_.terminate();} else {state_ = ToolState::Running;}
    });
  QObject::connect(
    &process_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), &process_,
    [this](int code, QProcess::ExitStatus exit) {
      drain(); deadline_.stop(); escalation_.stop();
      const bool stopped = state_ == ToolState::Stopping;
      state_ = stopped || (code == 0 && exit == QProcess::NormalExit) ?
      ToolState::Exited : ToolState::Failed;
      detail_ = QStringLiteral("退出码 %1，%2").arg(code).arg(
        exit == QProcess::NormalExit ? QStringLiteral("正常退出") : QStringLiteral("进程异常退出"));
    });
  QObject::connect(
    &process_, &QProcess::errorOccurred, &process_, [this](QProcess::ProcessError e) {
      detail_ = process_.errorString();
      if (e == QProcess::FailedToStart) {
        state_ = ToolState::Failed; deadline_.stop(); escalation_.stop();
      }
    });
  QObject::connect(
    &deadline_, &QTimer::timeout, &process_, [this] {
      detail_ = QStringLiteral("启动超时"); stop();
    });
  QObject::connect(
    &escalation_, &QTimer::timeout, &process_, [this] {
      if (active()) {process_.kill();}
    });
}
ToolProcess::~ToolProcess()
{
  // Emergency destruction only; normal GUI close waits for confirmed exit.
  // Disconnect before members disappear: QProcess destruction can emit signals.
  deadline_.stop(); escalation_.stop();
  QObject::disconnect(&process_, nullptr, &process_, nullptr);
  if (process_.state() != QProcess::NotRunning) {
    process_.kill(); process_.waitForFinished(1000);
  }
}
bool ToolProcess::active() const
{
  return state_ == ToolState::Starting || state_ == ToolState::Running ||
         state_ == ToolState::Stopping;
}
bool ToolProcess::start(
  ToolKind kind, const ToolOptions & options, const bridge::BridgeConfig & config, int preset)
{
  if (active()) {return false;}
  log_.clear(); detail_.clear();
  directory_ = std::make_unique<QTemporaryDir>();
  if (!directory_->isValid()) {
    state_ = ToolState::Failed; detail_ = "No temporary directory"; return false;
  }
  const auto session = directory_->path();
  const auto command = tool_command(kind, options, session, config, preset);
  if (!command.error.isEmpty()) {state_ = ToolState::Failed; detail_ = command.error; return false;}
  process_.setProgram(command.program); process_.setArguments(command.arguments);
  process_.setProcessEnvironment(command.environment);
  state_ = ToolState::Starting; process_.start(); deadline_.start(8000); return true;
}
void ToolProcess::stop()
{
  if (!active() || state_ == ToolState::Stopping) {return;}
  state_ = ToolState::Stopping; deadline_.stop(); process_.terminate(); escalation_.start(3000);
}
void ToolProcess::drain()
{
  while (process_.bytesAvailable()) {
    log_ += QString::fromUtf8(process_.read(4096));
    if (log_.size() > 32768) {log_ = log_.right(32768);}
  }
}
QString ToolProcess::status() const
{
  const QStringList names{QStringLiteral("未启动"), QStringLiteral("启动中"),
    QStringLiteral("本次工具包装进程运行中（不是连接确认）"), QStringLiteral("等待本次进程退出"),
    QStringLiteral("本次进程已退出"), QStringLiteral("启动/运行失败")};
  return names[static_cast<int>(state_)] + (detail_.isEmpty() ? "" : "\n" + detail_);
}
}  // namespace easyfly_desktop
