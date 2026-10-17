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

#include <QCoreApplication>
#include <QDateTime>
#include <QElapsedTimer>
#include <QFile>
#include <QDir>
#include <QJsonArray>
#include <QSettings>
#include <QStandardPaths>
#include <QUdpSocket>
#include <QFileInfo>
#include <QJsonDocument>
#include <QLocalServer>
#include <QLocalSocket>
#include <QPointer>
#include <QProcess>
#include <QSet>
#include <QTimer>
#include <QUuid>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdexcept>

#include "easyfly_connection_manager/supervisor.hpp"
#include "easyfly_connection_manager/profile.hpp"
#include "native_window.hpp"

namespace easyfly_connection_manager
{
namespace
{
class SessionProcess : public QProcess
{
protected:
  void setupChildProcess() override
  {
    if (::setsid() < 0) {::_exit(126);}
  }
};
bool group_active(qint64 group)
{
  if (group <= 0) {return false;}
  for (const auto & pid : QDir("/proc").entryList(QDir::Dirs | QDir::NoDotAndDotDot)) {
    bool numeric = false; pid.toLongLong(&numeric); if (!numeric) {continue;}
    QFile file("/proc/" + pid + "/stat");
    if (!file.open(QIODevice::ReadOnly)) {continue;}
    const auto data = file.readAll();
    const auto fields = data.mid(data.lastIndexOf(')') + 2).simplified().split(' ');
    if (fields.size() > 3 && fields[0] != "Z" && fields[2].toLongLong() == group &&
      fields[3].toLongLong() == group) {return true;}
  }
  return false;
}
QString uuid() {return QUuid::createUuid().toString(QUuid::WithoutBraces);}
void send(QLocalSocket * socket, const QJsonObject & value)
{
  if (socket && socket->state() == QLocalSocket::ConnectedState) {
    if (socket->bytesToWrite() > 65536) {socket->abort(); return;}
    socket->write(QJsonDocument(value).toJson(QJsonDocument::Compact) + '\n');
  }
}
bool same_user(QLocalSocket * socket)
{
  struct ucred credentials {};
  socklen_t size = sizeof(credentials);
  return ::getsockopt(
    socket->socketDescriptor(), SOL_SOCKET, SO_PEERCRED,
    &credentials, &size) == 0 && credentials.uid == ::getuid();
}
}  // namespace
struct Supervisor::Impl
{
  Supervisor * owner;
  QString directory, desktop, instance{uuid()}, session, token, phase{"idle"},
  message, output;
  QJsonObject active_profile, workspace;
  QString config_hash, manager_config;
  QLocalServer server;
  QProcess process, backend_process;
  SessionProcess qgc_process;
  qint64 qgc_group{0};
  QJsonObject backend_state;
  QByteArray backend_input;
  qint64 backend_heartbeat{-1};
  QString run_directory, qgc_message;
  bool unresolved{false};
  QPointer<QLocalSocket> worker, manager;
  QSet<QLocalSocket *> clients;
  QMap<QString, QPair<QByteArray, QJsonObject>> results;
  QElapsedTimer clock;
  qint64 last_heartbeat{-1};
  int lock_fd{-1};
  bool close_pending{false};

  Impl(Supervisor * parent, const QString & path, const QString & executable)
  : owner(parent), directory(private_directory(path)), desktop(executable)
  {
    if (QFile::encodeName(directory + "/control.sock").size() >= 104) {
      throw std::runtime_error("连接管理运行目录路径过长");
    }
    lock_fd = ::open(
      QFile::encodeName(directory + "/manager.lock").constData(),
      O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
    struct stat info {};
    if (lock_fd < 0 || ::fstat(lock_fd, &info) != 0 || !S_ISREG(info.st_mode) ||
      info.st_uid != ::getuid() || (info.st_mode & 0077) ||
      ::flock(lock_fd, LOCK_EX | LOCK_NB) != 0)
    {
      if (lock_fd >= 0) {::close(lock_fd); lock_fd = -1;}
      throw std::runtime_error("连接管理服务已运行或实例锁不可用");
    }
    const auto socket_path = directory + "/control.sock";
    if (::lstat(QFile::encodeName(socket_path).constData(), &info) == 0) {
      if (!S_ISSOCK(info.st_mode) || info.st_uid != ::getuid()) {
        ::close(lock_fd); lock_fd = -1; throw std::runtime_error(
                "已有 socket 路径不安全");
      }
      QFile::remove(socket_path);
    }
    server.setSocketOptions(QLocalServer::UserAccessOption);
    if (!server.listen(socket_path)) {
      ::close(lock_fd); lock_fd = -1; throw std::runtime_error(
              server.errorString().toStdString());
    }
    clock.start();
    QObject::connect(
      &server, &QLocalServer::newConnection, owner, [this] {
        accept();
      });
    QObject::connect(
      &process, &QProcess::started, owner, [this] {
        phase = "starting"; message = QStringLiteral(
          "工作进程已启动，等待实际观测状"
          "态");
      });
    QObject::connect(
      &process, &QProcess::errorOccurred, owner,
      [this](QProcess::ProcessError error) {
        message = process.errorString();
        if (error == QProcess::FailedToStart) {phase = "failed";}
      });
    QObject::connect(
      &process, QOverload<int, QProcess::ExitStatus>::of(
        &QProcess::finished), owner,
      [this](int code,
      QProcess::ExitStatus status) {
        phase = code == 0 && status == QProcess::NormalExit ? "stopped" : "failed";
        message = QStringLiteral("工作会话退出码 %1，%2").arg(code).arg(
          status ==
          QProcess::NormalExit ? QStringLiteral("正常退出") : QStringLiteral(
            "异常退出"));
        if (active_profile["features"].toObject()["mission_control"].toBool() ||
        active_profile["features"].toObject()["mission_import"].toBool())
        {
          unresolved = workspace["unresolved"].toBool(true);
        }
        workspace = {}; worker.clear(); close_pending = false; last_heartbeat = -1;
      });
    process.setProcessChannelMode(QProcess::MergedChannels);
    QObject::connect(
      &process, &QProcess::readyReadStandardOutput, owner, [this] {
        output += QString::fromUtf8(process.readAllStandardOutput());
        if (output.size() > 16384) {output = output.right(16384);}
      });
    backend_process.setProcessChannelMode(QProcess::SeparateChannels);
    QObject::connect(
      &backend_process, &QProcess::started, owner, [this] {
        backend_process.write(
          QJsonDocument(active_profile).toJson(
            QJsonDocument
            ::Compact) + '\n');
      });
    QObject::connect(
      &backend_process, &QProcess::readyReadStandardOutput, owner, [this] {
        backend_input += backend_process.readAllStandardOutput();
        if (backend_input.size() > 65536) {
          message = QStringLiteral(
            "后端状态消息超出预算；保留进程"
            "，不自动重启");
          backend_input.clear(); return;
        }
        while (backend_input.contains('\n')) {
          const auto end = backend_input.indexOf('\n');
          const auto value = QJsonDocument::fromJson(
            backend_input.left(
              end)).object();
          backend_input.remove(0, end + 1);
          if (value.isEmpty()) {continue;}
          backend_state = value; backend_heartbeat = clock.elapsed();
          if (value["process_running"].toBool() &&
          process.state() == QProcess::NotRunning &&
          phase == "starting_backend")
          {
            try {
              start_workspace(Profile::from_json(active_profile));
            } catch (const std::exception & error) {
              message = error.what(); phase = "failed";
            }
          }
        }
      });
    QObject::connect(
      &backend_process, &QProcess::readyReadStandardError, owner, [this] {
        output += QString::fromUtf8(backend_process.readAllStandardError());
        if (output.size() > 16384) {output = output.right(16384);}
      });
    QObject::connect(
      &backend_process, &QProcess::errorOccurred, owner,
      [this](
        QProcess::ProcessError error) {
        if (error == QProcess::FailedToStart) {
          backend_state = {{"phase", "failed"}, {"error",
              backend_process.errorString()}};
          phase = "failed";
        }
      });
    QObject::connect(
      &backend_process,
      QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
      owner, [this](int code, QProcess::ExitStatus) {
        if (backend_state["phase"] != "stopped") {
          backend_state["phase"] = "failed";
          if (!backend_state.contains("error")) {
            backend_state["error"] = QStringLiteral(
              "后端管理进程退出码 %1，请查看日"
              "志；不会自动接管遗留进程").arg(code);
          }
        }
        backend_heartbeat = -1;
      });
    QObject::connect(
      &qgc_process, &QProcess::started, owner, [this] {
        qgc_group = qgc_process.processId();
      });
    qgc_process.setProcessChannelMode(QProcess::MergedChannels);
    QObject::connect(
      &qgc_process, &QProcess::readyReadStandardOutput, owner, [this] {
        output += QString::fromUtf8(qgc_process.readAllStandardOutput());
        if (output.size() > 16384) {output = output.right(16384);}
      });
    QObject::connect(
      &qgc_process, &QProcess::errorOccurred, owner,
      [this](QProcess::ProcessError) {
        qgc_message = qgc_process.errorString();
      });
    QObject::connect(
      &qgc_process, QOverload<int, QProcess::ExitStatus>::of(
        &QProcess::finished), owner,
      [this](int code, QProcess::ExitStatus) {
        qgc_message = QStringLiteral("本次 QGC 已退出，退出码 %1").arg(code);
      });
  }
  ~Impl()
  {
    // Never destroy a live workspace as a side effect of QObject/QProcess teardown.
    // The server only accepts exit when no workspace remains.
    QObject::disconnect(&process, nullptr, owner, nullptr);
    QObject::disconnect(&backend_process, nullptr, owner, nullptr);
    QObject::disconnect(&qgc_process, nullptr, owner, nullptr);
    QObject::disconnect(&server, nullptr, owner, nullptr);
    // QLocalServer owns its sockets. Their destructors may emit disconnected
    // after the QSet member has already been destroyed; detach callbacks now.
    for (auto * socket : clients) {
      QObject::disconnect(socket, nullptr, owner, nullptr);
      socket->abort();
    }
    server.close();
    if (lock_fd >= 0) {::close(lock_fd);}
  }
  QJsonObject status() const
  {
    auto actual = workspace;
    const bool live = process.state() != QProcess::NotRunning;
    const bool fresh = worker &&
      worker->state() == QLocalSocket::ConnectedState &&
      last_heartbeat >= 0 && clock.elapsed() - last_heartbeat < 2500;
    const bool backend_live = backend_process.state() != QProcess::NotRunning;
    const bool backend_fresh = backend_live && backend_heartbeat >= 0 &&
      clock.elapsed() - backend_heartbeat < 2500;
    return {{"protocol", 2}, {"backend_running", backend_live},
      {"backend_fresh", backend_fresh},
      {"backend",
        backend_fresh ||
        !backend_live ? backend_state : QJsonObject{{"message",
          "后端状态未知，保留进程"}}},
      {"qgc_running", qgc_active()},
      {"qgc_message", qgc_message},
      {"unresolved", unresolved}, {"instance", instance}, {"session", session},
      {"phase", phase},
      {"process_running", live}, {"workspace_fresh", fresh},
      {"profile", active_profile},
      {"config_hash", config_hash},
      {"workspace", fresh ? actual : QJsonObject{}},
      {"message", fresh ? message : live ?
        QStringLiteral(
          "工作进程运行中，观测状态未确认"
          "；不会自动重启") : message},
      {"log_tail", output}, {"close_pending", close_pending}};
  }
  bool qgc_active() const
  {
    return qgc_process.state() != QProcess::NotRunning || group_active(qgc_group);
  }
  QString data_directory()
  {
    const auto base = QStandardPaths::writableLocation(
      QStandardPaths::GenericDataLocation);
    if (!QDir(base).exists() && !QDir().mkpath(base)) {
      throw std::runtime_error("无法创建用户数据目录");
    }
    return private_directory(base + "/easyfly-connections");
  }
  bool backend_fresh() const
  {
    return backend_process.state() != QProcess::NotRunning &&
           backend_heartbeat >= 0 &&
           clock.elapsed() - backend_heartbeat < 2500;
  }
  void start_workspace(const Profile & profile)
  {
    token = uuid() + uuid(); workspace = {}; last_heartbeat = -1;
    auto env = session_environment(profile, session);
    if (profile.mode != "observe") {
      const auto dds = backend_state["dds"].toObject();
      const auto library = dds["library_directory"].toString();
      const auto xml = dds["profiles_file"].toString();
      if (!QFileInfo(library).isDir() || !QFileInfo(xml).isFile()) {
        throw std::runtime_error(
                "仿真兼容通信环境尚未确认；保留"
                "后端，不启动混合版本工作窗口");
      }
      env.insert(
        "LD_LIBRARY_PATH",
        library + ":" + env.value("LD_LIBRARY_PATH"));
      env.insert("FASTRTPS_DEFAULT_PROFILES_FILE", xml);
      env.insert("FASTDDS_DEFAULT_PROFILES_FILE", xml);
    }
    env.insert("EASYFLY_CONNECTION_SOCKET", directory + "/control.sock");
    env.insert("EASYFLY_CONNECTION_CONFIG", manager_config);
    env.insert("EASYFLY_CONNECTION_TOKEN", token);
    env.insert(
      "EASYFLY_CONNECTION_OPERATIONS",
      profile.gcs_enabled ? "maintenance" : "configured");
    process.setProcessEnvironment(env); process.setWorkingDirectory(directory);
    QStringList arguments{"--profile", profile.selected_profile,
      "--front-topic", profile.front_topic, "--down-topic", profile.down_topic};
    if (!profile.simulation_time) {arguments << "--system-time";}
    if (profile.recording) {arguments << "--enable-recording";}
    if (profile.rviz_embed) {arguments << "--enable-rviz-embed";}
    if (profile.parameter_edit) {arguments << "--enable-parameter-edit";}
    if (profile.mission_control || profile.mission_import) {
      const auto profiles = private_directory(data_directory() + "/profiles");
      const auto root = private_directory(profiles + "/" + profile.id);
      if (profile.mission_control) {
        arguments << "--enable-mission-control" << "--mission-journal" <<
          private_directory(
          root + "/missions");
      }
      if (profile.mission_import) {
        arguments << "--enable-mission-import" << "--import-journal" <<
          private_directory(
          root + "/imports");
      }
    }
    phase = "starting"; process.start(desktop, arguments);
  }
  void start_qgc()
  {
    if (!backend_fresh() || !backend_state["safe_to_stop"].toBool() ||
      unresolved)
    {
      throw std::runtime_error(
              "需要新鲜、未解锁、在地面的后端"
              "状态，且没有未决任务");
    }
    const auto profile = Profile::from_json(active_profile);
    if (!profile.gcs_enabled || profile.mission_control ||
      profile.mission_import ||
      profile.parameter_edit)
    {
      throw std::runtime_error(
              "先在配置中启用独立的 QGC 联网维"
              "护模式并重新启动后端");
    }
    if (profile.gcs_host != "127.0.0.1" && profile.gcs_host != "localhost") {
      throw std::runtime_error(
              "QGC 位于远端机器，请在该机器打"
              "开；本机只持有 MAVROS 转发");
    }
    if (qgc_active()) {
      throw std::runtime_error("本次 QGC 已运行");
    }
    if (!QFileInfo(profile.qgc_path).isFile() ||
      !QFileInfo(profile.qgc_path).isExecutable())
    {
      throw std::runtime_error(
              "请在 QGC 转发页选择可执行的 QGroun"
              "dControl AppImage/程序并保存");
    }
    QUdpSocket probe;
    if (!probe.bind(
        QHostAddress::AnyIPv4, static_cast<quint16>(profile.gcs_port),
        QUdpSocket::DontShareAddress))
    {
      throw std::runtime_error(
              "QGC 接收端口已占用；不会接管现"
              "有 QGC，请选择其他空闲端口");
    }
    probe.close();
    const auto root = private_directory(run_directory + "/qgc");
    const auto config = private_directory(root + "/config");
    const auto data = private_directory(root + "/data");
    const auto saved = private_directory(data + "/saved");
    for (const auto & organization : {"QGroundControl", "QGroundControl.org"}) {
      const auto org = private_directory(config + "/" + organization);
      QSettings settings(org + "/QGroundControl.ini", QSettings::IniFormat);
      settings.setValue("savePath", saved);
      settings.beginGroup("AutoConnect");
      for (const auto & key : {"autoConnectPixhawk", "autoConnectSiKRadio", "autoConnectRTKGPS",
          "autoConnectLibrePilot", "autoConnectZeroConf"}) {settings.setValue(key, false);}
      settings.setValue("autoConnectUDP", true);
      settings.setValue("udpListenPort", profile.gcs_port);
      settings.setValue("udpTargetHostIP", "127.0.0.1");
      settings.setValue("udpTargetHostPort", profile.gcs_bind_port);
      settings.endGroup(); settings.sync();
      if (settings.status() != QSettings::NoError) {
        throw std::runtime_error("无法保存本次 QGC 的独立设置");
      }
    }
    auto env = QProcessEnvironment::systemEnvironment();
    for (const auto & key :
      {"LD_LIBRARY_PATH", "PYTHONPATH", "QT_PLUGIN_PATH",
        "QT_QPA_PLATFORM_PLUGIN_PATH",
        "QML2_IMPORT_PATH", "QML_IMPORT_PATH", "LD_PRELOAD"}) {env.remove(key);}
    if (env.value("DISPLAY").isEmpty()) {
      throw std::runtime_error("本机 QGC 图形管理需要 X11/XWayland 显示；当前 DISPLAY 不可用");
    }
    env.insert("QT_QPA_PLATFORM", "xcb");
    env.insert("XDG_CONFIG_HOME", config);
    env.insert("XDG_DATA_HOME", private_directory(root + "/data"));
    env.insert("XDG_CACHE_HOME", private_directory(root + "/cache"));
    qgc_process.setProcessEnvironment(env);
    qgc_process.setWorkingDirectory(root);
    qgc_process.setStandardOutputFile(root + "/qgc.log", QIODevice::Append);
    qgc_message = QStringLiteral("启动本次 QGC，等待端点收发与实际窗口确认");
    QStringList arguments;
    if (profile.qgc_path.endsWith(".AppImage", Qt::CaseInsensitive)) {
      arguments << "--appimage-extract-and-run";
    }
    qgc_process.start(profile.qgc_path, arguments);
  }
  void accept()
  {
    while (server.hasPendingConnections()) {
      auto * socket = server.nextPendingConnection();
      if (clients.size() >= 8 || !same_user(socket)) {
        socket->abort(); socket->deleteLater(); continue;
      }
      clients.insert(socket);
      auto input = std::make_shared<QByteArray>();
      QObject::connect(
        socket, &QLocalSocket::disconnected, owner, [this, socket] {
          clients.remove(socket); socket->deleteLater();
        });
      QObject::connect(
        socket, &QLocalSocket::readyRead, owner, [this, socket, input] {
          input->append(socket->readAll());
          if (input->size() > 32768) {socket->abort(); return;}
          int count = 0;
          while (input->contains('\n')) {
            if (++count > 32) {socket->abort(); return;}
            const int end = input->indexOf('\n');
            if (end > 16384) {socket->abort(); return;}
            QJsonParseError error;
            const auto doc = QJsonDocument::fromJson(input->left(end), &error);
            input->remove(0, end + 1);
            if (error.error != QJsonParseError::NoError || !doc.isObject()) {
              socket->abort(); return;
            }
            handle(socket, doc.object());
          }
        });
    }
  }
  void handle(QLocalSocket * socket, const QJsonObject & request)
  {
    const auto id = request["id"].toString();
    const auto op = request["op"].toString();
    if (request["v"] != 1 || id.isEmpty() || id.size() > 64 || op.size() > 32) {
      socket->abort(); return;
    }
    QJsonObject reply{{"v", 1}, {"id", id}, {"ok", true}};
    const auto fingerprint = QJsonDocument(request).toJson(
      QJsonDocument::Compact);
    const bool mutation = op == "start" || op == "close" ||
      op == "exit_service" ||
      op == "stop_backend" || op == "start_qgc" || op == "stop_qgc";
    if (mutation && results.contains(id)) {
      const auto previous = results.value(id);
      if (previous.first == fingerprint) {send(socket, previous.second);} else {
        send(
          socket, {{"v", 1}, {"id", id}, {"ok", false}, {"error",
              "request ID reused"}});
      }
      return;
    }
    const bool backend_operation =
      QStringList{"stop_backend", "start_qgc", "stop_qgc"}.contains(op);
    try {
      if (op == "status" || op == "attach_manager") {
        if (op == "attach_manager") {
          const auto file = request["config_file"].toString();
          if (!file.isEmpty()) {
            if (!QFileInfo(file).isAbsolute() || file.size() > 4096) {
              throw std::runtime_error("管理配置文件路径无效");
            }
            manager_config = file;
          }
          manager = socket;
        }
      } else if (op == "start") {
        if (process.state() != QProcess::NotRunning || close_pending) {
          throw std::runtime_error("已有工作会话；请先正常关闭原工作窗口");
        }
        if (results.size() >= 64) {
          throw std::runtime_error(
                  "本管理实例的修改请求预算已满；"
                  "停止会话后重启管理服务");
        }
        if (!request["profile"].isObject()) {
          throw std::runtime_error("缺少连接配置");
        }
        const auto profile = Profile::from_json(request["profile"].toObject());
        if (!QFileInfo(desktop).isAbsolute() ||
          !QFileInfo(desktop).isExecutable())
        {
          throw std::runtime_error("当前安装中的桌面程序不可用");
        }
        if (profile.recording || profile.rviz_embed) {
          const auto prefix =
            QFileInfo(desktop).dir().absoluteFilePath("../..");
          QFile features(prefix +
            "/share/easyfly_desktop/connection_protocol.json");
          if (!features.open(QIODevice::ReadOnly)) {
            throw std::runtime_error("无法检查工作界面功能构建");
          }
          const auto available =
            QJsonDocument::fromJson(features.readAll()).object();
          if ((profile.recording && !available["recording"].toBool()) ||
            (profile.rviz_embed && !available["rviz_embed"].toBool()))
          {
            throw std::runtime_error(
                    "当前桌面构建未启用所选录像/内"
                    "嵌三维功能，请取消选项或安装相"
                    "应构建");
          }
        }
        if (backend_process.state() != QProcess::NotRunning) {
          if (profile.hash() != config_hash) {
            throw std::runtime_error("先停止原后台，再切换连接配置");
          }
          start_workspace(profile);
        } else {
          if (qgc_active()) {
            throw std::runtime_error("请先关闭本次 QGC");
          }
          session = uuid(); active_profile = profile.json();
          config_hash = profile.hash();
          workspace = {}; output.clear(); last_heartbeat = -1;
          close_pending = false;
          if (profile.mode == "observe") {
            start_workspace(profile);
          } else {
            const auto runs = private_directory(data_directory() + "/runs");
            run_directory = private_directory(runs + "/" + session);
            backend_state = {}; backend_input.clear(); backend_heartbeat = -1;
            backend_process.setWorkingDirectory(run_directory);
            backend_process.setProcessEnvironment(
              session_environment(
                profile,
                session));
            phase = "starting_backend";
            backend_process.start(
              backend_program(), {"--directory", run_directory});
          }
        }
      } else if (backend_operation) {
        if (request["session"] != session) {
          throw std::runtime_error("会话已变化，请刷新后重试");
        }
        if (op == "start_qgc") {start_qgc();} else if (op == "stop_qgc") {
          if (!qgc_active()) {
            throw std::runtime_error("本次 QGC 未运行");
          }
          if (close_owned_windows(qgc_group, run_directory + "/qgc/config") == 0) {
            throw std::runtime_error("尚未找到本次 QGC 主窗口；请等待窗口就绪或在 QGC 中正常退出");
          }
          qgc_message = QStringLiteral("已请求 QGC 正常关闭，如有确认请在其窗口完成；不会强制结束");
        } else {
          if (process.state() != QProcess::NotRunning || unresolved) {
            throw std::runtime_error(
                    "请先处理未决操作并正常关闭工作"
                    "窗口，再停止后端");
          }
          if (qgc_active()) {
            throw std::runtime_error("请先关闭本次联网 QGC");
          }
          if (backend_process.state() == QProcess::NotRunning ||
            (backend_fresh() &&
            !backend_state["safe_to_stop"].toBool() &&
            !backend_state["can_abort_startup"].toBool()))
          {
            throw std::runtime_error("后端状态未知、已解锁或离地，拒绝停止");
          }
          backend_process.write("{\"op\":\"stop\"}\n");
        }
      } else if (op == "close") {
        if (request["session"] != session ||
          process.state() == QProcess::NotRunning)
        {
          throw std::runtime_error("工作会话已变化或没有可关闭的会话");
        }
        if (!worker || worker->state() != QLocalSocket::ConnectedState ||
          last_heartbeat < 0 || clock.elapsed() - last_heartbeat >= 2500 ||
          workspace["unresolved"].toBool(true))
        {
          throw std::runtime_error(
                  "状态未知或任务未决，请在原工作"
                  "窗口处理；不会强制结束");
        }
        close_pending = true;
        send(worker, {{"event", "close"}, {"session", session}});
        message = QStringLiteral(
          "已请求正常关闭；草稿或录像确认"
          "在工作窗口完成");
      } else if (op == "workspace") {
        if (request["session"] != session || request["token"] != token ||
          process.state() == QProcess::NotRunning ||
          !request["status"].isObject())
        {
          throw std::runtime_error("工作会话身份不匹配");
        }
        const auto incoming = request["status"].toObject();
        if (QJsonDocument(incoming).toJson(QJsonDocument::Compact).size() >
          8192)
        {
          throw std::runtime_error("工作快照超出预算");
        }
        worker = socket; last_heartbeat = clock.elapsed(); workspace = incoming;
        unresolved = workspace["unresolved"].toBool(true);
        if (close_pending && !workspace["closing"].toBool() &&
          !workspace["close_prompt"].toBool()) {close_pending = false;}
        if (!close_pending) {phase = "running";}
        message = backend_process.state() == QProcess::NotRunning ?
          QStringLiteral("已有平台 DDS 观测") : QStringLiteral(
          "本次仿真/通信与工作窗口由管理"
          "服务持有");
      } else if (op == "show_manager") {
        if (manager) {send(manager, {{"event", "raise"}});} else {
          const auto executable = QCoreApplication::applicationDirPath() +
            "/easyfly_connection_manager";
          QStringList args{"--runtime-directory", directory};
          if (!manager_config.isEmpty()) {
            args << "--config-file" << manager_config;
          }
          if (!QProcess::startDetached(executable, args)) {
            throw std::runtime_error("无法打开连接管理窗口");
          }
        }
      } else if (op == "exit_service") {
        if (process.state() != QProcess::NotRunning ||
          backend_process.state() != QProcess::NotRunning ||
          qgc_active())
        {
          throw std::runtime_error(
                  "工作窗口、后端或 QGC 仍在运行；"
                  "关闭管理窗口可以保留它们");
        }
        QTimer::singleShot(100, owner, [] {QCoreApplication::quit();});
      } else {throw std::runtime_error("不支持的管理操作");}
    } catch (const std::exception & error) {
      reply["ok"] = false; reply["error"] = QString::fromUtf8(error.what());
    }
    reply["status"] = status();
    if (mutation && results.size() < 64) {
      results.insert(id, {fingerprint, reply});
    }
    send(socket, reply);
  }
};
Supervisor::Supervisor(
  const QString & directory, const QString & desktop,
  QObject * parent)
: QObject(parent), impl_(std::make_unique<Impl>(this, directory, desktop)) {}
Supervisor::~Supervisor() = default;
QJsonObject Supervisor::status() const {return impl_->status();}
}  // namespace easyfly_connection_manager
