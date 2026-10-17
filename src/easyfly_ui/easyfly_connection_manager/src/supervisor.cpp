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

namespace easyfly_connection_manager
{
namespace
{
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
  QString directory, desktop, instance{uuid()}, session, token, phase{"idle"}, message, output;
  QJsonObject active_profile, workspace;
  QString config_hash, manager_config;
  QLocalServer server;
  QProcess process;
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
        ::close(lock_fd); lock_fd = -1; throw std::runtime_error("已有 socket 路径不安全");
      }
      QFile::remove(socket_path);
    }
    server.setSocketOptions(QLocalServer::UserAccessOption);
    if (!server.listen(socket_path)) {
      ::close(lock_fd); lock_fd = -1; throw std::runtime_error(server.errorString().toStdString());
    }
    clock.start();
    QObject::connect(&server, &QLocalServer::newConnection, owner, [this] {accept();});
    QObject::connect(
      &process, &QProcess::started, owner, [this] {
        phase = "starting"; message = QStringLiteral("工作进程已启动，等待实际观测状态");
      });
    QObject::connect(
      &process, &QProcess::errorOccurred, owner, [this](QProcess::ProcessError error) {
        message = process.errorString();
        if (error == QProcess::FailedToStart) {phase = "failed";}
      });
    QObject::connect(
      &process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), owner,
      [this](int code, QProcess::ExitStatus status) {
        phase = code == 0 && status == QProcess::NormalExit ? "stopped" : "failed";
        message = QStringLiteral("工作会话退出码 %1，%2").arg(code).arg(
          status == QProcess::NormalExit ? QStringLiteral("正常退出") : QStringLiteral("异常退出"));
        workspace = {}; worker.clear(); close_pending = false; last_heartbeat = -1;
      });
    process.setProcessChannelMode(QProcess::MergedChannels);
    QObject::connect(
      &process, &QProcess::readyReadStandardOutput, owner, [this] {
        output += QString::fromUtf8(process.readAllStandardOutput());
        if (output.size() > 16384) {output = output.right(16384);}
      });
  }
  ~Impl()
  {
    // Never destroy a live workspace as a side effect of QObject/QProcess teardown.
    // The server only accepts exit when no workspace remains.
    QObject::disconnect(&process, nullptr, owner, nullptr);
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
    const bool fresh = worker && worker->state() == QLocalSocket::ConnectedState &&
      last_heartbeat >= 0 && clock.elapsed() - last_heartbeat < 2500;
    return {{"instance", instance}, {"session", session}, {"phase", phase},
      {"process_running", live}, {"workspace_fresh", fresh}, {"profile", active_profile},
      {"config_hash", config_hash}, {"workspace", fresh ? actual : QJsonObject{}},
      {"message", fresh ? message : live ?
        QStringLiteral("工作进程运行中，观测状态未确认；不会自动重启") : message},
      {"log_tail", output}, {"close_pending", close_pending}};
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
    const auto fingerprint = QJsonDocument(request).toJson(QJsonDocument::Compact);
    const bool mutation = op == "start" || op == "close" || op == "exit_service";
    if (mutation && results.contains(id)) {
      const auto previous = results.value(id);
      if (previous.first == fingerprint) {send(socket, previous.second);} else {
        send(socket, {{"v", 1}, {"id", id}, {"ok", false}, {"error", "request ID reused"}});
      }
      return;
    }
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
          throw std::runtime_error("本管理实例的修改请求预算已满；停止会话后重启管理服务");
        }
        if (!request["profile"].isObject()) {throw std::runtime_error("缺少连接配置");}
        const auto profile = Profile::from_json(request["profile"].toObject());
        if (!QFileInfo(desktop).isAbsolute() || !QFileInfo(desktop).isExecutable()) {
          throw std::runtime_error("当前安装中的桌面程序不可用");
        }
        session = uuid(); token = uuid() + uuid(); active_profile = profile.json();
        config_hash = profile.hash(); workspace = {}; output.clear();
        last_heartbeat = -1; phase = "starting"; close_pending = false;
        auto env = session_environment(profile, session);
        env.insert("EASYFLY_CONNECTION_SOCKET", directory + "/control.sock");
        env.insert("EASYFLY_CONNECTION_TOKEN", token);
        process.setProcessEnvironment(env);
        process.setWorkingDirectory(directory);
        QStringList arguments{"--profile", profile.selected_profile,
          "--front-topic", profile.front_topic, "--down-topic", profile.down_topic};
        if (!profile.simulation_time) {arguments << "--system-time";}
        process.start(desktop, arguments);
      } else if (op == "close") {
        if (request["session"] != session || process.state() == QProcess::NotRunning) {
          throw std::runtime_error("工作会话已变化或没有可关闭的会话");
        }
        if (!worker || worker->state() != QLocalSocket::ConnectedState ||
          last_heartbeat < 0 || clock.elapsed() - last_heartbeat >= 2500 ||
          workspace["unresolved"].toBool(true))
        {
          throw std::runtime_error("状态未知或任务未决，请在原工作窗口处理；不会强制结束");
        }
        close_pending = true;
        send(worker, {{"event", "close"}, {"session", session}});
        message = QStringLiteral("已请求正常关闭；草稿或录像确认在工作窗口完成");
      } else if (op == "workspace") {
        if (request["session"] != session || request["token"] != token ||
          process.state() == QProcess::NotRunning || !request["status"].isObject())
        {
          throw std::runtime_error("工作会话身份不匹配");
        }
        const auto incoming = request["status"].toObject();
        if (QJsonDocument(incoming).toJson(QJsonDocument::Compact).size() > 8192) {
          throw std::runtime_error("工作快照超出预算");
        }
        worker = socket; last_heartbeat = clock.elapsed(); workspace = incoming;
        if (close_pending && !workspace["closing"].toBool() &&
          !workspace["close_prompt"].toBool()) {close_pending = false;}
        if (!close_pending) {phase = "running";}
        message = QStringLiteral("已有平台观测；不持有后端进程");
      } else if (op == "show_manager") {
        if (manager) {send(manager, {{"event", "raise"}});} else {
          const auto executable = QCoreApplication::applicationDirPath() +
            "/easyfly_connection_manager";
          QStringList args{"--runtime-directory", directory};
          if (!manager_config.isEmpty()) {args << "--config-file" << manager_config;}
          if (!QProcess::startDetached(executable, args)) {
            throw std::runtime_error("无法打开连接管理窗口");
          }
        }
      } else if (op == "exit_service") {
        if (process.state() != QProcess::NotRunning) {
          throw std::runtime_error("工作会话仍在运行；管理窗口可以独立关闭并保留它");
        }
        QTimer::singleShot(100, owner, [] {QCoreApplication::quit();});
      } else {throw std::runtime_error("不支持的管理操作");}
    } catch (const std::exception & error) {
      reply["ok"] = false; reply["error"] = QString::fromUtf8(error.what());
    }
    reply["status"] = status();
    if (mutation && results.size() < 64) {results.insert(id, {fingerprint, reply});}
    send(socket, reply);
  }
};
Supervisor::Supervisor(const QString & directory, const QString & desktop, QObject * parent)
: QObject(parent), impl_(std::make_unique<Impl>(this, directory, desktop)) {}
Supervisor::~Supervisor() = default;
QJsonObject Supervisor::status() const {return impl_->status();}
}  // namespace easyfly_connection_manager
