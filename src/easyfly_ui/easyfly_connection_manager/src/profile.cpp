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
#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QRegularExpression>
#include <QSaveFile>
#include <QSet>
#include <QStandardPaths>
#include <QUuid>
#include <sys/stat.h>
#include <unistd.h>
#include <cmath>
#include <stdexcept>

#include "easyfly_connection_manager/profile.hpp"

namespace easyfly_connection_manager
{
namespace
{
QString connection_program(const QString & name)
{
  const auto local = QCoreApplication::applicationDirPath() + "/" + name;
  if (QFileInfo(local).isExecutable()) {return local;}
  for (const auto & prefix :
    qEnvironmentVariable("AMENT_PREFIX_PATH").split(':', Qt::SkipEmptyParts))
  {
    if (!QFileInfo::exists(
        prefix +
        "/share/ament_index/resource_index/packages/easyfly_connection_manager")) {continue;}
    const auto program = prefix + "/lib/easyfly_connection_manager/" + name;
    if (QFileInfo(program).isExecutable()) {return program;}
  }
  return local;  // QProcess reports a missing/incomplete installation without throwing in a timer.
}
void require(bool valid, const QString & message)
{
  if (!valid) {throw std::runtime_error(message.toStdString());}
}
bool topic(const QString & value, bool empty)
{
  return (empty && value.isEmpty()) ||
         (value.size() <= 256 && QRegularExpression(
           "^/([A-Za-z_][A-Za-z0-9_]*)(/[A-Za-z_][A-Za-z0-9_]*)*$").match(value)
         .hasMatch());
}
void file_check(const QString & path)
{
  struct stat info {};
  if (::lstat(QFile::encodeName(path).constData(), &info) == 0) {
    require(
      S_ISREG(info.st_mode) && info.st_uid == ::getuid() &&
      !(info.st_mode & 0077),
      QStringLiteral("配置文件必须是当前用户的私有普通文件"));
  } else {require(errno == ENOENT, QStringLiteral("无法检查配置文件"));}
}
}  // namespace

Profile Profile::create(const QString & name)
{
  Profile p; p.id = QUuid::createUuid().toString(QUuid::WithoutBraces);
  p.name = name;
  QFile defaults(QFileInfo(backend_program()).dir().absoluteFilePath(
      "../../share/easyfly_connection_manager/installation.json"));
  if (defaults.open(QIODevice::ReadOnly) && defaults.size() <= 8192) {
    const auto path =
      QJsonDocument::fromJson(defaults.readAll()).object()["px4_directory"].
      toString();
    if (QFileInfo(path).isAbsolute() && QFileInfo(path).isDir()) {
      p.px4_directory = path;
    }
  }
  return p;
}
void Profile::validate() const
{
  require(
    QRegularExpression(
      "^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$")
    .match(id).hasMatch(), QStringLiteral("配置 ID 无效"));
  require(
    !name.trimmed().isEmpty() && name.size() <= 64 &&
    !QRegularExpression("[\\x00-\\x1f\\x7f]").match(name).hasMatch(),
    QStringLiteral("名称须为 1–64 个字符且不能含控制字符"));
  // Humble DDS port mapping supports 0..232; actual participant creation still validates ports.
  require(
    domain >= 0 && domain <= 232,
    QStringLiteral("ROS domain 范围应为 0–232"));
  require(
    QStringList{"base_sim", "full_perception", "navigation_dev", "landing_dev"}
    .contains(selected_profile), QStringLiteral("不支持的 UI profile"));
  require(
    topic(front_topic, true) && topic(down_topic, true),
    QStringLiteral("相机 topic 须为合法绝对名称，或留空禁用"));
  require(
    QStringList{"observe", "communication", "sitl"}.contains(
      mode), QStringLiteral("连接模式无效"));
  const auto host = [](const QString & value) {
      return QRegularExpression("^[A-Za-z0-9][A-Za-z0-9.-]{0,252}$").match(value)
             .hasMatch();
    };
  const auto port = [](int value) {return value >= 1 && value <= 65535;};
  require(
    host(bind_host) && host(remote_host) && host(gcs_host),
    QStringLiteral("主机地址须为 IPv4 或 DNS 名称"));
  require(
    port(bind_port) && port(remote_port) && port(gcs_bind_port) && port(
      gcs_port) &&
    gazebo_port >= 1024 && port(gazebo_port), QStringLiteral("端口范围无效"));
  require(
    QStringList{"udp", "tcp", "serial"}.contains(
      transport), QStringLiteral("飞控传输类型无效"));
  require(
    QRegularExpression("^/dev/[A-Za-z0-9_./-]+$").match(
      serial_device).hasMatch() &&
    QDir::cleanPath(
      serial_device) == serial_device && baud >= 1200 && baud <= 4000000,
    QStringLiteral("串口设备或波特率无效"));
  require(
    target_system >= 1 && target_system <= 255,
    QStringLiteral("飞控 system ID 须为 1–255"));
  require(
    QRegularExpression("^[a-z][a-z0-9_]{0,95}$").match(scene).hasMatch(),
    QStringLiteral("场景 ID 无效"));
  for (const auto & path : {px4_directory, qgc_path}) {
    require(
      path.size() <= 4096 && (path.isEmpty() || QDir::isAbsolutePath(path)) &&
      !QRegularExpression("[\\x00-\\x1f\\x7f]").match(path).hasMatch(),
      QStringLiteral("程序/目录须为无控制字符的绝对路径，或留空自动检测"));
  }
  require(
    mode != "sitl" || (simulation_time && local_only && transport == "udp" &&
    bind_host == "127.0.0.1" && bind_port == 14540 &&
    remote_host == "127.0.0.1" &&
    remote_port == 14580 && target_system == 1),
    QStringLiteral(
      "本机单机 SITL 使用模拟时间、本机"
      " DDS 和固定的 14540/14580 UDP 飞控链"
      "路"));
  require(
    !gcs_enabled || (mode != "observe" && gcs_bind_port != bind_port &&
    !(gcs_host == "127.0.0.1" && gcs_bind_port == gcs_port)),
    QStringLiteral("QGC 转发需要本程序持有通信后端，且本地端口不能冲突"));
  require(
    !(mode == "sitl" && gcs_enabled &&
    (gcs_host == "127.0.0.1" || gcs_host == "localhost") && gcs_port == 14550),
    QStringLiteral(
      "本机仿真 QGC 请使用 14560 等独立端"
      "口，避免 PX4 默认 14550 直连产生重"
      "复链路"));
  require(
    !mission_control || (mode == "sitl" && simulation_time),
    QStringLiteral("任务执行只允许显式启用的本机仿真"));
  require(
    !gcs_enabled || (!mission_control && !mission_import && !parameter_edit),
    QStringLiteral("QGC 联网维护模式不能同时启用 EasyFly 可变操作"));
}
QString Profile::fcu_url() const
{
  if (transport == "serial") {
    return serial_device + ":" + QString::number(baud);
  }
  if (transport == "tcp") {
    return "tcp://" + remote_host + ":" + QString::number(remote_port);
  }
  return QString("udp://%1:%2@%3:%4").arg(bind_host).arg(bind_port).arg(
    remote_host).arg(remote_port);
}
QString Profile::gcs_url() const
{
  return gcs_enabled ? QString("udp://%1:%2@%3:%4")
         .arg(
    gcs_host == "127.0.0.1" ||
    gcs_host == "localhost" ? "127.0.0.1" : "0.0.0.0")
         .arg(gcs_bind_port).arg(gcs_host).arg(gcs_port) : QString{};
}
QJsonObject Profile::json() const
{
  return {{"id", id}, {"name", name}, {"mode", mode}, {"domain", domain},
    {"scope", local_only ? "local" : "lan"}, {"rmw", "rmw_fastrtps_cpp"},
    {"discovery", "default"}, {"simulation_time", simulation_time},
    {"selected_profile", selected_profile}, {"front_topic", front_topic},
    {"down_topic", down_topic},
    {"simulation",
      QJsonObject{{"scene", scene}, {"px4_directory", px4_directory},
        {"gazebo_port", gazebo_port}, {"gazebo_gui", gazebo_gui}}},
    {"fcu",
      QJsonObject{{"transport", transport}, {"bind_host", bind_host},
        {"bind_port", bind_port},
        {"remote_host", remote_host}, {"remote_port", remote_port},
        {"serial_device", serial_device},
        {"baud", baud}, {"target_system", target_system}}},
    {"gcs",
      QJsonObject{{"enabled", gcs_enabled}, {"host", gcs_host},
        {"port", gcs_port},
        {"bind_port", gcs_bind_port}, {"qgc_path", qgc_path}}},
    {"features",
      QJsonObject{{"mission_control", mission_control},
        {"mission_import", mission_import},
        {"parameter_edit", parameter_edit}, {"recording", recording},
        {"rviz_embed", rviz_embed}}}};
}
Profile Profile::from_json(const QJsonObject & object)
{
  Profile p;
  auto normalized = object;
  auto legacy = p.json();
  for (const auto & key : {"simulation", "fcu", "gcs", "features"}) {
    legacy.remove(key);
  }
  const bool old = object.keys() == legacy.keys() &&
    object["mode"] == "observe";
  if (old) {
    for (const auto & key : {"simulation", "fcu", "gcs", "features"}) {
      normalized[key] = p.json()[key];
    }
  }
  require(
    normalized.keys() == p.json().keys(),
    QStringLiteral("配置字段缺失或存在不支持字段"));
  for (const auto & key : {"id", "name", "mode", "scope", "rmw", "discovery",
      "selected_profile", "front_topic", "down_topic"})
  {
    require(object.value(key).isString(), QStringLiteral("配置字段类型错误：") + key);
  }
  require(
    object["rmw"] == "rmw_fastrtps_cpp" &&
    object["discovery"] == "default", QStringLiteral("当前支持默认 Fast DDS 发现"));
  require(
    object["scope"] == "local" || object["scope"] == "lan",
    QStringLiteral("通信范围无效"));
  const auto domain = object.value("domain");
  require(
    domain.isDouble() && std::isfinite(domain.toDouble()) &&
    domain.toDouble() == std::floor(domain.toDouble()) &&
    domain.toDouble() >= 0 && domain.toDouble() <= 232,
    QStringLiteral("domain 类型或范围错误"));
  require(object.value("simulation_time").isBool(), QStringLiteral("时间源类型错误"));
  p.id = object["id"].toString(); p.name = object["name"].toString();
  p.domain = domain.toInt(); p.local_only = object["scope"] == "local";
  p.simulation_time = object["simulation_time"].toBool();
  p.selected_profile = object["selected_profile"].toString();
  p.front_topic = object["front_topic"].toString();
  p.down_topic = object["down_topic"].toString();
  p.mode = object["mode"].toString();
  const auto nested = [&](const char * key) {
      require(normalized[key].isObject(), QStringLiteral("配置对象类型错误：") + key);
      const auto value = normalized[key].toObject();
      const auto defaults = Profile{}.json()[key].toObject();
      require(
        value.keys() == defaults.keys(), QStringLiteral(
          "配置对象字段不匹配：") + key);
      for (const auto & name : value.keys()) {
        require(
          value[name].type() == defaults[name].type(),
          QStringLiteral("配置字段类型错误：") + name);
        if (defaults[name].isDouble()) {
          require(
            std::isfinite(value[name].toDouble()) &&
            value[name].toDouble() == std::floor(value[name].toDouble()) &&
            value[name].toDouble() >= 0 && value[name].toDouble() <= 4000000,
            QStringLiteral("整数字段范围错误：") + name);
        }
      }
      return value;
    };
  const auto sim = nested("simulation"), fcu = nested("fcu"),
    gcs = nested("gcs");
  const auto features = nested("features");
  p.scene = sim["scene"].toString();
  p.px4_directory = sim["px4_directory"].toString();
  p.gazebo_port = sim["gazebo_port"].toInt();
  p.gazebo_gui = sim["gazebo_gui"].toBool();
  p.transport = fcu["transport"].toString();
  p.bind_host = fcu["bind_host"].toString();
  p.bind_port = fcu["bind_port"].toInt();
  p.remote_host = fcu["remote_host"].toString();
  p.remote_port = fcu["remote_port"].toInt();
  p.serial_device = fcu["serial_device"].toString();
  p.baud = fcu["baud"].toInt(); p.target_system = fcu["target_system"].toInt();
  p.gcs_enabled = gcs["enabled"].toBool(); p.gcs_host = gcs["host"].toString();
  p.gcs_port = gcs["port"].toInt(); p.gcs_bind_port = gcs["bind_port"].toInt();
  p.qgc_path = gcs["qgc_path"].toString();
  p.mission_control = features["mission_control"].toBool();
  p.mission_import = features["mission_import"].toBool();
  p.parameter_edit = features["parameter_edit"].toBool();
  p.recording = features["recording"].toBool();
  p.rviz_embed = features["rviz_embed"].toBool();
  p.validate(); return p;
}
QString Profile::hash() const
{
  return QString::fromLatin1(
    QCryptographicHash::hash(
      QJsonDocument(json()).toJson(QJsonDocument::Compact),
      QCryptographicHash::Sha256).toHex());
}
QString private_directory(const QString & path, bool create)
{
  require(
    QDir::isAbsolutePath(path) && QDir::cleanPath(path) == path && path != "/",
    QStringLiteral("需要规范的私有绝对目录"));
  struct stat info {};
  if (::lstat(QFile::encodeName(path).constData(), &info) != 0) {
    require(errno == ENOENT && create, QStringLiteral("私有目录不可用"));
    const auto parent = QFileInfo(path).absolutePath();
    require(QFileInfo(parent).isDir(), QStringLiteral("请先创建配置目录的父目录"));
    require(
      ::mkdir(
        QFile::encodeName(
          path).constData(), 0700) == 0 || errno == EEXIST,
      QStringLiteral("不能创建私有目录"));
    require(
      ::lstat(QFile::encodeName(path).constData(), &info) == 0,
      QStringLiteral("不能检查私有目录"));
  }
  require(
    S_ISDIR(
      info.st_mode) && info.st_uid == ::getuid() && !(info.st_mode & 0077),
    QStringLiteral("目录必须归当前用户所有且权限为 0700；不自动修复权限"));
  require(
    QFileInfo(path).canonicalFilePath() == path,
    QStringLiteral("私有目录不能通过符号链接访问"));
  return path;
}
QString default_runtime_directory()
{
  const auto base = qEnvironmentVariable("XDG_RUNTIME_DIR", "/tmp");
  return QDir(base).absoluteFilePath(
    "easyfly-connections-" +
    QString::number(::getuid()));
}
QString default_config_file()
{
  const auto base = QStandardPaths::writableLocation(
    QStandardPaths::GenericConfigLocation);
  return QDir(base).absoluteFilePath("easyfly/connections.json");
}
QString desktop_program()
{
  for (const auto & prefix :
    qEnvironmentVariable("AMENT_PREFIX_PATH").split(':', Qt::SkipEmptyParts))
  {
    if (QFileInfo::exists(
        prefix +
        "/share/ament_index/resource_index/packages/easyfly_desktop"))
    {
      QFile manifest(prefix +
        "/share/easyfly_desktop/connection_protocol.json");
      if (!manifest.open(QIODevice::ReadOnly) || manifest.size() > 4096) {
        continue;
      }
      const auto protocol =
        QJsonDocument::fromJson(manifest.readAll()).object();
      if (protocol["connection_protocol"] != 2 ||
        protocol["snapshot_abi"] != 1 ||
        protocol["managed_default_read_only"] != true) {continue;}
      const auto path = prefix + "/lib/easyfly_desktop/easyfly_desktop";
      if (QFileInfo(path).isFile() && QFileInfo(path).isExecutable()) {
        return path;
      }
    }
  }
  throw std::runtime_error(
          "找不到支持连接协议和快照 ABI 1 "
          "的桌面安装；请使用随包图形启动"
          "器");
}
QString backend_program()
{
  return connection_program("easyfly_connection_backend");
}
QString supervisor_program()
{
  return connection_program("easyfly_connection_supervisor");
}
QProcessEnvironment session_environment(
  const Profile & profile,
  const QString & session)
{
  profile.validate();
  auto env = QProcessEnvironment::systemEnvironment();
  for (const auto & key :
    {"ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "RMW_IMPLEMENTATION",
      "FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE",
      "ROS_DISCOVERY_SERVER",
      "ROS_SUPER_CLIENT", "CYCLONEDDS_URI", "RMW_FASTRTPS_USE_QOS_FROM_XML",
      "RMW_FASTRTPS_PUBLICATION_MODE", "ROS_STATIC_PEERS",
      "ROS_AUTOMATIC_DISCOVERY_RANGE"})
  {
    env.remove(key);
  }
  env.insert("ROS_DOMAIN_ID", QString::number(profile.domain));
  env.insert("ROS_LOCALHOST_ONLY", profile.local_only ? "1" : "0");
  env.insert("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp");
  env.insert("EASYFLY_CONNECTION_ID", profile.id);
  env.insert("EASYFLY_CONNECTION_NAME", profile.name);
  env.insert("EASYFLY_CONNECTION_SESSION", session);
  env.insert("EASYFLY_CONNECTION_HASH", profile.hash());
  return env;
}
QVector<Profile> read_profiles(const QString & path)
{
  require(
    QDir::isAbsolutePath(path) && QDir::cleanPath(path) == path,
    QStringLiteral("配置文件需要规范的绝对路径"));
  private_directory(QFileInfo(path).absolutePath());
  file_check(path);
  QFile file(path);
  if (!file.exists()) {return {};}
  require(
    file.open(QIODevice::ReadOnly) && file.size() <= 262144,
    QStringLiteral("配置文件无法读取或超过 256 KiB"));
  QJsonParseError error;
  const auto doc = QJsonDocument::fromJson(file.readAll(), &error);
  require(
    error.error == QJsonParseError::NoError && doc.isObject(),
    QStringLiteral("配置文件不是合法 JSON"));
  const auto root = doc.object();
  require(
    root.keys() ==
    QStringList{"profiles",
      "schema_version"} &&
    (root["schema_version"] == 1 || root["schema_version"] == 2) &&
    root["profiles"].isArray() && root["profiles"].toArray().size() <= 32,
    QStringLiteral("不支持的配置版本或数量"));
  QVector<Profile> profiles; QSet<QString> ids;
  for (const auto & item : root["profiles"].toArray()) {
    require(item.isObject(), QStringLiteral("配置条目必须是对象"));
    auto p = Profile::from_json(item.toObject());
    require(!ids.contains(p.id), QStringLiteral("重复配置 ID"));
    ids.insert(p.id); profiles.push_back(p);
  }
  return profiles;
}
void write_profiles(const QString & path, const QVector<Profile> & profiles)
{
  require(
    QDir::isAbsolutePath(path) && QDir::cleanPath(path) == path,
    QStringLiteral("配置文件需要规范的绝对路径"));
  private_directory(QFileInfo(path).absolutePath()); file_check(path);
  require(profiles.size() <= 32, QStringLiteral("最多保存 32 条连接"));
  QJsonArray values; QSet<QString> ids;
  for (const auto & p : profiles) {
    p.validate(); require(!ids.contains(p.id), QStringLiteral("重复配置 ID"));
    ids.insert(p.id); values.append(p.json());
  }
  QSaveFile file(path); file.setDirectWriteFallback(false);
  require(file.open(QIODevice::WriteOnly), QStringLiteral("无法保存配置"));
  require(
    file.setPermissions(QFile::ReadOwner | QFile::WriteOwner),
    QStringLiteral("不能设置配置文件的私有权限"));
  const auto bytes = QJsonDocument(
    QJsonObject{{"schema_version", 2},
      {"profiles", values}})
    .toJson(QJsonDocument::Indented);
  require(
    bytes.size() <= 262144,
    QStringLiteral("配置总大小超过 256 KiB，请减少配置数量或路径长度"));
  require(
    file.write(bytes) == bytes.size() && file.commit(),
    QStringLiteral("配置原子保存失败"));
}
}  // namespace easyfly_connection_manager
