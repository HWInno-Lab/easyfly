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
void require(bool valid, const QString & message)
{
  if (!valid) {throw std::runtime_error(message.toStdString());}
}
bool topic(const QString & value, bool empty)
{
  return (empty && value.isEmpty()) ||
         (value.size() <= 256 && QRegularExpression(
           "^/([A-Za-z_][A-Za-z0-9_]*)(/[A-Za-z_][A-Za-z0-9_]*)*$").match(value).hasMatch());
}
void file_check(const QString & path)
{
  struct stat info {};
  if (::lstat(QFile::encodeName(path).constData(), &info) == 0) {
    require(
      S_ISREG(info.st_mode) && info.st_uid == ::getuid() && !(info.st_mode & 0077),
      QStringLiteral("配置文件必须是当前用户的私有普通文件"));
  } else {require(errno == ENOENT, QStringLiteral("无法检查配置文件"));}
}
}  // namespace

Profile Profile::create(const QString & name)
{
  Profile p; p.id = QUuid::createUuid().toString(QUuid::WithoutBraces); p.name = name;
  return p;
}
void Profile::validate() const
{
  require(
    QRegularExpression("^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$")
    .match(id).hasMatch(), QStringLiteral("配置 ID 无效"));
  require(
    !name.trimmed().isEmpty() && name.size() <= 64 &&
    !QRegularExpression("[\\x00-\\x1f\\x7f]").match(name).hasMatch(),
    QStringLiteral("名称须为 1–64 个字符且不能含控制字符"));
  // Humble DDS port mapping supports 0..232; actual participant creation still validates ports.
  require(domain >= 0 && domain <= 232, QStringLiteral("ROS domain 范围应为 0–232"));
  require(
    QStringList{"base_sim", "full_perception", "navigation_dev", "landing_dev"}
    .contains(selected_profile), QStringLiteral("不支持的 UI profile"));
  require(
    topic(front_topic, true) && topic(down_topic, true),
    QStringLiteral("相机 topic 须为合法绝对名称，或留空禁用"));
}
QJsonObject Profile::json() const
{
  return {{"id", id}, {"name", name}, {"mode", "observe"}, {"domain", domain},
    {"scope", local_only ? "local" : "lan"}, {"rmw", "rmw_fastrtps_cpp"},
    {"discovery", "default"}, {"simulation_time", simulation_time},
    {"selected_profile", selected_profile}, {"front_topic", front_topic},
    {"down_topic", down_topic}};
}
Profile Profile::from_json(const QJsonObject & object)
{
  Profile p;
  require(object.keys() == p.json().keys(), QStringLiteral("配置字段缺失或存在不支持字段"));
  for (const auto & key : {"id", "name", "mode", "scope", "rmw", "discovery",
      "selected_profile", "front_topic", "down_topic"})
  {
    require(object.value(key).isString(), QStringLiteral("配置字段类型错误：") + key);
  }
  require(
    object["mode"] == "observe" && object["rmw"] == "rmw_fastrtps_cpp" &&
    object["discovery"] == "default", QStringLiteral("当前仅支持默认 Fast DDS 的已有平台观测"));
  require(
    object["scope"] == "local" || object["scope"] == "lan",
    QStringLiteral("通信范围无效"));
  const auto domain = object.value("domain");
  require(
    domain.isDouble() && std::isfinite(domain.toDouble()) &&
    domain.toDouble() == std::floor(domain.toDouble()) &&
    domain.toDouble() >= 0 && domain.toDouble() <= 232, QStringLiteral("domain 类型或范围错误"));
  require(object.value("simulation_time").isBool(), QStringLiteral("时间源类型错误"));
  p.id = object["id"].toString(); p.name = object["name"].toString();
  p.domain = domain.toInt(); p.local_only = object["scope"] == "local";
  p.simulation_time = object["simulation_time"].toBool();
  p.selected_profile = object["selected_profile"].toString();
  p.front_topic = object["front_topic"].toString(); p.down_topic = object["down_topic"].toString();
  p.validate(); return p;
}
QString Profile::hash() const
{
  return QString::fromLatin1(
    QCryptographicHash::hash(
      QJsonDocument(json()).toJson(QJsonDocument::Compact), QCryptographicHash::Sha256).toHex());
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
      ::mkdir(QFile::encodeName(path).constData(), 0700) == 0 || errno == EEXIST,
      QStringLiteral("不能创建私有目录"));
    require(
      ::lstat(QFile::encodeName(path).constData(), &info) == 0,
      QStringLiteral("不能检查私有目录"));
  }
  require(
    S_ISDIR(info.st_mode) && info.st_uid == ::getuid() && !(info.st_mode & 0077),
    QStringLiteral("目录必须归当前用户所有且权限为 0700；不自动修复权限"));
  require(
    QFileInfo(path).canonicalFilePath() == path,
    QStringLiteral("私有目录不能通过符号链接访问"));
  return path;
}
QString default_runtime_directory()
{
  const auto base = qEnvironmentVariable("XDG_RUNTIME_DIR", "/tmp");
  return QDir(base).absoluteFilePath("easyfly-connections-" + QString::number(::getuid()));
}
QString default_config_file()
{
  const auto base = QStandardPaths::writableLocation(QStandardPaths::GenericConfigLocation);
  return QDir(base).absoluteFilePath("easyfly/connections.json");
}
QString desktop_program()
{
  for (const auto & prefix :
    qEnvironmentVariable("AMENT_PREFIX_PATH").split(':', Qt::SkipEmptyParts))
  {
    if (QFileInfo::exists(prefix + "/share/ament_index/resource_index/packages/easyfly_desktop")) {
      QFile manifest(prefix + "/share/easyfly_desktop/connection_protocol.json");
      if (!manifest.open(QIODevice::ReadOnly) || manifest.size() > 4096) {continue;}
      const auto protocol = QJsonDocument::fromJson(manifest.readAll()).object();
      if (protocol["connection_protocol"] != 1 || protocol["snapshot_abi"] != 1 ||
        protocol["managed_default_read_only"] != true) {continue;}
      const auto path = prefix + "/lib/easyfly_desktop/easyfly_desktop";
      if (QFileInfo(path).isFile() && QFileInfo(path).isExecutable()) {return path;}
    }
  }
  throw std::runtime_error("找不到支持连接协议和快照 ABI 1 的桌面安装；请使用随包图形启动器");
}
QProcessEnvironment session_environment(const Profile & profile, const QString & session)
{
  profile.validate();
  auto env = QProcessEnvironment::systemEnvironment();
  for (const auto & key : {"ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "RMW_IMPLEMENTATION",
      "FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE", "ROS_DISCOVERY_SERVER",
      "ROS_SUPER_CLIENT", "CYCLONEDDS_URI", "RMW_FASTRTPS_USE_QOS_FROM_XML",
      "RMW_FASTRTPS_PUBLICATION_MODE", "ROS_STATIC_PEERS", "ROS_AUTOMATIC_DISCOVERY_RANGE"})
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
    root.keys() == QStringList{"profiles", "schema_version"} && root["schema_version"] == 1 &&
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
  const auto bytes = QJsonDocument(QJsonObject{{"schema_version", 1}, {"profiles", values}})
    .toJson(QJsonDocument::Indented);
  require(file.write(bytes) == bytes.size() && file.commit(), QStringLiteral("配置原子保存失败"));
}
}  // namespace easyfly_connection_manager
