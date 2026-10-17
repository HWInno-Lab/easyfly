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
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLabel>
#include <QLineEdit>
#include <QLocalSocket>
#include <QPushButton>
#include <QSignalSpy>
#include <QSpinBox>
#include <QTemporaryDir>
#include <QTest>

#include <sys/stat.h>

#include <functional>

#include "easyfly_connection_manager/manager_window.hpp"
#include "easyfly_connection_manager/profile.hpp"
#include "easyfly_connection_manager/rpc.hpp"
#include "easyfly_connection_manager/supervisor.hpp"

namespace cm = easyfly_connection_manager;
namespace
{
bool until(const std::function<bool()> & condition, int milliseconds = 4000)
{
  for (int i = 0; i < milliseconds / 10; ++i) {
    QCoreApplication::processEvents();
    if (condition()) {return true;}
    QTest::qWait(10);
  }
  return condition();
}
struct Peer
{
  QLocalSocket socket;
  QByteArray buffer;
  explicit Peer(const QString & directory)
  {
    socket.connectToServer(directory + "/control.sock");
    if (!until([this] {return socket.state() == QLocalSocket::ConnectedState;})) {
      throw std::runtime_error("test peer connection timed out");
    }
  }
  QJsonObject request(QJsonObject object)
  {
    object["v"] = 1;
    if (!object.contains("id")) {
      object["id"] = QString::number(++sequence);
    }
    const auto id = object["id"].toString();
    socket.write(QJsonDocument(object).toJson(QJsonDocument::Compact) + '\n');
    QJsonObject answer;
    until(
      [&] {
        buffer += socket.readAll();
        while (buffer.contains('\n')) {
          const auto end = buffer.indexOf('\n');
          const auto value = QJsonDocument::fromJson(buffer.left(end)).object();
          buffer.remove(0, end + 1);
          if (value["id"] == id) {answer = value; return true;}
        }
        return false;
      });
    return answer;
  }
  int sequence{0};
};
}  // namespace

TEST(ConnectionProfile, RejectsMissingAndIncompatibleDesktopProtocol)
{
  QTemporaryDir root;
  struct RestoreEnvironment
  {
    QByteArray old{qgetenv("AMENT_PREFIX_PATH")};
    ~RestoreEnvironment()
    {
      if (old.isNull()) {qunsetenv("AMENT_PREFIX_PATH");} else {qputenv("AMENT_PREFIX_PATH", old);}
    }
  } restore;
  const auto share = root.path() + "/share";
  ASSERT_TRUE(QDir().mkpath(share + "/ament_index/resource_index/packages"));
  ASSERT_TRUE(QDir().mkpath(share + "/easyfly_desktop"));
  ASSERT_TRUE(QDir().mkpath(root.path() + "/lib/easyfly_desktop"));
  QFile marker(share + "/ament_index/resource_index/packages/easyfly_desktop");
  ASSERT_TRUE(marker.open(QIODevice::WriteOnly)); marker.close();
  QFile binary(root.path() + "/lib/easyfly_desktop/easyfly_desktop");
  ASSERT_TRUE(binary.open(QIODevice::WriteOnly)); binary.write("not executed"); binary.close();
  ASSERT_TRUE(binary.setPermissions(QFile::ReadOwner | QFile::WriteOwner | QFile::ExeOwner));
  qputenv("AMENT_PREFIX_PATH", root.path().toUtf8());
  EXPECT_THROW(cm::desktop_program(), std::runtime_error);
  QFile protocol(share + "/easyfly_desktop/connection_protocol.json");
  ASSERT_TRUE(protocol.open(QIODevice::WriteOnly));
  protocol.write(
    "{\"connection_protocol\":1,\"snapshot_abi\":2,"
    "\"managed_default_read_only\":true}");
  protocol.close();
  EXPECT_THROW(cm::desktop_program(), std::runtime_error);
  ASSERT_TRUE(protocol.open(QIODevice::WriteOnly | QIODevice::Truncate));
  protocol.write(
    "{\"connection_protocol\":1,\"snapshot_abi\":1,"
    "\"managed_default_read_only\":true}");
  protocol.close();
  EXPECT_EQ(cm::desktop_program(), binary.fileName());
}

TEST(ConnectionProfile, StrictRoundtripAndUnsupportedInputs)
{
  const auto p = cm::Profile::create(QStringLiteral("中文连接 $(touch NEVER)"));
  EXPECT_EQ(cm::Profile::from_json(p.json()).hash(), p.hash());
  auto object = p.json();
  object["domain"] = 232;
  EXPECT_EQ(cm::Profile::from_json(object).domain, 232);
  object["domain"] = true;
  EXPECT_THROW(cm::Profile::from_json(object), std::runtime_error);
  object = p.json(); object["domain"] = 1.5;
  EXPECT_THROW(cm::Profile::from_json(object), std::runtime_error);
  object = p.json(); object["domain"] = 233;
  EXPECT_THROW(cm::Profile::from_json(object), std::runtime_error);
  object = p.json(); object["name"] = "line\nbreak";
  EXPECT_THROW(cm::Profile::from_json(object), std::runtime_error);
  object = p.json(); object["exec"] = "/bin/sh";
  EXPECT_THROW(cm::Profile::from_json(object), std::runtime_error);
  object = p.json(); object["rmw"] = "unknown";
  EXPECT_THROW(cm::Profile::from_json(object), std::runtime_error);
  object = p.json(); object["front_topic"] = "/bad//topic";
  EXPECT_THROW(cm::Profile::from_json(object), std::runtime_error);
}
TEST(ConnectionProfile, PrivateAtomicStorePreservesInvalidData)
{
  QTemporaryDir root;
  const auto path = root.path() + "/连接配置.json";
  const auto p = cm::Profile::create("Saved");
  cm::write_profiles(path, {p});
  ASSERT_EQ(cm::read_profiles(path).size(), 1);
  EXPECT_EQ(cm::read_profiles(path)[0].hash(), p.hash());
  EXPECT_THROW(cm::write_profiles(path, {p, p}), std::runtime_error);
  QFile file(path);
  ASSERT_TRUE(file.open(QIODevice::WriteOnly | QIODevice::Truncate));
  file.write("{broken"); file.close();
  EXPECT_THROW(cm::read_profiles(path), std::runtime_error);
  ASSERT_TRUE(file.open(QIODevice::ReadOnly)); EXPECT_EQ(file.readAll(), "{broken"); file.close();
  ASSERT_EQ(::chmod(root.path().toUtf8().constData(), 0755), 0);
  EXPECT_THROW(cm::read_profiles(path), std::runtime_error);
  ASSERT_EQ(::chmod(root.path().toUtf8().constData(), 0700), 0);
}
TEST(ConnectionProfile, SelectedEnvironmentClearsInheritedDiscovery)
{
  const auto old_server = qgetenv("ROS_DISCOVERY_SERVER");
  const auto old_xml = qgetenv("FASTRTPS_DEFAULT_PROFILES_FILE");
  qputenv("ROS_DISCOVERY_SERVER", "wrong:11811");
  qputenv("FASTRTPS_DEFAULT_PROFILES_FILE", "/wrong.xml");
  auto p = cm::Profile::create("domain98"); p.domain = 98; p.local_only = false;
  const auto env = cm::session_environment(p, "session");
  EXPECT_EQ(env.value("ROS_DOMAIN_ID"), "98");
  EXPECT_EQ(env.value("ROS_LOCALHOST_ONLY"), "0");
  EXPECT_EQ(env.value("RMW_IMPLEMENTATION"), "rmw_fastrtps_cpp");
  EXPECT_FALSE(env.contains("ROS_DISCOVERY_SERVER"));
  EXPECT_FALSE(env.contains("FASTRTPS_DEFAULT_PROFILES_FILE"));
  if (old_server.isNull()) {qunsetenv("ROS_DISCOVERY_SERVER");} else {
    qputenv("ROS_DISCOVERY_SERVER", old_server);
  }
  if (old_xml.isNull()) {qunsetenv("FASTRTPS_DEFAULT_PROFILES_FILE");} else {
    qputenv("FASTRTPS_DEFAULT_PROFILES_FILE", old_xml);
  }
}
TEST(ConnectionSupervisor, IdempotentStartExactCloseAndCompleteEnvironmentSwitch)
{
  QTemporaryDir root;
  cm::Supervisor supervisor(root.path(), DESKTOP_BINARY_FROM_TEST);
  Peer client(root.path());
  auto p = cm::Profile::create("first"); p.domain = 97; p.simulation_time = false;
  const QJsonObject start{{"id", "start-once"}, {"op", "start"}, {"profile", p.json()}};
  const auto first = client.request(start);
  ASSERT_TRUE(first["ok"].toBool()) << QJsonDocument(first).toJson().constData();
  const auto session = first["status"].toObject()["session"].toString();
  EXPECT_EQ(client.request(start)["status"].toObject()["session"], session);
  auto altered = start; altered["profile"] = cm::Profile::create("other").json();
  EXPECT_FALSE(client.request(altered)["ok"].toBool());
  ASSERT_TRUE(until([&] {return supervisor.status()["workspace_fresh"].toBool();}));
  EXPECT_EQ(supervisor.status()["workspace"].toObject()["domain"], "97");
  EXPECT_FALSE(supervisor.status()["workspace"].toObject()["simulation_time"].toBool());
  EXPECT_FALSE(client.request({{"op", "start"}, {"profile", p.json()}})["ok"].toBool());
  EXPECT_FALSE(client.request({{"op", "close"}, {"session", "wrong"}})["ok"].toBool());
  ASSERT_TRUE(client.request({{"op", "close"}, {"session", session}})["ok"].toBool());
  ASSERT_TRUE(until([&] {return !supervisor.status()["process_running"].toBool();}));
  EXPECT_EQ(supervisor.status()["phase"], "stopped");
  p.domain = 98; p.simulation_time = true;
  ASSERT_TRUE(client.request({{"op", "start"}, {"profile", p.json()}})["ok"].toBool());
  ASSERT_TRUE(until([&] {return supervisor.status()["workspace_fresh"].toBool();}));
  EXPECT_EQ(supervisor.status()["workspace"].toObject()["domain"], "98");
  EXPECT_TRUE(supervisor.status()["workspace"].toObject()["simulation_time"].toBool());
  ASSERT_TRUE(
    client.request(
      {{"op", "close"},
        {"session", supervisor.status()["session"]}})["ok"].toBool());
  ASSERT_TRUE(until([&] {return !supervisor.status()["process_running"].toBool();}));
}
TEST(ConnectionSupervisor, PrivateLockAndInvalidWorkerNeverStartDesktop)
{
  QTemporaryDir root;
  cm::Supervisor supervisor(root.path(), DESKTOP_BINARY_FROM_TEST);
  EXPECT_THROW(cm::Supervisor duplicate(root.path(), DESKTOP_BINARY_FROM_TEST), std::runtime_error);
  Peer client(root.path());
  EXPECT_FALSE(
    client.request(
      {{"op", "workspace"}, {"session", "fake"},
        {"token", "wrong"}, {"status", QJsonObject{{"unresolved", false}}}})["ok"].toBool());
  EXPECT_FALSE(supervisor.status()["process_running"].toBool());
  EXPECT_FALSE(client.request({{"op", "unsupported"}})["ok"].toBool());
  client.socket.write(QByteArray(40000, 'x'));
  ASSERT_TRUE(until([&] {return client.socket.state() == QLocalSocket::UnconnectedState;}));
}
TEST(ConnectionManager, SavesWithoutConnectingAndReattachesToSameLiveWorkspace)
{
  QTemporaryDir root;
  const auto runtime = root.path() + "/runtime";
  cm::Supervisor supervisor(runtime, DESKTOP_BINARY_FROM_TEST);
  const auto config = root.path() + "/profiles.json";
  auto manager = std::make_unique<cm::ManagerWindow>(config, runtime); manager->show();
  auto * save = manager->findChild<QPushButton *>("saveConnection");
  ASSERT_TRUE(until([&] {return save->isEnabled();}));
  manager->findChild<QLineEdit *>("connectionName")->setText(QStringLiteral("测试连接"));
  manager->findChild<QSpinBox *>("connectionDomain")->setValue(98);
  manager->findChild<QCheckBox *>("connectionSimulationTime")->setChecked(false);
  QTest::mouseClick(save, Qt::LeftButton);
  ASSERT_EQ(cm::read_profiles(config).size(), 1);
  EXPECT_FALSE(supervisor.status()["process_running"].toBool());
  auto * start = manager->findChild<QPushButton *>("startConnection");
  ASSERT_TRUE(until([&] {return start->isEnabled();}));
  QTest::mouseClick(start, Qt::LeftButton);
  ASSERT_TRUE(until([&] {return supervisor.status()["workspace_fresh"].toBool();}));
  const auto session = supervisor.status().value("session").toString();
  manager.reset(); QCoreApplication::processEvents();
  EXPECT_TRUE(supervisor.status()["process_running"].toBool());
  manager = std::make_unique<cm::ManagerWindow>(config, runtime); manager->show();
  auto * stop = manager->findChild<QPushButton *>("closeConnection");
  ASSERT_TRUE(until([&] {return stop->isEnabled();}));
  EXPECT_EQ(supervisor.status()["session"], session);
  EXPECT_FALSE(manager->findChild<QLineEdit *>("connectionName")->isEnabled());
  if (qEnvironmentVariableIsSet("EASYFLY_CM_SCREENSHOT")) {
    ASSERT_TRUE(manager->grab().save(qEnvironmentVariable("EASYFLY_CM_SCREENSHOT")));
  }
  QTest::mouseClick(stop, Qt::LeftButton);
  ASSERT_TRUE(until([&] {return !supervisor.status()["process_running"].toBool();}));
}
