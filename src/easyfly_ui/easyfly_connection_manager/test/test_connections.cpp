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
#include <QCoreApplication>
#include <QDialog>
#include <QDoubleSpinBox>
#include <QListWidget>
#include <QMessageBox>
#include <QScrollArea>
#include <QScrollBar>
#include <QTabWidget>
#include <QTimer>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLabel>
#include <QLineEdit>
#include <QLocalSocket>
#include <QLocalServer>
#include <QMenuBar>
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
#include "../src/qgc_settings.hpp"

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

TEST(ConnectionProfile, Qgc4And5ReadTheSelectedUdpEndpointFromTheirNativeSettingsGroups)
{
  QTemporaryDir root;
  const auto path = root.path() + "/QGroundControl.ini";
  {
    QSettings settings(path, QSettings::IniFormat);
    settings.setValue("savePath", root.path() + "/saved");
    settings.setValue("LinkManager/udpListenPort", 14550);
    settings.setValue("AutoConnect/udpListenPort", 14550);
    cm::configure_qgc_udp(settings, 14562, 14559);
    settings.sync(); ASSERT_EQ(settings.status(), QSettings::NoError);
  }
  QSettings persisted(path, QSettings::IniFormat);
  EXPECT_EQ(persisted.value("savePath").toString(), root.path() + "/saved");
  for (const auto & version_group : {"LinkManager/", "AutoConnect/"}) {
    const QString group(version_group);
    EXPECT_TRUE(persisted.value(group + "autoConnectUDP").toBool());
    EXPECT_EQ(persisted.value(group + "udpListenPort").toInt(), 14562);
    EXPECT_EQ(persisted.value(group + "udpTargetHostIP").toString(), "127.0.0.1");
    EXPECT_EQ(persisted.value(group + "udpTargetHostPort").toInt(), 14559);
    EXPECT_FALSE(persisted.value(group + "autoConnectPixhawk", true).toBool());
    EXPECT_FALSE(persisted.value(group + "autoConnectSiKRadio", true).toBool());
    EXPECT_FALSE(persisted.value(group + "autoConnectRTKGPS", true).toBool());
    EXPECT_FALSE(persisted.value(group + "autoConnectLibrePilot", true).toBool());
    EXPECT_FALSE(persisted.value(group + "autoConnectZeroConf", true).toBool());
    EXPECT_FALSE(persisted.value(group + "autoConnectPX4Flow", true).toBool());
  }
}

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
    "{\"connection_protocol\":2,\"snapshot_abi\":1,"
    "\"managed_default_read_only\":true}");
  protocol.close();
  EXPECT_THROW(cm::desktop_program(), std::runtime_error);
  ASSERT_TRUE(protocol.open(QIODevice::WriteOnly | QIODevice::Truncate));
  protocol.write(
    "{\"connection_protocol\":2,\"snapshot_abi\":3,"
    "\"managed_default_read_only\":true}");
  protocol.close();
  EXPECT_EQ(cm::desktop_program(), binary.fileName());
}

TEST(ConnectionProfile, StrictRoundtripAndUnsupportedInputs)
{
  const auto p = cm::Profile::create(QStringLiteral("中文连接 $(touch NEVER)"));
  EXPECT_FALSE(p.gazebo_gui);
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
  QVector<cm::Profile> large;
  for (int i = 0; i < 32; ++i) {
    auto value = cm::Profile::create("Large");
    value.px4_directory = "/" + QString(4090, 'a');
    value.qgc_path = "/" + QString(4090, 'b'); large.push_back(value);
  }
  EXPECT_THROW(cm::write_profiles(path, large), std::runtime_error);
  ASSERT_EQ(cm::read_profiles(path).size(), 1);
  EXPECT_EQ(cm::read_profiles(path)[0].hash(), p.hash());
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
TEST(ConnectionSupervisor, CorruptRecoveryRecordKeepsServiceReadableAndRejectsNewLaunch)
{
  QTemporaryDir root;
  QFile record(root.path() + "/session.json");
  ASSERT_TRUE(record.open(QIODevice::WriteOnly)); record.write("broken JSON"); record.close();
  ASSERT_TRUE(record.setPermissions(QFile::ReadOwner | QFile::WriteOwner));
  cm::Supervisor supervisor(root.path(), DESKTOP_BINARY_FROM_TEST);
  Peer client(root.path());
  const auto status = client.request({{"op", "status"}})["status"].toObject();
  EXPECT_TRUE(status["recovery_blocked"].toBool());
  EXPECT_FALSE(status["recovery_error"].toString().isEmpty());
  EXPECT_FALSE(
    client.request(
      {{"op", "start"},
        {"profile", cm::Profile::create("blocked").json()}})["ok"].toBool());
  EXPECT_FALSE(supervisor.status()["process_running"].toBool());
}
TEST(ConnectionSupervisor, CompletedMutationReceiptSurvivesIdleServiceRestart)
{
  QTemporaryDir root;
  const auto profile = cm::Profile::create("durable receipt");
  const QJsonObject request{{"op", "start"}, {"id", "durable-start"}, {"profile", profile.json()}};
  QString session;
  {
    cm::Supervisor supervisor(root.path(), DESKTOP_BINARY_FROM_TEST);
    Peer client(root.path());
    ASSERT_TRUE(client.request(request)["ok"].toBool());
    ASSERT_TRUE(until([&] {return supervisor.status()["workspace_fresh"].toBool();}));
    session = supervisor.status()["session"].toString();
    ASSERT_TRUE(client.request({{"op", "close"}, {"session", session}})["ok"].toBool());
    ASSERT_TRUE(until([&] {return !supervisor.status()["process_running"].toBool();}));
  }
  cm::Supervisor restored(root.path(), DESKTOP_BINARY_FROM_TEST);
  Peer client(root.path());
  EXPECT_TRUE(client.request(request)["ok"].toBool());
  EXPECT_EQ(restored.status()["session"].toString(), session);
  EXPECT_FALSE(restored.status()["process_running"].toBool());
}
TEST(ConnectionSupervisor, UnresolvedRecoveryRequiresOriginalProfileAndKeepsJournalBytes)
{
  QTemporaryDir root;
  struct RestoreDataHome
  {
    QByteArray old{qgetenv("XDG_DATA_HOME")};
    ~RestoreDataHome()
    {if (old.isNull()) {qunsetenv("XDG_DATA_HOME");} else {qputenv("XDG_DATA_HOME", old);}}
  } restore;
  qputenv("XDG_DATA_HOME", (root.path() + "/data").toUtf8());
  const auto runtime = root.path() + "/runtime";
  auto profile = cm::Profile::create("recover journal"); profile.mission_import = true;
  {
    cm::Supervisor service(runtime, DESKTOP_BINARY_FROM_TEST); Peer peer(runtime);
    ASSERT_TRUE(peer.request({{"op", "start"}, {"profile", profile.json()}})["ok"].toBool());
    ASSERT_TRUE(until([&] {return service.status()["workspace_fresh"].toBool();}));
    ASSERT_TRUE(
      peer.request(
        {{"op", "close"},
          {"session", service.status()["session"]}})["ok"].toBool());
    ASSERT_TRUE(until([&] {return !service.status()["process_running"].toBool();}));
  }
  QFile record(runtime + "/session.json"); ASSERT_TRUE(record.open(QIODevice::ReadOnly));
  auto data = QJsonDocument::fromJson(record.readAll()).object(); record.close();
  data["unresolved"] = true;
  ASSERT_TRUE(record.open(QIODevice::WriteOnly | QIODevice::Truncate));
  record.write(QJsonDocument(data).toJson()); record.close();
  QFile journal(root.path() + "/data/easyfly-connections/profiles/" + profile.id +
    "/imports/proof");
  ASSERT_TRUE(journal.open(QIODevice::WriteOnly)); journal.write("preserved unresolved evidence");
  journal.close();
  cm::Supervisor service(runtime, DESKTOP_BINARY_FROM_TEST); Peer peer(runtime);
  EXPECT_TRUE(service.status()["unresolved"].toBool());
  EXPECT_FALSE(
    peer.request(
      {{"op", "start"}, {"id", "reject-switch"},
        {"profile", cm::Profile::create("other").json()}})["ok"].toBool());
  ASSERT_TRUE(
    peer.request(
      {{"op", "start"}, {"id", "resume-original"},
        {"profile", profile.json()}})["ok"].toBool());
  ASSERT_TRUE(until([&] {return service.status()["workspace_fresh"].toBool();}));
  ASSERT_TRUE(journal.open(QIODevice::ReadOnly));
  EXPECT_EQ(journal.readAll(), QByteArray("preserved unresolved evidence")); journal.close();
  ASSERT_TRUE(
    peer.request(
      {{"op", "close"}, {"id", "close-resumed-journal"},
        {"session", service.status()["session"]}})["ok"].toBool());
  ASSERT_TRUE(until([&] {return !service.status()["process_running"].toBool();}));
}
TEST(ConnectionManager, SavesWithoutConnectingAndReattachesToSameLiveWorkspace)
{
  QTemporaryDir root;
  const auto runtime = root.path() + "/runtime";
  cm::Supervisor supervisor(runtime, DESKTOP_BINARY_FROM_TEST);
  const auto config = root.path() + "/profiles.json";
  cm::write_profiles(config, {cm::Profile::create("Existing platform")});
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

TEST(ConnectionProfile, SimulationSerialAndMaintenanceContracts)
{
  auto p = cm::Profile::create("Simulation"); p.mode = "sitl";
  EXPECT_EQ(cm::Profile::from_json(p.json()).hash(), p.hash());
  p.gcs_enabled = true;
  EXPECT_EQ(p.gcs_url(), "udp://127.0.0.1:14558@127.0.0.1:14560");
  EXPECT_NO_THROW(p.validate());
  p.mission_control = true; EXPECT_THROW(p.validate(), std::runtime_error);
  p.mission_control = false; p.gcs_port = 14550; EXPECT_THROW(p.validate(), std::runtime_error);
  p.gcs_port = 14560; p.mode = "communication"; p.transport = "serial";
  EXPECT_EQ(p.fcu_url(), "/dev/ttyACM0:921600"); EXPECT_NO_THROW(p.validate());
  auto value = p.json(), fcu = value["fcu"].toObject();
  fcu["baud"] = 921600.5; value["fcu"] = fcu;
  EXPECT_THROW(cm::Profile::from_json(value), std::runtime_error);
  p.serial_device = "/dev/ttyUSB0;echo bad"; EXPECT_THROW(p.validate(), std::runtime_error);
}

TEST(ConnectionProfile, LegacyProfilesMigrateWithoutStartingOrOverwriting)
{
  QTemporaryDir root;
  auto value = cm::Profile::create("Legacy DDS").json();
  for (const auto & key : {"simulation", "fcu", "gcs", "features"}) {
    value.remove(key);
  }
  QFile file(root.path() + "/connections.json"); ASSERT_TRUE(file.open(QIODevice::WriteOnly));
  file.write(
    QJsonDocument(
      QJsonObject{{"schema_version", 1}, {"profiles",
          QJsonArray{value}}}).toJson());
  file.close(); file.setPermissions(QFile::ReadOwner | QFile::WriteOwner);
  const auto read = cm::read_profiles(file.fileName()); ASSERT_EQ(read.size(), 1);
  EXPECT_EQ(read[0].mode, "observe"); EXPECT_FALSE(read[0].gcs_enabled);
  EXPECT_FALSE(read[0].mission_control); EXPECT_FALSE(read[0].mission_import);
  cm::write_profiles(file.fileName(), read); ASSERT_TRUE(file.open(QIODevice::ReadOnly));
  EXPECT_EQ(QJsonDocument::fromJson(file.readAll()).object()["schema_version"], 2);
}

TEST(ConnectionManager, SimulationAndSerialCanBeConfiguredWithoutCommandLine)
{
  QTemporaryDir root;
  cm::Supervisor supervisor(root.path() + "/runtime", DESKTOP_BINARY_FROM_TEST);
  QMainWindow host;
  cm::ManagerWindow manager(root.path() + "/profiles.json", root.path() + "/runtime", &host);
  EXPECT_FALSE(manager.isWindow()); EXPECT_EQ(manager.window(), &host);
  EXPECT_EQ(manager.menuWidget(), nullptr); EXPECT_EQ(host.menuBar()->actions().size(), 1);
  auto * save = manager.findChild<QPushButton *>("saveConnection");
  ASSERT_TRUE(until([&] {return save->isEnabled();}));
  auto * mode = manager.findChild<QComboBox *>("connectionMode");
  ASSERT_NE(mode, nullptr); EXPECT_EQ(mode->currentData(), "sitl");
  EXPECT_TRUE(manager.findChild<QPushButton *>("startConnection")->text().contains("DDS"));
  ASSERT_NE(manager.findChild<QPushButton *>("startQgc"), nullptr);
  ASSERT_NE(manager.findChild<QPushButton *>("stopBackend"), nullptr);
  auto * ellipse = manager.findChild<QCheckBox *>("algorithm_ellipse_enabled");
  auto * strategy = manager.findChild<QComboBox *>("algorithm_strategy");
  ASSERT_NE(ellipse, nullptr); ASSERT_NE(strategy, nullptr);
  ellipse->setChecked(true); strategy->setCurrentIndex(strategy->findData("ego"));
  QTest::mouseClick(save, Qt::LeftButton);
  const auto algorithms = cm::read_profiles(root.path() + "/profiles.json");
  ASSERT_EQ(algorithms.size(), 1);
  EXPECT_TRUE(algorithms[0].algorithms["ellipse_enabled"].toBool());
  EXPECT_EQ(algorithms[0].algorithms["strategy"], "ego");
  EXPECT_FALSE(supervisor.status()["backend_running"].toBool());
  mode->setCurrentIndex(mode->findData("communication"));
  EXPECT_FALSE(ellipse->isEnabled());
  auto * transport = manager.findChild<QComboBox *>("connectionTransport");
  transport->setCurrentIndex(transport->findData("serial"));
  auto * device = manager.findChild<QComboBox *>("connectionSerialDevice");
  EXPECT_TRUE(device->isEnabled()); device->setCurrentText("/dev/ttyUSB7");
  manager.findChild<QComboBox *>("connectionBaud")->setCurrentText("57600");
  QTest::mouseClick(save, Qt::LeftButton);
  const auto saved = cm::read_profiles(root.path() + "/profiles.json"); ASSERT_EQ(saved.size(), 1);
  EXPECT_EQ(saved[0].transport, "serial"); EXPECT_EQ(saved[0].baud, 57600);
  EXPECT_FALSE(supervisor.status()["backend_running"].toBool());
  if (qEnvironmentVariableIsSet("EASYFLY_CM_FORM_SCREENSHOT")) {
    ASSERT_TRUE(manager.grab().save(qEnvironmentVariable("EASYFLY_CM_FORM_SCREENSHOT")));
  }
}
TEST(ConnectionManager, RecoveredBackendReopensFrozenProfileWithoutOverwritingAnotherSavedProfile)
{
  QTemporaryDir root;
  const auto runtime = cm::private_directory(root.path() + "/runtime");
  const auto saved = cm::Profile::create("saved draft");
  auto frozen = cm::Profile::create("recovered frozen"); frozen.mode = "sitl"; frozen.domain = 99;
  const auto file = root.path() + "/profiles.json";
  cm::write_profiles(file, {saved});
  QLocalServer service;
  ASSERT_TRUE(service.listen(runtime + "/control.sock"));
  QJsonObject requested;
  QObject::connect(
    &service, &QLocalServer::newConnection, &service, [&] {
      auto * peer = service.nextPendingConnection();
      auto input = std::make_shared<QByteArray>();
      QObject::connect(peer, &QLocalSocket::disconnected, peer, &QObject::deleteLater);
      QObject::connect(
        peer, &QLocalSocket::readyRead, peer, [&, peer, input] {
          input->append(peer->readAll());
          while (input->contains('\n')) {
            const auto end = input->indexOf('\n');
            const auto query = QJsonDocument::fromJson(input->left(end)).object();
            input->remove(0, end + 1);
            if (query["op"] == "start") {requested = query["profile"].toObject();}
            const QJsonObject status{{"session", "recovered"}, {"backend_running", true},
              {"backend_fresh", true}, {"profile", frozen.json()}, {"config_hash", frozen.hash()},
              {"process_running", !requested.isEmpty()}};
            peer->write(
              QJsonDocument(
                QJsonObject{{"id", query["id"]}, {"ok", true},
                  {"status", status}}).toJson(QJsonDocument::Compact) + '\n');
          }
        });
    });
  cm::ManagerWindow manager(file, runtime);
  auto * start = manager.findChild<QPushButton *>("startConnection");
  ASSERT_TRUE(until([&] {return start->isEnabled();}));
  EXPECT_EQ(manager.findChild<QLineEdit *>("connectionName")->text(), saved.name);
  EXPECT_TRUE(
    manager.findChild<QLabel *>("connectionEffectiveConfig")->text().contains(frozen.name));
  QTest::mouseClick(start, Qt::LeftButton);
  ASSERT_TRUE(until([&] {return !requested.isEmpty();}));
  EXPECT_EQ(requested, frozen.json());
  EXPECT_EQ(cm::read_profiles(file)[0].json(), saved.json());
}

TEST(ConnectionProfile, AlgorithmConfigurationIsFrozenAndLegacyHashesRemainStable)
{
  auto p = cm::Profile::create("algorithms");
  EXPECT_FALSE(p.json().contains("algorithms"));
  const auto legacy_hash = p.hash();
  EXPECT_EQ(cm::Profile::from_json(p.json()).hash(), legacy_hash);
  p.mode = "sitl"; p.algorithms["ellipse_enabled"] = true;
  p.algorithms["strategy"] = "ego"; p.algorithms["camera"] = "down";
  const auto restored = cm::Profile::from_json(p.json());
  EXPECT_EQ(restored.algorithms, p.algorithms); EXPECT_EQ(restored.hash(), p.hash());
  EXPECT_NE(restored.hash(), legacy_hash);
  p.algorithms["morphology_kernel"] = 2;
  EXPECT_THROW(p.validate(), std::runtime_error);
  p.algorithms["morphology_kernel"] = 3; p.down_topic.clear();
  EXPECT_THROW(p.validate(), std::runtime_error);
  p.down_topic = "/image"; p.algorithms["provider"] = "not_installed";
  EXPECT_THROW(p.validate(), std::runtime_error);
}

TEST(ConnectionManager, CompactConfigurationKeepsActionsVisibleAndSwitchCancellationConsistent)
{
  QTemporaryDir root;
  cm::Supervisor supervisor(root.path() + "/runtime", DESKTOP_BINARY_FROM_TEST);
  const auto file = root.path() + "/profiles.json";
  cm::write_profiles(file, {cm::Profile::create("First"), cm::Profile::create("Second")});
  cm::ManagerWindow manager(file, root.path() + "/runtime"); manager.show();
  auto * save = manager.findChild<QPushButton *>("saveConnection");
  ASSERT_TRUE(until([&] {return save->isEnabled();}));
  auto * selector = manager.findChild<QComboBox *>("connectionSelector");
  ASSERT_NE(selector, nullptr); EXPECT_EQ(selector->count(), 2);
  EXPECT_FALSE(manager.findChild<QDialog *>("connectionProfilesDialog")->isVisible());
  auto * tabs = manager.findChild<QTabWidget *>("connectionConfigurationTabs");
  for (const auto & size : {QSize(1200, 650), QSize(850, 570)}) {
    manager.resize(size); QTest::qWait(40);
    for (int i = 0; i < tabs->count(); ++i) {
      tabs->setCurrentIndex(i); QTest::qWait(20);
      auto * area = qobject_cast<QScrollArea *>(tabs->currentWidget()); ASSERT_NE(area, nullptr);
      EXPECT_EQ(area->horizontalScrollBar()->maximum(), 0);
      for (const auto * id :
        {"saveConnection", "startConnection", "closeConnection", "stopBackend"})
      {
        auto * action = manager.findChild<QPushButton *>(id); ASSERT_NE(action, nullptr);
        const QRect bounds(action->mapTo(&manager, QPoint()), action->size());
        EXPECT_TRUE(manager.rect().contains(bounds));
        for (auto * ancestor = action->parentWidget(); ancestor;
          ancestor = ancestor->parentWidget())
        {
          EXPECT_EQ(qobject_cast<QScrollArea *>(ancestor), nullptr);
        }
      }
    }
  }
  manager.findChild<QLineEdit *>("connectionName")->setText("Unsaved");
  QTimer::singleShot(
    0, &manager, [&manager] {
      auto * prompt = manager.findChild<QMessageBox *>();
      if (prompt) {prompt->done(QMessageBox::No);}
    });
  selector->setCurrentIndex(1);
  EXPECT_EQ(selector->currentIndex(), 0);
  EXPECT_EQ(manager.findChild<QListWidget *>("connectionList")->currentRow(), 0);
  EXPECT_EQ(manager.findChild<QLineEdit *>("connectionName")->text(), "Unsaved");
  save->click(); selector->setCurrentIndex(1);
  EXPECT_EQ(manager.findChild<QLineEdit *>("connectionName")->text(), "Second");
  EXPECT_FALSE(supervisor.status()["process_running"].toBool());
  EXPECT_FALSE(supervisor.status()["backend_running"].toBool());
}

TEST(ConnectionProfile, SlamDefaultsPreserveLegacyHashesAndCustomSettingsRoundTrip)
{
  cm::Profile profile; profile.id = "00000000-0000-0000-0000-000000000001";
  profile.name = "Legacy algorithms"; profile.mode = "sitl";
  profile.selected_profile = "navigation_dev";
  EXPECT_EQ(profile.hash(), "e558f62fadef6628125749d628bac7aaa3bc414d7bdd987d5c169076020e789d");
  profile.algorithms["ellipse_enabled"] = true;
  EXPECT_EQ(profile.hash(), "d59686845f4e51ef2f4171dd9346537abb6681099a9f9e537e1a81a32febe6c9");
  auto legacy = profile.json();
  EXPECT_FALSE(legacy["algorithms"].toObject().contains("slam"));
  auto restored = cm::Profile::from_json(legacy);
  EXPECT_EQ(restored.algorithms["slam"], "profile"); EXPECT_EQ(restored.hash(), profile.hash());
  restored.algorithms["slam"] = "fast_lio";
  restored.algorithms["slam_filter_size_map"] = .25;
  EXPECT_EQ(cm::Profile::from_json(restored.json()).algorithms, restored.algorithms);
  restored.selected_profile = "base_sim"; EXPECT_THROW(restored.validate(), std::runtime_error);
  restored.selected_profile = "navigation_dev";
  restored.algorithms["slam_imu_topic"] = "/bad;topic";
  EXPECT_THROW(restored.validate(), std::runtime_error);
}

TEST(ConnectionManager, SlamCanBeConfiguredAndSavedWithoutStartingAnyBackend)
{
  QTemporaryDir root;
  cm::Supervisor supervisor(root.path() + "/runtime", DESKTOP_BINARY_FROM_TEST);
  cm::ManagerWindow manager(root.path() + "/profiles.json", root.path() + "/runtime");
  manager.show(); auto * save = manager.findChild<QPushButton *>("saveConnection");
  ASSERT_TRUE(until([&] {return save->isEnabled();}));
  auto * slam = manager.findChild<QComboBox *>("algorithm_slam"); ASSERT_NE(slam, nullptr);
  EXPECT_EQ(slam->currentData(), "profile");
  auto * profile = manager.findChild<QComboBox *>("connectionProfile");
  profile->setCurrentIndex(profile->findData("navigation_dev"));
  slam->setCurrentIndex(slam->findData("fast_lio"));
  manager.findChild<QLineEdit *>("algorithm_slam_lidar_topic")->setText("/scanner/cloud");
  manager.findChild<QLineEdit *>("algorithm_slam_imu_topic")->setText("/scanner/imu");
  manager.findChild<QDoubleSpinBox *>("algorithm_slam_filter_size_map")->setValue(.25);
  save->click();
  const auto saved = cm::read_profiles(root.path() + "/profiles.json"); ASSERT_EQ(saved.size(), 1);
  EXPECT_EQ(saved[0].algorithms["slam"], "fast_lio");
  EXPECT_EQ(saved[0].algorithms["slam_lidar_topic"], "/scanner/cloud");
  EXPECT_DOUBLE_EQ(saved[0].algorithms["slam_filter_size_map"].toDouble(), .25);
  slam->setCurrentIndex(slam->findData("disabled"));
  EXPECT_TRUE(manager.findChild<QLabel *>("slamConfigurationSummary")->text().contains("不启动"));
  save->click();
  EXPECT_EQ(cm::read_profiles(root.path() + "/profiles.json")[0].algorithms["slam"], "disabled");
  EXPECT_FALSE(supervisor.status()["backend_running"].toBool());
  EXPECT_FALSE(supervisor.status()["process_running"].toBool());
}


TEST(Connections, OpenvinsSelectionIsDistinctFixedAndPreservedInEnvironment)
{
  auto profile = cm::Profile::create("openvins");
  profile.mode = "sitl"; profile.selected_profile = "navigation_dev";
  const auto previous = profile.hash();
  profile.algorithms["slam"] = "openvins";
  EXPECT_NO_THROW(profile.validate());
  EXPECT_NE(profile.hash(), previous);
  EXPECT_EQ(
    cm::session_environment(profile, "session").value("EASYFLY_LOCALIZATION_BACKEND"),
    "openvins");
  EXPECT_EQ(cm::Profile::from_json(profile.json()).algorithms["slam"], "openvins");
  profile.algorithms["slam_imu_topic"] = "/custom/imu";
  EXPECT_THROW(profile.validate(), std::exception);
  profile.algorithms["slam_imu_topic"] = "/livox/imu";
  profile.selected_profile = "base_sim";
  EXPECT_THROW(profile.validate(), std::exception);
}

TEST(Connections, FixedCalibrationAndFuelSelectionsAreValidatedBeforeStart)
{
  auto profile = cm::Profile::create("Algorithm admission");
  profile.mode = "sitl"; profile.selected_profile = "navigation_dev";
  for (const auto & slam : QStringList{"openvins", "fast_livo2", "orbslam3"}) {
    profile.algorithms = cm::algorithm_defaults(); profile.algorithms["slam"] = slam;
    EXPECT_NO_THROW(profile.validate());
    EXPECT_EQ(cm::Profile::from_json(profile.json()).algorithms, profile.algorithms);
    EXPECT_EQ(
      cm::session_environment(profile, "session").value(
        "EASYFLY_LOCALIZATION_BACKEND"), slam);
    profile.algorithms["slam_max_iteration"] = 5;
    EXPECT_THROW(profile.validate(), std::exception);
  }
  profile.algorithms = cm::algorithm_defaults();
  profile.algorithms["exploration"] = "fuel";
  EXPECT_THROW(profile.validate(), std::exception);
  profile.algorithms["strategy"] = "gcopter";
  EXPECT_NO_THROW(profile.validate());
  profile.algorithms["slam"] = "fast_lio";
  EXPECT_NO_THROW(profile.validate());
  profile.algorithms["slam"] = "orbslam3";
  EXPECT_THROW(profile.validate(), std::exception);
  profile.algorithms["slam"] = "profile";
  profile.selected_profile = "full_perception";
  EXPECT_THROW(profile.validate(), std::exception);
}

TEST(ConnectionManager, FixedCalibrationSwitchPreservesDraftAndSavesApplicableParameters)
{
  QTemporaryDir root;
  cm::Supervisor supervisor(root.path() + "/runtime", DESKTOP_BINARY_FROM_TEST);
  const auto file = root.path() + "/profiles.json";
  auto p = cm::Profile::create("Customized FAST-LIO");
  p.mode = "sitl"; p.selected_profile = "navigation_dev";
  p.algorithms["slam"] = "fast_lio"; p.algorithms["slam_imu_topic"] = "/custom/imu";
  cm::write_profiles(file, {p});
  for (const auto & choice : QStringList{"fast_livo2", "orbslam3", "openvins"}) {
    cm::ManagerWindow manager(file, root.path() + "/runtime");
    manager.show(); auto * save = manager.findChild<QPushButton *>("saveConnection");
    ASSERT_TRUE(until([&] {return save->isEnabled();}));
    auto * slam = manager.findChild<QComboBox *>("algorithm_slam");
    auto * imu = manager.findChild<QLineEdit *>("algorithm_slam_imu_topic");
    slam->setCurrentIndex(slam->findData("fast_lio"));
    imu->setText("/custom/imu"); ASSERT_TRUE(imu->isEnabled());
    slam->setCurrentIndex(slam->findData(choice)); EXPECT_FALSE(imu->isEnabled());
    EXPECT_TRUE(
      manager.findChild<QLabel *>("slamConfigurationSummary")->text().contains(
        "自定义参数不随本配置保存"));
    slam->setCurrentIndex(slam->findData("fast_lio"));
    EXPECT_TRUE(imu->isEnabled()); EXPECT_EQ(imu->text(), "/custom/imu");
    slam->setCurrentIndex(slam->findData(choice)); save->click();
    const auto saved = cm::read_profiles(file); ASSERT_EQ(saved.size(), 1);
    EXPECT_EQ(saved[0].algorithms["slam"], choice);
    EXPECT_EQ(saved[0].algorithms["slam_imu_topic"], "/livox/imu");
    EXPECT_NO_THROW(saved[0].validate());
    if (qEnvironmentVariableIsSet("EASYFLY_CM_SLAM_SCREENSHOTS")) {
      auto * tabs = manager.findChild<QTabWidget *>(); tabs->setCurrentIndex(1);
      manager.resize(850, 570); QCoreApplication::processEvents();
      auto * area = qobject_cast<QScrollArea *>(tabs->currentWidget());
      area->ensureWidgetVisible(manager.findChild<QLabel *>("slamConfigurationSummary"), 0, 10);
      QCoreApplication::processEvents();
      EXPECT_EQ(area->horizontalScrollBar()->maximum(), 0);
      ASSERT_TRUE(
        manager.grab().save(
          qEnvironmentVariable("EASYFLY_CM_SLAM_SCREENSHOTS") + "/" + choice + ".png"));
    }
    cm::ManagerWindow reopened(file, root.path() + "/runtime");
    EXPECT_EQ(reopened.findChild<QComboBox *>("algorithm_slam")->currentData(), choice);
    EXPECT_EQ(reopened.findChild<QLineEdit *>("algorithm_slam_imu_topic")->text(), "/livox/imu");
    EXPECT_FALSE(supervisor.status()["backend_running"].toBool());
    EXPECT_FALSE(supervisor.status()["process_running"].toBool());
  }
}

TEST(Connections, LegacyFixedCalibrationFileMigratesWithoutChangingFrozenPayloads)
{
  QTemporaryDir root;
  for (const auto & slam : QStringList{"fast_livo2", "orbslam3"}) {
    auto profile = cm::Profile::create("Old incompatible form");
    profile.mode = "sitl"; profile.selected_profile = "navigation_dev";
    profile.algorithms["slam"] = slam;
    profile.algorithms["slam_max_iteration"] = 5;
    const auto payload = profile.json();
    EXPECT_THROW(cm::Profile::from_json(payload), std::exception);
    const auto bytes = QJsonDocument(
      QJsonObject{{"schema_version", 2},
        {"profiles", QJsonArray{payload}}}).toJson();
    QFile file(root.path() + "/legacy.json"); ASSERT_TRUE(file.open(QIODevice::WriteOnly));
    file.setPermissions(QFile::ReadOwner | QFile::WriteOwner); file.write(bytes); file.close();
    const auto loaded = cm::read_profiles(file.fileName()); ASSERT_EQ(loaded.size(), 1);
    EXPECT_EQ(loaded[0].algorithms["slam"], slam);
    EXPECT_EQ(
      loaded[0].algorithms["slam_max_iteration"],
      cm::algorithm_defaults()["slam_max_iteration"]);
    ASSERT_TRUE(file.open(QIODevice::ReadOnly)); EXPECT_EQ(file.readAll(), bytes); file.close();
    cm::write_profiles(file.fileName(), loaded);
    EXPECT_EQ(cm::read_profiles(file.fileName())[0].hash(), loaded[0].hash());
  }
}

TEST(ConnectionManager, SceneAdmissionTracksProfileDraftAndMissingSelection)
{
  QTemporaryDir root;
  cm::Supervisor supervisor(root.path() + "/runtime", DESKTOP_BINARY_FROM_TEST);
  auto p = cm::Profile::create("Base scenes"); p.mode = "sitl";
  p.selected_profile = "base_sim";
  const auto file = root.path() + "/profiles.json"; cm::write_profiles(file, {p});
  cm::ManagerWindow manager(file, root.path() + "/runtime");
  auto * scenes = manager.findChild<QComboBox *>("connectionScene");
  auto * profile = manager.findChild<QComboBox *>("connectionProfile");
  auto * status = manager.findChild<QLabel *>("connectionSceneStatus");
  auto * start = manager.findChild<QPushButton *>("startConnection");
  ASSERT_TRUE(until([&] {return scenes->findData("empty") >= 0;}, 15000));
  ASSERT_TRUE(until([&] {return start->isEnabled();}));
  EXPECT_TRUE(scenes->currentText().contains(QStringLiteral(" · 可用")));
  profile->setCurrentIndex(profile->findData("full_perception"));
  EXPECT_FALSE(start->isEnabled());
  EXPECT_TRUE(status->text().contains(QStringLiteral("不支持")));
  EXPECT_TRUE(status->text().contains(QStringLiteral("基础仿真")));
  profile->setCurrentIndex(profile->findData("base_sim"));
  EXPECT_TRUE(start->isEnabled());
  scenes->setCurrentIndex(scenes->findData("empty"));
  EXPECT_FALSE(start->isEnabled()); EXPECT_TRUE(status->text().contains(QStringLiteral("草稿")));
  scenes->addItem("removed_scene", "removed_scene");
  scenes->setCurrentIndex(scenes->count() - 1);
  EXPECT_FALSE(start->isEnabled());
  EXPECT_TRUE(status->text().contains(QStringLiteral("不在已安装目录")));
  manager.findChild<QPushButton *>("refreshScenes")->click();
  EXPECT_FALSE(start->isEnabled());
  ASSERT_TRUE(
    until(
      [&] {return status->text().contains(QStringLiteral("不在已安装目录"));}, 15000));
  EXPECT_EQ(scenes->currentData(), "removed_scene");
  EXPECT_FALSE(start->isEnabled());
  EXPECT_FALSE(supervisor.status()["backend_running"].toBool());
  EXPECT_FALSE(supervisor.status()["process_running"].toBool());
}

TEST(ConnectionProfile, PeripheralOptInIsIndependentAndPreservesLegacyProfileHash)
{
  auto p = cm::Profile::create("Peripheral simulation"); p.mode = "sitl";
  const auto legacy = p.json(); const auto hash = p.hash();
  EXPECT_FALSE(legacy["features"].toObject().contains("peripheral_control"));
  EXPECT_EQ(cm::Profile::from_json(legacy).hash(), hash);
  p.peripheral_control = true; EXPECT_NO_THROW(p.validate());
  EXPECT_FALSE(p.mission_control); EXPECT_TRUE(cm::Profile::from_json(p.json()).peripheral_control);
  EXPECT_NE(p.hash(), hash);
  p.peripheral_control = false; EXPECT_EQ(p.hash(), hash);
  p.peripheral_control = true; p.mode = "observe"; EXPECT_THROW(p.validate(), std::runtime_error);
  p.mode = "sitl"; p.gcs_enabled = true; EXPECT_THROW(p.validate(), std::runtime_error);
  p.gcs_enabled = false; p.simulation_time = false; EXPECT_THROW(p.validate(), std::runtime_error);
  auto bad = legacy; auto features = bad["features"].toObject();
  features["peripheral_control"] = "true";
  bad["features"] = features; EXPECT_THROW(cm::Profile::from_json(bad), std::runtime_error);
}
