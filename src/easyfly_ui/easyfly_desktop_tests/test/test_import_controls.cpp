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
// Platform integration fixture; kept outside the desktop's minimal dependency closure.
#include <gtest/gtest.h>
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QProcess>
#include <QPushButton>
#include <QScrollArea>
#include <QTemporaryDir>
#include <QTest>
#include <cstdlib>
#include <atomic>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <easyfly_msgs/action/execute_mission.hpp>
#include <easyfly_msgs/srv/import_mission.hpp>
#include <easyfly_mission_format/mission_format.hpp>
#include "easyfly_desktop/import_controls.hpp"
#include "support.hpp"
namespace ui = easyfly_desktop;
namespace c = easyfly_ui_control;
namespace
{
constexpr char body[] = "# import only, never fly\nschema_version: 1\nframe_id: odom\n"
  "units: {position: m, angle: rad, time: s}\nsteps:\n- type: hover\n  duration: 2\n";
ui::ExportedDraft draft()
{return {1, "new.yaml", body, easyfly_mission_fsm::missionContentSha256(body)};}
struct State
{
  c::ImportSnapshot data;
  int creates{0}, reads{0}, stops{0}; bool inhibited{false};
  std::set<std::string> ids;
  State() {data.ready = true;}
};
class FakeImport final : public ui::ImportPort
{
public:
  explicit FakeImport(std::shared_ptr<State> state)
  : state_(state) {}
  std::shared_ptr<const c::ImportSnapshot> snapshot() const override
  {return std::make_shared<c::ImportSnapshot>(state_->data);}
  bool read_destination() override
  {
    if (state_->inhibited || state_->data.fault) {return false;}
    ++state_->reads;
    state_->data.destination = c::ImportDestination{std::string(32, 'a'), std::string(32, 'b'),
      "/tmp/mock-catalog", true, 256, 0, static_cast<double>(state_->reads)}; return true;
  }
  bool import_confirmed(
    const c::ImportDocument & doc, const c::ImportDestination & d,
    const std::string & id) override
  {
    if (state_->inhibited || state_->data.fault || state_->ids.count(id)) {return false;}
    state_->ids.insert(id); ++state_->creates;
    state_->data.intent = c::ImportIntent{"domain=129;localhost=1", d.instance, d.catalog,
      std::string(32, 'c'), doc.name, doc.hash, "imported", true, true};
    state_->data.phase = c::ImportPhase::Completed; return true;
  }
  void inhibit(bool active) override {state_->inhibited = active;}
  void stop() override {++state_->stops; state_->data.stopped = true;}

private:
  std::shared_ptr<State> state_;
};
ui::ImportFactory factory(std::shared_ptr<State> s)
{return [s](const auto &) {return std::make_unique<FakeImport>(s);};}
void confirm(ui::ImportPanel & panel, int answer = QMessageBox::Yes)
{
  auto * prompt = panel.findChild<QMessageBox *>("importPrompt"); ASSERT_NE(prompt, nullptr);
  EXPECT_EQ(prompt->defaultButton(), prompt->button(QMessageBox::Cancel)); prompt->done(answer);
}
class OwnedProcess final : public QProcess
{
public:
  ~OwnedProcess() override
  {
    if (state() == QProcess::NotRunning) {return;} terminate();
    if (!waitForFinished(2000)) {kill(); waitForFinished(2000);}
  }
};
}  // namespace
TEST(ImportPanel, DefaultWindowNeverCreatesImporterOrWriteButton)
{
  int creations = 0; auto observation = std::make_shared<desktop_test::FakeState>();
  ui::MainWindow window({}, desktop_test::factory(observation), nullptr, ui::default_tool_options(),
    {}, ui::control_factory(), {}, ui::parameter_factory(), {},
    [&](const auto &) -> std::unique_ptr<ui::ImportPort> {++creations; return {};});
  EXPECT_EQ(creations, 0); EXPECT_EQ(window.findChild<QPushButton *>("importCreate"), nullptr);
  window.close(); EXPECT_TRUE(desktop_test::until([&] {return window.cleanup_complete();}));
}
TEST(ImportPanel, CancelAndRetiredConfirmationCannotSubmit)
{
  auto s = std::make_shared<State>();
  ui::ImportPanel panel({true, "/tmp/mock-journal"}, factory(s));
  panel.observe(draft(), true);
  auto * create = panel.findChild<QPushButton *>("importCreate"); EXPECT_FALSE(create->isEnabled());
  panel.findChild<QPushButton *>("importReadDestination")->click(); create->click();
  auto * old = panel.findChild<QMessageBox *>("importPrompt"); ASSERT_NE(old, nullptr);
  confirm(panel, QMessageBox::Cancel); EXPECT_EQ(s->creates, 0);
  create->click(); auto * next = panel.findChild<QMessageBox *>("importPrompt"); ASSERT_NE(
    next,
    nullptr);
  old->done(QMessageBox::Yes); EXPECT_EQ(s->creates, 0);
  EXPECT_EQ(panel.findChild<QMessageBox *>("importPrompt"), next);
  confirm(panel); EXPECT_EQ(s->creates, 1); next->done(QMessageBox::Yes); EXPECT_EQ(s->creates, 1);
  panel.stop();
}
TEST(ImportPanel, LiveEditorRevisionIsCheckedBeforeTimerRefresh)
{
  auto s = std::make_shared<State>(); std::optional<ui::ExportedDraft> live = draft();
  ui::ImportPanel panel({true, "/tmp/mock-journal"}, factory(s), nullptr, [&] {return live;});
  panel.observe(live, true); panel.findChild<QPushButton *>("importReadDestination")->click();
  panel.findChild<QPushButton *>("importCreate")->click();
  live->revision++; confirm(panel); EXPECT_EQ(s->creates, 0);
  panel.observe(live, true); panel.findChild<QPushButton *>("importCreate")->click();
  live.reset(); confirm(panel); EXPECT_EQ(s->creates, 0); panel.stop();
}
TEST(ImportPanel, DisconnectInvalidDraftDisabledCatalogAndCapacityFailClosed)
{
  auto s = std::make_shared<State>();
  ui::ImportPanel panel({true, "/tmp/mock-journal"}, factory(s));
  panel.observe({}, true); panel.findChild<QPushButton *>("importReadDestination")->click();
  auto * create = panel.findChild<QPushButton *>("importCreate"); EXPECT_FALSE(create->isEnabled());
  panel.observe(draft(), true); create->click();
  auto * old = panel.findChild<QMessageBox *>("importPrompt"); ASSERT_NE(old, nullptr);
  panel.observe(draft(), false); old->done(QMessageBox::Yes); EXPECT_EQ(s->creates, 0);
  s->data.destination->enabled = false; panel.observe(draft(), true); EXPECT_FALSE(
    create->isEnabled());
  s->data.destination->enabled = true; s->data.confirmations = 256;
  panel.observe(draft(), true); EXPECT_FALSE(create->isEnabled()); panel.stop();
}
TEST(ImportPanel, UnresolvedExitKeepsJournalAndDefaultsToStayEvenWhenDisconnected)
{
  auto s = std::make_shared<State>();
  auto observation = std::make_shared<desktop_test::FakeState>();
  ui::MainWindow window({}, desktop_test::factory(observation), nullptr, ui::default_tool_options(),
    {}, ui::control_factory(), {}, ui::parameter_factory(), {true, "/tmp/mock-journal"},
    factory(s));
  window.show();
  s->data.active_create = true; window.close(); EXPECT_EQ(s->stops, 0);
  EXPECT_EQ(window.findChild<QMessageBox *>("importExitPrompt"), nullptr);
  s->data.active_create = false; s->data.fault = true;
  window.findChild<QPushButton *>("observerToggle")->click();
  ASSERT_TRUE(desktop_test::until([&] {return window.cleanup_complete();}));
  window.close(); auto * prompt = window.findChild<QMessageBox *>("importExitPrompt"); ASSERT_NE(
    prompt, nullptr);
  EXPECT_EQ(prompt->defaultButton(), prompt->button(QMessageBox::Cancel)); prompt->done(
    QMessageBox::Cancel);
  EXPECT_TRUE(window.isVisible()); EXPECT_EQ(s->stops, 0); EXPECT_TRUE(s->inhibited);
  window.close(); prompt = window.findChild<QMessageBox *>("importExitPrompt"); ASSERT_NE(
    prompt,
    nullptr);
  prompt->done(QMessageBox::Yes); ASSERT_TRUE(
    desktop_test::until(
      [&] {
        return window.cleanup_complete();
      }));
  EXPECT_EQ(s->stops, 1); EXPECT_EQ(s->creates, 0);
}
TEST(ImportStack, ExportedDraftRealServiceReadbackAndReadOnlyCatalogView)
{
  ASSERT_STREQ(std::getenv("ROS_DOMAIN_ID"), "129"); ASSERT_STREQ(
    std::getenv(
      "ROS_LOCALHOST_ONLY"), "1");
  QTemporaryDir directory; ASSERT_TRUE(directory.isValid());
  ASSERT_TRUE(QDir().mkdir(directory.path() + "/catalog"));
  ASSERT_TRUE(
    QFile::setPermissions(
      directory.path() + "/catalog",
      QFile::ReadOwner | QFile::WriteOwner | QFile::ExeOwner));
  OwnedProcess process;
  process.start(
    QString::fromStdString(ament_index_cpp::get_package_prefix("easyfly_mission_srvs")) +
    "/lib/easyfly_mission_srvs/mission_services", {"--ros-args", "-p",
      "mission_directory:=" + directory.path() + "/catalog", "-p", "enable_mission_import:=true"});
  ASSERT_TRUE(process.waitForStarted(2000));
  easyfly_ui_bridge::BridgeConfig config; config.use_sim_time = false;
  config.selected_profile = "base_sim";
  ui::MainWindow window(config, ui::bridge_factory(), nullptr, ui::default_tool_options(), {},
    ui::control_factory(), {}, ui::parameter_factory(),
    {true, (directory.path() + "/journal").toStdString()});
  window.show(); window.findChild<QListWidget *>("navigation")->setCurrentRow(6);
  auto * editor = window.findChild<ui::MissionEditor *>(); ASSERT_NE(editor, nullptr);
  editor->prepare_directory(directory.path() + "/drafts");
  ASSERT_TRUE(
    desktop_test::until(
      [&] {
        return !editor->busy() &&
        window.findChild<QLineEdit *>("draftDirectory")->text() == directory.path() + "/drafts" &&
        QFile::exists(directory.path() + "/drafts/.easyfly-drafts-v1");
      }));
  window.findChild<QPlainTextEdit *>("draftBody")->setPlainText(QString::fromUtf8(body));
  ASSERT_TRUE(
    desktop_test::until(
      [&] {
        return window.findChild<QPushButton *>("draftExport")->isEnabled();
      }));
  window.findChild<QLineEdit *>("draftFilename")->setText("new.yaml"); editor->export_file(
    "new.yaml");
  ASSERT_TRUE(desktop_test::until([&] {return editor->exported_draft().has_value();}));
  auto * panel = window.findChild<ui::ImportPanel *>(); ASSERT_NE(panel, nullptr);
  auto * read = window.findChild<QPushButton *>("importReadDestination");
  auto * create = window.findChild<QPushButton *>("importCreate");
  ASSERT_TRUE(
    desktop_test::until(
      [&] {
        if (read->isEnabled() && !create->isEnabled()) {read->click();} return create->isEnabled();
      }, 7000));
  create->click(); confirm(*panel);
  ASSERT_TRUE(
    desktop_test::until(
      [&] {
        return window.findChild<QLabel *>("importStatus")->property("importPhase").toInt() ==
        static_cast<int>(c::ImportPhase::Completed);
      }, 6000));
  QFile published(directory.path() + "/catalog/new.yaml");
  ASSERT_TRUE(published.open(QIODevice::ReadOnly));
  EXPECT_EQ(published.readAll(), QByteArray(body)); EXPECT_FALSE(panel->unresolved());
  window.findChild<QLabel *>("requestNote")->setText(
    QStringLiteral(
      "独立domain129 · 文件导入测试 · 无飞行节点/未执行"));
  auto * ancestor = panel->parentWidget();
  while (ancestor && !qobject_cast<QScrollArea *>(ancestor)) {ancestor = ancestor->parentWidget();}
  ASSERT_NE(ancestor, nullptr); qobject_cast<QScrollArea *>(ancestor)->ensureWidgetVisible(
    panel, 5,
    5);
  QTest::qWait(100); QDir().mkpath(EASYFLY_IMPORT_SCREENSHOT_DIRECTORY);
  EXPECT_TRUE(
    window.grab().save(
      QString(EASYFLY_IMPORT_SCREENSHOT_DIRECTORY) +
      "/import-isolated.png"));
  window.findChild<QListWidget *>("navigation")->setCurrentRow(1);
  auto * list = window.findChild<QPushButton *>("listMissions"); ASSERT_NE(list, nullptr);
  list->click();
  ASSERT_TRUE(
    desktop_test::until(
      [&] {
        return window.findChild<QListWidget *>("missionList")->count() == 1;
      }, 5000));
  EXPECT_EQ(window.findChild<QListWidget *>("missionList")->item(0)->text(), "new.yaml");
  window.findChild<QListWidget *>("missionList")->setCurrentRow(0);
  auto * inspect = window.findChild<QPushButton *>("inspectMission");
  ASSERT_TRUE(desktop_test::until([&] {return inspect->isEnabled();})); inspect->click();
  ASSERT_TRUE(
    desktop_test::until(
      [&] {
        return window.findChild<QPlainTextEdit *>("missionContent")->toPlainText() ==
        QString::fromUtf8(body);
      }, 5000));
  window.findChild<QPlainTextEdit *>("draftBody")->appendPlainText("# unapplied new revision");
  EXPECT_FALSE(editor->exported_draft());
  EXPECT_EQ(window.findChild<QPushButton *>("managedStart"), nullptr);
  window.close();
  auto * discard = window.findChild<QMessageBox *>("draftExitPrompt"); ASSERT_NE(discard, nullptr);
  EXPECT_EQ(discard->defaultButton(), discard->button(QMessageBox::Cancel)); discard->done(
    QMessageBox::Yes);
  ASSERT_TRUE(desktop_test::until([&] {return window.cleanup_complete();}));
}
// C04: actual editor/importer/observer/service, isolated private files, no flight backend.
class ImportChain : public ::testing::Test
{
protected:
  using Api = easyfly_msgs::srv::ImportMission;
  using Action = easyfly_msgs::action::ExecuteMission;
  QTemporaryDir directory;
  OwnedProcess process;
  std::unique_ptr<ui::MainWindow> window;
  ui::Source * observed_source{nullptr};
  std::shared_ptr<rclcpp::Context> context;
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  rclcpp::Client<Api>::SharedPtr forward;
  rclcpp::Service<Api>::SharedPtr tap;
  rclcpp_action::Server<Action>::SharedPtr guard;
  std::thread thread;
  std::mutex mutex;
  std::vector<Api::Request> requests;
  std::vector<Api::Response> replies;
  std::atomic<int> goals{0};
  void SetUp() override
  {
    desktop_test::application(); ASSERT_TRUE(directory.isValid());
    ASSERT_STREQ(std::getenv("ROS_DOMAIN_ID"), "129");
    ASSERT_STREQ(std::getenv("ROS_LOCALHOST_ONLY"), "1");
    ASSERT_TRUE(QDir().mkdir(directory.path() + "/catalog"));
    ASSERT_TRUE(
      QFile::setPermissions(
        directory.path() + "/catalog",
        QFile::ReadOwner | QFile::WriteOwner | QFile::ExeOwner));
    context = std::make_shared<rclcpp::Context>(); context->init(0, nullptr);
    rclcpp::NodeOptions options; options.context(context).use_global_arguments(false);
    node = std::make_shared<rclcpp::Node>("c04_readback_probe_no_flight", options);
    forward = node->create_client<Api>("/c04/backend/import");
    tap = node->create_service<Api>(
      "/platform/mission/import",
      [this](const std::shared_ptr<rclcpp::Service<Api>> & server,
      const std::shared_ptr<rmw_request_id_t> & header, Api::Request::SharedPtr request) {
        {std::lock_guard<std::mutex> lock(mutex); requests.push_back(*request);}
        forward->async_send_request(
          request,
          [this, server, header](rclcpp::Client<Api>::SharedFuture future) {
            const auto reply = future.get();
            {std::lock_guard<std::mutex> lock(mutex); replies.push_back(*reply);}
            server->send_response(*header, *reply);
          });
      });
    guard = rclcpp_action::create_server<Action>(
      node, "/execute_mission",
      [this](const auto &, auto) {++goals; return rclcpp_action::GoalResponse::REJECT;},
      [](auto) {return rclcpp_action::CancelResponse::REJECT;}, [](auto) {});
    rclcpp::ExecutorOptions eo; eo.context = context;
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(eo);
    executor->add_node(node); thread = std::thread([this] {executor->spin();});
  }
  template<typename T> T * widget(const char * name) {return window->findChild<T *>(name);}
  static QByteArray bytes(const QString & path)
  {QFile file(path); return file.open(QIODevice::ReadOnly) ? file.readAll() : QByteArray{};}
  void open(bool enabled)
  {
    process.start(
      QString::fromStdString(ament_index_cpp::get_package_prefix("easyfly_mission_srvs")) +
      "/lib/easyfly_mission_srvs/mission_services",
      {"--ros-args", "-p", "mission_directory:=" + directory.path() + "/catalog", "-p",
        enabled ? "enable_mission_import:=true" : "enable_mission_import:=false",
        "-r", "/platform/mission/import:=/c04/backend/import"});
    ASSERT_TRUE(process.waitForStarted(2000));
    easyfly_ui_bridge::BridgeConfig config; config.use_sim_time = false;
    config.selected_profile = "c04_file_only_no_flight"; config.image_topics = {"", ""};
    window = std::make_unique<ui::MainWindow>(
      config, [this](const auto & cfg, auto history) {
        auto source = ui::bridge_factory()(cfg, history); observed_source = source.get();
        return source;
      }, nullptr,
      ui::default_tool_options(), ui::MissionOptions{},
      ui::control_factory(), ui::ParameterOptions{},
      ui::parameter_factory(), ui::ImportOptions{true,
        (directory.path() + "/journal").toStdString()});
    window->show(); widget<QListWidget>("navigation")->setCurrentRow(6);
    EXPECT_EQ(widget<QPushButton>("startMission"), nullptr);
    EXPECT_EQ(widget<QPushButton>("cancelOwnedMission"), nullptr);
  }
  void export_draft(const QString & leaf, const QByteArray & content)
  {
    auto * editor = window->findChild<ui::MissionEditor *>(); ASSERT_NE(editor, nullptr);
    editor->prepare_directory(directory.path() + "/" + leaf);
    ASSERT_TRUE(
      desktop_test::until(
        [&] {
          return !editor->busy() &&
          widget<QLineEdit>("draftDirectory")->text() == directory.path() + "/" + leaf;
        }));
    widget<QPlainTextEdit>("draftBody")->setPlainText(QString::fromUtf8(content));
    widget<QLineEdit>("draftFilename")->setText("c04.yaml");
    ASSERT_TRUE(desktop_test::until([&] {return widget<QPushButton>("draftExport")->isEnabled();}));
    editor->export_file("c04.yaml");
    ASSERT_TRUE(desktop_test::until([&] {return editor->exported_draft().has_value();}));
    const auto exported = *editor->exported_draft();
    EXPECT_EQ(exported.content, content.toStdString());
    EXPECT_EQ(exported.hash, easyfly_mission_fsm::missionContentSha256(content.toStdString()));
    EXPECT_EQ(bytes(directory.path() + "/" + leaf + "/c04.yaml"), content);
  }
  unsigned creates()
  {
    std::lock_guard<std::mutex> lock(mutex); unsigned count = 0;
    for (const auto & r : requests) {count += r.operation == "create";} return count;
  }
  void refresh_destination(bool can_create)
  {
    auto * read = widget<QPushButton>("importReadDestination");
    ASSERT_TRUE(desktop_test::until([&] {return read->isEnabled();}, 5000)); read->click();
    if (can_create) {
      ASSERT_TRUE(
        desktop_test::until(
          [&] {return widget<QPushButton>("importCreate")->isEnabled();},
          5000));
    } else {
      ASSERT_TRUE(
        desktop_test::until(
          [&] {
            std::lock_guard<std::mutex> lock(mutex); return !replies.empty();
          }, 5000));
      QTest::qWait(150); EXPECT_FALSE(widget<QPushButton>("importCreate")->isEnabled());
    }
  }
  void submit(c::ImportPhase phase = c::ImportPhase::Completed)
  {
    const auto expected = creates() + 1;
    refresh_destination(true); widget<QPushButton>("importCreate")->click();
    confirm(*window->findChild<ui::ImportPanel *>());
    ASSERT_TRUE(
      desktop_test::until(
        [&] {
          return creates() == expected && !window->findChild<ui::ImportPanel *>()->busy() &&
          widget<QLabel>("importStatus")->property("importPhase").toInt() ==
          static_cast<int>(phase);
        }, 6000));
  }
  void inspect(const QByteArray & expected)
  {
    // List and validate share a production endpoint with an unchanged 1s read limit.
    QTest::qWait(1050);
    ASSERT_NE(observed_source, nullptr);
    const auto previous = observed_source->snapshot()->missions.at(
      easyfly_ui_bridge::Key::MissionList).evidence.request_id;
    widget<QListWidget>("navigation")->setCurrentRow(1);
    widget<QPushButton>("listMissions")->click();
    auto * list = widget<QListWidget>("missionList");
    ASSERT_TRUE(
      desktop_test::until(
        [&] {
          const auto snapshot = observed_source->snapshot();
          return snapshot->missions.at(easyfly_ui_bridge::Key::MissionList).evidence.request_id >
          previous;
        }, 5000));
    // Let the GUI consume that actual new list reply, not its retained old list rows.
    QTest::qWait(150); ASSERT_EQ(list->count(), 1);
    EXPECT_EQ(list->item(0)->text(), "c04.yaml"); list->setCurrentRow(0);
    ASSERT_TRUE(
      desktop_test::until(
        [&] {
          return widget<QPushButton>(
            "inspectMission")->isEnabled();
        }));
    widget<QPushButton>("inspectMission")->click();
    ASSERT_TRUE(
      desktop_test::until(
        [&] {
          return widget<QPlainTextEdit>("missionContent")->toPlainText() ==
          QString::fromUtf8(expected);
        }, 5000)) << widget<QLabel>("missionDetailsSummary")->text().toStdString();
    EXPECT_TRUE(widget<QLabel>("missionDetailsSummary")->text().contains("yaml_v1"));
    EXPECT_TRUE(
      widget<QLabel>("missionDetailsSummary")->text().contains(
        QString::fromStdString(
          easyfly_mission_fsm::missionContentSha256(expected.toStdString()))));
    ASSERT_TRUE(
      desktop_test::until(
        [&] {
          return widget<QPushButton>("validateMission")->isEnabled();
        }));
    QTest::qWait(1050); widget<QPushButton>("validateMission")->click();
    auto * validation = widget<QLabel>("validationResult");
    ASSERT_TRUE(
      desktop_test::until(
        [&] {
          return validation->text().contains(QStringLiteral("上游状态：valid"));
        }, 5000)) << validation->text().toStdString() << " / " <<
      widget<QLabel>("requestNote")->text().toStdString();
    EXPECT_EQ(goals, 0);
  }
  void TearDown() override
  {
    if (window) {
      if (auto * p = widget<QMessageBox>("importPrompt")) {p->done(QMessageBox::Cancel);}
      window->close();
      if (auto * p = widget<QMessageBox>("draftExitPrompt")) {p->done(QMessageBox::Yes);}
      if (auto * p = widget<QMessageBox>("importExitPrompt")) {p->done(QMessageBox::Yes);}
      EXPECT_TRUE(desktop_test::until([&] {return window->cleanup_complete();}, 8000));
      window.reset();
    }
    EXPECT_EQ(goals, 0);
    const auto evidence = qEnvironmentVariable("EASYFLY_C04_EVIDENCE_ROOT");
    if (!evidence.isEmpty()) {
      directory.setAutoRemove(false); QJsonArray trace;
      {std::lock_guard<std::mutex> lock(mutex);
        for (const auto & q : requests) {
          trace.append(
            QJsonObject{{"operation", QString::fromStdString(q.operation)},
              {"instance", QString::fromStdString(q.server_instance_id)},
              {"catalog", QString::fromStdString(q.catalog_id)},
              {"request", QString::fromStdString(q.client_request_id)},
              {"hash", QString::fromStdString(q.expected_content_sha256)}});
        }
      }
      QFile output(evidence + "/" +
        ::testing::UnitTest::GetInstance()->current_test_info()->name() +
        ".json");
      EXPECT_TRUE(output.open(QIODevice::WriteOnly | QIODevice::NewOnly));
      output.write(
        QJsonDocument(
          QJsonObject{{"artifacts", directory.path()}, {"trace", trace},
            {"flight_goals", goals.load()}, {"assertions_passed",
              !::testing::Test::HasFailure()}}).toJson());
    }
    if (process.state() != QProcess::NotRunning) {
      process.terminate();
      EXPECT_TRUE(process.waitForFinished(3000));
    }
    if (context && context->is_valid()) {context->shutdown("C04 file-only probe complete");}
    if (thread.joinable()) {thread.join();}
    guard.reset(); tap.reset(); forward.reset(); executor.reset(); node.reset(); context.reset();
  }
};
TEST_F(ImportChain, DraftRevisionExactReadbackNoOverwriteAndNoExecution)
{
  open(true); ASSERT_FALSE(::testing::Test::HasFatalFailure());
  export_draft("original-drafts", QByteArray(body));
  ASSERT_FALSE(::testing::Test::HasFatalFailure());
  refresh_destination(true); widget<QPushButton>("importCreate")->click();
  confirm(*window->findChild<ui::ImportPanel *>(), QMessageBox::Cancel); EXPECT_EQ(creates(), 0u);
  widget<QPushButton>("importCreate")->click();
  // Change the live editor before the next GUI timer. Retired snapshot must not be submitted.
  const QByteArray current = QByteArray(body) + "# C04 revision two\n";
  widget<QPlainTextEdit>("draftBody")->setPlainText(QString::fromUtf8(current));
  confirm(*window->findChild<ui::ImportPanel *>()); EXPECT_EQ(creates(), 0u);
  export_draft("current-drafts", current); ASSERT_FALSE(::testing::Test::HasFatalFailure());
  submit(); ASSERT_FALSE(::testing::Test::HasFatalFailure()); EXPECT_EQ(creates(), 1u);
  EXPECT_EQ(bytes(directory.path() + "/catalog/c04.yaml"), current);
  EXPECT_EQ(bytes(directory.path() + "/original-drafts/c04.yaml"), QByteArray(body));
  inspect(current); ASSERT_FALSE(::testing::Test::HasFatalFailure());
  widget<QListWidget>("navigation")->setCurrentRow(6);
  const QByteArray collision = current + "# same basename, different bytes\n";
  export_draft("collision-drafts", collision); ASSERT_FALSE(::testing::Test::HasFatalFailure());
  submit(c::ImportPhase::Rejected); ASSERT_FALSE(::testing::Test::HasFatalFailure());
  EXPECT_EQ(creates(), 2u);
  EXPECT_EQ(bytes(directory.path() + "/catalog/c04.yaml"), current);
  {
    std::lock_guard<std::mutex> lock(mutex); unsigned published = 0, refused = 0;
    for (const auto & r : replies) {
      published += r.status == "imported" && r.published;
      refused += r.status == "exists" && !r.published && r.resolved;
    }
    EXPECT_EQ(published, 1u); EXPECT_EQ(refused, 1u);
  }
  inspect(current); EXPECT_EQ(goals, 0);
}
TEST_F(ImportChain, ExplicitUiCannotCreateWhenServerImportDisabled)
{
  open(false); ASSERT_FALSE(::testing::Test::HasFatalFailure());
  export_draft("drafts", QByteArray(body)); ASSERT_FALSE(::testing::Test::HasFatalFailure());
  refresh_destination(false); EXPECT_EQ(creates(), 0u); EXPECT_EQ(goals, 0);
  EXPECT_FALSE(QFile::exists(directory.path() + "/catalog/c04.yaml"));
}
