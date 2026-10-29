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
#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QLabel>
#include <QListWidget>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QProcess>
#include <QProcessEnvironment>
#include <QPushButton>
#include <QScrollArea>
#include <QTemporaryDir>
#include <QTest>
#include <cstdlib>
#include <memory>
#include <set>
#include <string>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "easyfly_desktop/parameter_controls.hpp"
#include "support.hpp"
namespace ui = easyfly_desktop;
namespace p = easyfly_ui_control;
namespace
{
class OwnedProcess final : public QProcess
{
public:
  ~OwnedProcess() override
  {
    if (state() == QProcess::NotRunning) {return;}
    terminate();
    if (!waitForFinished(2000)) {kill(); waitForFinished(2000);}
  }
};
struct FakeParameters
{
  p::ParameterSnapshot data;
  int reads{0}, writes{0}, stops{0}; bool inhibited{false}, hold{false};
  uint64_t sequence{0}; std::set<std::string> confirmations;
  FakeParameters() {data.ready = true;}
};
class FakePort final : public ui::ParameterPort
{
public:
  explicit FakePort(std::shared_ptr<FakeParameters> state)
  : s_(state) {}
  std::shared_ptr<const p::ParameterSnapshot> snapshot() const override
  {
    return std::make_shared<p::ParameterSnapshot>(s_->data);
  }
  bool read(p::LocalParameter key) override
  {
    if (s_->inhibited || s_->data.pending || s_->data.stopped) {return false;}
    ++s_->reads; s_->data.review = p::ParameterReview{++s_->sequence, key,
      key == p::LocalParameter::OperatorNote ? "\"old\"" : "false", p::parameter_steady_seconds()};
    return true;
  }
  bool write_confirmed(
    const p::ParameterReview & review, const p::ParameterValue & desired,
    const std::string & id) override
  {
    if (s_->inhibited || s_->data.fault || s_->data.pending || !s_->data.review ||
      s_->data.review->id != review.id || id.size() != 32 || s_->confirmations.count(id))
    {
      return false;
    }
    s_->confirmations.insert(id); ++s_->writes; s_->data.dispatched = true;
    if (s_->hold) {s_->data.pending = true; s_->data.phase = p::ParameterPhase::Writing;} else {
      s_->data.review = p::ParameterReview{++s_->sequence, review.parameter,
        p::parameter_json(review.parameter, desired), p::parameter_steady_seconds()};
      s_->data.phase = p::ParameterPhase::Verified; s_->data.acknowledged = true;
    }
    return true;
  }
  void inhibit(bool active) override {s_->inhibited = active;}
  void stop() override {++s_->stops; s_->data.stopped = true; s_->data.ready = false;}

private:
  std::shared_ptr<FakeParameters> s_;
};
ui::ParameterFactory factory(std::shared_ptr<FakeParameters> s)
{return [s] {return std::make_unique<FakePort>(s);};}
void confirm(ui::ParameterPanel & panel, int answer = QMessageBox::Yes)
{
  auto * prompt = panel.findChild<QMessageBox *>("parameterEditPrompt"); ASSERT_NE(prompt, nullptr);
  EXPECT_EQ(prompt->defaultButton(), prompt->button(QMessageBox::Cancel)); prompt->done(answer);
}
void close(ui::MainWindow & window)
{window.close(); EXPECT_TRUE(desktop_test::until([&] {return window.cleanup_complete();}));}
}  // namespace
TEST(ParameterPanel, DefaultWindowNeverConstructsWriter)
{
  int creations = 0; auto observation = std::make_shared<desktop_test::FakeState>();
  ui::MainWindow window({}, desktop_test::factory(observation), nullptr, ui::default_tool_options(),
    {},
    ui::control_factory(), {},
    [&]() -> std::unique_ptr<ui::ParameterPort> {++creations; return {};});
  window.show(); EXPECT_EQ(creations, 0);
  EXPECT_EQ(window.findChild<QPushButton *>("parameterEditWrite"), nullptr); close(window);
}
TEST(ParameterPanel, ConfirmationCancelAndRetiredDialogCannotWrite)
{
  auto s = std::make_shared<FakeParameters>(); ui::ParameterPanel panel({true}, factory(s));
  panel.observer_available(true); panel.show();
  auto * write = panel.findChild<QPushButton *>("parameterEditWrite"); EXPECT_FALSE(
    write->isEnabled());
  panel.findChild<QPushButton *>("parameterEditRead")->click(); EXPECT_EQ(s->reads, 1);
  panel.findChild<QPlainTextEdit *>("parameterEditNote")->setPlainText(QStringLiteral("新值"));
  write->click(); auto * old = panel.findChild<QMessageBox *>("parameterEditPrompt"); ASSERT_NE(
    old,
    nullptr);
  confirm(panel, QMessageBox::Cancel); EXPECT_EQ(s->writes, 0);
  write->click(); auto * next = panel.findChild<QMessageBox *>("parameterEditPrompt"); ASSERT_NE(
    next, nullptr);
  old->done(QMessageBox::Yes); EXPECT_EQ(s->writes, 0);
  EXPECT_EQ(panel.findChild<QMessageBox *>("parameterEditPrompt"), next);
  confirm(panel); EXPECT_EQ(s->writes, 1); EXPECT_EQ(s->data.review->json, "\"新值\"");
  panel.stop(); EXPECT_TRUE(panel.stopped());
}
TEST(ParameterPanel, CandidateChangesDisconnectAndOversizeFailClosed)
{
  auto s = std::make_shared<FakeParameters>(); ui::ParameterPanel panel({true}, factory(s));
  panel.observer_available(true);
  auto * read = panel.findChild<QPushButton *>("parameterEditRead"); read->click();
  auto * note = panel.findChild<QPlainTextEdit *>("parameterEditNote");
  auto * write = panel.findChild<QPushButton *>("parameterEditWrite");
  note->setPlainText(QString(513, 'x')); EXPECT_FALSE(write->isEnabled());
  note->setPlainText("proposal"); write->click(); note->setPlainText("changed after prompt");
  confirm(panel); EXPECT_EQ(s->writes, 0);
  write->click(); auto * old = panel.findChild<QMessageBox *>("parameterEditPrompt"); ASSERT_NE(
    old,
    nullptr);
  panel.observer_available(false); old->done(QMessageBox::Yes); EXPECT_EQ(s->writes, 0);
  EXPECT_TRUE(s->inhibited); EXPECT_FALSE(write->isEnabled()); panel.stop();
}
TEST(ParameterPanel, OnlyTwoTypedKeysAndBooleanNoDefaultCurrentValue)
{
  auto s = std::make_shared<FakeParameters>(); ui::ParameterPanel panel({true}, factory(s));
  panel.observer_available(true);
  auto * key = panel.findChild<QComboBox *>("parameterEditKey"); ASSERT_EQ(key->count(), 2);
  EXPECT_EQ(key->itemText(0), "operator_note"); EXPECT_EQ(key->itemText(1), "export_pretty");
  key->setCurrentIndex(1);
  EXPECT_TRUE(panel.findChild<QPlainTextEdit *>("parameterEditCurrent")->toPlainText().isEmpty());
  panel.findChild<QPushButton *>("parameterEditRead")->click(); panel.findChild<QCheckBox *>(
    "parameterEditPretty")->setChecked(true);
  panel.findChild<QPushButton *>("parameterEditWrite")->click(); confirm(panel);
  EXPECT_EQ(s->data.review->json, "true"); EXPECT_EQ(s->writes, 1); panel.stop();
}
TEST(ParameterPanel, FaultAndCapacityAreNotClearedByReads)
{
  auto s = std::make_shared<FakeParameters>(); ui::ParameterPanel panel({true}, factory(s));
  panel.observer_available(true);
  s->data.fault = true; panel.findChild<QPushButton *>("parameterEditRead")->click();
  EXPECT_TRUE(panel.fault()); EXPECT_FALSE(
    panel.findChild<QPushButton *>(
      "parameterEditWrite")->isEnabled());
  s->data.fault = false; s->data.confirmations = 256; panel.findChild<QPushButton *>(
    "parameterEditRead")->click();
  EXPECT_FALSE(panel.findChild<QPushButton *>("parameterEditWrite")->isEnabled()); EXPECT_EQ(
    s->writes, 0); panel.stop();
}
TEST(ParameterPanel, BusyExitAndFaultExitRequireRealStateAndDefaultStay)
{
  auto s = std::make_shared<FakeParameters>();
  auto observation = std::make_shared<desktop_test::FakeState>();
  ui::MainWindow window({}, desktop_test::factory(observation), nullptr, ui::default_tool_options(),
    {},
    ui::control_factory(), {true}, factory(s)); window.show();
  s->data.pending = true; s->data.dispatched = true; window.close();
  EXPECT_TRUE(window.isVisible()); EXPECT_EQ(s->stops, 0); EXPECT_EQ(observation->stop_calls, 0);
  s->data.pending = false; s->data.fault = true; window.close();
  auto * prompt = window.findChild<QMessageBox *>("parameterExitPrompt");
  ASSERT_NE(prompt, nullptr);
  EXPECT_EQ(prompt->defaultButton(), prompt->button(QMessageBox::Cancel)); prompt->done(
    QMessageBox::Cancel);
  EXPECT_TRUE(window.isVisible()); EXPECT_EQ(s->stops, 0); EXPECT_FALSE(s->inhibited);
  window.close(); prompt = window.findChild<QMessageBox *>("parameterExitPrompt"); ASSERT_NE(
    prompt,
    nullptr);
  prompt->done(QMessageBox::Yes); EXPECT_TRUE(
    desktop_test::until(
      [&] {
        return window.cleanup_complete();
      }));
  EXPECT_EQ(s->stops, 1); EXPECT_EQ(s->writes, 0);
}
TEST(ParameterPanel, DisconnectedObserverDoesNotBypassFaultExit)
{
  auto s = std::make_shared<FakeParameters>();
  auto observation = std::make_shared<desktop_test::FakeState>();
  ui::MainWindow window({}, desktop_test::factory(observation), nullptr, ui::default_tool_options(),
    {}, ui::control_factory(), {true}, factory(s));
  window.show();
  window.findChild<QPushButton *>("observerToggle")->click();
  ASSERT_TRUE(desktop_test::until([&] {return window.cleanup_complete();}));
  s->data.fault = true;
  window.close();
  auto * prompt = window.findChild<QMessageBox *>("parameterExitPrompt");
  ASSERT_NE(prompt, nullptr);
  EXPECT_EQ(s->stops, 0);
  prompt->done(QMessageBox::Cancel);
  EXPECT_TRUE(window.isVisible());
  EXPECT_TRUE(s->inhibited);
  window.close();
  prompt = window.findChild<QMessageBox *>("parameterExitPrompt");
  ASSERT_NE(prompt, nullptr);
  prompt->done(QMessageBox::Yes);
  ASSERT_TRUE(desktop_test::until([&] {return window.cleanup_complete();}));
  EXPECT_EQ(s->stops, 1);
  EXPECT_EQ(s->writes, 0);
}
TEST(ParameterStack, RealParameterManagerReadWriteReadbackAndPrettyExport)
{
  ASSERT_STREQ(std::getenv("ROS_DOMAIN_ID"), "124"); ASSERT_STREQ(
    std::getenv(
      "ROS_LOCALHOST_ONLY"), "1");
  QTemporaryDir directory; OwnedProcess process;
  auto environment = QProcessEnvironment::systemEnvironment();
  environment.insert("ROS_LOG_DIR", directory.path()); process.setProcessEnvironment(environment);
  const auto binary = ament_index_cpp::get_package_prefix("easyfly_params") +
    "/lib/easyfly_params/parameter_manager";
  process.start(
    QString::fromStdString(
      binary), {"--ros-args", "-p", "profile:=base_sim", "-p", "use_sim_time:=false"});
  ASSERT_TRUE(process.waitForStarted(2000));
  auto observation = std::make_shared<desktop_test::FakeState>();
  observation->data = std::make_shared<easyfly_ui_bridge::UiSnapshot>();
  observation->data->running = true;
  easyfly_ui_bridge::BridgeConfig config;
  config.use_sim_time = false; config.selected_profile = "base_sim";
  ui::MainWindow window(config, desktop_test::factory(observation), nullptr,
    ui::default_tool_options(),
    {},
    ui::control_factory(), {true}); window.show();
  window.findChild<QListWidget *>("navigation")->setCurrentRow(4);
  auto * panel = window.findChild<ui::ParameterPanel *>(); ASSERT_NE(panel, nullptr);
  auto * read = window.findChild<QPushButton *>("parameterEditRead");
  auto * write = window.findChild<QPushButton *>("parameterEditWrite");
  ASSERT_TRUE(
    desktop_test::until(
      [&] {
        if (read->isEnabled() && !write->isEnabled()) {read->click();}
        return write->isEnabled();
      }, 7000));
  window.findChild<QPlainTextEdit *>("parameterEditNote")->setPlainText(
    QStringLiteral("隔离界面测试"));
  write->click(); confirm(*panel);
  ASSERT_TRUE(
    desktop_test::until(
      [&] {
        return !panel->busy() && window.findChild<QLabel *>("parameterEditStatus")->property(
          "parameterPhase").toInt() == static_cast<int>(p::ParameterPhase::Verified) &&
        window.findChild<QPlainTextEdit *>("parameterEditCurrent")->toPlainText().contains(
          QStringLiteral("隔离界面测试"));
      }, 4000));
  EXPECT_FALSE(panel->fault());
  EXPECT_TRUE(
    window.findChild<QPlainTextEdit *>("parameterEditCurrent")->toPlainText().contains(
      QStringLiteral("隔离界面测试")));
  window.findChild<QComboBox *>("parameterEditKey")->setCurrentIndex(1); read->click();
  ASSERT_TRUE(desktop_test::until([&] {return write->isEnabled();}, 4000));
  window.findChild<QCheckBox *>("parameterEditPretty")->setChecked(true); write->click(); confirm(
    *panel);
  ASSERT_TRUE(
    desktop_test::until(
      [&] {
        return !panel->busy() && window.findChild<QLabel *>("parameterEditStatus")->property(
          "parameterPhase").toInt() == static_cast<int>(p::ParameterPhase::Verified) &&
        window.findChild<QPlainTextEdit *>("parameterEditCurrent")->toPlainText() == "true";
      }, 4000));
  EXPECT_EQ(window.findChild<QPlainTextEdit *>("parameterEditCurrent")->toPlainText(), "true");
  EXPECT_FALSE(panel->fault());
  auto context = std::make_shared<rclcpp::Context>(); context->init(0, nullptr);
  rclcpp::NodeOptions options; options.context(context).use_global_arguments(false);
  auto node = std::make_shared<rclcpp::Node>("read_only_export_check", options);
  auto client = node->create_client<std_srvs::srv::Trigger>("/platform/params/export");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(3)));
  auto response = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  rclcpp::ExecutorOptions eo; eo.context = context;
  rclcpp::executors::SingleThreadedExecutor executor(eo); executor.add_node(node);
  ASSERT_TRUE(
    desktop_test::until(
      [&] {
        executor.spin_some();
        return response.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready;
      }, 4000));
  const auto exported = response.get();
  EXPECT_TRUE(exported->success);
  EXPECT_NE(exported->message.find('\n'), std::string::npos);
  EXPECT_NE(exported->message.find("隔离界面测试"), std::string::npos);
  window.findChild<QLabel *>("requestNote")->setText(
    QStringLiteral(
      "独立domain124，仅元数据服务 · 非飞行 · 不接生产后端"));
  auto * scroll = qobject_cast<QScrollArea *>(
    panel->parentWidget()->parentWidget()->parentWidget()->parentWidget());
  if (scroll) {scroll->ensureWidgetVisible(panel, 10, 10);} QTest::qWait(100);
  QDir().mkpath(EASYFLY_PARAMETER_SCREENSHOT_DIRECTORY);
  EXPECT_TRUE(
    window.grab().save(
      QString(EASYFLY_PARAMETER_SCREENSHOT_DIRECTORY) +
      "/parameters-isolated.png"));
  EXPECT_EQ(observation->list_calls + observation->validate_calls + observation->query_calls, 0);
  close(window); executor.remove_node(node); client.reset(); node.reset(); context->shutdown(
    "test end");
  process.terminate(); if (!process.waitForFinished(2000)) {
    process.kill(); process.waitForFinished(2000);
  }
}
