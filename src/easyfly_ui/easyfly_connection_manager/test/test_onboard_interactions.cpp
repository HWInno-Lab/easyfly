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
#include <QAbstractButton>
#include <QApplication>
#include <QComboBox>
#include <QCheckBox>
#include <QCryptographicHash>
#include <QDir>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPlainTextEdit>
#include <QMap>
#include <QMessageBox>
#include <QPushButton>
#include <QSaveFile>
#include <QTabWidget>
#include <QTemporaryDir>
#include <QTest>
#include <QTimer>
#include <QTreeWidget>
#include <QTreeWidgetItemIterator>
#include <functional>

#include "easyfly_connection_manager/onboard_page.hpp"

namespace
{
bool until(const std::function<bool()> & condition, int timeout = 8000)
{
  QElapsedTimer elapsed; elapsed.start();
  while (!condition() && elapsed.elapsed() < timeout) {QTest::qWait(10);}
  return condition();
}

class FakeBridge
{
public:
  QTemporaryDir directory;
  QJsonObject control;

  FakeBridge()
  {
    const auto source = QFileInfo(QString::fromUtf8(__FILE__)).absoluteDir();
    const auto package = directory.path() + "/modules/easyfly_connection_manager";
    QDir().mkpath(package);
    write(package + "/__init__.py", {});
    write(
      package + "/onboard_ui_bridge.py",
      "import os,runpy\n"
      "runpy.run_path(os.environ['EASYFLY_TEST_ONBOARD_FIXTURE'],run_name='__main__')\n");
    set("EASYFLY_TEST_ONBOARD_FIXTURE", source.filePath("fixture_onboard_bridge.py").toUtf8());
    set(
      "EASYFLY_TEST_ONBOARD_PROFILES",
      source.filePath("../easyfly_connection_manager/onboard_profiles.py").toUtf8());
    set("EASYFLY_TEST_ONBOARD_CONTROL", controlPath().toUtf8());
    set("EASYFLY_TEST_ONBOARD_LOG", logPath().toUtf8());
    set("PYTHONPATH", (directory.path() + "/modules:").toUtf8() + qgetenv("PYTHONPATH"));
    saveControl();
  }

  ~FakeBridge()
  {
    for (auto it = previous_.cbegin(); it != previous_.cend(); ++it) {
      if (present_.value(it.key())) {qputenv(it.key().constData(), it.value());} else {
        qunsetenv(it.key().constData());
      }
    }
  }

  QString runtime() const {return directory.path() + "/runtime";}
  QString controlPath() const {return directory.path() + "/control.json";}
  QString logPath() const {return directory.path() + "/requests.jsonl";}

  void saveControl()
  {
    QSaveFile file(controlPath());
    ASSERT_TRUE(file.open(QIODevice::WriteOnly));
    ASSERT_GT(file.write(QJsonDocument(control).toJson(QJsonDocument::Compact)), 0);
    ASSERT_TRUE(file.commit());
  }

  QList<QJsonObject> requests(const QString & op = {}) const
  {
    QFile file(logPath()); if (!file.open(QIODevice::ReadOnly)) {return {};}
    QList<QJsonObject> result;
    for (const auto & line : file.readAll().split('\n')) {
      const auto object = QJsonDocument::fromJson(line).object();
      if (!object.isEmpty() && (op.isEmpty() || object["op"].toString() == op)) {
        result.append(object);
      }
    }
    return result;
  }

private:
  QMap<QByteArray, QByteArray> previous_;
  QMap<QByteArray, bool> present_;
  void set(const QByteArray & name, const QByteArray & value)
  {
    present_[name] = qEnvironmentVariableIsSet(name.constData());
    previous_[name] = qgetenv(name.constData());
    qputenv(name.constData(), value);
  }
  void write(const QString & path, const QByteArray & content)
  {
    QFile file(path); ASSERT_TRUE(file.open(QIODevice::WriteOnly));
    ASSERT_EQ(file.write(content), content.size());
  }
};

QMessageBox * messageBox(const QString & title)
{
  for (auto * window : QApplication::topLevelWidgets()) {
    auto * box = qobject_cast<QMessageBox *>(window);
    if (box && box->isVisible() && box->windowTitle() == title) {return box;}
  }
  return nullptr;
}

bool connectLocal(easyfly_connection_manager::OnboardPage & page)
{
  auto * local = page.findChild<QPushButton *>("onboardConnectLocal");
  auto * name = page.findChild<QLineEdit *>("onboardRecipeName");
  auto * start = page.findChild<QPushButton *>("onboardStart");
  if (!local || !name || !start || !until(
      [&] {
        return page.findChild<QTreeWidget *>("onboardAlgorithmTree")->topLevelItemCount() > 0 &&
        local->isEnabled();
      })) {return false;}
  local->click();
  return until([&] {return name->text() == "platform-A" && start->isEnabled();});
}

QTreeWidgetItem * component(QTreeWidget * tree, const QString & id)
{
  QTreeWidgetItemIterator it(tree);
  for (; *it; ++it) {
    if ((*it)->data(0, Qt::UserRole).toString() == id) {return *it;}
  }
  return nullptr;
}

QComboBox * strategySelector(QWidget * page)
{
  for (auto * selector : page->findChildren<QComboBox *>()) {
    if (selector->findData("astar") >= 0 && selector->findData("ego") >= 0 &&
      selector->findData("combined") >= 0) {return selector;}
  }
  return nullptr;
}
}  // namespace

TEST(OnboardInteractions, DiscardThenConnectLoadsTheNewTargetAndVersion)
{
  FakeBridge bridge;
  easyfly_connection_manager::OnboardPage page(bridge.runtime()); page.show();
  ASSERT_TRUE(connectLocal(page));
  auto * name = page.findChild<QLineEdit *>("onboardRecipeName");
  name->setFocus(); name->selectAll(); QTest::keyClicks(name, "unsaved-A");
  ASSERT_TRUE(page.property("onboardHasUnsavedChanges").toBool());
  auto * recent = page.findChild<QComboBox *>("onboardRecentPlatforms");
  auto * connect = page.findChild<QPushButton *>("onboardConnectRecent");
  ASSERT_TRUE(until([&] {return recent->count() && connect->isEnabled();}));
  recent->setCurrentIndex(0);
  bool answered = false; QTimer response; response.setInterval(10);
  QObject::connect(
    &response, &QTimer::timeout, &page, [&] {
      if (auto * box = messageBox(QStringLiteral("保留未保存编辑"))) {
        answered = true; box->button(QMessageBox::Discard)->click();
      }
    });
  response.start(); connect->click(); response.stop();
  ASSERT_TRUE(answered);
  ASSERT_TRUE(until([&] {return name->text() == "platform-B";})) <<
    "name=" << name->text().toStdString() << "; result=" <<
    page.findChild<QLabel *>("onboardResult")->text().toStdString() << "; remote requests=" <<
    bridge.requests("connect_remote").size();
  auto * state = page.findChild<QLabel *>("onboardRecipeState");
  ASSERT_NE(state, nullptr); EXPECT_TRUE(state->text().contains(QString(64, 'b')));
  auto * save = page.findChild<QPushButton *>("onboardSaveRecipe");
  ASSERT_TRUE(until([&] {return save->isEnabled();})); save->click();
  ASSERT_TRUE(until([&] {return !bridge.requests("operate").isEmpty();}));
  const auto data = bridge.requests("operate").last()["data"].toObject();
  EXPECT_EQ(data["operation"].toString(), "configure");
  const auto values = data["values"].toObject();
  EXPECT_EQ(values["expected_hash"].toString(), QString(64, 'b'));
  EXPECT_EQ(values["recipe"].toObject()["name"].toString(), "platform-B");
}

TEST(OnboardInteractions, TemplateEditsRestoreAndCustomFileUseOneParameterSource)
{
  FakeBridge bridge;
  bridge.control["capabilities"] = QJsonObject{{"parameter_templates", 1}};
  bridge.control["parameter_templates"] = QJsonObject{{"flight_control", QJsonObject{
      {"parameters", QJsonObject{{"max_horizontal_speed", 20.0}}},
      {"accepts_file", true}, {"required", QJsonObject{}}}}};
  bridge.saveControl();
  easyfly_connection_manager::OnboardPage page(bridge.runtime()); page.show();
  ASSERT_TRUE(connectLocal(page));
  auto * tree = page.findChild<QTreeWidget *>("onboardAlgorithmTree");
  auto * control = component(tree, "flight_control"); ASSERT_NE(control, nullptr);
  tree->setCurrentItem(control);
  page.findChild<QTabWidget *>("onboardManagementTabs")->setCurrentIndex(2);
  auto * editor = page.findChild<QWidget *>("onboardAlgorithmParameterEditor");
  ASSERT_NE(editor, nullptr); EXPECT_FALSE(editor->isVisible());
  auto * edit = page.findChild<QPushButton *>("onboardEditParameters_flight_control");
  ASSERT_NE(edit, nullptr); edit->click();
  EXPECT_TRUE(editor->isVisible());
  EXPECT_FALSE(page.property("onboardHasUnsavedChanges").toBool());
  auto * speed = page.findChild<QDoubleSpinBox *>(
    "onboardParameter_flight_control_max_horizontal_speed");
  ASSERT_NE(speed, nullptr); speed->setValue(8.0);
  auto * back = page.findChild<QPushButton *>("onboardBackToAlgorithmDetails");
  ASSERT_NE(back, nullptr); back->click(); EXPECT_FALSE(editor->isVisible());
  page.findChild<QPushButton *>("onboardEditParameters_flight_control")->click();
  EXPECT_TRUE(editor->isVisible());
  EXPECT_EQ(
    page.findChild<QDoubleSpinBox *>(
      "onboardParameter_flight_control_max_horizontal_speed")->value(), 8.0);
  control->setCheckState(0, Qt::Checked);
  auto * save = page.findChild<QPushButton *>("onboardSaveRecipe");
  ASSERT_TRUE(until([&] {return !bridge.requests("prepare").isEmpty() && save->isEnabled();}));
  const auto upload_file = [&](const QString & selected_file) {
      bool shown = false, timed_out = false;
      QElapsedTimer deadline; deadline.start(); QTimer response; response.setInterval(10);
      QObject::connect(
        &response, &QTimer::timeout, &page, [&] {
          for (auto * window : QApplication::topLevelWidgets()) {
            auto * dialog = qobject_cast<QFileDialog *>(window);
            if (!dialog || !dialog->isVisible()) {continue;}
            if (deadline.elapsed() > 4000) {
              timed_out = true; dialog->reject(); return;
            }
            if (shown) {return;}
            shown = true;
            if (selected_file.isEmpty()) {dialog->reject();} else {
              auto * filename = dialog->findChild<QLineEdit *>("fileNameEdit");
              if (!filename) {timed_out = true; dialog->reject(); return;}
              filename->setText(selected_file);
              QMetaObject::invokeMethod(dialog, "accept", Qt::QueuedConnection);
            }
            break;
          }
        });
      response.start();
      page.findChild<QPushButton *>("onboardUploadFile_flight_control")->click();
      response.stop(); return shown && !timed_out;
    };
  ASSERT_TRUE(upload_file({}));
  EXPECT_TRUE(bridge.requests("operate").isEmpty());
  EXPECT_EQ(
    page.findChild<QComboBox *>("onboardParameterSource_flight_control")->currentData(),
    "builtin");
  EXPECT_EQ(
    page.findChild<QDoubleSpinBox *>(
      "onboardParameter_flight_control_max_horizontal_speed")->value(), 8.0);
  const auto path = bridge.directory.path() + "/controller.yaml";
  QFile uploaded(path); ASSERT_TRUE(uploaded.open(QIODevice::WriteOnly));
  uploaded.write("/**:\n  ros__parameters:\n    max_horizontal_speed: 5.0\n"); uploaded.close();
  bridge.control["reject_upload"] = true; bridge.saveControl();
  ASSERT_TRUE(upload_file(path));
  ASSERT_TRUE(until([&] {return bridge.requests("operate").size() == 1 && save->isEnabled();}));
  EXPECT_EQ(
    page.findChild<QComboBox *>("onboardParameterSource_flight_control")->currentData(),
    "builtin");
  EXPECT_EQ(
    page.findChild<QDoubleSpinBox *>(
      "onboardParameter_flight_control_max_horizontal_speed")->value(), 8.0);
  bridge.control["reject_upload"] = false; bridge.saveControl();
  ASSERT_TRUE(upload_file(path));
  ASSERT_TRUE(
    until(
      [&] {
        auto * file = page.findChild<QLineEdit *>("onboardParameterFile_flight_control");
        return file && !file->isReadOnly() && file->text() == "/fixture/controller.yaml";
      }));
  EXPECT_EQ(
    page.findChild<QDoubleSpinBox *>(
      "onboardParameter_flight_control_max_horizontal_speed"), nullptr);
  auto * file = page.findChild<QLineEdit *>("onboardParameterFile_flight_control");
  const auto uploads = bridge.requests("operate").size(); save->click();
  ASSERT_TRUE(
    until(
      [&] {
        return bridge.requests("operate").size() > uploads && save->isEnabled();
      }));
  auto values = bridge.requests("operate").last()["data"].toObject()["values"].toObject();
  auto submitted = values["recipe"].toObject()["components"].toObject();
  EXPECT_TRUE(submitted["flight_control"].toObject()["parameters"].toObject().isEmpty());
  EXPECT_TRUE(values["templates"].toArray().isEmpty());
  auto * reset = page.findChild<QPushButton *>("onboardResetParameters_flight_control");
  ASSERT_NE(reset, nullptr); reset->click();
  ASSERT_TRUE(
    until(
      [&] {
        auto * field = page.findChild<QDoubleSpinBox *>(
          "onboardParameter_flight_control_max_horizontal_speed");
        return field && field->value() == 20.0;
      }));
  file = page.findChild<QLineEdit *>("onboardParameterFile_flight_control");
  ASSERT_TRUE(file->isReadOnly()); EXPECT_TRUE(file->text().isEmpty());
  const auto previous = bridge.requests("operate").size(); save->click();
  ASSERT_TRUE(
    until(
      [&] {
        return bridge.requests("operate").size() > previous &&
        page.findChild<QLineEdit *>("onboardParameterFile_flight_control")->text().contains(
          "-easyfly-template-flight_control.yaml");
      }));
  values = bridge.requests("operate").last()["data"].toObject()["values"].toObject();
  EXPECT_EQ(values["templates"].toArray(), QJsonArray{"flight_control"});
  submitted = values["recipe"].toObject()["components"].toObject();
  EXPECT_TRUE(submitted["flight_control"].toObject()["enabled"].toBool());
  EXPECT_TRUE(submitted["mavlink"].toObject()["enabled"].toBool());
  EXPECT_EQ(submitted["navigation"].toObject()["parameters"].toObject()["strategy"], "astar");
  const QStringList permitted{"configure", "configuration_upload"};
  for (const auto & call : bridge.requests("operate")) {
    EXPECT_TRUE(permitted.contains(call["data"].toObject()["operation"].toString()));
  }
}

TEST(OnboardInteractions, ConfirmationCannotAuthorizeAChangedRecipeVersion)
{
  FakeBridge bridge;
  easyfly_connection_manager::OnboardPage page(bridge.runtime()); page.show();
  ASSERT_TRUE(connectLocal(page));
  auto * start = page.findChild<QPushButton *>("onboardStart");
  auto * state = page.findChild<QLabel *>("onboardRecipeState");
  bool saw_dialog = false, saw_changed_version = false;
  QElapsedTimer deadline; deadline.start(); QTimer response; response.setInterval(10);
  QObject::connect(
    &response, &QTimer::timeout, &page, [&] {
      auto * box = messageBox(QStringLiteral("启动机载平台")); if (!box) {return;}
      if (!saw_dialog) {
        saw_dialog = true; bridge.control["hash_A"] = QString(64, 'd');
        bridge.control["name_A"] = "external-new-version"; bridge.saveControl();
      }
      if (state->text().contains(QString(64, 'd'))) {
        saw_changed_version = true; box->button(QMessageBox::Yes)->click();
      } else if (deadline.elapsed() > 6000) {box->button(QMessageBox::No)->click();}
    });
  response.start(); start->click(); response.stop();
  ASSERT_TRUE(saw_dialog); ASSERT_TRUE(saw_changed_version);
  QTest::qWait(100);
  for (const auto & request : bridge.requests("operate")) {
    const auto data = request["data"].toObject();
    if (data["operation"].toString() == "start") {
      EXPECT_EQ(data["values"].toObject()["recipe_hash"].toString(), QString(64, 'a'));
    }
  }
}

TEST(OnboardInteractions, DependencyPreparationSurvivesSlowStatusAndStaleReplies)
{
  FakeBridge bridge;
  easyfly_connection_manager::OnboardPage page(bridge.runtime()); page.show();
  ASSERT_TRUE(connectLocal(page));
  auto * tabs = page.findChild<QTabWidget *>("onboardManagementTabs");
  for (int i = 0; i < tabs->count(); ++i) {
    if (tabs->tabText(i) == QStringLiteral("算法配置")) {tabs->setCurrentIndex(i); break;}
  }
  bridge.control["hold_status"] = true; bridge.control["hold_prepare"] = true;
  bridge.saveControl();
  const auto status_count = bridge.requests("status").size();
  ASSERT_TRUE(until([&] {return bridge.requests("status").size() > status_count;}));
  auto * tree = page.findChild<QTreeWidget *>("onboardAlgorithmTree");
  ASSERT_TRUE(tree->isEnabled());
  auto * navigation = component(tree, "navigation"); ASSERT_NE(navigation, nullptr);
  tree->setCurrentItem(navigation); navigation->setCheckState(0, Qt::Checked);
  auto * selector = strategySelector(&page); ASSERT_NE(selector, nullptr);
  selector->setCurrentIndex(selector->findData("ego"));
  EXPECT_TRUE(bridge.requests("prepare").isEmpty());
  bridge.control["hold_status"] = false; bridge.saveControl();
  ASSERT_TRUE(until([&] {return !bridge.requests("prepare").isEmpty();}));
  const auto first = bridge.requests("prepare").first()["data"].toObject()["recipe"].toObject();
  EXPECT_EQ(
    first["components"].toObject()["navigation"].toObject()["parameters"].toObject()[
      "strategy"].toString(), "ego");

  auto * advanced = page.findChild<QPushButton *>("onboardAdvancedRecipe");
  ASSERT_NE(advanced, nullptr); EXPECT_FALSE(advanced->isEnabled());
  // A later real edit must survive the older dependency response and its retry.
  auto * name = page.findChild<QLineEdit *>("onboardRecipeName");
  ASSERT_TRUE(name->isEnabled());
  name->setFocus(); name->selectAll(); QTest::keyClicks(name, "newer-local-edit");
  bridge.control["hold_prepare"] = false; bridge.saveControl();
  ASSERT_TRUE(until([&] {return bridge.requests("prepare").size() >= 2;}));
  ASSERT_TRUE(
    until(
      [&] {
        const auto * local = component(tree, "local_planner");
        return local && local->checkState(0) == Qt::Checked && tree->isEnabled();
      }));
  EXPECT_EQ(name->text(), "newer-local-edit");
  const auto latest = bridge.requests("prepare").last()["data"].toObject()["recipe"].toObject();
  EXPECT_EQ(latest["name"].toString(), "newer-local-edit");
  EXPECT_EQ(component(tree, "navigation")->checkState(0), Qt::Checked);
}


TEST(OnboardInteractions, SharedMissionCandidateKeepsReviewAndTargetBoundaries)
{
  FakeBridge bridge;
  const QString content =
    "# Preserve original CRLF bytes\r\nschema_version: 1\r\nframe_id: odom\r\n"
    "units: {position: m, angle: rad, time: s}\r\nfailure_policy: abort\r\n"
    "steps: [{type: land, timeout: 60}]\r\n";
  const auto displayed = QString(content).replace("\r\n", "\n");
  bridge.control["mission_catalog"] = QJsonObject{{"original.yaml", content}};
  bridge.control["task_request"] = QString(32, 'e'); bridge.saveControl();
  easyfly_connection_manager::OnboardPage page(bridge.runtime()); page.show();
  ASSERT_TRUE(connectLocal(page));
  ASSERT_NE(page.missionTransferPage(), nullptr);
  auto * list = page.findChild<QListWidget *>("onboardMissionCatalog");
  ASSERT_NE(list, nullptr); ASSERT_TRUE(until([&] {return list->count() == 1;}));
  auto * read = page.findChild<QPushButton *>("onboardMissionRead");
  auto * save = page.findChild<QPushButton *>("onboardMissionSave");
  auto * edit = page.findChild<QPushButton *>("onboardMissionEdit");
  auto * lookup = page.findChild<QPushButton *>("onboardMissionLookup");
  auto * body = page.findChild<QPlainTextEdit *>("onboardMissionEditor");
  auto * permit = page.findChild<QCheckBox *>("onboardMissionPermit");
  ASSERT_NE(read, nullptr); ASSERT_NE(save, nullptr); ASSERT_NE(edit, nullptr);
  ASSERT_NE(lookup, nullptr); ASSERT_NE(body, nullptr); ASSERT_NE(permit, nullptr);
  EXPECT_TRUE(body->isReadOnly());
  const auto target =
    page.property("onboardMissionContext").value<QJsonObject>()["target"].toObject();
  const auto support = page.property("onboardMissionContext").value<QJsonObject>()[
    "step_support"].toObject();
  EXPECT_EQ(support["explore"].toObject()["state"], "unknown");
  EXPECT_EQ(support["land"].toObject()["state"], "unknown");
  QJsonObject candidate{{"name", "copy.yaml"}, {"content", content}, {"revision", 1},
    {"target", target}, {"hash", QString::fromLatin1(
        QCryptographicHash::hash(content.toUtf8(), QCryptographicHash::Sha256).toHex())}};
  page.setMissionCandidate(candidate);
  EXPECT_FALSE(page.property("onboardHasUnsavedChanges").toBool());
  ASSERT_TRUE(until([&] {return read->isEnabled() && save->isEnabled();})); read->click();
  ASSERT_TRUE(until([&] {return body->toPlainText() == displayed && permit->isEnabled();}));
  EXPECT_TRUE(page.property("onboardMissionDocument").value<QJsonObject>().isEmpty());
  page.setMissionCandidate(candidate); EXPECT_TRUE(permit->isEnabled());
  edit->click(); auto document = page.property("onboardMissionDocument").value<QJsonObject>();
  EXPECT_EQ(document["content"], content); EXPECT_EQ(document["target"].toObject(), target);
  EXPECT_NE(body->toPlainText(), content);
  EXPECT_EQ(
    QString::fromLatin1(
      QCryptographicHash::hash(
        document["content"].toString().toUtf8(), QCryptographicHash::Sha256).toHex()),
    candidate["hash"].toString());
  const auto serial = document["serial"].toDouble(); edit->click();
  EXPECT_GT(
    page.property(
      "onboardMissionDocument").value<QJsonObject>()["serial"].toDouble(), serial);
  page.setMissionCandidate(QJsonObject{{"revision", 2}, {"target", target}, {"valid", false}});
  EXPECT_FALSE(permit->isEnabled()); EXPECT_FALSE(save->isEnabled());
  candidate["revision"] = 3; candidate["name"] = QStringLiteral("巡检任务.yaml");
  page.setMissionCandidate(candidate);
  ASSERT_TRUE(until([&] {return save->isEnabled();}));
  const auto before = bridge.requests("operate").size();
  bool answered = false; QTimer response; response.setInterval(10);
  QObject::connect(
    &response, &QTimer::timeout, &page, [&] {
      if (auto * box = messageBox(QStringLiteral("保存任务到机载平台"))) {
        answered = true; candidate["revision"] = 4; page.setMissionCandidate(candidate);
        box->button(QMessageBox::Yes)->click();
      }
    });
  response.start(); save->click(); response.stop();
  ASSERT_TRUE(answered); EXPECT_EQ(bridge.requests("operate").size(), before);
  QObject::disconnect(&response, nullptr, &page, nullptr);
  QObject::connect(
    &response, &QTimer::timeout, &page, [&] {
      if (auto * box = messageBox(QStringLiteral("保存任务到机载平台"))) {
        box->button(QMessageBox::Yes)->click();
      }
    });
  ASSERT_TRUE(until([&] {return save->isEnabled();}));
  response.start(); save->click(); response.stop();
  ASSERT_TRUE(until([&] {return bridge.requests("operate").size() > before && save->isEnabled();}));
  EXPECT_EQ(bridge.requests("operate").last()["data"].toObject()["operation"], "mission_save");
  const auto saved = bridge.requests("operate").last()["data"].toObject()["values"].toObject();
  EXPECT_EQ(saved["name"].toString(), QStringLiteral("巡检任务.yaml"));
  EXPECT_EQ(saved["content"].toString(), content);
  const auto acknowledgement = page.property("onboardMissionSaved").value<QJsonObject>();
  EXPECT_EQ(acknowledgement["name"].toString(), QStringLiteral("巡检任务.yaml"));
  EXPECT_EQ(acknowledgement["sha256"], candidate["hash"]);
  EXPECT_EQ(acknowledgement["target"].toObject(), target);
  const auto requests_before_focus = bridge.requests("operate").size();
  page.focusMission(QStringLiteral("巡检任务.yaml"));
  EXPECT_EQ(list->currentItem()->text(), QStringLiteral("巡检任务.yaml"));
  EXPECT_EQ(bridge.requests("operate").size(), requests_before_focus);
  EXPECT_FALSE(permit->isEnabled());
  page.setMissionCenterActive(false);
  EXPECT_FALSE(read->isEnabled()); EXPECT_FALSE(save->isEnabled());
  EXPECT_FALSE(lookup->isEnabled());
  EXPECT_EQ(
    page.property("onboardMissionContext").value<QJsonObject>()["task_request"],
    QString(32, 'e'));
  page.setMissionCenterActive(true);
  ASSERT_TRUE(until([&] {return lookup->isEnabled();}));
  auto wrong_target = target; wrong_target["server_id"] = "another-platform";
  candidate["target"] = wrong_target; page.setMissionCandidate(candidate);
  EXPECT_FALSE(save->isEnabled());
  candidate["target"] = target;
  const QStringList invalid_names{"mission.yml", "mission.v2.yaml", "e\u0301.yaml",
    QString(50, QChar(0x5de1)) + ".yaml"};
  for (const auto & name : invalid_names) {
    candidate["name"] = name; page.setMissionCandidate(candidate);
    EXPECT_FALSE(save->isEnabled());
    EXPECT_TRUE(
      page.findChild<QLabel *>("onboardMissionCandidate")->text().contains(
        name.toUtf8().size() > 128 ? QStringLiteral("128 字节") : QStringLiteral("仅支持 .yaml")));
  }
  for (const auto & call : bridge.requests("operate")) {
    const auto operation = call["data"].toObject()["operation"].toString();
    EXPECT_TRUE(operation == "mission_read" || operation == "mission_save");
  }
}

TEST(OnboardInteractions, NotReadyReadStepExplainsWhyExecutionIsUnavailable)
{
  FakeBridge bridge;
  const QString content =
    "schema_version: 1\nframe_id: odom\nunits: {position: m, angle: rad, time: s}\n"
    "failure_policy: abort\nsteps: [{type: explore, min_bound: [-2,-2,0.5], "
    "max_bound: [2,2,2.5], return_home: true}]\n";
  bridge.control["mission_catalog"] = QJsonObject{{"explore.yaml", content}};
  bridge.control["task_step_support"] = QJsonObject{{"explore", QJsonObject{
      {"state", "not_ready"}, {"reason", "FUEL input map is not ready"}}}};
  bridge.control["mission_step_types"] = QJsonArray{"explore"}; bridge.saveControl();
  easyfly_connection_manager::OnboardPage page(bridge.runtime()); page.show();
  ASSERT_TRUE(connectLocal(page));
  auto * read = page.findChild<QPushButton *>("onboardMissionRead");
  ASSERT_TRUE(until([&] {return read->isEnabled();})); read->click();
  auto * policy = page.findChild<QLabel *>("onboardTaskPolicy");
  ASSERT_TRUE(until([&] {return policy->text().contains("FUEL");}));
  auto * execute = page.findChild<QPushButton *>("onboardMissionStart");
  EXPECT_FALSE(execute->isEnabled()); execute->click();
  for (const auto & call : bridge.requests("operate")) {
    EXPECT_NE(call["data"].toObject()["operation"].toString(), "mission_start");
  }
}
