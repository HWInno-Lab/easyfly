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
#include <QCommandLineParser>
#include <QDir>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QDialog>
#include <QJsonArray>
#include <QJsonDocument>
#include <QFile>
#include <QScrollArea>
#include <QScrollBar>
#include <QTemporaryDir>
#include <QToolButton>
#include <QTabWidget>
#include <QVBoxLayout>
#include "easyfly_desktop/mission_editor.hpp"
#include "support.hpp"
#include "connection_page.hpp"
int main(int argc, char ** argv)
{
  QApplication app(argc, argv); app.setFont(QFont("Noto Sans CJK SC", 10));
  QCommandLineParser parser; parser.addHelpOption();
  parser.addOption({"config-file", "Private fixture config", "path"});
  parser.addOption({"runtime-directory", "Private fixture supervisor", "path"});
  parser.addOption({"output-directory", "Fixture screenshots", "path"}); parser.process(app);
  auto state = std::make_shared<desktop_test::FakeState>(); state->data = desktop_test::sample();
  easyfly_desktop::MainWindow window({}, desktop_test::factory(state));
  const auto page = easyfly_desktop::load_connection_page(
    &window,
    parser.value("config-file"), parser.value("runtime-directory"), false);
  if (!page.widget) {return 2;}
  auto * mark = new QLabel(QStringLiteral("构造界面预览 · 非实时飞机 · 不执行任务"), &window);
  static_cast<QVBoxLayout *>(window.centralWidget()->layout())->insertWidget(0, mark);
  window.show(); QTest::qWait(2500);
  const QDir output(parser.value("output-directory")); if (!output.mkpath(".")) {return 2;}
  bool ok = true; QJsonArray geometry;
  auto * editor = window.findChild<easyfly_desktop::MissionEditor *>();
  window.findChild<QPushButton *>("draftNew")->click();
  if (!desktop_test::until([&] {return editor->dirty() && !editor->busy();})) {return 2;}
  const auto shot = [&](const QString & name) {
      state->data = desktop_test::sample(); QTest::qWait(130);
      ok = window.grab().save(output.filePath(name + ".png")) && ok;
      QJsonArray widgets;
      for (const auto * id :
        {"navigation", "missionCatalogPage", "saveConnection", "startConnection",
          "cameraCanvas", "cameraControlsScroll", "perceptionObjectTabs", "draftSteps",
          "draftFields"})
      {
        auto * widget = window.findChild<QWidget *>(id);
        if (widget) {
          const auto point = widget->mapTo(&window, QPoint());
          widgets.append(
            QJsonObject{{"name", id}, {"visible", widget->isVisible()},
              {"x", point.x()}, {"y", point.y()}, {"width", widget->width()},
              {"height", widget->height()}});
        }
      }
      geometry.append(
        QJsonObject{{"screenshot", name}, {"width", window.width()},
          {"height", window.height()}, {"widgets", widgets}});
    };
  for (const auto & size : {QSize(1440, 900), QSize(1366, 768), QSize(1024, 720)}) {
    window.resize(size); const auto suffix = QString::number(size.width());
    for (const auto & key :
      {"connection", "overview", "tasks", "perception", "space", "records", "settings"})
    {
      window.show_page(key);
      if (QString(key) == "space") {
        window.findChild<QTabWidget *>("spaceViewTabs")->setCurrentIndex(1);
      }
      shot(QString(key) + "-" + suffix);
    }
    window.show_page("perception");
    auto * results = window.findChild<QToolButton *>("togglePerceptionResults"); results->click();
    shot("perception-results-" + suffix);
    auto * focus = window.findChild<QToolButton *>("cameraFocusMode"); focus->click();
    shot("perception-focus-" + suffix); focus->click(); results->click();
    window.show_connection_page(); auto * tabs = window.findChild<QTabWidget *>(
      "connectionConfigurationTabs");
    for (int i = 0; i < tabs->count(); ++i) {
      tabs->setCurrentIndex(i); shot("connection-tab-" + QString::number(i) + "-" + suffix);
    }
    if (size.width() == 1366) {
      window.findChild<QPushButton *>("manageConnections")->click(); QTest::qWait(80);
      auto * dialog = window.findChild<QDialog *>("connectionProfilesDialog");
      ok = dialog->grab().save(output.filePath("profile-management.png")) && ok; dialog->hide();
    }
    tabs->setCurrentIndex(1);
    auto * advanced = window.findChild<QPushButton *>("toggleAlgorithmAdvanced"); advanced->click();
    shot("algorithm-advanced-" + suffix); advanced->click();
    window.show_page("tasks"); tabs = window.findChild<QTabWidget *>("taskCenterTabs");
    tabs->setCurrentIndex(0);
    auto * execution = window.findChild<QToolButton *>("toggleTaskExecution");
    execution->click(); shot("task-execution-" + suffix); execution->click();
    for (int i = 0; i < tabs->count(); ++i) {
      tabs->setCurrentIndex(i); shot("task-tab-" + QString::number(i) + "-" + suffix);
    }
  }
  QFile report(output.filePath("layout-geometry.json"));
  if (!report.open(QIODevice::WriteOnly)) {return 2;}
  report.write(QJsonDocument(geometry).toJson()); report.close();
  QTemporaryDir drafts;
  editor->prepare_directory(drafts.path() + "/drafts");
  if (!desktop_test::until([&] {return !editor->busy();})) {return 2;}
  editor->export_file("fixture.yaml");
  if (!desktop_test::until([&] {return !editor->dirty();})) {return 2;}
  window.close();
  if (!desktop_test::until([&] {return window.cleanup_complete();})) {return 2;}
  return ok ? 0 : 2;
}
