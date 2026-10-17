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
#include <QTabWidget>
#include <QVBoxLayout>
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
  bool ok = true;
  const auto shot = [&](const QString & name) {
      state->data = desktop_test::sample(); QTest::qWait(130);
      ok = window.grab().save(output.filePath(name + ".png")) && ok;
    };
  for (const auto & size : {QSize(1440, 900), QSize(1024, 720)}) {
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
    window.show_connection_page(); auto * tabs = window.findChild<QTabWidget *>(
      "connectionConfigurationTabs");
    for (int i = 0; i < tabs->count(); ++i) {
      tabs->setCurrentIndex(i); shot("connection-tab-" + QString::number(i) + "-" + suffix);
    }
    window.show_page("tasks"); tabs = window.findChild<QTabWidget *>("taskCenterTabs");
    for (int i = 0; i < tabs->count(); ++i) {
      tabs->setCurrentIndex(i); shot("task-tab-" + QString::number(i) + "-" + suffix);
    }
  }
  window.close();
  if (!desktop_test::until([&] {return window.cleanup_complete();})) {return 2;}
  return ok ? 0 : 2;
}
