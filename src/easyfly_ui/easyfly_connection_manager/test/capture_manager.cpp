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

#include <QApplication>
#include <QCommandLineParser>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QPushButton>
#include <QScrollArea>
#include <QScrollBar>
#include <QTabWidget>
#include <QTimer>
#include <cstdio>
#include "easyfly_connection_manager/manager_window.hpp"
#include "easyfly_connection_manager/profile.hpp"

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);
  QCommandLineParser parser; parser.addHelpOption();
  parser.addOption({"config-file", "Existing private configuration", "path"});
  parser.addOption({"runtime-directory", "Existing supervisor runtime", "path"});
  parser.addOption({"output-directory", "New private screenshot directory", "path"});
  parser.process(app);
  try {
    const auto runtime =
      easyfly_connection_manager::private_directory(parser.value("runtime-directory"), false);
    if (!QFileInfo::exists(runtime + "/control.sock")) {return 2;}
    const auto output =
      easyfly_connection_manager::private_directory(parser.value("output-directory"));
    if (!QDir(output).entryList(QDir::Files | QDir::NoDotAndDotDot).isEmpty()) {return 2;}
    easyfly_connection_manager::ManagerWindow window(parser.value("config-file"), runtime);
    window.show();
    QTimer::singleShot(
      2500, &app, [&] {
        auto * tabs = window.findChild<QTabWidget *>();
        if (!tabs) {app.exit(2); return;}
        for (int i = 0; i < tabs->count(); ++i) {
          tabs->setCurrentIndex(i); QApplication::processEvents();
          if (!window.grab().save(output + QString("/page-%1.png").arg(i))) {app.exit(2); return;}
        }
        tabs->setCurrentIndex(1);
        auto * expand = window.findChild<QPushButton *>("toggleSlamAdvanced");
        if (!expand) {app.exit(2); return;}
        expand->click();
        for (const auto & size : {QSize(1200, 740), QSize(850, 570)}) {
          window.resize(size); QApplication::processEvents();
          auto * area = qobject_cast<QScrollArea *>(tabs->currentWidget());
          area->ensureWidgetVisible(window.findChild<QWidget *>("slamAdvanced"), 0, 10);
          QApplication::processEvents();
          if (area->horizontalScrollBar()->maximum() != 0 ||
          !window.grab().save(output + QString("/slam-advanced-%1.png").arg(size.width())))
          {app.exit(2); return;}
        }
        app.quit();
      });
    return app.exec();
  } catch (const std::exception & error) {
    std::fprintf(stderr, "%s\n", error.what()); return 2;
  }
}
