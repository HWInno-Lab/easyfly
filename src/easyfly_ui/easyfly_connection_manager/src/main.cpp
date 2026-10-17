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
#include <QFont>
#include <QTextStream>

#include "easyfly_connection_manager/manager_window.hpp"
#include "easyfly_connection_manager/profile.hpp"

int main(int argc, char ** argv)
{
  QApplication app(argc, argv); app.setApplicationName("EasyFly Connections");
  app.setFont(QFont("Noto Sans CJK SC", 10));
  QCommandLineParser parser; parser.addHelpOption();
  parser.addOption(
    {"config-file", "Private absolute configuration file", "path",
      easyfly_connection_manager::default_config_file()});
  parser.addOption(
    {"runtime-directory", "Private absolute runtime directory", "path",
      easyfly_connection_manager::default_runtime_directory()});
  parser.process(app);
  try {
    easyfly_connection_manager::ManagerWindow window(parser.value("config-file"),
      parser.value("runtime-directory"));
    window.show(); return app.exec();
  } catch (const std::exception & error) {
    QTextStream(stderr) << error.what() << '\n'; return 2;
  }
}
