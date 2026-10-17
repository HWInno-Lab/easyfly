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
#include <QCommandLineParser>
#include <QTextStream>

#include "easyfly_connection_manager/profile.hpp"
#include "easyfly_connection_manager/supervisor.hpp"

int main(int argc, char ** argv)
{
  QCoreApplication app(argc, argv);
  QCoreApplication::setApplicationName("easyfly_connection_supervisor");
  QCommandLineParser parser; parser.addHelpOption();
  parser.addOption({"runtime-directory", "Private absolute runtime directory", "path"});
  parser.process(app);
  try {
    easyfly_connection_manager::Supervisor supervisor(parser.value("runtime-directory"),
      easyfly_connection_manager::desktop_program());
    return app.exec();
  } catch (const std::exception & error) {
    QTextStream(stderr) << error.what() << '\n'; return 2;
  }
}
