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
#include <QDir>
#include <QFileInfo>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTextStream>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <vector>

#include "easyfly_connection_manager/profile.hpp"

int main(int argc, char ** argv)
{
  QCoreApplication app(argc, argv); app.setApplicationName("EasyFly");
  QCommandLineParser parser; parser.addHelpOption();
  parser.addOption(
    {"config-file", "Private absolute configuration file", "path",
      easyfly_connection_manager::default_config_file()});
  parser.addOption(
    {"runtime-directory", "Private absolute runtime directory", "path",
      easyfly_connection_manager::default_runtime_directory()});
  parser.process(app);
  try {
    const auto program = easyfly_connection_manager::desktop_program();
    QFile manifest(QFileInfo(program).dir().absoluteFilePath(
        "../../share/easyfly_desktop/connection_protocol.json"));
    if (!manifest.open(QIODevice::ReadOnly) || manifest.size() > 4096 ||
      QJsonDocument::fromJson(manifest.readAll()).object()["connection_page_abi"].toInt() != 1)
    {
      throw std::runtime_error("当前桌面缺少整合连接页，请同步安装 EasyFly 桌面和连接管理组件");
    }
    for (const auto * key : {"EASYFLY_CONNECTION_SOCKET", "EASYFLY_CONNECTION_SESSION",
        "EASYFLY_CONNECTION_TOKEN", "EASYFLY_CONNECTION_OPERATIONS", "EASYFLY_CONNECTION_ID",
        "EASYFLY_CONNECTION_HASH", "EASYFLY_CONNECTION_NAME", "EASYFLY_CONNECTION_CONFIG"})
    {
      qunsetenv(key);
    }
    QList<QByteArray> arguments{QFile::encodeName(program), "--connection-lobby",
      "--connection-config", parser.value("config-file").toUtf8(),
      "--connection-runtime", parser.value("runtime-directory").toUtf8()};
    std::vector<char *> values;
    for (auto & argument : arguments) {
      values.push_back(argument.data());
    }
    values.push_back(nullptr);
    ::execv(values.front(), values.data());
    QTextStream(stderr) << "Cannot open EasyFly: " << std::strerror(errno) << '\n';
    return 2;
  } catch (const std::exception & error) {
    QTextStream(stderr) << error.what() << '\n'; return 2;
  }
}
