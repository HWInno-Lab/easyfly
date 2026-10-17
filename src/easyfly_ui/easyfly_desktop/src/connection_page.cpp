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

#include <QDir>
#include <QLibrary>
#include <QStandardPaths>
#include <unistd.h>
#include <exception>
#include <ament_index_cpp/get_package_prefix.hpp>
#include "connection_page.hpp"
#include "easyfly_desktop/main_window.hpp"

namespace easyfly_desktop
{
ConnectionPageHandle load_connection_page(
  MainWindow * window, const QString & config_file, const QString & runtime, bool lobby)
{
  ConnectionPageHandle result;
  result.config_file = config_file.isEmpty() ? QDir(
    QStandardPaths::writableLocation(
      QStandardPaths::GenericConfigLocation)).absoluteFilePath("easyfly/connections.json") :
    config_file;
  result.runtime = runtime.isEmpty() ? QDir(qEnvironmentVariable("XDG_RUNTIME_DIR", "/tmp"))
    .absoluteFilePath("easyfly-connections-" + QString::number(::getuid())) : runtime;
  try {
    const auto prefix = QString::fromStdString(
      ament_index_cpp::get_package_prefix("easyfly_connection_manager"));
    auto * library = new QLibrary(prefix + "/lib/libeasyfly_connection_page.so", window);
    library->setLoadHints(QLibrary::PreventUnloadHint);
    using Version = int (*)();
    using Create = QWidget * (*)(QWidget *, const char *, const char *);
    const auto version = reinterpret_cast<Version>(library->resolve("easyfly_connection_page_abi"));
    const auto create =
      reinterpret_cast<Create>(library->resolve("easyfly_create_connection_page"));
    if (!version || !create || version() != 1) {
      result.error = QStringLiteral("连接页组件缺失或版本不匹配：") + library->errorString();
      return result;
    }
    const auto file = result.config_file.toUtf8();
    const auto directory = result.runtime.toUtf8();
    result.widget = create(window, file.constData(), directory.constData());
    if (!result.widget) {
      result.error = QStringLiteral("无法创建连接页，请检查配置路径、权限及运行日志。");
      return result;
    }
    result.launcher = prefix + "/lib/easyfly_connection_manager/easyfly_connection_manager";
    window->set_connection_page(result.widget, lobby);
  } catch (const std::exception & error) {
    result.error = QString::fromUtf8(error.what());
  }
  return result;
}
}  // namespace easyfly_desktop
