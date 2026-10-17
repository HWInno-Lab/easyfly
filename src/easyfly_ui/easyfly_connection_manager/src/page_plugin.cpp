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

#include <QDebug>
#include <exception>
#include "easyfly_connection_manager/manager_window.hpp"

extern "C" Q_DECL_EXPORT int easyfly_connection_page_abi()
{
  return 1;
}
extern "C" Q_DECL_EXPORT QWidget * easyfly_create_connection_page(
  QWidget * parent, const char * config_file, const char * runtime)
{
  if (!parent || !config_file || !runtime) {return nullptr;}
  try {
    auto * page = new easyfly_connection_manager::ManagerWindow(
      QString::fromUtf8(config_file), QString::fromUtf8(runtime), parent);
    page->setObjectName("integratedConnectionPage");
    return page;
  } catch (const std::exception & error) {
    qWarning("EasyFly connection page: %s", error.what());
    return nullptr;
  }
}
