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
#pragma once
#include <QString>
class QWidget;
namespace easyfly_desktop
{
class MainWindow;
struct ConnectionPageHandle
{
  QWidget * widget{nullptr};
  QString launcher, config_file, runtime, error;
};
ConnectionPageHandle load_connection_page(
  MainWindow * window, const QString & config_file, const QString & runtime, bool lobby);
}  // namespace easyfly_desktop
