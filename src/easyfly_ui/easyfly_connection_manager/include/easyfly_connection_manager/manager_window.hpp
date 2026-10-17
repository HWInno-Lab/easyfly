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
#include <QMainWindow>
#include <memory>

namespace easyfly_connection_manager
{
class ManagerWindow : public QMainWindow
{
public:
  ManagerWindow(const QString & config_file, const QString & runtime, QWidget * parent = nullptr);
  ~ManagerWindow() override;

protected:
  void resizeEvent(QResizeEvent * event) override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace easyfly_connection_manager
