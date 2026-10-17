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

#ifndef EASYFLY_DESKTOP__MAIN_WINDOW_HPP_
#define EASYFLY_DESKTOP__MAIN_WINDOW_HPP_
#include <QMainWindow>
#include <QJsonObject>

#include <memory>

#include "easyfly_desktop/source.hpp"
#include "easyfly_desktop/tools.hpp"
#include "easyfly_desktop/mission_controls.hpp"
#include "easyfly_desktop/parameter_controls.hpp"
#include "easyfly_desktop/import_controls.hpp"
#include "easyfly_desktop/rviz_view.hpp"
#include "easyfly_desktop/recording_panel.hpp"
namespace easyfly_desktop
{
class MainWindow : public QMainWindow
{
public:
  explicit MainWindow(
    const bridge::BridgeConfig & config = bridge::BridgeConfig{},
    SourceFactory factory = bridge_factory(), QWidget * parent = nullptr,
    const ToolOptions & tools = default_tool_options(),
    const MissionOptions & missions = MissionOptions{},
    ControlFactory controls = control_factory(), const ParameterOptions & parameters = {},
    ParameterFactory parameter_controls = parameter_factory(), const ImportOptions & imports = {},
    ImportFactory import_controls = import_factory(), const RvizOptions & spatial = {},
    const RecordingOptions & recording = {});
  ~MainWindow() override;
  bool cleanup_complete() const;
  // Bounded read-only management status; no mission content, paths or control handles.
  QJsonObject connection_status() const;

protected:
  void closeEvent(QCloseEvent * event) override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace easyfly_desktop
#endif  // EASYFLY_DESKTOP__MAIN_WINDOW_HPP_
