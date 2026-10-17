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

#ifndef EASYFLY_DESKTOP__TOOLS_HPP_
#define EASYFLY_DESKTOP__TOOLS_HPP_
#include <QProcess>
#include <QProcessEnvironment>
#include <QString>
#include <QStringList>
#include <QTemporaryDir>
#include <QTimer>
#include <memory>

#include "easyfly_desktop/source.hpp"
namespace easyfly_desktop
{
enum class ToolKind {Rviz, Qgc};
struct ToolOptions
{
  QString rviz_path;
  QString qgc_path;
  QString sandbox_path;
};
struct ToolCommand
{
  QString program, error;
  QStringList arguments;
  QProcessEnvironment environment;
};
ToolOptions default_tool_options();
QString rviz_preview(const bridge::BridgeConfig & config, int preset);
ToolCommand tool_command(
  ToolKind kind, const ToolOptions & options, const QString & directory,
  const bridge::BridgeConfig & config, int preset);
enum class ToolState {Idle, Starting, Running, Stopping, Exited, Failed};
// Owns exactly one launched process. Never discovers or signals external PIDs.
// Production commands wrap descendants in an owned PID namespace.
class ToolProcess
{
public:
  ToolProcess();
  ~ToolProcess();
  bool start(
    ToolKind kind, const ToolOptions & options, const bridge::BridgeConfig & config,
    int preset = 3);
  bool active() const;
  void stop();
  ToolState state() const {return state_;}
  QString status() const;
  QString log() const {return log_;}
  qint64 pid() const {return process_.processId();}

private:
  QProcess process_;
  QTimer deadline_, escalation_;
  std::unique_ptr<QTemporaryDir> directory_;
  ToolState state_{ToolState::Idle};
  QString log_, detail_;
  void drain();
};
}  // namespace easyfly_desktop
#endif  // EASYFLY_DESKTOP__TOOLS_HPP_
