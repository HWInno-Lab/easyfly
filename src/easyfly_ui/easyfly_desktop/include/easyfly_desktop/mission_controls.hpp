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
#ifndef EASYFLY_DESKTOP__MISSION_CONTROLS_HPP_
#define EASYFLY_DESKTOP__MISSION_CONTROLS_HPP_
#include <QWidget>
#include <functional>
#include <memory>
#include <string>
#include <easyfly_ui_control/controller.hpp>
#include "easyfly_desktop/source.hpp"
namespace easyfly_desktop
{
namespace control = easyfly_ui_control;
struct MissionOptions
{
  bool enabled{false};  // Explicit opt-in; no controls or controller in the default window.
  std::string journal_directory;
};
class ControlPort
{
public:
  virtual ~ControlPort() = default;
  virtual std::shared_ptr<const control::ControlSnapshot> snapshot() const = 0;
  virtual bool start_confirmed(
    const easyfly_msgs::srv::GetMissionDetails::Response &, double, const std::string &,
    const std::string & confirmation) = 0;
  virtual bool cancel_confirmed() = 0;
  virtual void stop() = 0;
};
using ControlFactory = std::function<std::unique_ptr<ControlPort>(const control::ControlConfig &)>;
ControlFactory control_factory();
// Fixed whitelisted reads through the existing bounded Observer; never a control RPC.
using ReviewRead = std::function<bridge::RequestReceipt(bridge::Key, const std::string &)>;
class MissionPanel : public QWidget
{
public:
  explicit MissionPanel(
    const MissionOptions & options, ControlFactory factory, QWidget * parent = nullptr,
    ReviewRead read = {});
  ~MissionPanel() override;
  void observe(
    std::shared_ptr<const bridge::UiSnapshot> snapshot, const std::string & selection,
    bool details_waiting, bool observer_available, bool simulation_clock);
  bool unresolved() const;
  bool stopped() const;
  std::shared_ptr<const control::ControlSnapshot> record_state() const;
  void stop();
  void exit_question(bool active);  // Dismiss any operation prompt and disable new operations.

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace easyfly_desktop
#endif  // EASYFLY_DESKTOP__MISSION_CONTROLS_HPP_
