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
#ifndef EASYFLY_DESKTOP__SPECIAL_OBSERVATION_PANEL_HPP_
#define EASYFLY_DESKTOP__SPECIAL_OBSERVATION_PANEL_HPP_
#include <QWidget>
#include <memory>
#include <easyfly_ui_bridge/model.hpp>
#include <easyfly_ui_control/controller.hpp>
namespace easyfly_desktop
{
class SpecialObservationPanel : public QWidget
{
public:
  explicit SpecialObservationPanel(bool landing, QWidget * parent = nullptr);
  ~SpecialObservationPanel() override;
  void observe(
    const easyfly_ui_bridge::UiSnapshot &, double now,
    std::shared_ptr<const easyfly_ui_control::ControlSnapshot> owned);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace easyfly_desktop
#endif  // EASYFLY_DESKTOP__SPECIAL_OBSERVATION_PANEL_HPP_
