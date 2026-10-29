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
#ifndef EASYFLY_DESKTOP__PARAMETER_CONTROLS_HPP_
#define EASYFLY_DESKTOP__PARAMETER_CONTROLS_HPP_
#include <QWidget>
#include <functional>
#include <memory>
#include <string>
#include <easyfly_ui_control/parameters.hpp>
namespace easyfly_desktop
{
struct ParameterOptions {bool enabled{false};};
class ParameterPort
{
public:
  virtual ~ParameterPort() = default;
  virtual std::shared_ptr<const easyfly_ui_control::ParameterSnapshot> snapshot() const = 0;
  virtual bool read(easyfly_ui_control::LocalParameter parameter) = 0;
  virtual bool write_confirmed(
    const easyfly_ui_control::ParameterReview & review,
    const easyfly_ui_control::ParameterValue & desired, const std::string & confirmation) = 0;
  virtual void inhibit(bool active) = 0;
  virtual void stop() = 0;
};
using ParameterFactory = std::function<std::unique_ptr<ParameterPort>()>;
ParameterFactory parameter_factory();
class ParameterPanel : public QWidget
{
  Q_OBJECT

public:
  explicit ParameterPanel(
    const ParameterOptions & options, ParameterFactory factory,
    QWidget * parent = nullptr);
  ~ParameterPanel() override;
  void observer_available(bool available);
  void exit_question(bool active);
  bool busy() const;
  bool fault() const;
  bool stopped() const;
  void stop();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace easyfly_desktop
#endif  // EASYFLY_DESKTOP__PARAMETER_CONTROLS_HPP_
