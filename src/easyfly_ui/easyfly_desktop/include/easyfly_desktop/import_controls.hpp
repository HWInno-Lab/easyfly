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

#ifndef EASYFLY_DESKTOP__IMPORT_CONTROLS_HPP_
#define EASYFLY_DESKTOP__IMPORT_CONTROLS_HPP_
#include <QWidget>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <easyfly_ui_control/importer.hpp>
#include "easyfly_desktop/mission_editor.hpp"
namespace easyfly_desktop
{
struct ImportOptions {bool enabled{false}; std::string journal_directory;};
class ImportPort
{
public:
  virtual ~ImportPort() = default;
  virtual std::shared_ptr<const easyfly_ui_control::ImportSnapshot> snapshot() const = 0;
  virtual bool read_destination() = 0;
  virtual bool import_confirmed(
    const easyfly_ui_control::ImportDocument & document,
    const easyfly_ui_control::ImportDestination & destination,
    const std::string & confirmation) = 0;
  virtual void inhibit(bool active) = 0;
  virtual void stop() = 0;
};
using ImportFactory =
  std::function<std::unique_ptr<ImportPort>(const easyfly_ui_control::ImportConfig &)>;
ImportFactory import_factory();
class ImportPanel : public QWidget
{
  Q_OBJECT

public:
  ImportPanel(
    const ImportOptions & options, ImportFactory factory, QWidget * parent = nullptr,
    std::function<std::optional<ExportedDraft>()> live_draft = {});
  ~ImportPanel() override;
  void observe(std::optional<ExportedDraft> draft, bool available);
  void exit_question(bool active);
  bool busy() const;
  bool unresolved() const;
  bool stopped() const;
  void stop();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace easyfly_desktop
#endif  // EASYFLY_DESKTOP__IMPORT_CONTROLS_HPP_
