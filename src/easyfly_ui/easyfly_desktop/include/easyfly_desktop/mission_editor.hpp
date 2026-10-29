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
#ifndef EASYFLY_DESKTOP__MISSION_EDITOR_HPP_
#define EASYFLY_DESKTOP__MISSION_EDITOR_HPP_
#include <QWidget>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>
namespace easyfly_desktop
{
struct ExportedDraft {uint64_t revision; std::string name, content, hash;};
// Offline only: exports a value snapshot to a separate opt-in import panel.
// No flight clients, task start/cancel or catalog write capability.
class MissionEditor : public QWidget
{
  Q_OBJECT

public:
  explicit MissionEditor(
    QWidget * parent = nullptr, std::vector<std::string> protected_paths = {},
    std::function<void()> changed = {});
  ~MissionEditor() override;
  bool dirty() const;
  bool busy() const;
  bool stopped() const;
  std::optional<ExportedDraft> exported_draft() const;
  void stop();
  void open_file(const QString & path);
  void prepare_directory(const QString & path);
  void export_file(const QString & filename);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace easyfly_desktop
#endif  // EASYFLY_DESKTOP__MISSION_EDITOR_HPP_
