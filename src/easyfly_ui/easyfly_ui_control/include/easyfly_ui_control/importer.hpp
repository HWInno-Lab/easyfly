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
#ifndef EASYFLY_UI_CONTROL__IMPORTER_HPP_
#define EASYFLY_UI_CONTROL__IMPORTER_HPP_
#include <memory>
#include <optional>
#include <string>
namespace easyfly_ui_control
{
struct ImportDocument {std::string name, content, hash;};
struct ImportDestination
{
  std::string instance, catalog, directory;
  bool enabled{false};
  unsigned capacity{0}, retained{0};
  double received{0};
};
struct ImportIntent
{
  std::string scope, instance, catalog, request, name, hash, status;
  bool resolved{false}, success{false};
};
enum class ImportPhase {Idle, Reading, Ready, Precheck, Sent, Uncertain, Readback,
  Completed, Rejected, Blocked};
struct ImportSnapshot
{
  bool stopped{false}, ready{false}, pending{false}, active_create{false}, fault{false};
  ImportPhase phase{ImportPhase::Idle};
  std::optional<ImportDestination> destination;
  std::optional<ImportIntent> intent;
  std::string message;
  size_t confirmations{0};
};
struct ImportConfig {std::string journal_directory; double response_timeout{2};};
// Independent metadata/file client. No task execution, SDK, publishers or rollback.
// Durable identity before a single create; lookup-only after uncertainty/restart.
class MissionImporter
{
public:
  explicit MissionImporter(const ImportConfig & config);
  ~MissionImporter();
  MissionImporter(const MissionImporter &) = delete;
  MissionImporter & operator=(const MissionImporter &) = delete;
  std::shared_ptr<const ImportSnapshot> snapshot() const;
  bool read_destination();
  bool import_confirmed(
    const ImportDocument & document, const ImportDestination & destination,
    const std::string & confirmation);
  void inhibit(bool active);
  void stop();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace easyfly_ui_control
#endif  // EASYFLY_UI_CONTROL__IMPORTER_HPP_
