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

#ifndef EASYFLY_DESKTOP__MISSION_DRAFT_HPP_
#define EASYFLY_DESKTOP__MISSION_DRAFT_HPP_
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <easyfly_mission_format/mission_format.hpp>
namespace easyfly_desktop
{
// Protect relocated installed packages and the executable's installed prefix, not just build paths.
std::vector<std::string> local_storage_protected_paths(
  std::vector<std::string> configured,
  const std::map<std::string, std::string> & installed_packages,
  const std::string & executable_path);
struct DraftField {std::string key, unit; double value; bool integer{false};};
std::vector<DraftField> draft_fields(const easyfly_mission_fsm::MissionStep & step);
bool draft_set_field(
  easyfly_mission_fsm::MissionStep & step, const std::string & key,
  double value);
easyfly_mission_fsm::MissionStep draft_default_step(const std::string & type);
// Writes effective values explicitly. Comments/formatting are not preserved.
std::string draft_yaml(const std::vector<easyfly_mission_fsm::MissionStep> & steps);
struct DraftFingerprint
{
  bool exists{false};
  uint64_t device{0}, inode{0};
  uint64_t directory_device{0}, directory_inode{0};
  int64_t modified_ns{0}, changed_ns{0};
  std::string hash;
  bool operator==(const DraftFingerprint & other) const;
};
enum class DraftOperation {Validate, Normalize, Read, PrepareDirectory, InspectExport, Export};
struct DraftJob
{
  DraftOperation operation{DraftOperation::Validate};
  uint64_t revision{0};
  std::string content, path, filename;
  DraftFingerprint expected;
  std::vector<easyfly_mission_fsm::MissionStep> steps;
};
struct DraftResult
{
  uint64_t id{0};
  DraftJob job;
  bool success{false};
  std::string error, content, format, hash, path, backup;
  easyfly_mission_fsm::MissionValidation plan;
  DraftFingerprint fingerprint;
};
struct DraftWorkerState {bool active{false}, stopped{false}; size_t pending{0};};
// No ROS context, publishers, clients, flight or task-directory import capability.
class DraftWorker
{
public:
  explicit DraftWorker(std::vector<std::string> protected_paths = {});
  ~DraftWorker();
  DraftWorker(const DraftWorker &) = delete;
  DraftWorker & operator=(const DraftWorker &) = delete;
  uint64_t submit(DraftJob job);  // One active + one latest pending validation; I/O never replaced.
  std::shared_ptr<const DraftResult> latest() const;
  DraftWorkerState state() const;
  void stop();  // Nonblocking; release after stopped. Confirmed active I/O finishes normally.

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace easyfly_desktop
#endif  // EASYFLY_DESKTOP__MISSION_DRAFT_HPP_
