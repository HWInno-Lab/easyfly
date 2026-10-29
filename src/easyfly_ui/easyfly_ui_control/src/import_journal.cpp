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
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <json/json.h>
#include <algorithm>
#include <cerrno>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <vector>
#include "import_journal.hpp"
#include <easyfly_mission_format/mission_format.hpp>
namespace easyfly_ui_control
{
namespace
{
struct Fd {int fd; ~Fd() {if (fd >= 0) {close(fd);}}};
std::string json(const Json::Value & v)
{Json::StreamWriterBuilder b; b["indentation"] = ""; return Json::writeString(b, v);}
void check(const ImportIntent & v)
{
  bool scope = false;
  for (int d = 0; d <= 232; ++d) {
    if (v.scope == "domain=" + std::to_string(d) + ";localhost=1") {scope = true;}
  }
  const auto ext = std::filesystem::path(v.name).extension();
  if (!scope || !hex_id(v.instance, 32) || !hex_id(v.catalog, 32) || !hex_id(v.request, 32) ||
    !catalog_name(v.name) || (ext != ".yaml" && ext != ".yml") || !hex_id(v.hash, 64) ||
    v.status.empty() || v.status.size() > 64 || !easyfly_mission_fsm::missionTextUtf8(v.status) ||
    (v.success && !v.resolved)) {throw std::runtime_error("invalid import intent identity");}
}
}  // namespace
ImportJournal::ImportJournal(const std::string & directory)
: lock_(directory)
{
  directory_ = open(directory.c_str(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
  struct stat info {};
  const int flight = directory_ < 0 ? 0 : fstatat(
    directory_, "intent.json", &info,
    AT_SYMLINK_NOFOLLOW);
  if (directory_ < 0 || flight == 0 || errno != ENOENT) {
    if (directory_ >= 0) {close(directory_); directory_ = -1;}
    throw std::runtime_error("dedicated import journal required; flight journal not allowed");
  }
}
ImportJournal::~ImportJournal() {if (directory_ >= 0) {close(directory_);}}
std::optional<ImportIntent> ImportJournal::load() const
{
  Fd file{openat(directory_, "import.json", O_RDONLY | O_NOFOLLOW | O_NONBLOCK | O_CLOEXEC)};
  if (file.fd < 0 && errno == ENOENT) {return {};}
  struct stat info {};
  if (file.fd < 0 || fstat(file.fd, &info) || !S_ISREG(info.st_mode) || info.st_uid != geteuid() ||
    (info.st_mode & 077) || info.st_nlink != 1 || info.st_size < 0 || info.st_size > 4096)
  {throw std::runtime_error("invalid private import journal file");}
  std::string bytes(4097, '\0'); size_t size = 0;
  while (size < bytes.size()) {
    const auto n = read(file.fd, bytes.data() + size, bytes.size() - size);
    if (n < 0 && errno == EINTR) {continue;}
    if (n < 0) {throw std::runtime_error("import journal read failed");}
    if (!n) {break;} size += static_cast<size_t>(n);
  }
  if (size > 4096) {throw std::runtime_error("import journal too large");} bytes.resize(size);
  Json::CharReaderBuilder b; b["rejectDupKeys"] = true; b["failIfExtra"] = true;
  b["allowComments"] = false; b["allowTrailingCommas"] = false; b["stackLimit"] = 16;
  std::unique_ptr<Json::CharReader> reader(b.newCharReader()); Json::Value root; std::string error;
  if (!reader->parse(
      bytes.data(), bytes.data() + bytes.size(), &root,
      &error) || !root.isObject())
  {
    throw std::runtime_error("corrupt import journal");
  }
  std::vector<std::string> keys{"schema_version", "kind", "scope", "instance", "catalog",
    "request", "name", "hash", "status", "resolved", "success", "checksum"};
  std::sort(keys.begin(), keys.end());
  if (root.getMemberNames() != keys || !root["schema_version"].isInt() ||
    root["schema_version"].asInt() != 1 || !root["resolved"].isBool() || !root["success"].isBool())
  {throw std::runtime_error("invalid import journal schema");}
  for (const auto & k : {"kind", "scope", "instance", "catalog", "request", "name", "hash",
      "status", "checksum"})
  {
    if (!root[k].isString()) {throw std::runtime_error("invalid import journal field");}
  }
  if (root["kind"].asString() != "catalog_import") {throw std::runtime_error("wrong journal kind");}
  const auto checksum = root["checksum"].asString(); root.removeMember("checksum");
  if (!hex_id(checksum, 64) || checksum != easyfly_mission_fsm::missionContentSha256(json(root))) {
    throw std::runtime_error("import journal integrity mismatch");
  }
  ImportIntent v{root["scope"].asString(), root["instance"].asString(), root["catalog"].asString(),
    root["request"].asString(), root["name"].asString(), root["hash"].asString(),
    root["status"].asString(), root["resolved"].asBool(), root["success"].asBool()};
  check(v); return v;
}
void ImportJournal::save(const ImportIntent & v) const
{
  check(v); Json::Value root;
  root["schema_version"] = 1; root["kind"] = "catalog_import"; root["scope"] = v.scope;
  root["instance"] = v.instance; root["catalog"] = v.catalog; root["request"] = v.request;
  root["name"] = v.name; root["hash"] = v.hash; root["status"] = v.status;
  root["resolved"] = v.resolved; root["success"] = v.success;
  root["checksum"] = easyfly_mission_fsm::missionContentSha256(json(root));
  const auto bytes = json(root); const auto stage = "import-" + new_id() + ".tmp";
  Fd file{openat(
      directory_,
      stage.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0600)};
  if (file.fd < 0) {throw std::runtime_error("cannot prepare import journal");}
  try {
    size_t count = 0;
    while (count < bytes.size()) {
      const auto n = write(file.fd, bytes.data() + count, bytes.size() - count);
      if (n < 0 && errno == EINTR) {continue;}
      if (n <= 0) {throw std::runtime_error("import journal write failed");}
      count += static_cast<size_t>(n);
    }
    if (fsync(file.fd) || renameat(directory_, stage.c_str(), directory_, "import.json") ||
      fsync(directory_)) {throw std::runtime_error("import journal durable commit failed");}
  } catch (...) {
    unlinkat(directory_, stage.c_str(), 0); throw;
  }
}
}  // namespace easyfly_ui_control
