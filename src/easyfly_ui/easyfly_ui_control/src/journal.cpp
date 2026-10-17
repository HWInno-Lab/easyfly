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
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>
#include <json/json.h>
#include <algorithm>
#include <cerrno>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <easyfly_mission_format/mission_format.hpp>
#include "journal.hpp"

namespace easyfly_ui_control
{
namespace
{
struct Guard {int fd; ~Guard() {if (fd >= 0) {close(fd);}}};
void secureFile(int fd, size_t limit)
{
  struct stat st {};
  if (fstat(fd, &st) || !S_ISREG(st.st_mode) || st.st_uid != geteuid() ||
    st.st_nlink != 1 || (st.st_mode & 077) || st.st_size < 0 ||
    static_cast<size_t>(st.st_size) > limit)
  {throw std::runtime_error("journal file ownership/type/mode/size invalid");}
}
bool scopeValid(const std::string & scope)
{
  for (int domain = 0; domain <= 232; ++domain) {
    const auto prefix = "domain=" + std::to_string(domain) + ";localhost=";
    if (scope == prefix + "0" || scope == prefix + "1") {return true;}
  }
  return false;
}
void check(const Intent & v)
{
  if (!scopeValid(v.scope) || !hex_id(v.instance, 32) || !hex_id(v.request, 32) ||
    !catalog_name(v.name) || !hex_id(v.hash, 64) ||
    (!v.execution.empty() && !hex_id(v.execution, 32)) ||
    (!v.goal.empty() && (!hex_id(v.goal, 32) || v.execution.empty())) ||
    (v.cancel_dispatched && v.execution.empty()))
  {throw std::runtime_error("invalid journal identity");}
}
}  // namespace
bool hex_id(const std::string & value, size_t size)
{
  return value.size() == size && std::all_of(
    value.begin(), value.end(), [](char c) {
      return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
    });
}
bool catalog_name(const std::string & value)
{
  if (value.empty() || value.size() > 128 || value.front() == '.' ||
    std::filesystem::path(value).filename() != value ||
    !easyfly_mission_fsm::missionTextUtf8(value)) {return false;}
  const auto ext = std::filesystem::path(value).extension().string();
  return ext == ".mission" || ext == ".yaml" || ext == ".yml";
}
std::string new_id()
{
  std::random_device random; std::ostringstream out;
  for (int i = 0; i < 16; ++i) {
    out << std::hex << std::setw(2) << std::setfill('0') << (random() & 255);
  }
  return out.str();
}
Journal::Journal(const std::string & directory)
{
  const auto target = std::filesystem::path(directory);
  if (!target.is_absolute() || target.filename().empty() || target.filename() == "." ||
    target.filename() == "..")
  {
    throw std::invalid_argument("dedicated absolute journal directory required");
  }
  const bool created = mkdir(directory.c_str(), 0700) == 0;
  if (!created && errno != EEXIST) {
    throw std::runtime_error("cannot create journal directory");
  }
  if (created) {
    Guard parent{open(
        std::filesystem::path(directory).parent_path().c_str(),
        O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC)};
    if (parent.fd < 0 || fsync(parent.fd)) {
      throw std::runtime_error("cannot durably create journal directory");
    }
  }
  try {
    directory_ = open(directory.c_str(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
    struct stat st {};
    if (directory_ < 0 || fstat(directory_, &st) || st.st_uid != geteuid() || (st.st_mode & 077)) {
      throw std::runtime_error("private owned journal directory required");
    }
    lock_ = openat(directory_, "lock", O_RDWR | O_CREAT | O_NOFOLLOW | O_CLOEXEC, 0600);
    if (lock_ < 0) {throw std::runtime_error("cannot open journal lock");}
    secureFile(lock_, 0);
    if (flock(lock_, LOCK_EX | LOCK_NB)) {
      throw std::runtime_error("journal already owned by another controller");
    }
  } catch (...) {
    if (lock_ >= 0) {close(lock_);}
    if (directory_ >= 0) {close(directory_);}
    throw;
  }
}
Journal::~Journal()
{
  if (lock_ >= 0) {close(lock_);}
  if (directory_ >= 0) {close(directory_);}
}
std::optional<Intent> Journal::load() const
{
  Guard fd{openat(directory_, "intent.json", O_RDONLY | O_NOFOLLOW | O_NONBLOCK | O_CLOEXEC)};
  if (fd.fd < 0 && errno == ENOENT) {return std::nullopt;}
  if (fd.fd < 0) {throw std::runtime_error("cannot read journal");}
  secureFile(fd.fd, 4096);
  std::string text(4097, '\0'); size_t count = 0;
  while (count < text.size()) {
    const auto n = read(fd.fd, text.data() + count, text.size() - count);
    if (n < 0 && errno == EINTR) {continue;}
    if (n < 0) {throw std::runtime_error("journal read failed");}
    if (!n) {break;}
    count += static_cast<size_t>(n);
  }
  if (count > 4096) {throw std::runtime_error("journal too large");}
  text.resize(count);
  Json::CharReaderBuilder builder;
  builder["collectComments"] = false; builder["allowComments"] = false;
  builder["rejectDupKeys"] = true; builder["failIfExtra"] = true;
  builder["allowTrailingCommas"] = false; builder["stackLimit"] = 16;
  std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
  Json::Value root; std::string errors;
  if (!reader->parse(text.data(), text.data() + text.size(), &root, &errors) ||
    !root.isObject() || !root["schema_version"].isInt() || root["schema_version"].asInt() != 1)
  {throw std::runtime_error("corrupt or unsupported journal schema");}
  std::vector<std::string> keys{"schema_version", "scope", "instance", "request", "name", "hash",
    "execution", "goal", "cancel_dispatched", "resolved", "checksum"};
  std::sort(keys.begin(), keys.end());
  if (root.getMemberNames() != keys || !root["cancel_dispatched"].isBool() ||
    !root["resolved"].isBool())
  {throw std::runtime_error("invalid journal fields");}
  if (!root["checksum"].isString() || !hex_id(root["checksum"].asString(), 64)) {
    throw std::runtime_error("missing journal integrity checksum");
  }
  const auto checksum = root["checksum"].asString(); root.removeMember("checksum");
  Json::StreamWriterBuilder writer; writer["indentation"] = "";
  if (checksum != easyfly_mission_fsm::missionContentSha256(Json::writeString(writer, root))) {
    throw std::runtime_error("journal integrity mismatch");
  }
  for (const auto & key : {"scope", "instance", "request", "name", "hash", "execution", "goal"}) {
    if (!root[key].isString()) {throw std::runtime_error("invalid journal text field");}
  }
  Intent v{root["scope"].asString(), root["instance"].asString(), root["request"].asString(),
    root["name"].asString(), root["hash"].asString(), root["execution"].asString(),
    root["goal"].asString(), root["cancel_dispatched"].asBool(), root["resolved"].asBool()};
  check(v); return v;
}
void Journal::save(const Intent & v) const
{
  check(v);
  Json::Value root;
  root["schema_version"] = 1; root["scope"] = v.scope; root["instance"] = v.instance;
  root["request"] = v.request; root["name"] = v.name; root["hash"] = v.hash;
  root["execution"] = v.execution; root["goal"] = v.goal;
  root["cancel_dispatched"] = v.cancel_dispatched; root["resolved"] = v.resolved;
  Json::StreamWriterBuilder builder; builder["indentation"] = "";
  root["checksum"] = easyfly_mission_fsm::missionContentSha256(Json::writeString(builder, root));
  const auto text = Json::writeString(builder, root);
  const auto temporary = "intent-" + new_id() + ".tmp";
  Guard fd{openat(
      directory_, temporary.c_str(), O_WRONLY | O_CREAT | O_EXCL |
      O_NOFOLLOW | O_CLOEXEC, 0600)};
  if (fd.fd < 0) {throw std::runtime_error("cannot prepare journal write");}
  try {
    size_t written = 0;
    while (written < text.size()) {
      const auto n = write(fd.fd, text.data() + written, text.size() - written);
      if (n < 0 && errno == EINTR) {continue;}
      if (n <= 0) {throw std::runtime_error("journal write failed");}
      written += static_cast<size_t>(n);
    }
    if (fsync(fd.fd) || renameat(directory_, temporary.c_str(), directory_, "intent.json") ||
      fsync(directory_)) {throw std::runtime_error("journal durable commit failed");}
  } catch (...) {
    unlinkat(directory_, temporary.c_str(), 0); throw;
  }
}
}  // namespace easyfly_ui_control
