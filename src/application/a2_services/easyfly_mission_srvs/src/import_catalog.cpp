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
#include <sys/syscall.h>
#include <unistd.h>
#include <algorithm>
#include <cerrno>
#include <filesystem>
#include <map>
#include <mutex>
#include <stdexcept>
#include <utility>
#include <easyfly_mission_fsm/mission_format.hpp>
#include "import_catalog.hpp"
namespace fs = std::filesystem;
namespace
{
constexpr const char * marker_name = ".easyfly-mission-catalog-v1";
constexpr char marker_text[] = "easyfly-mission-catalog-v1\n";
constexpr size_t marker_size = sizeof(marker_text) - 1;
struct Fd {int fd{-1}; ~Fd() {if (fd >= 0) {close(fd);}}};
bool hex(const std::string & s, size_t size)
{
  return s.size() == size && std::all_of(
    s.begin(), s.end(), [](char c) {
      return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
    });
}
bool name_valid(const std::string & s)
{
  if (s.empty() || s.size() > 128 || s[0] == '.' || fs::path(s).filename() != s ||
    !easyfly_mission_fsm::missionTextUtf8(s)) {return false;}
  const auto ext = fs::path(s).extension();
  return ext == ".yaml" || ext == ".yml";
}
bool within(const fs::path & path, const fs::path & parent)
{
  auto p = path.begin();
  for (auto q = parent.begin(); q != parent.end(); ++q, ++p) {
    if (p == path.end() || *p != *q) {return false;}
  }
  return true;
}
void write_all(int fd, const std::string & bytes)
{
  size_t count = 0;
  while (count < bytes.size()) {
    const auto n = write(fd, bytes.data() + count, bytes.size() - count);
    if (n < 0 && errno == EINTR) {continue;}
    if (n <= 0) {throw std::runtime_error("catalog write failed");}
    count += static_cast<size_t>(n);
  }
  if (fsync(fd)) {throw std::runtime_error("catalog file fsync failed");}
}
}  // namespace
struct ImportCatalog::Impl
{
  bool enabled;
  std::string instance, catalog;
  int capacity;
  fs::path root;
  Fd directory, marker;
  dev_t device{}; ino_t inode{};
  std::map<std::string, Api::Response> records;
  std::mutex mutex;
  Impl(
    bool active, const std::string & path, const std::string & id, int limit,
    const std::vector<std::string> & protected_roots)
  : enabled(active), instance(id), capacity(limit)
  {
    if (capacity < 1 || capacity > 256) {
      throw std::invalid_argument("import capacity must be 1..256");
    }
    if (!enabled) {return;}
    const fs::path named(path);
    if (!named.is_absolute() || named.filename().empty() || named.filename() == "." ||
      named.filename() == ".." || path.size() > 4096 ||
      !easyfly_mission_fsm::missionTextUtf8(path))
    {
      throw std::invalid_argument("explicit private import catalog required");
    }
    root = fs::canonical(named);
    if (root != named.lexically_normal()) {
      throw std::invalid_argument("catalog symlinks forbidden");
    }
    for (const auto & protected_root : protected_roots) {
      if (within(root, fs::weakly_canonical(protected_root))) {
        throw std::invalid_argument("cannot import into source/install/package catalog");
      }
    }
    directory.fd = open(root.c_str(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
    struct stat info {};
    if (directory.fd < 0 || fstat(directory.fd, &info) || info.st_uid != geteuid() ||
      (info.st_mode & 077)) {throw std::invalid_argument("private owned catalog required");}
    device = info.st_dev; inode = info.st_ino;
    marker.fd = openat(directory.fd, marker_name, O_RDWR | O_NOFOLLOW | O_NONBLOCK | O_CLOEXEC);
    if (marker.fd < 0 && errno == ENOENT) {
      if (!fs::is_empty(root)) {
        throw std::invalid_argument("existing ordinary catalog cannot be adopted");
      }
      marker.fd = openat(
        directory.fd, marker_name,
        O_RDWR | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0600);
      if (marker.fd < 0) {throw std::runtime_error("cannot create catalog marker");}
      write_all(marker.fd, marker_text);
      if (fsync(directory.fd)) {throw std::runtime_error("catalog marker fsync failed");}
    }
    if (marker.fd < 0 || fstat(marker.fd, &info) || !S_ISREG(info.st_mode) ||
      info.st_uid != geteuid() || (info.st_mode & 077) || info.st_nlink != 1 ||
      info.st_size != static_cast<off_t>(marker_size))
    {throw std::invalid_argument("invalid private catalog marker");}
    char bytes[sizeof(marker_text)]{};
    if (pread(marker.fd, bytes, marker_size, 0) != static_cast<ssize_t>(marker_size) ||
      std::string(bytes, marker_size) != marker_text)
    {
      throw std::invalid_argument("invalid catalog marker contents");
    }
    if (flock(marker.fd, LOCK_EX | LOCK_NB)) {
      throw std::runtime_error("catalog already owned by another importer");
    }
    catalog = easyfly_mission_fsm::missionContentSha256(
      instance + ":" + root.string() + ":" + std::to_string(device) + ":" +
      std::to_string(inode)).substr(0, 32);
  }
  bool live() const
  {
    if (!enabled) {return false;}
    struct stat named {}, held {}, tagged {};
    return lstat(root.c_str(), &named) == 0 && S_ISDIR(named.st_mode) &&
           named.st_uid == geteuid() && !(named.st_mode & 077) && named.st_dev == device &&
           named.st_ino == inode && fstat(directory.fd, &held) == 0 && held.st_nlink > 0 &&
           fstatat(directory.fd, marker_name, &tagged, AT_SYMLINK_NOFOLLOW) == 0 &&
           fstat(marker.fd, &held) == 0 && tagged.st_dev == held.st_dev &&
           tagged.st_ino == held.st_ino && tagged.st_nlink == 1 &&
           tagged.st_uid == geteuid() && !(tagged.st_mode & 077);
  }
  Api::Response response(
    const Api::Request & q, const std::string & status,
    const std::string & message, bool success = false) const
  {
    Api::Response r;
    r.success = success; r.status = status; r.message = message;
    r.server_instance_id = instance; r.catalog_id = catalog; r.enabled = live();
    r.catalog_directory = root.string();
    r.client_request_id = hex(q.client_request_id, 32) ? q.client_request_id : "";
    r.request_capacity = capacity; r.retained_requests = records.size();
    if (q.operation == "create" && name_valid(q.mission_name) &&
      hex(q.expected_content_sha256, 64) && q.content.size() <= 65536 &&
      easyfly_mission_fsm::missionContentSha256(q.content) == q.expected_content_sha256)
    {
      r.mission_name = q.mission_name; r.content_sha256 = q.expected_content_sha256;
      r.resolved = true;  // Direct refusal can be known; missing replies still require lookup.
    }
    return r;
  }
  Api::Response handle(const Api::Request & q)
  {
    std::lock_guard<std::mutex> lock(mutex);
    if (q.operation == "session") {
      return response(q, "session", "catalog identity only; never flight admission", true);
    }
    if (q.operation != "create" && q.operation != "lookup") {
      return response(q, "unsupported", "supported: session, create, lookup");
    }
    if (q.server_instance_id != instance || q.catalog_id != catalog) {
      return response(q, "instance_mismatch", "original instance/catalog required; do not replay");
    }
    if (!hex(q.client_request_id, 32)) {
      return response(q, "invalid", "32 lowercase hex request ID required");
    }
    auto found = records.find(q.client_request_id);
    if (found != records.end()) {
      const auto & stored = found->second;
      if (q.operation == "create" && (q.mission_name != stored.mission_name ||
        q.expected_content_sha256 != stored.content_sha256 ||
        easyfly_mission_fsm::missionContentSha256(q.content) != stored.content_sha256))
      {return response(q, "request_conflict", "request bound to other content; no write");}
      auto r = stored; r.enabled = live(); r.retained_requests = records.size(); return r;
    }
    if (q.operation == "lookup") {
      return response(q, "not_found", "unknown request; not proof of no write");
    }
    if (!live()) {return response(q, "disabled", "import disabled or catalog identity changed");}
    if (!name_valid(q.mission_name) || !hex(q.expected_content_sha256, 64) ||
      q.content.empty() || q.content.size() > 65536 ||
      !easyfly_mission_fsm::missionTextUtf8(q.content) ||
      easyfly_mission_fsm::missionContentSha256(q.content) != q.expected_content_sha256)
    {return response(q, "invalid", "valid YAML basename/UTF-8 <=64KiB and exact hash required");}
    if (records.size() >= static_cast<size_t>(capacity)) {
      return response(q, "request_capacity", "no eviction; new imports refused");
    }
    auto r = response(q, "rejected", "no file published");
    r.resolved = false;
    r.mission_name = q.mission_name; r.content_sha256 = q.expected_content_sha256;
    records.emplace(q.client_request_id, r);
    std::string stage;
    try {
      const auto check = easyfly_mission_fsm::validateMissionText(q.content);
      if (!check.valid() || easyfly_mission_fsm::missionFormatName(q.content) != "yaml_v1") {
        throw std::invalid_argument("only shared-validator-approved YAML v1 can be imported");
      }
      size_t entries = 0;
      for (const auto & unused : fs::directory_iterator(root)) {
        (void)unused;
        if (++entries >= 256) {throw std::runtime_error("catalog directory entry limit reached");}
      }
      stage = ".easyfly-import-" + q.client_request_id + ".tmp";
      Fd file{openat(
          directory.fd, stage.c_str(), O_WRONLY | O_CREAT | O_EXCL |
          O_NOFOLLOW | O_CLOEXEC, 0600)};
      if (file.fd < 0) {
        stage.clear(); throw std::runtime_error("cannot create private staging file");
      }
      write_all(file.fd, q.content);
      if (!live()) {throw std::runtime_error("catalog changed before publish");}
      if (syscall(
          SYS_renameat2, directory.fd, stage.c_str(), directory.fd,
          q.mission_name.c_str(), RENAME_NOREPLACE))
      {
        r.status = errno == EEXIST ? "exists" : "rejected";
        throw std::runtime_error("no-replace publish refused; existing target never overwritten");
      }
      stage.clear(); r.published = true;
      if (fsync(directory.fd) || !live()) {
        r.status = "unconfirmed";
        r.message = "published but durability/catalog not confirmed; no rollback";
      } else {
        r.success = r.resolved = true; r.status = "imported";
        r.message = "new validated file published; not executed; independent readback required";
      }
    } catch (const std::exception & error) {
      r.resolved = !r.published; r.message = error.what();
      if (!stage.empty()) {unlinkat(directory.fd, stage.c_str(), 0);}
    }
    r.retained_requests = records.size(); r.enabled = live();
    r.message = r.message.substr(0, 2048);
    if (!easyfly_mission_fsm::missionTextUtf8(r.message)) {r.message = "catalog operation failed";}
    records[q.client_request_id] = r; return r;
  }
};
ImportCatalog::ImportCatalog(
  bool enabled, const std::string & directory, const std::string & instance,
  int capacity, const std::vector<std::string> & roots)
: impl_(std::make_unique<Impl>(enabled, directory, instance, capacity, roots)) {}
ImportCatalog::~ImportCatalog() = default;
ImportCatalog::Api::Response ImportCatalog::handle(const Api::Request & request)
{return impl_->handle(request);}
