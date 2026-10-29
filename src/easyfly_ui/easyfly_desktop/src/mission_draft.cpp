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
#include <linux/fs.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <condition_variable>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <locale>
#include <mutex>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>
#include "easyfly_desktop/mission_draft.hpp"
namespace easyfly_desktop
{
namespace fs = std::filesystem;
namespace mission = easyfly_mission_fsm;
std::vector<std::string> local_storage_protected_paths(
  std::vector<std::string> configured,
  const std::map<std::string, std::string> & installed_packages,
  const std::string & executable_path)
{
  for (const auto & package : installed_packages) {
    if (package.first.rfind("easyfly_", 0) == 0 && fs::path(package.second).is_absolute()) {
      configured.push_back(package.second);
    }
  }
  const fs::path executable(executable_path);
  const auto directory = executable.parent_path();
  if (executable.is_absolute() && directory.filename() == "easyfly_desktop" &&
    directory.parent_path().filename() == "lib")
  {
    // Retain protection even if the ament index is missing or points at another overlay.
    configured.push_back(directory.parent_path().parent_path().string());
  }
  return configured;
}
std::vector<DraftField> draft_fields(const mission::MissionStep & s)
{
  std::vector<DraftField> f;
  const auto add = [&](const char * key, double v, const char * unit, bool integer = false) {
      f.push_back({key, unit, v, integer});
    };
  if (s.type == "takeoff") {
    add("altitude", s.altitude, "m"); add("climb_speed", s.climb_speed, "m/s");
  }
  if (s.type == "hover") {add("duration", s.wait, "s");}
  if (s.type == "waypoint" || s.type == "navigate") {
    add("x", s.x, "m"); add("y", s.y, "m"); add("z", s.z, "m");
    add("max_speed", s.max_speed, "m/s");
    if (s.type == "waypoint") {
      add("yaw", s.yaw, "rad"); add("acceptance_radius", s.acceptance_radius, "m");
    }
  }
  if (s.type == "track") {
    add("target_id", s.target_id, "id", true); add("duration", s.duration, "s");
    add("follow_distance", s.follow_distance, "m"); add("follow_height", s.follow_height, "m");
  }
  if (s.type == "search") {
    add("center_x", s.center_x, "m"); add("center_y", s.center_y, "m");
    add("area_width", s.area_width, "m"); add("area_height", s.area_height, "m");
    add("altitude", s.altitude, "m"); add("max_speed", s.max_speed, "m/s");
  }
  if (s.type == "explore") {
    add("min_x", s.min_bound[0], "m"); add("min_y", s.min_bound[1], "m");
    add("min_z", s.min_bound[2], "m"); add("max_x", s.max_bound[0], "m");
    add("max_y", s.max_bound[1], "m"); add("max_z", s.max_bound[2], "m");
  }
  if (s.type == "precision_land") {add("tag_id", s.tag_id, "id", true);}
  add("timeout", s.timeout, "s"); return f;
}
bool draft_set_field(mission::MissionStep & s, const std::string & key, double value)
{
  if (!std::isfinite(value)) {return false;}
  for (const auto & field : draft_fields(s)) {
    if (field.key != key) {continue;}
    if (field.integer && (std::floor(value) != value || value < -1 ||
      value > std::numeric_limits<int>::max())) {return false;}
    if (key == "duration") {(s.type == "hover" ? s.wait : s.duration) = value; return true;}
    if (key == "tag_id") {s.tag_id = static_cast<int>(value); return true;}
    if (key == "target_id") {s.target_id = static_cast<int>(value); return true;}
    const std::vector<std::pair<std::string, double *>> values{
      {"altitude", &s.altitude}, {"climb_speed", &s.climb_speed}, {"x", &s.x}, {"y", &s.y},
      {"z", &s.z}, {"yaw", &s.yaw}, {"max_speed", &s.max_speed}, {"timeout", &s.timeout},
      {"acceptance_radius", &s.acceptance_radius}, {"follow_distance", &s.follow_distance},
      {"follow_height", &s.follow_height}, {"center_x", &s.center_x}, {"center_y", &s.center_y},
      {"area_width", &s.area_width}, {"area_height", &s.area_height},
      {"min_x", &s.min_bound[0]}, {"min_y", &s.min_bound[1]}, {"min_z", &s.min_bound[2]},
      {"max_x", &s.max_bound[0]}, {"max_y", &s.max_bound[1]}, {"max_z", &s.max_bound[2]}};
    for (const auto & entry : values) {
      if (entry.first == key) {
        *entry.second = value; return true;
      }
    }
  }
  return false;
}
mission::MissionStep draft_default_step(const std::string & type)
{
  mission::MissionStep s; s.type = type; s.altitude = s.z = 1.5;
  s.wait = s.duration = 5; s.area_width = s.area_height = 10; s.tag_id = 0;
  return s;
}
std::string draft_yaml(const std::vector<mission::MissionStep> & steps)
{
  std::ostringstream out; out.imbue(std::locale::classic());
  out << std::setprecision(std::numeric_limits<double>::max_digits10);
  out << "schema_version: 1\nframe_id: odom\nunits: {position: m, angle: rad, time: s}\n"
    "failure_policy: abort\nsteps:\n";
  for (const auto & s : steps) {
    const std::vector<std::string> types{"takeoff", "hover", "waypoint", "navigate", "track",
      "search", "land", "precision_land", "explore"};
    if (std::find(types.begin(), types.end(), s.type) == types.end()) {
      throw std::invalid_argument("不支持的步骤类型");
    }
    out << "  - type: " << s.type << '\n';
    for (const auto & f : draft_fields(s)) {
      if (s.type == "explore" && f.key != "timeout") {continue;}
      out << "    " << f.key << ": " << f.value << '\n';
    }
    if (s.type == "search") {out << "    pattern: boustrophedon\n";}
    if (s.type == "explore") {
      const auto point = [&](const std::array<double, 3> & p) {
          out << '[' << p[0] << ", " << p[1] << ", " << p[2] << ']';
        };
      out << "    min_bound: "; point(s.min_bound);
      out << "\n    max_bound: "; point(s.max_bound);
      out << "\n    return_home: " << (s.return_home ? "true" : "false") << '\n';
      if (!s.forbidden_boxes.empty()) {
        out << "    forbidden_boxes:\n";
        for (const auto & box : s.forbidden_boxes) {
          out << "      - min: "; point(box.min);
          out << "\n        max: "; point(box.max); out << '\n';
        }
      }
    }
  }
  const auto body = out.str();
  const auto checked = mission::validateMissionText(body);
  if (!checked.valid()) {throw std::invalid_argument(checked.error);}
  return body;
}
bool DraftFingerprint::operator==(const DraftFingerprint & b) const
{
  return exists == b.exists && device == b.device && inode == b.inode &&
         directory_device == b.directory_device && directory_inode == b.directory_inode &&
         modified_ns == b.modified_ns && changed_ns == b.changed_ns && hash == b.hash;
}
namespace
{
constexpr const char * marker_name = ".easyfly-drafts-v1";
constexpr const char * marker_text = "EasyFly dedicated local draft directory v1\n";
struct Fd {int fd; ~Fd() {if (fd >= 0) {close(fd);}}};
void require(bool ok, const char * error) {if (!ok) {throw std::runtime_error(error);}}
std::string identity()
{
  std::random_device random; std::ostringstream id;
  for (int i = 0; i < 16; ++i) {
    id << std::hex << std::setw(2) << std::setfill('0') << (random() & 255);
  }
  return id.str();
}
std::string read_regular(int fd)
{
  struct stat st {};
  require(
    fd >= 0 && !fstat(fd, &st) && S_ISREG(st.st_mode) && st.st_size >= 0 &&
    st.st_size <= 65536, "仅支持普通文件且不超过64KiB");
  std::string bytes(65537, '\0'); size_t size = 0;
  while (size < bytes.size()) {
    const auto n = read(fd, bytes.data() + size, bytes.size() - size);
    if (n < 0 && errno == EINTR) {continue;}
    require(n >= 0, "文件读取失败"); if (!n) {break;} size += static_cast<size_t>(n);
  }
  require(size <= 65536, "文件超过64KiB"); bytes.resize(size); return bytes;
}
void write_all(int fd, const std::string & bytes)
{
  size_t size = 0;
  while (size < bytes.size()) {
    const auto n = write(fd, bytes.data() + size, bytes.size() - size);
    if (n < 0 && errno == EINTR) {continue;}
    require(n > 0, "写入失败，未确认导出成功"); size += static_cast<size_t>(n);
  }
  require(!fsync(fd), "文件同步失败，未确认导出成功");
}
bool inside(const fs::path & value, const fs::path & root)
{
  auto a = value.begin(), b = root.begin();
  for (; b != root.end(); ++a, ++b) {
    if (a == value.end() || *a != *b) {
      return false;
    }
  }
  return true;
}
void allowed_path(const fs::path & path, const std::vector<std::string> & protected_paths)
{
  for (const auto & root : protected_paths) {
    require(!inside(path, fs::weakly_canonical(root)), "不允许导出到源码、安装或已知任务目录");
  }
}
int draft_directory(const std::string & path)
{
  const fs::path p(path);
  require(
    p.is_absolute() && !p.filename().empty() && p.filename() != "." && p.filename() != "..",
    "草稿目录必须是明确的绝对目录");
  const int fd = open(path.c_str(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
  Fd guard{fd}; struct stat st {};
  require(
    fd >= 0 && !fstat(fd, &st) && st.st_uid == geteuid() && !(st.st_mode & 077),
    "草稿目录须由当前用户独占，权限0700，且不可是符号链接");
  Fd marker{openat(fd, marker_name, O_RDONLY | O_NONBLOCK | O_NOFOLLOW | O_CLOEXEC)};
  require(
    marker.fd >= 0 && !fstat(marker.fd, &st) && st.st_uid == geteuid() &&
    st.st_nlink == 1 && !(st.st_mode & 077) && read_regular(marker.fd) == marker_text,
    "拒绝已有普通目录：请选择新建专用草稿目录或既有EasyFly草稿目录");
  guard.fd = -1; return fd;
}
std::string prepare_directory(
  const std::string & path,
  const std::vector<std::string> & protected_paths)
{
  const fs::path p(path);
  require(
    p.is_absolute() && !p.filename().empty() && p.filename() != "." && p.filename() != "..",
    "请选择父目录已存在的新草稿目录，或既有EasyFly专用草稿目录");
  const auto parent = fs::canonical(p.parent_path()); const auto target = parent / p.filename();
  allowed_path(target, protected_paths);
  Fd parent_fd{open(parent.c_str(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC)};
  require(parent_fd.fd >= 0, "父目录不可用");
  if (mkdirat(parent_fd.fd, p.filename().c_str(), 0700) == 0) {
    Fd directory{openat(
        parent_fd.fd,
        p.filename().c_str(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC)};
    require(directory.fd >= 0, "新草稿目录不可用，目录已保留供检查");
    Fd marker{openat(
        directory.fd, marker_name,
        O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0600)};
    require(marker.fd >= 0, "草稿目录标记创建失败，目录已保留供检查");
    write_all(marker.fd, marker_text);
    require(!fsync(directory.fd) && !fsync(parent_fd.fd), "目录同步失败，目录已保留供检查");
  } else {require(errno == EEXIST, "无法创建草稿目录");}
  Fd checked{draft_directory(target.string())}; return target.string();
}
void filename(const std::string & name)
{
  const fs::path p(name);
  require(
    !name.empty() && name.size() <= 128 && name.front() != '.' && p.filename() == p &&
    (p.extension() == ".yaml" || p.extension() == ".yml") && mission::missionTextUtf8(name) &&
    std::none_of(
      name.begin(), name.end(), [](unsigned char c) {
        return c < 32 || c == 127 || c == '\\';
      }),
    "导出文件名须为不含路径的.yaml/.yml名称（不超过128字节）");
}
DraftFingerprint inspect(int dir, const std::string & name)
{
  struct stat directory {};
  require(!fstat(dir, &directory), "草稿目录身份不可确认");
  DraftFingerprint result;
  result.directory_device = directory.st_dev; result.directory_inode = directory.st_ino;
  Fd fd{openat(dir, name.c_str(), O_RDONLY | O_NONBLOCK | O_NOFOLLOW | O_CLOEXEC)};
  if (fd.fd < 0 && errno == ENOENT) {return result;}
  require(fd.fd >= 0, "目标不可读取或是符号链接，拒绝覆盖"); struct stat st {};
  require(
    !fstat(fd.fd, &st) && st.st_uid == geteuid() && st.st_nlink == 1,
    "目标须为当前用户所有、无硬链接的普通文件");
  const auto body = read_regular(fd.fd);
  struct stat after {}; require(
    !fstat(fd.fd, &after) && st.st_size == after.st_size &&
    st.st_mtim.tv_sec == after.st_mtim.tv_sec && st.st_mtim.tv_nsec == after.st_mtim.tv_nsec &&
    st.st_ctim.tv_sec == after.st_ctim.tv_sec && st.st_ctim.tv_nsec == after.st_ctim.tv_nsec,
    "目标读取期间变化，须重新确认");
  result.exists = true; result.device = st.st_dev; result.inode = st.st_ino;
  result.modified_ns = st.st_mtim.tv_sec * 1000000000LL + st.st_mtim.tv_nsec;
  result.changed_ns = st.st_ctim.tv_sec * 1000000000LL + st.st_ctim.tv_nsec;
  result.hash = mission::missionContentSha256(body); return result;
}
bool move_new(int dir, const std::string & from, const std::string & to)
{
  return syscall(SYS_renameat2, dir, from.c_str(), dir, to.c_str(), RENAME_NOREPLACE) == 0;
}
void export_file(DraftResult & r, const std::vector<std::string> & protected_paths)
{
  const auto & j = r.job; filename(j.filename);
  allowed_path(fs::canonical(j.path), protected_paths); Fd dir{draft_directory(j.path)};
  require(
    mission::missionTextUtf8(j.content) && mission::missionFormatName(j.content) == "yaml_v1" &&
    mission::validateMissionText(j.content).valid(), "仅导出通过共享校验器的正式v1 YAML");
  require(inspect(dir.fd, j.filename) == j.expected, "导出目标已变化；未覆盖，须重新确认");
  const auto temporary = ".export-" + identity();
  Fd output{openat(
      dir.fd,
      temporary.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0600)};
  require(output.fd >= 0, "不能创建导出暂存文件");
  bool moved = false;
  const auto backup = ".backup-" + identity();
  try {
    write_all(output.fd, j.content);
    if (j.expected.exists) {
      require(move_new(dir.fd, j.filename, backup), "目标备份失败；未覆盖"); moved = true;
      r.backup = (fs::path(j.path) / backup).string();
      // rename changes ctime: compare the captured inode/content/mtime, not its old ctime.
      auto captured = inspect(dir.fd, backup); captured.changed_ns = j.expected.changed_ns;
      require(captured == j.expected, "备份捕获的目标已变化；未覆盖");
    }
    require(
      move_new(dir.fd, temporary, j.filename), "目标被其他写入占用或原子提交失败；未覆盖该目标");
    require(!fsync(dir.fd), "目录同步失败；新文件可能已存在，请检查，未确认成功");
    r.path = (fs::path(j.path) / j.filename).string();
  } catch (...) {
    // Only our O_EXCL-created temporary, never a user file.
    unlinkat(dir.fd, temporary.c_str(), 0);
    if (moved && move_new(dir.fd, backup, j.filename)) {r.backup.clear(); fsync(dir.fd);}
    throw;
  }
}
}  // namespace
struct DraftWorker::Impl
{
  std::vector<std::string> protected_paths;
  mutable std::mutex mutex;
  std::condition_variable wake;
  std::optional<std::pair<uint64_t, DraftJob>> pending;
  std::shared_ptr<const DraftResult> output;
  DraftWorkerState state;
  DraftOperation active_kind{DraftOperation::Validate};
  uint64_t sequence{0}; bool closing{false};
  std::thread worker;
  explicit Impl(std::vector<std::string> paths)
  : protected_paths(std::move(paths)), worker([this] {run();}) {}
  ~Impl() {stop(); if (worker.joinable()) {worker.join();}}
  void stop()
  {
    std::lock_guard<std::mutex> lock(mutex); closing = true; pending.reset(); wake.notify_all();
  }
  void run()
  {
    std::unique_lock<std::mutex> lock(mutex);
    while (!closing) {
      wake.wait(lock, [this] {return closing || pending.has_value();}); if (closing) {break;}
      auto job = std::move(*pending); pending.reset(); state.active = true;
      active_kind = job.second.operation;
      lock.unlock(); auto r = std::make_shared<DraftResult>(); r->id = job.first;
      r->job = std::move(job.second);
      const bool validation = r->job.operation == DraftOperation::Validate ||
        r->job.operation == DraftOperation::Normalize;
      try {
        if (r->job.operation == DraftOperation::Read) {
          Fd fd{open(r->job.path.c_str(), O_RDONLY | O_NONBLOCK | O_NOFOLLOW | O_CLOEXEC)};
          r->content = read_regular(fd.fd); require(
            mission::missionTextUtf8(
              r->content), "文件不是有效UTF-8正文");
          r->path = fs::canonical(r->job.path).string(); r->success = true;
        } else if (validation) {
          r->content = r->job.operation ==
            DraftOperation::Normalize ? draft_yaml(r->job.steps) : r->job.content;
          require(mission::missionTextUtf8(r->content), "正文不是有效UTF-8");
          r->plan = mission::validateMissionText(r->content);
          r->format = mission::missionFormatName(r->content);
          r->success = r->plan.valid(); r->error = r->plan.error;
          if (r->success) {r->hash = mission::missionContentSha256(r->content);}
        } else if (r->job.operation == DraftOperation::PrepareDirectory) {
          r->path = prepare_directory(r->job.path, protected_paths); r->success = true;
        } else if (r->job.operation == DraftOperation::InspectExport) {
          filename(r->job.filename); allowed_path(fs::canonical(r->job.path), protected_paths);
          Fd fd{draft_directory(r->job.path)}; r->fingerprint = inspect(fd.fd, r->job.filename);
          r->success = true;
        } else {export_file(*r, protected_paths); r->success = true;}
      } catch (const std::exception & e) {r->error = std::string(e.what()).substr(0, 2048);}
      lock.lock(); state.active = false;
      if (r->id == sequence) {output = r;}
    }
    state.stopped = true;
  }
};
DraftWorker::DraftWorker(std::vector<std::string> paths)
: impl_(std::make_unique<Impl>(std::move(paths))) {}
DraftWorker::~DraftWorker() = default;
uint64_t DraftWorker::submit(DraftJob job)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->closing || job.content.size() > 65536 || job.path.size() > 4096 ||
    job.filename.size() > 128 ||
    job.steps.size() > 256 || job.path.find('\0') != std::string::npos ||
    (impl_->state.active && impl_->active_kind != DraftOperation::Validate) ||
    (impl_->pending && impl_->pending->second.operation != DraftOperation::Validate)) {return 0;}
  const auto id = ++impl_->sequence; impl_->pending = std::make_pair(id, std::move(job));
  impl_->output.reset(); impl_->wake.notify_all(); return id;
}
std::shared_ptr<const DraftResult> DraftWorker::latest() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex); return impl_->output;
}
DraftWorkerState DraftWorker::state() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex); auto state = impl_->state;
  state.pending = impl_->pending ? 1 : 0; return state;
}
void DraftWorker::stop() {impl_->stop();}
}  // namespace easyfly_desktop
