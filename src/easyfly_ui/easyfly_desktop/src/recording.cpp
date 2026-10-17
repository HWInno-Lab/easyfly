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

#include "easyfly_desktop/recording.hpp"
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <unistd.h>
#include <QDateTime>
#include <QCryptographicHash>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QUuid>
#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <condition_variable>
#include <filesystem>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>
#ifdef EASYFLY_ENABLE_MEDIA
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#endif
namespace easyfly_desktop
{
std::string recording_clock_end_reason(const RecordingPacket & packet, const ImageBinding & binding)
{
  const auto & s = packet.observation;
  if (!s || !s->running || s->stopped) {return "observation_disconnected";}
  if (s->observation_session_id != binding.session || s->time_epoch != binding.epoch) {
    return "camera_session_clock_changed";
  }
  if (s->clock.quality == bridge::Quality::ClockPaused) {return "clock_paused";}
  if (s->clock.quality != bridge::Quality::ValidAtObservation) {return "clock_unavailable";}
  const auto age = bridge::detail::steady_seconds() - s->produced_steady;
  if (!std::isfinite(age) || age < 0 || age > 1) {return "clock_evidence_stale";}
  return {};
}
namespace
{
namespace fs = std::filesystem;
constexpr const char * marker = ".easyfly-recordings-v1";
constexpr const char * marker_text = "EasyFly private media directory v1\n";
QString str(const std::string & s) {return QString::fromStdString(s).left(2048);}
[[maybe_unused]] QString integer(uint64_t n)
{
  return QString::number(static_cast<qulonglong>(n));
}
QString utc() {return QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);}
void require(bool ok, const char * message)
{
  if (!ok) {throw std::runtime_error(message);}
}
struct Fd
{
  int value{-1};
  Fd() = default;
  explicit Fd(int fd)
  : value(fd) {require(fd >= 0, "file open failed");}
  ~Fd() {if (value >= 0) {::close(value);}}
  Fd(const Fd &) = delete;
  Fd & operator=(const Fd &) = delete;
};
void write_bytes(int fd, const void * data, std::size_t size)
{
  auto * bytes = static_cast<const char *>(data);
  while (size) {
    const auto n = ::write(fd, bytes, size);
    if (n < 0 && errno == EINTR) {continue;}
    require(n > 0, "record write failed (disk full or I/O error)");
    bytes += n; size -= static_cast<std::size_t>(n);
  }
}
void sync(int fd) {require(::fsync(fd) == 0, "record fsync failed");}
[[maybe_unused]] void json_line(int fd, QJsonObject object)
{
  object["written_utc"] = utc();
  const auto data = QJsonDocument(object).toJson(QJsonDocument::Compact) + '\n';
  require(data.size() < 65536, "metadata entry too large");
  write_bytes(fd, data.constData(), data.size());
}
bool private_dir(int fd)
{
  struct stat s {};
  return fstat(fd, &s) == 0 && S_ISDIR(s.st_mode) && s.st_uid == geteuid() &&
         (s.st_mode & 0777) == 0700;
}
bool within(const fs::path & path, const fs::path & base)
{
  auto p = path.begin();
  for (auto b = base.begin(); b != base.end(); ++b, ++p) {
    if (p == path.end() || *p != *b) {return false;}
  }
  return true;
}
QByteArray read_small(int dir, const char * name, std::size_t limit = 65536)
{
  Fd file(openat(dir, name, O_RDONLY | O_NOFOLLOW | O_NONBLOCK | O_CLOEXEC));
  struct stat s {};
  require(
    fstat(file.value, &s) == 0 && S_ISREG(s.st_mode) && s.st_uid == geteuid() &&
    (s.st_mode & 0777) == 0600 && s.st_nlink == 1 && s.st_size >= 0 &&
    static_cast<uint64_t>(s.st_size) <= limit, "unsafe or oversized record file");
  QByteArray result(static_cast<int>(s.st_size), '\0');
  std::size_t offset = 0;
  while (offset < static_cast<std::size_t>(result.size())) {
    const auto n = read(file.value, result.data() + offset, result.size() - offset);
    if (n < 0 && errno == EINTR) {continue;}
    require(n > 0, "record read failed"); offset += n;
  }
  return result;
}
std::unique_ptr<Fd> root_directory(
  const std::string & input, const std::vector<std::string> & protected_paths, bool create)
{
  const fs::path raw(input);
  require(
    raw.is_absolute() && !raw.filename().empty() && raw.filename() != "." &&
    raw.filename() != ".." && raw == raw.lexically_normal(), "choose an absolute clean directory");
  const auto parent = fs::canonical(raw.parent_path());
  const auto path = parent / raw.filename();
  require(
    path != parent.root_path() && path != fs::path(qEnvironmentVariable("HOME").toStdString()),
    "choose an independent recording directory");
  for (const auto & protected_path : protected_paths) {
    if (protected_path.empty()) {continue;}
    const auto p = fs::weakly_canonical(protected_path);
    require(!within(path, p) && !within(p, path), "protected project/journal directory");
  }
  Fd p(open(parent.c_str(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC));
  bool made = false;
  if (create) {
    made = mkdirat(p.value, raw.filename().c_str(), 0700) == 0;
    require(made || errno == EEXIST, "record directory creation failed");
  }
  auto root = std::make_unique<Fd>(
    openat(
      p.value, raw.filename().c_str(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC));
  require(private_dir(root->value), "directory must be owned mode0700 (not adopted or repaired)");
  if (made) {
    Fd m(openat(root->value, marker, O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0600));
    write_bytes(m.value, marker_text, std::char_traits<char>::length(marker_text));
    sync(m.value); sync(root->value); sync(p.value);
  }
  require(
    read_small(root->value, marker, 128) == marker_text,
    "not an EasyFly recording directory; choose a new independent directory");
  return root;
}
[[maybe_unused]] std::unique_ptr<Fd> new_file(int dir, const std::string & name)
{
  return std::make_unique<Fd>(
    openat(
      dir, name.c_str(), O_RDWR | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0600));
}
uint64_t size_of(int fd)
{
  struct stat s {}; require(fstat(fd, &s) == 0 && s.st_size >= 0, "record stat failed");
  return s.st_size;
}
QString digest(int fd)
{
  const auto size = size_of(fd); require(size <= 64 * 1024 * 1024, "segment too large");
  QCryptographicHash hash(QCryptographicHash::Sha256);
  std::array<char, 65536> buffer{}; uint64_t offset = 0;
  while (offset < size) {
    const auto n =
      pread(fd, buffer.data(), std::min<uint64_t>(buffer.size(), size - offset), offset);
    if (n < 0 && errno == EINTR) {continue;}
    require(n > 0, "segment hash read failed"); hash.addData(buffer.data(), n); offset += n;
  }
  return QString::fromLatin1(hash.result().toHex());
}
#ifdef EASYFLY_ENABLE_MEDIA
// Classic RIFF AVI, single MJPEG stream. Exact exclusive-created FD, never a path-based
// encoder open/truncate. Segments are bounded below 64MiB; no OpenDML extension required.
class Avi
{
public:
  Avi(int dir, const std::string & name, int width, int height)
  : file_(new_file(dir, name)), width_(width), height_(height)
  {
    tag("RIFF"); riff_size_ = word(0); tag("AVI ");
    tag("LIST"); const auto hdrl = word(0); tag("hdrl");
    tag("avih"); word(56); word(100000); word(0); word(0); word(0x10);
    total_ = word(0); word(0); word(1); word(width * height * 3); word(width); word(height);
    for (int i = 0; i < 4; ++i) {word(0);}
    tag("LIST"); const auto strl = word(0); tag("strl");
    tag("strh"); word(56); tag("vids"); tag("MJPG"); word(0); word(0); word(0);
    word(1); word(10); word(0); length_ = word(0); word(width * height * 3);
    word(0xffffffff); word(0); word(0); word((height << 16) | width);
    tag("strf"); word(40); word(40); word(width); word(height); word((24 << 16) | 1);
    tag("MJPG"); word(width * height * 3); word(0); word(0); word(0); word(0);
    patch(strl, position() - strl - 4); patch(hdrl, position() - hdrl - 4);
    tag("LIST"); movi_size_ = word(0); movi_start_ = position(); tag("movi");
  }
  void append(const std::vector<unsigned char> & jpeg)
  {
    require(
      jpeg.size() < 8 * 1024 * 1024 && bytes() < 64 * 1024 * 1024,
      "segment size limit reached");
    index_.push_back(
      {static_cast<uint32_t>(position() - movi_start_),
        static_cast<uint32_t>(jpeg.size())});
    tag("00dc"); word(jpeg.size()); write_bytes(file_->value, jpeg.data(), jpeg.size());
    if (jpeg.size() % 2) {const char pad = 0; write_bytes(file_->value, &pad, 1);}
  }
  void finish()
  {
    require(!index_.empty(), "empty AVI segment");
    patch(movi_size_, position() - movi_size_ - 4);
    tag("idx1"); word(index_.size() * 16);
    for (const auto & item : index_) {tag("00dc"); word(0x10); word(item.first); word(item.second);}
    patch(riff_size_, position() - 8); patch(total_, index_.size()); patch(length_, index_.size());
    sync(file_->value);
    const auto path = "/proc/self/fd/" + std::to_string(file_->value);
    cv::VideoCapture reader(path, cv::CAP_OPENCV_MJPEG);
    require(reader.isOpened(), "AVI validation open failed");
    cv::Mat frame; unsigned count = 0;
    while (reader.read(frame)) {
      require(
        frame.cols == width_ && frame.rows == height_ && frame.type() == CV_8UC3,
        "AVI validation frame mismatch");
      require(++count <= index_.size(), "AVI validation extra frames");
    }
    require(count == index_.size(), "AVI validation truncated segment");
  }
  uint64_t bytes() const {return size_of(file_->value);}
  unsigned frames() const {return index_.size();}
  QString hash() const {return digest(file_->value);}

private:
  std::unique_ptr<Fd> file_;
  int width_, height_;
  off_t riff_size_, total_, length_, movi_size_, movi_start_;
  std::vector<std::pair<uint32_t, uint32_t>> index_;
  off_t position() const
  {
    const auto p = lseek(file_->value, 0, SEEK_CUR); require(p >= 0, "AVI seek failed"); return p;
  }
  void tag(const char * value) {write_bytes(file_->value, value, 4);}
  off_t word(uint32_t value)
  {
    const auto p = position(); const auto data = little(value);
    write_bytes(file_->value, data.data(), data.size()); return p;
  }
  static std::array<unsigned char, 4> little(uint32_t n)
  {
    return {static_cast<unsigned char>(n), static_cast<unsigned char>(n >> 8),
      static_cast<unsigned char>(n >> 16), static_cast<unsigned char>(n >> 24)};
  }
  void patch(off_t offset, uint32_t value)
  {
    const auto data = little(value); std::size_t sent = 0;
    while (sent < data.size()) {
      const auto n = pwrite(file_->value, data.data() + sent, data.size() - sent, offset + sent);
      if (n < 0 && errno == EINTR) {continue;}
      require(n > 0, "AVI header finalization failed"); sent += n;
    }
  }
};
class Session
{
public:
  RecordingConfig config;
  std::string id, path;
  uint64_t frames{0};
  QJsonArray segments;
  Session(const RecordingConfig & c, const std::vector<std::string> & protected_paths)
  : config(c), root_(root_directory(c.directory, protected_paths, true))
  {
    require(
      c.segment_frames > 0 && c.segment_frames <= 600 && c.byte_limit >= 65536 &&
      c.byte_limit <= 512ULL * 1024 * 1024, "invalid media budgets");
    space(); id = QUuid::createUuid().toString(QUuid::WithoutBraces).toStdString();
    require(mkdirat(root_->value, id.c_str(), 0700) == 0, "record ID directory already exists");
    dir_ = std::make_unique<Fd>(
      openat(
        root_->value, id.c_str(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC));
    require(private_dir(dir_->value), "unsafe recording session");
    path = (fs::canonical(c.directory) / id).string();
    auto initial = new_file(dir_->value, "inprogress.json");
    json_line(initial->value, {{"schema", 1}, {"id", str(id)}, {"status", "incomplete"}});
    sync(initial->value); sync(dir_->value); sync(root_->value);
    auto settings = new_file(dir_->value, "config.json");
    json_line(
      settings->value, {{"schema", 1}, {"ui_version", "0.1.0"},
        {"codec", "MJPEG/AVI"}, {"opencv_version", CV_VERSION}, {"fps", 10},
        {"timing", "fixed playback rate; source/steady/UTC in frames.jsonl; no gap filling"},
        {"profile_label_unconfirmed", str(c.profile)}, {"simulation_clock", c.simulation_clock},
        {"source_topic", str(c.binding.topic)}, {"camera", c.binding.camera},
        {"observation_session", integer(c.binding.session)}, {"epoch", integer(c.binding.epoch)},
        {"segment_frames", static_cast<int>(c.segment_frames)},
        {"byte_limit", integer(c.byte_limit)}, {"disk_reserve_bytes", integer(c.reserve_bytes)},
        {"capture", "UI sampled frames only; upstream losses unknown; no audio/cloud/desktop"}});
    sync(settings->value); config_hash_ = digest(settings->value);
    frame_log_ = new_file(dir_->value, "frames.jsonl");
    event_log_ = new_file(dir_->value, "events.jsonl");
    json_line(event_log_->value, {{"kind", "recording_started"}, {"attribution", "local_media"}});
    initializing_ = false;
  }
  void metadata(const RecordingPacket & p)
  {
    space();
    if (p.control) {
      const auto & c = *p.control;
      QJsonObject value{{"kind", "control_snapshot"}, {"phase", static_cast<int>(c.phase)},
        {"status", str(c.status)}, {"message", str(c.message)},
        {"server_instance", str(c.server_instance)},
        {"attribution", "local_controller_only_not_global_phase"}};
      if (c.intent) {
        const auto & i = *c.intent;
        value["request_id"] = str(i.request); value["instance_id"] = str(i.instance);
        value["execution_id"] = str(i.execution); value["action_uuid"] = str(i.goal);
        value["mission_name"] = str(i.name); value["content_hash"] = str(i.hash);
        value["resolved"] = i.resolved;
      }
      const auto fingerprint = QJsonDocument(value).toJson(QJsonDocument::Compact);
      if (fingerprint != control_fingerprint_) {
        control_fingerprint_ = fingerprint; json_line(event_log_->value, value);
      }
    }
    if (!p.observation) {return;}
    const auto & s = *p.observation;
    const bool image_available = p.frame && !p.frame->image.isNull() && p.frame->error.empty();
    if (!seen_image_state_ || image_available != image_available_) {
      seen_image_state_ = true; image_available_ = image_available;
      json_line(
        event_log_->value, {{"kind", "decoded_image_availability"},
          {"available", image_available}, {"observed_steady", s.produced_steady},
          {"clock_ns", QString::number(static_cast<qlonglong>(s.clock.now_ns))},
          {"reason", image_available ? "fresh_GUI_sample" : "no_fresh_decoded_frame"}});
    }
    if (s.counters.dropped_images != upstream_dropped_) {
      upstream_dropped_ = s.counters.dropped_images;
      json_line(
        event_log_->value, {{"kind", "observer_drop_counter"},
          {"count", integer(
              upstream_dropped_)}, {"semantics", "observer-local; upstream loss unknown"}});
    }
    const auto query = s.missions.find(bridge::Key::MissionQuery);
    if (query != s.missions.end() && query->second.value &&
      query->second.evidence.request_id != query_request_)
    {
      query_request_ = query->second.evidence.request_id;
      const auto & value = *query->second.value;
      if (!value.execution_id.empty()) {
        json_line(
          event_log_->value, {{"kind", "mission_query_result"},
            {"attribution", "queried_execution_only"}, {"execution_id", str(value.execution_id)},
            {"request_id", integer(query_request_)}, {"status", str(value.status)},
            {"success", value.success}, {"message", str(value.message)},
            {"quality_at_observation", static_cast<int>(query->second.evidence.quality)},
            {"received_steady", query->second.evidence.received_steady},
            {"execution_pending", value.execution_pending}});
      }
    }
    const auto profile = s.parameters.find(bridge::Key::Profile);
    if (profile != s.parameters.end() && profile->second.value &&
      profile->second.evidence.request_id != profile_request_)
    {
      profile_request_ = profile->second.evidence.request_id;
      json_line(
        event_log_->value, {{"kind", "queried_profile"},
          {"value_json", str(profile->second.value->current_value)},
          {"success", profile->second.value->success},
          {"quality_at_observation", static_cast<int>(profile->second.evidence.quality)},
          {"received_steady", profile->second.evidence.received_steady}});
    }
    unsigned appended = 0;
    for (const auto & e : s.events) {
      if (e.steady <= event_tail_) {continue;}
      if (++appended > 64) {
        json_line(event_log_->value, {{"kind", "event_gap"}, {"reason", "snapshot event budget"}});
        event_tail_ = s.events.back().steady; break;
      }
      json_line(
        event_log_->value, {{"kind", "observer_event"}, {"steady", e.steady},
          {"epoch", integer(e.epoch)}, {"text", str(
              e.text)}, {"attribution", "global_unattributed"}});
      event_tail_ = e.steady;
    }
    const auto & global = s.last_mission_event;
    if (global.value) {
      const auto stamp = bridge::detail::stamp_ns(global.value->header.stamp);
      if (stamp != global_stamp_) {
        global_stamp_ = stamp;
        json_line(
          event_log_->value, {{"kind", "global_mission_status"},
            {"source_ns", QString::number(static_cast<qlonglong>(stamp))},
            {"phase_name", str(global.value->phase_name)},
            {"quality_at_observation", static_cast<int>(global.evidence.quality)},
            {"first_received_steady", global.evidence.first_source_steady},
            {"attribution", "global_unattributed"}});
      }
    }
  }
  void append(const ConvertedFrame & f)
  {
    space();
    require(
      !f.image.isNull() && f.image.width() <= 1920 && f.image.height() <= 1080 &&
      f.image.width() > 0 && f.image.height() > 0, "invalid recording image");
    const auto image = f.image.scaled(
      std::min(1280, f.image.width()),
      std::min(720, f.image.height()), Qt::KeepAspectRatio, Qt::FastTransformation)
      .convertToFormat(QImage::Format_RGB888);
    if (avi_ && (avi_->frames() >= config.segment_frames || avi_->bytes() >= 56 * 1024 * 1024 ||
      image.width() != width_ || image.height() != height_)) {segment_finish();}
    if (!avi_) {
      require(segments.size() < 128, "segment count budget reached");
      width_ = image.width(); height_ = image.height();
      name_ = "segment-" + std::to_string(segments.size()) + ".avi";
      avi_ = std::make_unique<Avi>(dir_->value, name_, width_, height_);
    }
    cv::Mat rgb(image.height(), image.width(), CV_8UC3,
      const_cast<uchar *>(image.constBits()), image.bytesPerLine());
    cv::Mat bgr; cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    std::vector<unsigned char> jpeg;
    require(
      cv::imencode(".jpg", bgr, jpeg, {cv::IMWRITE_JPEG_QUALITY, 80}),
      "JPEG encoding failed");
    require(total_bytes() + jpeg.size() + 65536 < config.byte_limit, "record byte limit reached");
    avi_->append(jpeg);
    json_line(
      frame_log_->value, {{"frame", integer(frames)}, {"segment", str(name_)},
        {"segment_frame", static_cast<int>(avi_->frames() - 1)},
        {"source_ns", QString::number(static_cast<qlonglong>(f.stamp_ns))},
        {"first_received_steady", f.first_received},
        {"encoded_steady", bridge::detail::steady_seconds()},
        {"decoder_sequence", integer(f.sequence)}});
    ++frames;
  }
  void finish(const RecordingState & state, bool failed)
  {
    finalizing_ = true;  // Consume the reserved header/index/manifest budget on this exact session.
    if (!failed && avi_) {segment_finish();}
    json_line(
      event_log_->value, {{"kind", "recording_finished"}, {"reason", str(state.reason)},
        {"error", str(state.error)}, {"queue_replaced", integer(state.replaced)},
        {"rate_dropped", integer(state.rate_dropped)}, {"stale_dropped", integer(
            state.stale_dropped)},
        {"metadata_replaced", integer(state.metadata_replaced)}});
    sync(frame_log_->value); sync(event_log_->value);
    auto manifest = new_file(dir_->value, "manifest.json");
    json_line(
      manifest->value, {{"schema", 1}, {"id", str(id)},
        {"status", failed ? "failed" : (frames ? "verified" : "empty")},
        {"frames", integer(frames)}, {"segments", segments}, {"reason", str(state.reason)},
        {"error", str(state.error)}, {"config_sha256", config_hash_},
        {"frames_sha256", digest(frame_log_->value)},
        {"events_sha256", digest(event_log_->value)}});
    sync(manifest->value); sync(dir_->value); sync(root_->value);
  }

private:
  std::unique_ptr<Fd> root_, dir_, frame_log_, event_log_;
  std::unique_ptr<Avi> avi_;
  int width_{0}, height_{0};
  std::string name_;
  uint64_t closed_bytes_{0};
  double event_tail_{-1};
  int64_t global_stamp_{-1};
  QByteArray control_fingerprint_;
  QString config_hash_;
  uint64_t query_request_{0}, profile_request_{0};
  bool seen_image_state_{false}, image_available_{false};
  uint64_t upstream_dropped_{0};
  bool initializing_{true}, finalizing_{false};
  void json_line(int fd, QJsonObject value)
  {
    if (!initializing_ && !finalizing_) {
      const auto bytes = QJsonDocument(value).toJson(QJsonDocument::Compact).size() + 128;
      require(total_bytes() + bytes + 65536 < config.byte_limit, "record byte limit reached");
      require(
        size_of(frame_log_->value) + size_of(event_log_->value) + bytes <= 8 * 1024 * 1024,
        "metadata size limit reached");
    }
    ::easyfly_desktop::json_line(fd, std::move(value));
  }
  uint64_t total_bytes() const
  {
    return closed_bytes_ + (avi_ ? avi_->bytes() : 0) +
           size_of(frame_log_->value) + size_of(event_log_->value);
  }
  void space() const
  {
    struct statvfs s {};
    require(fstatvfs(root_->value, &s) == 0, "disk space query failed");
    require(
      static_cast<uint64_t>(s.f_bavail) * s.f_frsize > config.reserve_bytes,
      "disk reserve reached; recording stopped");
    if (frame_log_ && event_log_) {
      require(
        size_of(frame_log_->value) + size_of(event_log_->value) < 8 * 1024 * 1024,
        "metadata size limit reached");
    }
  }
  void segment_finish()
  {
    avi_->finish(); closed_bytes_ += avi_->bytes();
    segments.append(
      QJsonObject{{"name", str(name_)}, {"frames", static_cast<int>(avi_->frames())},
        {"bytes", integer(avi_->bytes())}, {"sha256", avi_->hash()}, {"readback_verified", true}});
    avi_.reset();
  }
};
#endif
std::vector<RecordingEntry> index_directory(
  const std::string & input, const std::vector<std::string> & protected_paths)
{
  auto root = root_directory(input, protected_paths, false);
  const auto anchored = fs::path("/proc/self/fd/" + std::to_string(root->value));
  std::vector<RecordingEntry> entries;
  unsigned inspected = 0;
  uint64_t hash_bytes = 0;
  for (const auto & child : fs::directory_iterator(anchored)) {
    require(++inspected <= 1024, "index directory entry budget exceeded");
    const auto name = child.path().filename().string();
    const auto id = QUuid(str(name));
    if (id.isNull() || id.toString(QUuid::WithoutBraces).toStdString() != name) {continue;}
    require(entries.size() < 256, "index limit 256; choose a separate directory for more records");
    RecordingEntry entry{name, "incomplete/corrupt", "not finalized or invalid manifest", 0};
    try {
      Fd dir(openat(root->value, name.c_str(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC));
      require(private_dir(dir.value), "unsafe session directory");
      const auto doc = QJsonDocument::fromJson(read_small(dir.value, "manifest.json"));
      require(doc.isObject(), "invalid manifest"); const auto m = doc.object();
      require(
        m["schema"].toInt() == 1 && m["id"].toString() == str(
          name), "manifest identity mismatch");
      const auto status = m["status"].toString();
      for (const auto & file : {"config", "frames", "events"}) {
        const auto filename = std::string(file) +
          (std::string(file) == "config" ? ".json" : ".jsonl");
        const auto bytes = read_small(dir.value, filename.c_str(), 8 * 1024 * 1024);
        const auto hash = QCryptographicHash::hash(bytes, QCryptographicHash::Sha256).toHex();
        require(
          m[QString::fromLatin1(file) + "_sha256"].toString() == QString::fromLatin1(hash),
          "metadata integrity mismatch");
        hash_bytes += bytes.size();
        require(hash_bytes <= 512ULL * 1024 * 1024, "index integrity byte budget exceeded");
      }
      require(
        status == "verified" || status == "failed" || status == "empty",
        "invalid record status");
      bool ok = false; entry.frames = m["frames"].toString().toULongLong(&ok);
      require(ok && entry.frames <= 36000 && m["segments"].isArray(), "invalid manifest counts");
      const auto segments = m["segments"].toArray(); uint64_t count = 0;
      require(segments.size() <= 36000, "invalid segment count");
      for (int i = 0; i < segments.size(); ++i) {
        const auto segment = segments[i].toObject();
        const auto filename = "segment-" + std::to_string(i) + ".avi";
        require(
          segment["name"].toString() == str(filename) &&
          segment["readback_verified"].toBool(), "invalid segment entry");
        Fd video(openat(
            dir.value, filename.c_str(),
            O_RDONLY | O_NOFOLLOW | O_NONBLOCK | O_CLOEXEC));
        struct stat s {};
        require(
          fstat(video.value, &s) == 0 && S_ISREG(s.st_mode) && s.st_nlink == 1 &&
          s.st_uid == geteuid() && (s.st_mode & 0777) == 0600, "unsafe segment");
        require(
          segment["bytes"].toString().toULongLong() == size_of(
            video.value), "segment size changed");
        hash_bytes += size_of(video.value);
        require(hash_bytes <= 512ULL * 1024 * 1024, "index integrity byte budget exceeded");
        require(
          segment["sha256"].toString().size() == 64 &&
          segment["sha256"].toString() == digest(video.value), "segment integrity mismatch");
        require(
          segment["frames"].toInt() > 0 && segment["frames"].toInt() <= 600,
          "invalid segment frame count");
        count += segment["frames"].toInt();
      }
      if (status == "verified") {
        require(count == entry.frames && count > 0, "frame count mismatch");
      }
      entry.status = status.toStdString(); entry.reason = m["reason"].toString().toStdString();
    } catch (const std::exception & e) {
      entry.reason = e.what();
    }
    entries.push_back(std::move(entry));
  }
  std::sort(
    entries.begin(), entries.end(),
    [](const auto & a, const auto & b) {return a.id < b.id;});
  return entries;
}
bool active(RecordingPhase p)
{
  return p == RecordingPhase::Starting || p == RecordingPhase::Recording ||
         p == RecordingPhase::Stopping;
}
}  // namespace
struct RecordingWorker::Impl
{
  mutable std::mutex mutex;
  std::condition_variable changed;
  RecordingState status;
  ImageBinding binding;
  std::vector<std::string> protected_paths;
  std::optional<RecordingConfig> start_request;
  std::optional<std::string> scan_request;
  RecordingPacket metadata;
  std::shared_ptr<const ConvertedFrame> pending;
  bool metadata_pending{false}, finish_requested{false}, shutdown{false}, scanning{false};
  std::thread thread;
  explicit Impl(std::vector<std::string> paths)
  : protected_paths(std::move(paths))
  {
    thread = std::thread([this] {run();});
  }
  ~Impl()
  {
    {std::lock_guard<std::mutex> lock(mutex); shutdown = true; changed.notify_one();}
    thread.join();
  }
  RecordingState snapshot() const {std::lock_guard<std::mutex> lock(mutex); return status;}
  void run()
  {
#ifdef EASYFLY_ENABLE_MEDIA
    std::unique_ptr<Session> session;
    double last_steady = -1; int64_t last_stamp = -1; uint64_t last_sequence = 0;
#endif
    while (true) {
      std::optional<RecordingConfig> start;
      std::optional<std::string> scan;
      RecordingPacket packet;
      [[maybe_unused]] bool finish = false;
      bool exit = false;
      {
        std::unique_lock<std::mutex> lock(mutex);
        changed.wait(
          lock, [this] {
            return start_request || scan_request || pending || metadata_pending ||
            finish_requested || shutdown;
          });
        start = std::move(start_request); start_request.reset();
        scan = std::move(scan_request); scan_request.reset();
        packet = std::move(metadata); metadata = {}; metadata_pending = false;
        packet.frame = std::move(pending); status.pending = 0;
        finish = finish_requested || shutdown; finish_requested = false; exit = shutdown;
      }
      if (scan) {
        try {
          auto entries = index_directory(*scan, protected_paths);
          std::lock_guard<std::mutex> lock(mutex); status.index = std::move(entries);
          status.index_error.clear();
        } catch (const std::exception & e) {
          std::lock_guard<std::mutex> lock(mutex); status.index.clear();
          status.index_error = e.what();
        }
        std::lock_guard<std::mutex> lock(mutex); scanning = false; status.index_pending = false;
      }
#ifdef EASYFLY_ENABLE_MEDIA
      try {
        if (start) {
          session = std::make_unique<Session>(*start, protected_paths);
          last_steady = -1; last_stamp = -1; last_sequence = 0;
          std::lock_guard<std::mutex> lock(mutex);
          status.id = session->id; status.path = session->path;
          if (!finish && !finish_requested && !shutdown) {status.phase = RecordingPhase::Recording;}
        }
        if (session && !finish) {
          session->metadata(packet);
          const auto & frame = packet.frame;
          if (frame && frame->sequence != last_sequence) {
            last_sequence = frame->sequence;
            const auto now = bridge::detail::steady_seconds();
            const auto & observation = packet.observation;
            const auto source_age = observation ?
              (static_cast<double>(observation->clock.now_ns) - frame->stamp_ns) / 1e9 : 2;
            const bool fresh = frame->binding == session->config.binding && frame->error.empty() &&
              frame->stamp_ns > 0 && std::isfinite(frame->first_received) &&
              now - frame->first_received >= 0 && now - frame->first_received <= 1 &&
              observation && observation->running && !observation->stopped &&
              observation->clock.quality == bridge::Quality::ValidAtObservation &&
              observation->observation_session_id == session->config.binding.session &&
              observation->time_epoch == session->config.binding.epoch &&
              now - observation->produced_steady >= 0 && now - observation->produced_steady <= 1 &&
              source_age >= -.2 && source_age <= 1;
            if (!fresh || frame->stamp_ns <= last_stamp || frame->first_received <= last_steady) {
              std::lock_guard<std::mutex> lock(mutex); ++status.stale_dropped;
            } else if (last_steady >= 0 && frame->first_received - last_steady < .1) {
              std::lock_guard<std::mutex> lock(mutex); ++status.rate_dropped;
            } else {
              require(session->frames < 36000, "record frame budget reached");
              session->append(*frame); last_steady = frame->first_received;
              last_stamp = frame->stamp_ns;
              std::lock_guard<std::mutex> lock(mutex);
              status.frames = session->frames; status.segments = session->segments.size();
            }
          }
        }
        if (session && finish) {
          auto s = snapshot(); session->finish(s, false);
          const auto count = session->segments.size(); session.reset();
          std::lock_guard<std::mutex> lock(mutex); status.segments = count;
          status.phase = RecordingPhase::Completed;
        }
        if (finish && !session && snapshot().phase == RecordingPhase::Stopping) {
          std::lock_guard<std::mutex> lock(mutex); status.phase = RecordingPhase::Completed;
        }
      } catch (const std::exception & e) {
        {
          std::lock_guard<std::mutex> lock(mutex); status.phase = RecordingPhase::Stopping;
          status.error = e.what(); status.reason = "local_recording_error";
          pending.reset(); metadata = {}; metadata_pending = false;
        }
        if (session) {
          try {session->finish(snapshot(), true);} catch (const std::exception & final_error) {
            std::lock_guard<std::mutex> lock(mutex);
            status.error += std::string("; finalization unconfirmed: ") + final_error.what();
          }
          session.reset();
        }
        std::lock_guard<std::mutex> lock(mutex); status.phase = RecordingPhase::Failed;
      }
#else
      if (start) {
        std::lock_guard<std::mutex> lock(mutex); status.phase = RecordingPhase::Failed;
        status.error = "media backend not enabled in this build";
      }
#endif
      if (exit) {break;}
    }
    std::lock_guard<std::mutex> lock(mutex); status.stopped = true;
  }
};
RecordingWorker::RecordingWorker(std::vector<std::string> paths)
: impl_(std::make_unique<Impl>(std::move(paths))) {}
RecordingWorker::~RecordingWorker() = default;
bool RecordingWorker::available()
{
#ifdef EASYFLY_ENABLE_MEDIA
  return true;
#else
  return false;
#endif
}
bool RecordingWorker::start(const RecordingConfig & config, const RecordingPacket & first)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!available() || impl_->shutdown || active(impl_->status.phase) || impl_->scanning ||
    !first.frame || !(first.frame->binding == config.binding) ||
    !recording_clock_end_reason(first, config.binding).empty()) {return false;}
  impl_->status = {}; impl_->status.phase = RecordingPhase::Starting;
  impl_->binding = config.binding;
  impl_->start_request = config; impl_->metadata = first; impl_->metadata.frame.reset();
  impl_->pending = first.frame; impl_->status.pending = 1; impl_->metadata_pending = true;
  impl_->finish_requested = false; impl_->changed.notify_one(); return true;
}
void RecordingWorker::offer(const RecordingPacket & packet)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->shutdown || (impl_->status.phase != RecordingPhase::Starting &&
    impl_->status.phase != RecordingPhase::Recording)) {return;}
  const auto reason = recording_clock_end_reason(packet, impl_->binding);
  if (!reason.empty()) {
    impl_->status.phase = RecordingPhase::Stopping; impl_->status.reason = reason;
    impl_->finish_requested = true; impl_->pending.reset(); impl_->status.pending = 0;
    impl_->metadata = packet; impl_->metadata.frame.reset(); impl_->metadata_pending = true;
    impl_->changed.notify_one(); return;
  }
  if (impl_->metadata_pending) {++impl_->status.metadata_replaced;}
  impl_->metadata = packet; impl_->metadata.frame.reset(); impl_->metadata_pending = true;
  if (packet.frame) {
    if (impl_->pending && impl_->pending->sequence != packet.frame->sequence) {
      ++impl_->status.replaced;
    }
    impl_->pending = packet.frame; impl_->status.pending = 1;
  }
  impl_->changed.notify_one();
}
void RecordingWorker::finish(const std::string & reason)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!active(impl_->status.phase) || impl_->status.phase == RecordingPhase::Stopping) {return;}
  impl_->status.phase = RecordingPhase::Stopping; impl_->status.reason = reason;
  impl_->finish_requested = true; impl_->pending.reset(); impl_->status.pending = 0;
  impl_->changed.notify_one();
}
bool RecordingWorker::scan(const std::string & directory)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->shutdown || active(impl_->status.phase) || impl_->scanning) {return false;}
  impl_->scanning = true; impl_->status.index_pending = true;
  impl_->scan_request = directory; impl_->changed.notify_one(); return true;
}
void RecordingWorker::stop()
{
  finish("window_closed");
  std::lock_guard<std::mutex> lock(impl_->mutex); impl_->shutdown = true;
  impl_->changed.notify_one();
}
RecordingState RecordingWorker::state() const {return impl_->snapshot();}
}  // namespace easyfly_desktop
