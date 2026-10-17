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

#include "easyfly_desktop/playback.hpp"
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <QCryptographicHash>
#include <QDateTime>
#include <QJsonArray>
#include <QJsonDocument>
#include <QUuid>
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <condition_variable>
#include <filesystem>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
namespace easyfly_desktop
{
namespace
{
void require(bool ok, const char * message)
{
  if (!ok) {throw std::runtime_error(message);}
}
struct File
{
  int fd;
  explicit File(int value)
  : fd(value) {require(fd >= 0, "local record file open failed");}
  ~File() {::close(fd);}
  File(const File &) = delete;
  File & operator=(const File &) = delete;
};
void directory(int fd)
{
  struct stat s {};
  require(
    fstat(fd, &s) == 0 && S_ISDIR(s.st_mode) && s.st_uid == geteuid() &&
    (s.st_mode & 0777) == 0700, "record directory must be owned mode0700");
}
std::unique_ptr<File> file(int dir, const QString & name, uint64_t limit)
{
  auto result = std::make_unique<File>(
    openat(
      dir, name.toUtf8().constData(),
      O_RDONLY | O_NONBLOCK | O_NOFOLLOW | O_CLOEXEC));
  struct stat s {};
  require(
    fstat(result->fd, &s) == 0 && S_ISREG(s.st_mode) && s.st_uid == geteuid() &&
    (s.st_mode & 0777) == 0600 && s.st_nlink == 1 && s.st_size >= 0 &&
    static_cast<uint64_t>(s.st_size) <= limit, "unsafe or oversized record file");
  return result;
}
uint64_t size(int fd)
{
  struct stat s {}; require(fstat(fd, &s) == 0 && s.st_size >= 0, "local record stat failed");
  return s.st_size;
}
QByteArray read(int fd, uint64_t offset, uint64_t count)
{
  require(count <= 64 * 1024 * 1024, "local read budget exceeded");
  QByteArray bytes(static_cast<int>(count), '\0'); uint64_t done = 0;
  while (done < count) {
    const auto n = pread(fd, bytes.data() + done, count - done, offset + done);
    if (n < 0 && errno == EINTR) {continue;}
    require(n > 0, "local record read failed"); done += n;
  }
  return bytes;
}
QByteArray hash(const QByteArray & bytes)
{
  return QCryptographicHash::hash(bytes, QCryptographicHash::Sha256).toHex();
}
QJsonObject object(const QByteArray & bytes)
{
  QJsonParseError error;
  const auto value = QJsonDocument::fromJson(bytes, &error);
  require(error.error == QJsonParseError::NoError && value.isObject(), "invalid record JSON");
  return value.object();
}
uint64_t integer(const QJsonValue & value)
{
  bool ok = false; const auto number = value.toString().toULongLong(&ok);
  require(ok && value.toString() == QString::number(number), "invalid exact integer");
  return number;
}
uint32_t word(const QByteArray & bytes, uint64_t offset)
{
  require(offset + 4 <= static_cast<uint64_t>(bytes.size()), "truncated AVI header");
  const auto * b = reinterpret_cast<const unsigned char *>(bytes.constData() + offset);
  return b[0] | (uint32_t(b[1]) << 8) | (uint32_t(b[2]) << 16) | (uint32_t(b[3]) << 24);
}
bool tag(const QByteArray & bytes, uint64_t at, const char * expected)
{
  return at + 4 <= static_cast<uint64_t>(bytes.size()) && bytes.mid(at, 4) == expected;
}
void jpeg_size(const QByteArray & bytes, int width, int height)
{
  // Only the baseline JPEG format emitted by the v1 recorder, before allocating decoded pixels.
  require(
    bytes.size() >= 4 && static_cast<unsigned char>(bytes[0]) == 0xff &&
    static_cast<unsigned char>(bytes[1]) == 0xd8, "not a recorder JPEG");
  int at = 2; bool dimensions = false;
  while (at + 4 <= bytes.size()) {
    require(static_cast<unsigned char>(bytes[at]) == 0xff, "invalid JPEG marker");
    const auto marker = static_cast<unsigned char>(bytes[at + 1]);
    require(marker != 0xd9, "JPEG dimensions missing");
    const int length = (static_cast<unsigned char>(bytes[at + 2]) << 8) |
      static_cast<unsigned char>(bytes[at + 3]);
    require(length >= 2 && at + length + 2 <= bytes.size(), "truncated JPEG marker");
    if (marker == 0xda) {
      require(dimensions, "JPEG dimensions missing"); return;
    }
    if (marker == 0xc0) {
      require(
        !dimensions && length == 17 && static_cast<unsigned char>(bytes[at + 4]) == 8 &&
        ((static_cast<unsigned char>(bytes[at + 5]) << 8) |
        static_cast<unsigned char>(bytes[at + 6])) == height &&
        ((static_cast<unsigned char>(bytes[at + 7]) << 8) |
        static_cast<unsigned char>(bytes[at + 8])) == width &&
        static_cast<unsigned char>(bytes[at + 9]) == 3, "JPEG dimensions differ from bounded AVI");
      dimensions = true;
    } else {
      require(
        (marker >= 0xe0 && marker <= 0xef) || marker == 0xdb || marker == 0xc4 ||
        marker == 0xfe, "unsupported nonbaseline JPEG marker");
    }
    at += length + 2;
  }
  require(false, "JPEG dimensions missing");
}
struct Slice {uint64_t offset; unsigned bytes; QByteArray sha;};
struct Segment
{
  std::unique_ptr<File> video;
  int width{0}, height{0};
  std::vector<Slice> slices;
};
struct Selection {unsigned segment, frame;};
struct Session
{
  std::shared_ptr<PlaybackRecord> record{std::make_shared<PlaybackRecord>()};
  std::vector<Segment> segments;
  std::vector<Selection> positions;
};
template<typename Cancel>
std::unique_ptr<Session> load(const QString & path, const QString & id, Cancel cancel)
{
  namespace fs = std::filesystem;
  const fs::path raw(path.toStdString());
  require(
    raw.is_absolute() && raw == raw.lexically_normal() && !raw.filename().empty() &&
    raw.filename() != "." && raw.filename() != "..", "choose a clean absolute record root");
  require(
    !QUuid(id).isNull() && QUuid(id).toString(QUuid::WithoutBraces) == id,
    "choose an exact record UUID, not a path");
  File root(::open(raw.c_str(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC));
  directory(root.fd);
  auto marker = file(root.fd, ".easyfly-recordings-v1", 128);
  require(
    read(marker->fd, 0, size(marker->fd)) == "EasyFly private media directory v1\n",
    "not an EasyFly private recording root");
  File dir(openat(
      root.fd, id.toUtf8().constData(),
      O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC)); directory(dir.fd);
  auto session = std::make_unique<Session>(); auto & r = *session->record; r.id = id;
  auto manifest = file(dir.fd, "manifest.json", 65536);
  r.manifest = object(read(manifest->fd, 0, size(manifest->fd)));
  require(
    r.manifest["schema"].toInt() == 1 && r.manifest["id"].toString() == id &&
    r.manifest["status"].toString() == "verified", "record is incomplete, failed or not verified");
  const auto total = integer(r.manifest["frames"]);
  require(
    total > 0 && total <= 36000 && r.manifest["segments"].isArray(),
    "invalid record frame counts");
  uint64_t metadata_bytes = 0, budget = 0;
  for (const auto & name : {"config", "frames", "events"}) {
    cancel();
    const QString key = name;
    auto f = file(dir.fd, key + (key == "config" ? ".json" : ".jsonl"), 8 * 1024 * 1024);
    const auto bytes = read(f->fd, 0, size(f->fd));
    metadata_bytes += bytes.size(); budget += bytes.size();
    require(metadata_bytes <= 8 * 1024 * 1024, "metadata budget exceeded");
    require(
      QString::fromLatin1(hash(bytes)) == r.manifest[key + "_sha256"].toString(),
      "metadata integrity mismatch");
    if (key == "config") {r.config = object(bytes); continue;}
    auto & output = key == "frames" ? r.frames : r.events;
    // Parse without creating an unbounded list of lines.
    int at = 0;
    while (at < bytes.size()) {
      cancel(); const int end = bytes.indexOf('\n', at);
      require(end >= at && end - at > 0 && end - at < 65536, "invalid metadata line");
      require(output.size() < 36000, "metadata entry budget exceeded");
      output.push_back(object(bytes.mid(at, end - at))); at = end + 1;
    }
  }
  require(
    r.config["schema"].toInt() == 1 && r.config["codec"].toString() == "MJPEG/AVI" &&
    r.config["fps"].toInt() == 10 && r.config["simulation_clock"].isBool(),
    "unsupported recorder config or clock domain");
  require(r.frames.size() == total, "frame log count mismatch");
  const auto entries = r.manifest["segments"].toArray();
  require(!entries.empty() && entries.size() <= 128, "segment budget exceeded");
  for (int i = 0; i < entries.size(); ++i) {
    cancel(); const auto entry = entries[i].toObject();
    const auto name = QString("segment-%1.avi").arg(i);
    require(
      entry["name"].toString() == name && entry["readback_verified"].toBool(),
      "invalid segment identity or readback status");
    Segment s; s.video = file(dir.fd, name, 64 * 1024 * 1024);
    const auto length = size(s.video->fd); budget += length;
    require(
      budget <= 512ULL * 1024 * 1024 && integer(entry["bytes"]) == length,
      "record size mismatch or read budget exceeded");
    const auto bytes = read(s.video->fd, 0, length); cancel();
    require(
      QString::fromLatin1(hash(bytes)) == entry["sha256"].toString(),
      "video integrity mismatch");
    // Deliberately narrow: only classic AVI headers and chunk layout written by our v1 recorder.
    require(
      length >= 248 && tag(bytes, 0, "RIFF") && word(bytes, 4) + 8ULL == length &&
      tag(bytes, 8, "AVI ") && tag(bytes, 12, "LIST") && word(bytes, 16) == 192 &&
      tag(bytes, 20, "hdrl") && tag(bytes, 24, "avih") && word(bytes, 28) == 56 &&
      word(bytes, 32) == 100000 && word(bytes, 56) == 1 &&
      tag(bytes, 88, "LIST") && word(bytes, 92) == 116 && tag(bytes, 96, "strl") &&
      tag(bytes, 100, "strh") && word(bytes, 104) == 56 && tag(bytes, 108, "vids") &&
      tag(bytes, 112, "MJPG") && word(bytes, 128) == 1 && word(bytes, 132) == 10 &&
      tag(bytes, 164, "strf") && word(bytes, 168) == 40 && word(bytes, 172) == 40 &&
      word(bytes, 184) == ((24 << 16) | 1) && tag(bytes, 188, "MJPG") &&
      tag(bytes, 212, "LIST") && tag(bytes, 220, "movi"), "unsupported or malformed recorder AVI");
    s.width = word(bytes, 64); s.height = word(bytes, 68);
    require(
      s.width > 0 && s.width <= 1280 && s.height > 0 && s.height <= 720 &&
      word(bytes, 176) == static_cast<unsigned>(s.width) &&
      word(bytes, 180) == static_cast<unsigned>(s.height), "AVI image dimensions out of bounds");
    const auto count = entry["frames"].toInt();
    require(
      count > 0 && count <= 600 && word(bytes, 48) == static_cast<unsigned>(count) &&
      word(bytes, 140) == static_cast<unsigned>(count), "AVI frame header mismatch");
    const uint64_t end = 220ULL + word(bytes, 216);
    require(
      end >= 224 && end + 8 <= length && tag(bytes, end, "idx1") &&
      word(bytes, end + 4) == count * 16u && end + 8 + count * 16ULL == length,
      "invalid AVI final index");
    uint64_t at = 224;
    for (int frame = 0; frame < count; ++frame) {
      cancel();
      require(at + 8 <= end && tag(bytes, at, "00dc"), "invalid AVI frame chunk");
      const auto n = word(bytes, at + 4);
      require(
        n > 0 && n < 8 * 1024 * 1024 && at + 8 + n + n % 2 <= end,
        "AVI frame byte budget exceeded");
      const auto index = end + 8 + frame * 16ULL;
      require(
        tag(bytes, index, "00dc") && word(bytes, index + 8) == at - 220 &&
        word(bytes, index + 12) == n, "AVI chunk/index mismatch");
      const auto jpeg = bytes.mid(at + 8, n); jpeg_size(jpeg, s.width, s.height);
      s.slices.push_back({at + 8, n, hash(jpeg)});
      session->positions.push_back({static_cast<unsigned>(i), static_cast<unsigned>(frame)});
      at += 8 + n + n % 2;
    }
    require(at == end, "extra AVI chunks");
    session->segments.push_back(std::move(s));
  }
  require(session->positions.size() == total, "segment/log counts disagree");
  double previous = -1; uint64_t source = 0;
  for (std::size_t i = 0; i < r.frames.size(); ++i) {
    const auto & f = r.frames[i]; const auto p = session->positions[i];
    const auto steady = f["first_received_steady"].toDouble(-1);
    const auto stamp = integer(f["source_ns"]);
    require(
      integer(f["frame"]) == i &&
      f["segment"].toString() == QString("segment-%1.avi").arg(p.segment) &&
      f["segment_frame"].toInt(-1) == static_cast<int>(p.frame),
      "frame log/AVI mapping mismatch");
    require(
      std::isfinite(steady) && steady >= 0 && steady > previous && stamp > source &&
      f["encoded_steady"].isDouble() && std::isfinite(f["encoded_steady"].toDouble()) &&
      f["encoded_steady"].toDouble() >= steady &&
      QDateTime::fromString(f["written_utc"].toString(), Qt::ISODateWithMs).isValid(),
      "invalid or nonmonotonic frame timing");
    previous = steady; source = stamp;
  }
  return session;
}
QImage decode(Session & session, int index)
{
  const auto position = session.positions.at(index);
  const auto & segment = session.segments.at(position.segment);
  const auto & slice = segment.slices.at(position.frame);
  const auto bytes = read(segment.video->fd, slice.offset, slice.bytes);
  require(hash(bytes) == slice.sha, "frame changed after integrity check; playback stopped");
  jpeg_size(bytes, segment.width, segment.height);
  const cv::Mat encoded(1, bytes.size(), CV_8UC1, const_cast<char *>(bytes.constData()));
  auto bgr = cv::imdecode(encoded, cv::IMREAD_COLOR);
  require(
    !bgr.empty() && bgr.cols == segment.width && bgr.rows == segment.height &&
    bgr.type() == CV_8UC3, "local JPEG decode failed");
  cv::Mat rgb; cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
  return QImage(rgb.data, rgb.cols, rgb.rows, rgb.step, QImage::Format_RGB888).copy();
}
}  // namespace
struct PlaybackWorker::Impl
{
  mutable std::mutex mutex;
  std::condition_variable changed;
  PlaybackState status;
  uint64_t generation{0};
  bool shutdown{false};
  std::optional<std::pair<QString, QString>> opening;
  std::optional<int> selection;
  std::thread thread;
  Impl()
  : thread([this] {run();}) {}
  PlaybackState snapshot() const
  {
    std::lock_guard<std::mutex> lock(mutex); return status;
  }
  void run()
  {
    std::unique_ptr<Session> session;
    for (;; ) {
      std::optional<std::pair<QString, QString>> request; std::optional<int> frame;
      uint64_t token;
      {
        std::unique_lock<std::mutex> lock(mutex);
        changed.wait(lock, [this] {return shutdown || opening || selection;});
        if (shutdown) {break;}
        request = std::move(opening); opening.reset();
        frame = selection; selection.reset(); token = generation;
      }
      auto cancel = [this, token] {
          std::lock_guard<std::mutex> lock(mutex);
          require(!shutdown && token == generation, "local playback request superseded");
        };
      try {
        if (request) {
          session.reset(); session = load(request->first, request->second, cancel); frame = 0;
        }
        cancel(); require(session && frame, "no loaded recording");
        const auto image = decode(*session, *frame); cancel();
        std::lock_guard<std::mutex> lock(mutex);
        if (token == generation && !shutdown) {
          status.loading = false; status.pending = selection.has_value(); status.error.clear();
          status.record = session->record; status.frame = *frame; status.image = image;
          ++status.revision;
        }
      } catch (const std::exception & error) {
        session.reset();
        std::lock_guard<std::mutex> lock(mutex);
        if (token == generation && !shutdown) {
          status.loading = false; status.pending = false; status.record.reset();
          status.image = {}; status.frame = -1; status.error = error.what(); selection.reset();
          ++status.revision;
        }
      }
    }
    session.reset();
    std::lock_guard<std::mutex> lock(mutex);
    status.stopped = true; status.loading = status.pending = false;
    status.record.reset(); status.image = {}; ++status.revision;
  }
};
PlaybackWorker::PlaybackWorker()
: impl_(std::make_unique<Impl>()) {}
PlaybackWorker::~PlaybackWorker() {stop(); impl_->thread.join();}
bool PlaybackWorker::open(const QString & path, const QString & id)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->shutdown) {return false;}
  ++impl_->generation; impl_->status = {}; impl_->status.loading = true;
  impl_->opening = std::make_pair(path, id); impl_->selection.reset(); impl_->changed.notify_one();
  return true;
}
bool PlaybackWorker::seek(int frame)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->shutdown || impl_->status.loading || !impl_->status.record || frame < 0 ||
    static_cast<std::size_t>(frame) >= impl_->status.record->frames.size()) {return false;}
  impl_->selection = frame; impl_->status.pending = true; impl_->changed.notify_one(); return true;
}
void PlaybackWorker::stop()
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->shutdown = true; ++impl_->generation; impl_->opening.reset(); impl_->selection.reset();
  impl_->changed.notify_one();
}
PlaybackState PlaybackWorker::state() const {return impl_->snapshot();}
}  // namespace easyfly_desktop
