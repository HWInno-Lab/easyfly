// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_landing_detection/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project

#pragma once
#include <fcntl.h>
#include <unistd.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>

namespace ef_timing
{
inline int64_t wall_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}
struct Event
{
  int64_t source_ns{0}, writer_wall_ns{0}, receive_wall_ns{0}, entry_wall_ns{0};
  int64_t entry_ros_ns{0}, publish_begin_wall_ns{0}, publish_end_wall_ns{0};
  bool input{false}, valid{false};
  char reason[64]{};
};
// Single-executor recorder: no additional reader, thread or callback-time I/O.
// Last 8192 publications only. Count/drop fields make truncation explicit.
class Trace
{
public:
  explicit Trace(const std::string & path)
  {
    if (path.empty()) {
      return;
    }
    events_ = std::make_unique<std::array<Event, capacity>>();
    int fd = open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
    if (fd < 0) {
      throw std::runtime_error("cannot exclusively create landing timing trace");
    }
    file_ = fdopen(fd, "w");
    if (!file_) {
      close(fd);
      throw std::runtime_error("cannot open landing timing stream");
    }
  }
  ~Trace()
  {
    if (!file_) {
      return;
    }
    const auto retained = std::min(count_, capacity);
    fprintf(file_, "# total=%zu retained=%zu dropped=%zu\n", count_, retained, count_ - retained);
    fprintf(
      file_, "sequence\tinput\tsource_ns\twriter_wall_ns\treceive_wall_ns\t"
      "entry_wall_ns\tentry_ros_ns\tpublish_begin_wall_ns\tpublish_end_wall_ns\tvalid\treason\n");
    for (size_t i = count_ - retained; i < count_; ++i) {
      const auto & e = (*events_)[i % capacity];
      fprintf(
        file_, "%zu\t%d\t%" PRId64 "\t%" PRId64 "\t%" PRId64 "\t%" PRId64
        "\t%" PRId64 "\t%" PRId64 "\t%" PRId64 "\t%d\t%s\n",
        i, e.input, e.source_ns, e.writer_wall_ns, e.receive_wall_ns, e.entry_wall_ns,
        e.entry_ros_ns, e.publish_begin_wall_ns, e.publish_end_wall_ns, e.valid, e.reason);
    }
    const bool failed = ferror(file_) != 0;
    if (fclose(file_) != 0 || failed) {
      fprintf(stderr, "landing timing trace write failed\n");
    }
  }
  bool enabled() const {return file_ != nullptr;}
  void record(Event event, const std::string & reason)
  {
    if (!enabled()) {
      return;
    }
    std::strncpy(event.reason, reason.c_str(), sizeof(event.reason) - 1);
    (*events_)[count_++ % capacity] = event;
  }

private:
  static constexpr size_t capacity = 8192;
  std::unique_ptr<std::array<Event, capacity>> events_;
  FILE * file_{nullptr};
  size_t count_{0};
};
}  // namespace ef_timing
