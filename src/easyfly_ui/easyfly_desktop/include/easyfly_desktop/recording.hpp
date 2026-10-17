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

#ifndef EASYFLY_DESKTOP__RECORDING_HPP_
#define EASYFLY_DESKTOP__RECORDING_HPP_
#include <memory>
#include <string>
#include <vector>
#include "easyfly_desktop/image_pipeline.hpp"
#include "easyfly_ui_control/controller.hpp"
namespace easyfly_desktop
{
struct RecordingOptions {bool enabled{false};};
struct RecordingConfig
{
  std::string directory, profile;
  ImageBinding binding;
  bool simulation_clock{true};
  // Local media budgets only; never flight parameters.
  unsigned segment_frames{600};
  uint64_t byte_limit{512ULL * 1024 * 1024}, reserve_bytes{256ULL * 1024 * 1024};
};
struct RecordingPacket
{
  std::shared_ptr<const ConvertedFrame> frame;
  std::shared_ptr<const bridge::UiSnapshot> observation;
  std::shared_ptr<const easyfly_ui_control::ControlSnapshot> control;
};
// Empty means trusted clock evidence; image freshness remains a separate per-frame check.
std::string recording_clock_end_reason(
  const RecordingPacket & packet,
  const ImageBinding & binding);
enum class RecordingPhase {Idle, Starting, Recording, Stopping, Completed, Failed};
struct RecordingEntry
{
  std::string id, status, reason;
  uint64_t frames{0};
  QString started, source, execution;
  double duration{-1};
  QImage thumbnail;
};
struct RecordingState
{
  RecordingPhase phase{RecordingPhase::Idle};
  bool stopped{false};
  bool index_pending{false};
  std::size_t pending{0};
  uint64_t frames{0}, replaced{0}, rate_dropped{0}, stale_dropped{0}, metadata_replaced{0};
  unsigned segments{0};
  std::string id, path, reason, error, index_error;
  std::vector<RecordingEntry> index;
};
// No ROS clients, control writes or GUI calls. One pending frame plus one metadata snapshot.
// All filesystem access, JPEG encoding, finalization and validation happen on its worker.
// Invalid clock offers latch finalization before another offer can replace the evidence.
class RecordingWorker
{
public:
  explicit RecordingWorker(std::vector<std::string> protected_paths = {});
  ~RecordingWorker();
  RecordingWorker(const RecordingWorker &) = delete;
  RecordingWorker & operator=(const RecordingWorker &) = delete;
  static bool available();
  bool start(const RecordingConfig & config, const RecordingPacket & first);
  void offer(const RecordingPacket & packet);
  void finish(const std::string & reason);
  bool scan(const std::string & directory);
  void stop();  // Nonblocking shutdown; waits for real file finalization on worker.
  RecordingState state() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace easyfly_desktop
#endif  // EASYFLY_DESKTOP__RECORDING_HPP_
