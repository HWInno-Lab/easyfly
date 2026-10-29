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
#ifndef EASYFLY_DESKTOP__PLAYBACK_HPP_
#define EASYFLY_DESKTOP__PLAYBACK_HPP_
#include <QImage>
#include <QJsonObject>
#include <QString>
#include <memory>
#include <vector>
namespace easyfly_desktop
{
struct PlaybackRecord
{
  QString id;
  QJsonObject config, manifest;
  std::vector<QJsonObject> frames, events;
};
struct PlaybackState
{
  bool loading{false}, pending{false}, stopped{false};
  uint64_t revision{0};
  int frame{-1};
  QString error;
  QImage image;
  std::shared_ptr<const PlaybackRecord> record;
};
// Local files only: this library has no ROS dependencies, nodes or control ports.
// One pending selection and one immutable decoded image; I/O and decode off the GUI thread.
class PlaybackWorker
{
public:
  PlaybackWorker();
  ~PlaybackWorker();
  PlaybackWorker(const PlaybackWorker &) = delete;
  PlaybackWorker & operator=(const PlaybackWorker &) = delete;
  bool open(const QString & directory, const QString & id);
  bool seek(int frame);
  void stop();
  PlaybackState state() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace easyfly_desktop
#endif  // EASYFLY_DESKTOP__PLAYBACK_HPP_
