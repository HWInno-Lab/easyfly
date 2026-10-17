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

#ifndef EASYFLY_UI_BRIDGE__CAMERA_OBSERVATIONS_HPP_
#define EASYFLY_UI_BRIDGE__CAMERA_OBSERVATIONS_HPP_
#include <deque>
#include "easyfly_ui_bridge/model.hpp"
namespace easyfly_ui_bridge
{
// Single observer-worker owner. Snapshots expose at most two matched images,
// not the whole cache. Old snapshots remain immutable.
class CameraObservationBuffer
{
public:
  static constexpr size_t max_images = 8;
  static constexpr size_t max_bytes = 8 * 1024 * 1024;
  explicit CameraObservationBuffer(CameraPerceptionBinding binding);
  void image(sensor_msgs::msg::Image::ConstSharedPtr message, double now, uint64_t epoch);
  void detections(
    easyfly_msgs::msg::DetectionInfoArray::ConstSharedPtr message,
    double now, uint64_t epoch);
  void tracks(
    easyfly_msgs::msg::TrackInfoArray::ConstSharedPtr message,
    double now, uint64_t epoch);
  void targets(
    easyfly_msgs::msg::Target3DArray::ConstSharedPtr message,
    double now, uint64_t epoch);
  CameraObservations snapshot(const ClockState & clock, double now);
  void clear();

private:
  CameraObservations data_;
  std::deque<Observation<sensor_msgs::msg::Image>> images_;
  size_t bytes_{0};
  int64_t image_watermark_{0}, detection_watermark_{0}, track_watermark_{0}, target_watermark_{0};
};
}  // namespace easyfly_ui_bridge
#endif  // EASYFLY_UI_BRIDGE__CAMERA_OBSERVATIONS_HPP_
