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
#ifndef CAMERA_STREAM_HPP_
#define CAMERA_STREAM_HPP_
#include <easyfly_perception_contracts/contracts.hpp>
#include <array>
#include <optional>
#include <set>
#include <string>
#include <type_traits>
#include <utility>
#include "easyfly_ui_bridge/camera_observations.hpp"
namespace easyfly_ui_bridge
{
namespace detail
{
namespace pc = easyfly::perception;
// Worker-private transaction cache. Public buffer/model layouts stay unchanged.
class CameraStream
{
public:
  explicit CameraStream(CameraPerceptionBinding binding)
  : buffer_(std::move(binding)) {}
  void clear()
  {native_detections_.reset(); native_tracks_.reset(); native_targets_.reset(); buffer_.clear(); pairs_ = {}; conflict_stamps_ = {}; completed_headers_ = {}; epoch_.reset();}
  void source_identity(const std::string & identity)
  {
    if (identity != identity_) {clear(); identity_ = identity;}
  }
  void detections_v2(easyfly_msgs::msg::DetectionArrayV2::ConstSharedPtr m, double now, const ClockState & clock)
  {native(m, native_detections_, gates_[0], now, clock, &CameraObservationBuffer::detections);}
  void tracks_v2(easyfly_msgs::msg::ImageTrackArrayV2::ConstSharedPtr m, double now, const ClockState & clock)
  {native(m, native_tracks_, gates_[1], now, clock, &CameraObservationBuffer::tracks);}
  void targets_v2(easyfly_msgs::msg::WorldTargetArrayV2::ConstSharedPtr m, double now, const ClockState & clock)
  {native(m, native_targets_, gates_[2], now, clock, &CameraObservationBuffer::targets);}
  void image(sensor_msgs::msg::Image::ConstSharedPtr message, double now, const ClockState & clock)
  {
    origin(clock);
    if (message) {
      for (size_t index = 0; index < pairs_.size(); ++index) {
        const auto & pair = pairs_[index];
        if (!pair) {
          // The byte budget may have evicted the original pixels while a desktop
          // still holds this transaction. A same-stamp replay cannot be verified.
          if (completed_headers_[index] &&
            completed_headers_[index]->stamp == message->header.stamp)
          {conflict_stamps_[index] = stamp_ns(message->header.stamp);}
          continue;
        }
        bool conflict = false;
        for (const auto * image : { & pair->detection_image, & pair->track_image,
            & pair->ellipse_image, & pair->color_line_image})
        {
          if (image->value && image->value->header.stamp == message->header.stamp &&
            *image->value != *message) {conflict = true;}
        }
        if (conflict) {
          conflict_stamps_[index] = stamp_ns(message->header.stamp); pairs_[index].reset();
        }
      }
    }
    buffer_.image(std::move(message), now, clock.epoch); capture(clock, now);
  }
  void detections(
    easyfly_msgs::msg::DetectionInfoArray::ConstSharedPtr m, double now, const ClockState & clock)
  {receive(m, now, clock, &CameraObservationBuffer::detections);}
  void tracks(
    easyfly_msgs::msg::TrackInfoArray::ConstSharedPtr m, double now, const ClockState & clock)
  {receive(m, now, clock, &CameraObservationBuffer::tracks);}
  void targets(
    easyfly_msgs::msg::Target3DArray::ConstSharedPtr m, double now, const ClockState & clock)
  {receive(m, now, clock, &CameraObservationBuffer::targets);}
  void ellipses(
    easyfly_msgs::msg::EllipseObservationArray::ConstSharedPtr m, double now,
    const ClockState & clock)
  {receive(m, now, clock, &CameraObservationBuffer::ellipses);}
  void color_lines(
    easyfly_msgs::msg::ColorLineObservationArray::ConstSharedPtr m,
    double now, const ClockState & clock)
  {receive(m, now, clock, &CameraObservationBuffer::color_lines);}
  CameraObservations snapshot(const ClockState & clock, double now)
  {
    origin(clock); auto result = capture(clock, now);
    fallback(
      0, &CameraObservations::detections, &CameraObservations::detection_image,
      result, clock, now, 1.);
    fallback(
      1, &CameraObservations::tracks, &CameraObservations::track_image,
      result, clock, now, 1.);
    fallback(
      2, &CameraObservations::ellipses, &CameraObservations::ellipse_image,
      result, clock, now, .5);
    fallback(
      3, &CameraObservations::color_lines, &CameraObservations::color_line_image,
      result, clock, now, .5);
    reject_conflict(
      0, &CameraObservations::detections, &CameraObservations::detection_image,
      result);
    reject_conflict(1, &CameraObservations::tracks, &CameraObservations::track_image, result);
    reject_conflict(2, &CameraObservations::ellipses, &CameraObservations::ellipse_image, result);
    reject_conflict(
      3, &CameraObservations::color_lines, &CameraObservations::color_line_image,
      result);
    if (native_detections_ && result.detections.value && result.detections.value->header == native_detections_->context.header) {
      result.native_detections = native_detections_;
      if (result.detection_image.value && (result.detection_image.value->width != native_detections_->context.image_width ||
          result.detection_image.value->height != native_detections_->context.image_height)) {
        result.detections = {}; result.detections.evidence.quality = Quality::Invalid;
      }
    }
    if (native_tracks_ && result.tracks.value && result.tracks.value->header == native_tracks_->context.header) {
      result.native_tracks = native_tracks_;
      if (result.track_image.value && (result.track_image.value->width != native_tracks_->context.image_width ||
          result.track_image.value->height != native_tracks_->context.image_height)) {
        result.tracks = {}; result.tracks.evidence.quality = Quality::Invalid;
      }
    }
    if (native_targets_ && result.targets.value && result.targets.value->header == native_targets_->context.header) {
      result.native_targets = native_targets_;
    }
    return result;
  }
  size_t pair_bytes() const
  {
    std::set<const sensor_msgs::msg::Image *> unique;
    size_t bytes = 0;
    for (const auto & pair : pairs_) {
      if (!pair) {continue;}
      for (const auto * image : { & pair->detection_image, & pair->track_image,
          & pair->ellipse_image, & pair->color_line_image})
      {
        if (image->value && unique.insert(image->value.get()).second) {
          bytes += image->value->data.size();
        }
      }
    }
    return bytes;
  }

private:
  template<typename Message, typename Callback>
  void native(Message m, Message & retained, pc::SourceGate & gate, double now, const ClockState & clock, Callback callback)
  {
    origin(clock);
    const auto state = gate.observe(m->context);
    if (state == pc::SourceGate::rejected) {return;}
    if (state == pc::SourceGate::changed) {clear(); epoch_ = clock.epoch;}
    try {
      auto numeric = pc::numeric_snapshot(*m);
      using Numeric = decltype(numeric);
      (buffer_.*callback)(std::make_shared<const Numeric>(std::move(numeric)), now, clock.epoch);
      retained = m; capture(clock, now);
    } catch (const std::exception &) {clear(); epoch_ = clock.epoch;}
  }
  easyfly_msgs::msg::DetectionArrayV2::ConstSharedPtr native_detections_;
  easyfly_msgs::msg::ImageTrackArrayV2::ConstSharedPtr native_tracks_;
  easyfly_msgs::msg::WorldTargetArrayV2::ConstSharedPtr native_targets_;
  std::array<pc::SourceGate, 3> gates_;
  void origin(const ClockState & clock)
  {
    if (!epoch_ || *epoch_ != clock.epoch) {clear(); epoch_ = clock.epoch;}
  }
  template<typename Message, typename Callback>
  void receive(Message message, double now, const ClockState & clock, Callback callback)
  {
    origin(clock); (buffer_.*callback)(std::move(message), now, clock.epoch); capture(clock, now);
  }
  template<typename Message>
  static Observation<Message> aged(
    Observation<Message> value, const ClockState & clock, double now, double ttl)
  {
    if (!value.value) {return value;}
    auto & e = value.evidence; e.receipt_age = now - e.first_source_steady;
    e.source_age_available = clock.now_ns > 0;
    e.source_age = e.source_age_available ?
      static_cast<double>(clock.now_ns - stamp_ns(value.value->header.stamp)) / 1e9 : 0;
    e.quality = e.time_epoch != clock.epoch ? Quality::ClockReset :
      source_quality(value.value->header, clock, e.receipt_age, ttl, false);
    return value;
  }
  template<typename Message>
  void retain(
    size_t index, Observation<Message> CameraObservations::* metadata,
    Observation<sensor_msgs::msg::Image> CameraObservations::* image,
    const CameraObservations & current)
  {
    const auto & m = current.*metadata; const auto & i = current.*image;
    if (m.evidence.quality != Quality::ValidAtObservation || !m.value) {
      pairs_[index].reset(); return;
    }
    if (i.evidence.quality != Quality::ValidAtObservation || !i.value) {return;}
    if (conflict_stamps_[index] > 0 && stamp_ns(i.value->header.stamp) <= conflict_stamps_[index]) {
      return;
    }
    if constexpr (std::is_same_v<Message, easyfly_msgs::msg::EllipseObservationArray>||
      std::is_same_v<Message, easyfly_msgs::msg::ColorLineObservationArray>)
    {
      if (m.value->image_width != i.value->width || m.value->image_height != i.value->height) {
        pairs_[index].reset(); return;
      }
    }
    CameraObservations pair; pair.binding = current.binding;
    pair.*metadata = m; pair.*image = i;
    if (index == 1 && current.targets.value &&
      current.targets.value->header.stamp == m.value->header.stamp)
    {pair.targets = current.targets;}
    conflict_stamps_[index] = 0;
    completed_headers_[index] = i.value->header;
    pairs_[index] = std::move(pair);
  }
  CameraObservations capture(const ClockState & clock, double now)
  {
    auto current = buffer_.snapshot(clock, now);
    retain(0, &CameraObservations::detections, &CameraObservations::detection_image, current);
    retain(1, &CameraObservations::tracks, &CameraObservations::track_image, current);
    retain(2, &CameraObservations::ellipses, &CameraObservations::ellipse_image, current);
    retain(3, &CameraObservations::color_lines, &CameraObservations::color_line_image, current);
    if (pairs_[1] && current.targets.evidence.quality != Quality::ValidAtObservation) {
      pairs_[1]->targets = {};
    }
    while (pair_bytes() > CameraObservationBuffer::max_bytes) {
      size_t oldest = pairs_.size(); double receipt = 0;
      for (size_t i = 0; i < pairs_.size(); ++i) {
        if (!pairs_[i]) {continue;}
        const auto & pair = *pairs_[i];
        const auto * image = i == 0 ? &pair.detection_image : i == 1 ? &pair.track_image :
          i == 2 ? &pair.ellipse_image : &pair.color_line_image;
        if (oldest == pairs_.size() || image->evidence.first_source_steady < receipt) {
          oldest = i; receipt = image->evidence.first_source_steady;
        }
      }
      if (oldest == pairs_.size()) {break;}
      pairs_[oldest].reset();
    }
    return current;
  }
  template<typename Message>
  void reject_conflict(
    size_t index, Observation<Message> CameraObservations::* metadata,
    Observation<sensor_msgs::msg::Image> CameraObservations::* image,
    CameraObservations & current)
  {
    if (conflict_stamps_[index] <= 0) {return;}
    auto & m = current.*metadata; m.value.reset(); m.evidence.quality = Quality::Invalid;
    m.evidence.reason = "same-stamp raw image conflict; awaiting a new complete pair";
    current.*image = {}; (current.*image).evidence.quality = Quality::Invalid;
  }
  template<typename Message>
  void fallback(
    size_t index, Observation<Message> CameraObservations::* metadata,
    Observation<sensor_msgs::msg::Image> CameraObservations::* image,
    CameraObservations & current, const ClockState & clock, double now, double ttl)
  {
    if (!pairs_[index]) {return;}
    const auto m = aged((*pairs_[index]).*metadata, clock, now, ttl);
    const auto i = aged((*pairs_[index]).*image, clock, now, 1.);
    if (m.evidence.quality != Quality::ValidAtObservation ||
      i.evidence.quality != Quality::ValidAtObservation)
    {pairs_[index].reset(); return;}
    const auto & latest = current.*metadata; const auto & latest_image = current.*image;
    if (latest.evidence.quality == Quality::ValidAtObservation && latest.value &&
      !latest_image.value && latest_image.evidence.quality == Quality::Unavailable)
    {
      current.*metadata = m; current.*image = i;
      if (index == 1 && current.targets.evidence.quality == Quality::ValidAtObservation) {
        current.targets = aged(pairs_[index]->targets, clock, now, 1.);
      }
    }
  }
  CameraObservationBuffer buffer_;
  std::array<std::optional<CameraObservations>, 4> pairs_;
  std::array<int64_t, 4> conflict_stamps_{};
  std::array<std::optional<std_msgs::msg::Header>, 4> completed_headers_;
  std::optional<uint64_t> epoch_;
  std::string identity_;
};
}  // namespace detail
}  // namespace easyfly_ui_bridge
#endif  // CAMERA_STREAM_HPP_
