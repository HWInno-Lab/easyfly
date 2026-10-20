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

#include <algorithm>
#include <cmath>
#include <utility>
#include "easyfly_ui_bridge/camera_observations.hpp"
namespace easyfly_ui_bridge
{
namespace
{
bool header(const std_msgs::msg::Header & value)
{
  return detail::stamp_ns(value.stamp) > 0 && !value.frame_id.empty() &&
         value.frame_id.size() <= 256;
}
template<typename Box> bool box(const Box & value, const std_msgs::msg::Header & parent)
{
  return value.header == parent && value.class_name.size() <= 128 &&
         std::isfinite(value.confidence) && value.confidence >= 0 && value.confidence <= 1 &&
         std::isfinite(value.bbox_center_x) && value.bbox_center_x >= 0 &&
         value.bbox_center_x <= 1 &&
         std::isfinite(value.bbox_center_y) && value.bbox_center_y >= 0 &&
         value.bbox_center_y <= 1 &&
         std::isfinite(value.bbox_width) && value.bbox_width > 0 && value.bbox_width <= 1 &&
         std::isfinite(value.bbox_height) && value.bbox_height > 0 && value.bbox_height <= 1;
}
template<typename Message>
void receive(
  std::shared_ptr<const Message> message, Observation<Message> & current,
  int64_t & watermark, const std::string & topic, double now, uint64_t epoch, bool valid)
{
  if (!message) {return;}
  const auto ns = detail::stamp_ns(message->header.stamp);
  if (current.value && ns == watermark && *message == *current.value) {return;}
  current = {};
  current.evidence.endpoint = topic; current.evidence.time_epoch = epoch;
  current.evidence.received_steady = current.evidence.first_source_steady = now;
  if (!valid || !header(message->header) || ns <= watermark) {
    current.evidence.quality = Quality::Invalid;
    current.evidence.reason = "invalid/budgeted, out-of-order or conflicting camera observation";
    return;
  }
  watermark = ns; current.value = std::move(message);
  current.evidence.quality = Quality::ValidAtObservation;
}
template<typename Message>
Observation<Message> age(
  Observation<Message> current, const ClockState & clock, double now,
  double lifetime = 1.0)
{
  if (!current.value) {return current;}
  auto & evidence = current.evidence;
  evidence.receipt_age = now - evidence.first_source_steady;
  evidence.source_age = static_cast<double>(clock.now_ns -
    detail::stamp_ns(current.value->header.stamp)) / 1e9;
  evidence.source_age_available = clock.now_ns > 0;
  evidence.quality = evidence.time_epoch != clock.epoch ? Quality::ClockReset :
    detail::source_quality(current.value->header, clock, evidence.receipt_age, lifetime, false);
  return current;
}
}  // namespace
CameraObservationBuffer::CameraObservationBuffer(CameraPerceptionBinding binding)
{data_.binding = std::move(binding);}
void CameraObservationBuffer::clear()
{
  const auto binding = data_.binding; data_ = {}; data_.binding = binding;
  images_.clear(); bytes_ = 0;
  image_watermark_ = detection_watermark_ = track_watermark_ = target_watermark_ = 0;
  ellipse_watermark_ = color_line_watermark_ = 0;
}
void CameraObservationBuffer::image(
  sensor_msgs::msg::Image::ConstSharedPtr message,
  double now, uint64_t epoch)
{
  if (!message) {return;}
  const auto ns = detail::stamp_ns(message->header.stamp);
  if (ns <= image_watermark_) {
    for (auto it = images_.begin(); it != images_.end(); ++it) {
      if (it->value->header.stamp != message->header.stamp) {continue;}
      if (*it->value != *message) {bytes_ -= it->value->data.size(); images_.erase(it);}
      return;  // Duplicates never renew receipt lifetime; conflicts remove the match.
    }
    return;
  }
  if (!header(message->header) || detail::validate_image(*message) != Quality::ValidAtObservation ||
    message->data.size() > max_bytes) {return;}
  image_watermark_ = ns;
  Observation<sensor_msgs::msg::Image> entry; entry.value = std::move(message);
  entry.evidence.endpoint = data_.binding.image_topic; entry.evidence.time_epoch = epoch;
  entry.evidence.first_source_steady = entry.evidence.received_steady = now;
  entry.evidence.quality = Quality::ValidAtObservation;
  bytes_ += entry.value->data.size(); images_.push_back(std::move(entry));
  while (images_.size() > max_images || bytes_ > max_bytes) {
    bytes_ -= images_.front().value->data.size(); images_.pop_front();
  }
}
void CameraObservationBuffer::detections(
  easyfly_msgs::msg::DetectionInfoArray::ConstSharedPtr message, double now, uint64_t epoch)
{
  if (!message) {return;}
  const bool valid = message->detections.size() <= 100 &&
    std::all_of(
    message->detections.begin(), message->detections.end(), [&](const auto & value) {
      return value.track_id >= -1 && box(value, message->header);
    });
  receive(
    message, data_.detections, detection_watermark_, data_.binding.detections_topic,
    now, epoch, valid);
}
void CameraObservationBuffer::tracks(
  easyfly_msgs::msg::TrackInfoArray::ConstSharedPtr message, double now, uint64_t epoch)
{
  if (!message) {return;}
  const bool valid = message->tracks.size() <= 100 &&
    std::all_of(
    message->tracks.begin(), message->tracks.end(), [&](const auto & value) {
      return value.track_id >= 0 && box(value, message->header) &&
      value.trajectory_x.size() <= 30 && value.trajectory_y.size() <= 30 &&
      value.trajectory_z.size() <= 30;
    });
  receive(message, data_.tracks, track_watermark_, data_.binding.tracks_topic, now, epoch, valid);
}
void CameraObservationBuffer::targets(
  easyfly_msgs::msg::Target3DArray::ConstSharedPtr message, double now, uint64_t epoch)
{
  if (!message) {return;}
  const bool valid = message->header.frame_id == data_.binding.target_frame &&
    message->targets.size() <= 100 &&
    std::all_of(
    message->targets.begin(), message->targets.end(), [&](const auto & value) {
      return value.header == message->header && value.track_id >= -1 &&
      value.class_name.size() <= 128;
    });
  receive(
    message, data_.targets, target_watermark_, data_.binding.targets_topic, now, epoch,
    valid);
}
namespace
{
template<typename Message> bool geometry_envelope(const Message & message)
{
  return message.valid && message.reason.size() <= 256 && message.image_width > 0 &&
         message.image_height > 0 &&
         static_cast<uint64_t>(message.image_width) * message.image_height <= 2073600 &&
         message.observations.size() <= 32;
}
bool point(double x, double y, uint32_t width, uint32_t height)
{
  return std::isfinite(x) && std::isfinite(y) && x >= 0 && y >= 0 && x < width && y < height;
}
bool score(double value) {return std::isfinite(value) && value >= 0 && value <= 1;}
}  // namespace
void CameraObservationBuffer::ellipses(
  easyfly_msgs::msg::EllipseObservationArray::ConstSharedPtr message, double now, uint64_t epoch)
{
  if (!message) {return;}
  const bool valid = geometry_envelope(*message) && std::all_of(
    message->observations.begin(), message->observations.end(), [&](const auto & value) {
      return point(value.center_x, value.center_y, message->image_width, message->image_height) &&
      std::isfinite(value.major_axis_px) && std::isfinite(value.minor_axis_px) &&
      value.minor_axis_px > 0 && value.major_axis_px >= value.minor_axis_px &&
      value.major_axis_px <= 8192 && std::isfinite(value.angle_rad) &&
      value.angle_rad >= 0 && value.angle_rad < 3.141593 && score(value.score) &&
      std::isfinite(value.radial_error) && value.radial_error >= 0;
    });
  receive(
    message, data_.ellipses, ellipse_watermark_, data_.binding.ellipse_topic, now, epoch,
    valid);
}
void CameraObservationBuffer::color_lines(
  easyfly_msgs::msg::ColorLineObservationArray::ConstSharedPtr message, double now, uint64_t epoch)
{
  if (!message) {return;}
  const bool valid = geometry_envelope(*message) && std::all_of(
    message->observations.begin(), message->observations.end(), [&](const auto & value) {
      return point(value.center_x, value.center_y, message->image_width, message->image_height) &&
      point(value.start_x, value.start_y, message->image_width, message->image_height) &&
      point(value.end_x, value.end_y, message->image_width, message->image_height) &&
      std::isfinite(value.length_px) && value.length_px > 0 && value.length_px <= 8192 &&
      std::isfinite(value.width_px) && value.width_px > 0 && value.width_px <= 8192 &&
      std::isfinite(value.area_px) && value.area_px > 0 && value.area_px <= 2073600 &&
      std::isfinite(value.angle_rad) && std::abs(value.angle_rad) <= 1.570797 &&
      std::isfinite(value.offset_x_normalized) && std::abs(value.offset_x_normalized) <= 1 &&
      score(value.score);
    });
  receive(
    message, data_.color_lines, color_line_watermark_, data_.binding.color_line_topic, now,
    epoch, valid);
}
CameraObservations CameraObservationBuffer::snapshot(const ClockState & clock, double now)
{
  while (!images_.empty() && now - images_.front().evidence.first_source_steady > 1) {
    bytes_ -= images_.front().value->data.size(); images_.pop_front();
  }
  auto result = data_; result.cached_images = images_.size(); result.cached_bytes = bytes_;
  result.detections = age(result.detections, clock, now);
  result.ellipses = age(result.ellipses, clock, now, .5);
  result.color_lines = age(result.color_lines, clock, now, .5);
  result.tracks = age(result.tracks, clock, now); result.targets = age(result.targets, clock, now);
  const auto match = [&](const auto & observation) {
      Observation<sensor_msgs::msg::Image> found;
      if (observation.evidence.quality != Quality::ValidAtObservation || !observation.value) {
        return found;
      }
      for (const auto & image : images_) {
        if (image.value->header == observation.value->header) {return age(image, clock, now);}
      }
      found.evidence.quality = Quality::Unavailable;
      found.evidence.reason = "no exact image header match in bounded cache";
      return found;
    };
  result.detection_image = match(result.detections); result.track_image = match(result.tracks);
  result.ellipse_image = match(result.ellipses);
  result.color_line_image = match(result.color_lines);
  return result;
}
}  // namespace easyfly_ui_bridge
