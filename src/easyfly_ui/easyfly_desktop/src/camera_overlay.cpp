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
#include "easyfly_desktop/camera_overlay.hpp"
#include "easyfly_desktop/presentation.hpp"
namespace easyfly_desktop
{
namespace
{
template<typename Message>
bool same_frame(
  const bridge::Observation<Message> & observation, const ConvertedFrame & frame,
  const bridge::UiSnapshot & snapshot, double now)
{
  return observation.value && display_quality(observation.evidence, snapshot, now, 1, true) ==
         bridge::Quality::ValidAtObservation &&
         now - observation.evidence.first_source_steady <= 1 &&
         bridge::detail::stamp_ns(observation.value->header.stamp) == frame.stamp_ns &&
         observation.value->header.frame_id == frame.source_frame;
}
template<typename Box>
bool valid_box(const Box & box, const std_msgs::msg::Header & header)
{
  return box.header == header && box.class_name.size() <= 128 &&
         std::isfinite(box.confidence) && box.confidence >= 0 && box.confidence <= 1 &&
         std::isfinite(box.bbox_center_x) && box.bbox_center_x >= 0 && box.bbox_center_x <= 1 &&
         std::isfinite(box.bbox_center_y) && box.bbox_center_y >= 0 && box.bbox_center_y <= 1 &&
         std::isfinite(box.bbox_width) && box.bbox_width > 0 && box.bbox_width <= 1 &&
         std::isfinite(box.bbox_height) && box.bbox_height > 0 && box.bbox_height <= 1;
}
template<typename Box>
CameraOverlayBox present(const Box & box, bool tracking)
{
  CameraOverlayBox result;
  result.normalized = QRectF(
    box.bbox_center_x - box.bbox_width / 2,
    box.bbox_center_y - box.bbox_height / 2, box.bbox_width, box.bbox_height).
    intersected(QRectF(0, 0, 1, 1));
  result.class_name = text(box.class_name, 128); result.confidence = box.confidence;
  result.tracking = tracking; result.id = tracking ? box.track_id : -1;
  result.label =
    (tracking ? QStringLiteral("SORT #%1 ").arg(box.track_id) : QStringLiteral("检测 ")) +
    result.class_name + QStringLiteral(" %1%").arg(number(box.confidence * 100, 1));
  result.depth_status = tracking ? QStringLiteral("深度未匹配") : QStringLiteral("无跟踪关联");
  return result;
}
void depth(
  CameraOverlayBox & box, const bridge::CameraObservations & observations,
  const ConvertedFrame & frame, const bridge::UiSnapshot & snapshot, double now)
{
  const auto & targets = observations.targets;
  if (!targets.value || display_quality(targets.evidence, snapshot, now, 1, true) !=
    bridge::Quality::ValidAtObservation ||
    targets.value->targets.size() > 100 ||
    now - targets.evidence.first_source_steady > 1 ||
    bridge::detail::stamp_ns(targets.value->header.stamp) != frame.stamp_ns ||
    targets.value->header.frame_id != observations.binding.target_frame) {return;}
  const easyfly_msgs::msg::Target3D * match = nullptr;
  for (const auto & target : targets.value->targets) {
    if (target.track_id != box.id) {continue;}
    if (match) {box.depth_status = QStringLiteral("ID关联不唯一"); return;}
    match = &target;
  }
  if (!match || match->class_name != box.class_name.toStdString() ||
    match->header != targets.value->header) {return;}
  if (!match->position_valid || !std::isfinite(match->distance) || match->distance <= 0 ||
    !std::isfinite(match->confidence) || match->confidence < 0 || match->confidence > 1 ||
    match->distance > 1e6 || !std::isfinite(match->position_x) ||
    !std::isfinite(match->position_y) || !std::isfinite(match->position_z))
  {
    box.depth_status = QStringLiteral("上游深度/定位无效"); return;
  }
  // Pixel-to-3D samples depth at the original track center. Do not compare
  // against the clipped rectangle center, which differs at image boundaries.
  const auto & tracks = observations.tracks.value->tracks;
  const auto source = std::find_if(
    tracks.begin(), tracks.end(),
    [&](const auto & track) {return track.track_id == box.id;});
  if (source == tracks.end() || match->pixel_u < 0 || match->pixel_v < 0 ||
    match->pixel_u >= frame.image.width() || match->pixel_v >= frame.image.height() ||
    std::abs(match->pixel_u - source->bbox_center_x * frame.image.width()) > 1 ||
    std::abs(match->pixel_v - source->bbox_center_y * frame.image.height()) > 1)
  {
    box.depth_status = QStringLiteral("像素中心不匹配"); return;
  }
  box.axial_depth = match->distance;
  box.depth_status = QStringLiteral("同帧上游融合光轴深度");
  box.label += QStringLiteral(" · z=%1m").arg(number(*box.axial_depth));
}
}  // namespace
QRect camera_image_rect(QSize canvas, QSize image)
{
  if (canvas.width() <= 0 || canvas.height() <= 48 || image.isEmpty()) {return {};}
  const auto available = QSize(canvas.width(), canvas.height() - 48);
  const auto size = image.scaled(available, Qt::KeepAspectRatio);
  return QRect(
    (available.width() - size.width()) / 2, (available.height() - size.height()) / 2,
    size.width(), size.height());
}
QRect camera_label_rect(
  QRect bounds, QRect image, QSize label,
  const std::vector<QRect> & occupied)
{
  if (image.isEmpty() || label.isEmpty()) {return {};}
  label.setWidth(std::min(label.width(), image.width()));
  label.setHeight(std::min(label.height(), image.height()));
  const int x = std::clamp(bounds.left(), image.left(), image.right() + 1 - label.width());
  const auto candidate = [&](int y) {
      QRect result(x, std::clamp(y, image.top(), image.bottom() + 1 - label.height()),
        label.width(), label.height());
      for (const auto & previous : occupied) {
        if (result.adjusted(-1, -1, 1, 1).intersects(previous)) {return QRect{};}
      }
      return result;
    };
  const int rows = image.height() / (label.height() + 2) + 1;
  for (int row = 0; row < rows; ++row) {
    const auto result = candidate(bounds.top() - (row + 1) * (label.height() + 2));
    if (!result.isEmpty()) {return result;}
  }
  for (int row = 0; row < rows; ++row) {
    const auto result = candidate(image.top() + row * (label.height() + 2));
    if (!result.isEmpty()) {return result;}
  }
  return {};
}
CameraOverlay camera_overlay(
  const bridge::UiSnapshot & snapshot,
  const ConvertedFrame & frame, int channel, CameraOverlayMode mode, double now,
  const bridge::CameraObservations * selected)
{
  CameraOverlay result;
  if (mode == CameraOverlayMode::Raw) {
    result.status = QStringLiteral("实时原图，不叠框"); return result;
  }
  result.status = QStringLiteral("没有新鲜的同帧感知，未叠框");
  if (channel < 0 || channel >= 2 || !snapshot.running || snapshot.stopped ||
    frame.binding.session != snapshot.observation_session_id ||
    frame.binding.epoch != snapshot.time_epoch || frame.binding.camera != channel ||
    snapshot.clock.epoch != snapshot.time_epoch ||
    snapshot.clock.quality != bridge::Quality::ValidAtObservation || frame.image.isNull() ||
    !frame.error.empty() ||
    frame.stamp_ns <= 0 || frame.source_frame.empty() || now - frame.first_received > 1 ||
    frame.stamp_ns - snapshot.clock.now_ns > 200000000LL ||
    snapshot.clock.now_ns - frame.stamp_ns > 1000000000LL) {return result;}
  const auto & observations = selected ? *selected : snapshot.camera_perception[channel];
  if (observations.binding.image_topic.empty() ||
    observations.binding.image_topic != frame.binding.topic)
  {
    result.status = QStringLiteral("该相机没有明确感知绑定，未叠框"); return result;
  }
  if (mode == CameraOverlayMode::Detections) {
    const auto & detections = observations.detections;
    if (!same_frame(
        detections, frame, snapshot,
        now) || detections.value->detections.size() > 100)
    {
      return result;
    }
    for (const auto & box : detections.value->detections) {
      if (!valid_box(box, detections.value->header)) {result.boxes.clear(); return result;}
      result.boxes.push_back(present(box, false));
    }
  } else {
    const auto & tracks = observations.tracks;
    if (!same_frame(tracks, frame, snapshot, now) || tracks.value->tracks.size() > 100) {
      return result;
    }
    for (const auto & box : tracks.value->tracks) {
      if (box.track_id < 0 || !valid_box(box, tracks.value->header) ||
        std::count_if(
          tracks.value->tracks.begin(), tracks.value->tracks.end(),
          [&](const auto & other) {return other.track_id == box.track_id;}) != 1)
      {
        result.boxes.clear(); return result;
      }
      auto value = present(box, true); depth(value, observations, frame, snapshot, now);
      result.boxes.push_back(std::move(value));
    }
  }
  result.matched = true;
  result.status = QStringLiteral("同帧%1 · %2个%3 · frame=%4 · stamp=%5").arg(
    mode == CameraOverlayMode::Detections ? QStringLiteral("检测") : QStringLiteral("上游跟踪"),
    QString::number(result.boxes.size()), result.boxes.empty() ? QStringLiteral("（有效空）") : "",
    text(frame.source_frame, 256)).arg(frame.stamp_ns);
  return result;
}
}  // namespace easyfly_desktop
