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

#ifndef EASYFLY_DESKTOP__CAMERA_OVERLAY_HPP_
#define EASYFLY_DESKTOP__CAMERA_OVERLAY_HPP_
#include <QRectF>
#include <QString>
#include <vector>
#include "easyfly_desktop/image_pipeline.hpp"
namespace easyfly_desktop
{
enum class CameraOverlayMode {Raw, Detections, Tracks};
struct CameraOverlayBox
{
  QRectF normalized;
  QString class_name, label, depth_status;
  int id{-1};
  double confidence{0};
  bool tracking{false};
  std::optional<double> axial_depth;
};
struct CameraOverlay
{
  bool matched{false};
  QString status;
  std::vector<CameraOverlayBox> boxes;
};
CameraOverlay camera_overlay(
  const bridge::UiSnapshot & snapshot,
  const ConvertedFrame & frame, int channel, CameraOverlayMode mode, double now,
  const bridge::CameraObservations * selected = nullptr);
QRect camera_image_rect(QSize canvas, QSize image);
QRect camera_label_rect(
  QRect bounds, QRect image, QSize label,
  const std::vector<QRect> & occupied);
}  // namespace easyfly_desktop
#endif  // EASYFLY_DESKTOP__CAMERA_OVERLAY_HPP_
