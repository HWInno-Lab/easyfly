// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_camera_calib/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
//

#pragma once
#include <string>
#include "easyfly_camera_calib/calibration.hpp"
#include <sensor_msgs/msg/camera_info.hpp>

namespace easyfly_camera_calib
{
inline std::string cameraMismatch(
  const sensor_msgs::msg::CameraInfo & info,
  const Calibration & reference, double tolerance)
{
  if (!reference.has_camera) {return "camera_calibration_missing";}
  const auto & i = reference.camera;
  if (info.header.frame_id != reference.camera_frame) {return "camera_frame_mismatch";}
  if (info.width != static_cast<uint32_t>(i.width) ||
    info.height != static_cast<uint32_t>(i.height)) {return "resolution_mismatch";}
  for (double v : info.k) {
    if (!std::isfinite(v)) {
      return "invalid_K";
    }
  }
  if (info.k[0] <= 0 || info.k[4] <= 0 || std::abs(info.k[1]) > 1e-9 ||
    std::abs(info.k[3]) > 1e-9 ||
    std::abs(info.k[6]) > 1e-9 || std::abs(info.k[7]) > 1e-9 || std::abs(info.k[8] - 1) > 1e-9)
  {
    return "invalid_K";
  }
  if (info.distortion_model != i.distortion_model || info.d.size() != i.d.size()) {
    return "distortion_model_mismatch";
  }
  for (size_t k = 0; k < info.d.size(); ++k) {
    if (!std::isfinite(info.d[k]) || std::abs(info.d[k] - i.d[k]) > 1e-6) {
      return "distortion_mismatch";
    }
  }
  if (std::abs(info.k[0] - i.fx) > tolerance || std::abs(info.k[4] - i.fy) > tolerance ||
    std::abs(info.k[2] - i.cx) > tolerance || std::abs(info.k[5] - i.cy) > tolerance)
  {
    return "intrinsics_mismatch";
  }
  return "valid";
}
}  // namespace easyfly_camera_calib
