// Copyright 2026 EasyFly Project
//
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
#ifndef EASYFLY_ELLIPSE__DETECTOR_HPP_
#define EASYFLY_ELLIPSE__DETECTOR_HPP_

#include <vector>

#include <opencv2/core.hpp>

namespace easyfly::ellipse
{
struct Parameters
{
  double min_axis_px{12.0};
  double max_axis_ratio{8.0};
  double max_radial_error{0.06};
  double max_area_error{0.15};
  double canny_low{50.0};
  double canny_high{150.0};
  int max_detections{32};
};

struct Observation
{
  cv::Point2d center;
  double major_axis_px{0.0};  // Full diameters, major >= minor.
  double minor_axis_px{0.0};
  double angle_rad{0.0};  // Major axis: image +x towards image +y, [0, pi).
  double radial_error{0.0};  // RMS dimensionless elliptical-radius residual.
  double score{0.0};  // Geometric fit quality, not a calibrated probability.
};

void validate(const Parameters & parameters);
// Accepts CV_8UC1 or BGR CV_8UC3, <= 2073600 pixels. Empty images yield no detections.
// Invalid parameters/types/sizes throw invalid_argument; contour budget throws runtime_error.
std::vector<Observation> detect(const cv::Mat & image, const Parameters & parameters = {});
}  // namespace easyfly::ellipse

#endif  // EASYFLY_ELLIPSE__DETECTOR_HPP_
