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
#ifndef EASYFLY_COLOR_LINE__DETECTOR_HPP_
#define EASYFLY_COLOR_LINE__DETECTOR_HPP_

#include <vector>

#include <opencv2/core.hpp>

namespace easyfly::color_line
{
struct Parameters
{
  // OpenCV HSV: hue [0,179], saturation/value [0,255]. hue_min > hue_max wraps red.
  int hue_min{170}, hue_max{10};
  int saturation_min{100}, saturation_max{255};
  int value_min{60}, value_max{255};
  int morphology_kernel{3};  // Odd [1,15]; 1 disables opening/closing.
  double min_area_px{80.0};
  double min_length_px{25.0};
  double min_aspect_ratio{3.0};
  double min_fill_ratio{0.5};
  int max_detections{32};
};

struct Observation
{
  cv::Point2d center, start, end;
  double angle_rad{0.0};  // Undirected axis, image +x towards +y, [-pi/2, pi/2).
  double length_px{0.0}, width_px{0.0};
  double area_px{0.0};
  double offset_x_normalized{0.0};  // (center.x - (image.width-1)/2) / ((width-1)/2).
  double score{0.0};  // Contour / fitted rectangle area; not a probability.
};

void validate(const Parameters & parameters);
// BGR CV_8UC3 only, <= 2073600 pixels. Empty images yield no detections.
// Each sufficiently long/straight connected colored component is a separate line.
std::vector<Observation> detect(const cv::Mat & image, const Parameters & parameters = {});
}  // namespace easyfly::color_line

#endif  // EASYFLY_COLOR_LINE__DETECTOR_HPP_
