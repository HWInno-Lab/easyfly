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
#include "easyfly_color_line/detector.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace easyfly::color_line
{
void validate(const Parameters & p)
{
  if (p.hue_min < 0 || p.hue_min > 179 || p.hue_max < 0 || p.hue_max > 179 ||
    p.saturation_min < 0 || p.saturation_max > 255 ||
    p.saturation_min > p.saturation_max || p.value_min < 0 || p.value_max > 255 ||
    p.value_min > p.value_max || p.morphology_kernel < 1 || p.morphology_kernel > 15 ||
    p.morphology_kernel % 2 == 0 || !std::isfinite(p.min_area_px) || p.min_area_px < 1 ||
    !std::isfinite(p.min_length_px) || p.min_length_px < 2 ||
    !std::isfinite(p.min_aspect_ratio) || p.min_aspect_ratio < 1 ||
    !std::isfinite(p.min_fill_ratio) || p.min_fill_ratio <= 0 || p.min_fill_ratio > 1 ||
    p.max_detections < 1 || p.max_detections > 256)
  {
    throw std::invalid_argument("invalid color line detector parameters");
  }
}

std::vector<Observation> detect(const cv::Mat & image, const Parameters & p)
{
  validate(p);
  if (image.empty()) {return {};}
  if (image.type() != CV_8UC3 || image.total() > 2073600 || image.rows < 8 || image.cols < 8) {
    throw std::invalid_argument("color line requires bounded BGR8 image");
  }
  cv::Mat hsv, mask;
  cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
  if (p.hue_min <= p.hue_max) {
    cv::inRange(
      hsv, cv::Scalar(p.hue_min, p.saturation_min, p.value_min),
      cv::Scalar(p.hue_max, p.saturation_max, p.value_max), mask);
  } else {
    cv::Mat upper;
    cv::inRange(
      hsv, cv::Scalar(0, p.saturation_min, p.value_min),
      cv::Scalar(p.hue_max, p.saturation_max, p.value_max), mask);
    cv::inRange(
      hsv, cv::Scalar(p.hue_min, p.saturation_min, p.value_min),
      cv::Scalar(179, p.saturation_max, p.value_max), upper);
    cv::bitwise_or(mask, upper, mask);
  }
  if (p.morphology_kernel > 1) {
    const auto kernel = cv::getStructuringElement(
      cv::MORPH_RECT, cv::Size(p.morphology_kernel, p.morphology_kernel));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
  }
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  if (contours.size() > 4096) {throw std::runtime_error("color line contour budget exceeded");}
  std::vector<Observation> result;
  for (const auto & contour : contours) {
    const double area = cv::contourArea(contour);
    if (area < p.min_area_px) {continue;}
    const auto rectangle = cv::minAreaRect(contour);
    const double length = std::max(rectangle.size.width, rectangle.size.height);
    const double width = std::min(rectangle.size.width, rectangle.size.height);
    if (width <= 0 || length < p.min_length_px || length / width < p.min_aspect_ratio) {
      continue;
    }
    const double fill = area / (length * width);
    if (fill < p.min_fill_ratio) {continue;}
    double angle = (rectangle.angle +
      (rectangle.size.width < rectangle.size.height ? 90.0 : 0.0)) * CV_PI / 180.0;
    while (angle >= CV_PI / 2.0) {angle -= CV_PI;}
    while (angle < -CV_PI / 2.0) {angle += CV_PI;}
    const cv::Point2d direction(std::cos(angle), std::sin(angle));
    Observation o;
    o.center = rectangle.center;
    o.start = o.center - direction * (length / 2.0);
    o.end = o.center + direction * (length / 2.0);
    // Rotated-box axis endpoints can project a fraction beyond the raster.
    for (auto * point : {&o.start, &o.end}) {
      point->x = std::clamp(point->x, 0.0, static_cast<double>(image.cols - 1));
      point->y = std::clamp(point->y, 0.0, static_cast<double>(image.rows - 1));
    }
    o.angle_rad = angle;
    o.length_px = length;
    o.width_px = width;
    o.area_px = area;
    o.score = std::clamp(fill, 0.0, 1.0);
    o.offset_x_normalized = (o.center.x - (image.cols - 1) / 2.0) / ((image.cols - 1) / 2.0);
    result.push_back(o);
  }
  std::sort(
    result.begin(), result.end(), [](const auto & a, const auto & b) {
      if (a.area_px != b.area_px) {return a.area_px > b.area_px;}
      if (a.center.x != b.center.x) {return a.center.x < b.center.x;}
      return a.center.y < b.center.y;
    });
  if (result.size() > static_cast<size_t>(p.max_detections)) {result.resize(p.max_detections);}
  return result;
}
}  // namespace easyfly::color_line
