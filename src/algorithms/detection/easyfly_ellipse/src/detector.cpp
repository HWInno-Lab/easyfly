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
#include "easyfly_ellipse/detector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace easyfly::ellipse
{
void validate(const Parameters & p)
{
  if (!std::isfinite(p.min_axis_px) || p.min_axis_px < 4.0 ||
    !std::isfinite(p.max_axis_ratio) || p.max_axis_ratio < 1.0 || p.max_axis_ratio > 20.0 ||
    !std::isfinite(p.max_radial_error) || p.max_radial_error <= 0 ||
    p.max_radial_error > 0.25 || !std::isfinite(p.max_area_error) ||
    p.max_area_error <= 0 || p.max_area_error > 0.5 ||
    !std::isfinite(p.canny_low) || !std::isfinite(p.canny_high) ||
    p.canny_low <= 0 || p.canny_high <= p.canny_low || p.canny_high > 4096 ||
    p.max_detections < 1 || p.max_detections > 256)
  {
    throw std::invalid_argument("invalid ellipse detector parameters");
  }
}

std::vector<Observation> detect(const cv::Mat & image, const Parameters & p)
{
  validate(p);
  if (image.empty()) {
    return {};
  }
  if ((image.type() != CV_8UC1 && image.type() != CV_8UC3) ||
    image.total() > 2073600 || image.rows < 8 || image.cols < 8)
  {
    throw std::invalid_argument("ellipse requires bounded 8-bit gray/BGR image");
  }
  cv::Mat gray, edges;
  if (image.channels() == 3) {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  } else {
    gray = image;
  }
  cv::GaussianBlur(gray, edges, cv::Size(5, 5), 1.0);
  cv::Canny(edges, edges, p.canny_low, p.canny_high);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(edges, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
  if (contours.size() > 4096) {
    throw std::runtime_error("ellipse contour budget exceeded");
  }
  std::vector<Observation> candidates;
  for (const auto & contour : contours) {
    if (contour.size() < 20) {
      continue;
    }
    const auto bounds = cv::boundingRect(contour);
    // Full closed targets only; border-clipped arcs are intentionally rejected.
    if (bounds.x <= 1 || bounds.y <= 1 || bounds.br().x >= image.cols - 1 ||
      bounds.br().y >= image.rows - 1)
    {
      continue;
    }
    const auto fit = cv::fitEllipse(contour);
    Observation o;
    o.center = fit.center;
    o.major_axis_px = std::max(fit.size.width, fit.size.height);
    o.minor_axis_px = std::min(fit.size.width, fit.size.height);
    o.angle_rad = std::fmod(
      (fit.angle + (fit.size.width < fit.size.height ? 90.0 : 0.0)) * CV_PI / 180.0,
      CV_PI);
    if (!std::isfinite(o.major_axis_px) || !std::isfinite(o.minor_axis_px) ||
      !std::isfinite(o.center.x) || !std::isfinite(o.center.y) ||
      !std::isfinite(o.angle_rad) || o.minor_axis_px < p.min_axis_px ||
      o.major_axis_px > 2.0 * std::max(image.cols, image.rows) ||
      o.major_axis_px / o.minor_axis_px > p.max_axis_ratio)
    {
      continue;
    }
    const double a = o.major_axis_px / 2.0, b = o.minor_axis_px / 2.0;
    const double area_error = std::abs(cv::contourArea(contour) / (CV_PI * a * b) - 1.0);
    const double c = std::cos(o.angle_rad), s = std::sin(o.angle_rad);
    double squared_error = 0.0;
    std::array<bool, 16> sectors{};
    for (const auto & point : contour) {
      const double dx = point.x - o.center.x, dy = point.y - o.center.y;
      const double x = (c * dx + s * dy) / a, y = (-s * dx + c * dy) / b;
      const double error = std::hypot(x, y) - 1.0;
      squared_error += error * error;
      const double angle = std::atan2(y, x) + CV_PI;
      sectors[std::min(15, static_cast<int>(angle * 16.0 / (2.0 * CV_PI)))] = true;
    }
    o.radial_error = std::sqrt(squared_error / contour.size());
    if (o.radial_error > p.max_radial_error || area_error > p.max_area_error ||
      std::count(sectors.begin(), sectors.end(), true) < 15)
    {
      continue;
    }
    o.score = std::clamp(1.0 - o.radial_error / p.max_radial_error, 0.0, 1.0);
    candidates.push_back(o);
  }
  std::sort(
    candidates.begin(), candidates.end(), [](const auto & a, const auto & b) {
      if (a.score != b.score) {return a.score > b.score;}
      if (a.center.x != b.center.x) {return a.center.x < b.center.x;}
      return a.center.y < b.center.y;
    });
  std::vector<Observation> result;
  for (const auto & candidate : candidates) {
    // Canny often returns both sides of a one-pixel edge. Keep one geometric target.
    const bool duplicate = std::any_of(
      result.begin(), result.end(), [&candidate](const auto & old) {
        return cv::norm(candidate.center - old.center) < 3.0 &&
        std::abs(candidate.major_axis_px - old.major_axis_px) < 4.0 &&
        std::abs(candidate.minor_axis_px - old.minor_axis_px) < 4.0;
      });
    if (!duplicate) {
      result.push_back(candidate);
      if (result.size() == static_cast<size_t>(p.max_detections)) {break;}
    }
  }
  return result;
}
}  // namespace easyfly::ellipse
