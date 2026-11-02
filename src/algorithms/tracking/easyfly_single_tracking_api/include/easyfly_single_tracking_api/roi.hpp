// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cmath>
#include <opencv2/core.hpp>
namespace easyfly::single_tracking
{
inline bool roi(double x, double y, double w, double h)
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(w) && std::isfinite(h) && x >= 0 &&
         y >= 0 && w > 0 && h > 0 && x + w <= 1 && y + h <= 1;
}
inline cv::Rect pixels(double x, double y, double w, double h, int width, int height)
{
  return {static_cast<int>(std::floor(x * width)), static_cast<int>(std::floor(y * height)),
    static_cast<int>(std::floor(w * width)),
    static_cast<int>(std::floor(h * height))};
}
}  // namespace easyfly::single_tracking

