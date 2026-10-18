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

#pragma once
#include <cmath>
#include <opencv2/imgproc.hpp>
#include <opencv2/tracking.hpp>
namespace easyfly::csrt
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
}  // namespace easyfly::csrt

#include <memory>
#include <stdexcept>
#include "easyfly_single_tracking_api/tracker.hpp"
namespace easyfly::csrt
{
class CsrtTracker final : public easyfly::single_tracking::Tracker
{
public:
  std::string backend() const override {return "csrt";}
  bool initialize(const cv::Mat & bgr, const cv::Rect & box) override
  {
    reset();
    if (bgr.empty() || bgr.type() != CV_8UC3 || box.x < 0 || box.y < 0 ||
      box.width < 8 || box.height < 8 || box.width > bgr.cols || box.height > bgr.rows ||
      box.x > bgr.cols - box.width || box.y > bgr.rows - box.height) return false;
    auto tracker = cv::TrackerCSRT::create();
    tracker->init(bgr, box);
    tracker_ = std::move(tracker); box_ = box;
    return true;
  }
  easyfly::single_tracking::Result update(const cv::Mat & bgr) override
  {
    if (!tracker_ || bgr.empty() || bgr.type() != CV_8UC3) return {};
    const bool ok = tracker_->update(bgr, box_);
    return {ok, box_, std::nullopt};
  }
  void reset() override {tracker_.release(); box_ = {};}
  bool initialized() const override {return !tracker_.empty();}
private:
  cv::Ptr<cv::TrackerCSRT> tracker_;
  cv::Rect box_;
};
inline std::unique_ptr<easyfly::single_tracking::Tracker> make_tracker(const std::string & backend)
{
  if (backend == "csrt") return std::make_unique<CsrtTracker>();
  throw std::invalid_argument("unknown single tracking backend: " + backend);
}
}
