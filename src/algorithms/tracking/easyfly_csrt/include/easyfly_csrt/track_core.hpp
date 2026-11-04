// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_csrt/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
//

#pragma once
#include <cmath>
#include <opencv2/imgproc.hpp>
#include <opencv2/tracking.hpp>
#include "easyfly_single_tracking_api/roi.hpp"
namespace easyfly::csrt {
using easyfly::single_tracking::roi;
using easyfly::single_tracking::pixels;
}

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
