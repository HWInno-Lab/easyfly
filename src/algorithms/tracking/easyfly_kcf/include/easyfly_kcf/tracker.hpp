// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <opencv2/tracking.hpp>
#include "easyfly_single_tracking_api/tracker.hpp"

namespace easyfly::kcf
{
struct Parameters
{
  double detect_thresh{0.5};  // Native correlation response threshold, not a probability.
};

class KcfTracker final : public single_tracking::Tracker
{
public:
  explicit KcfTracker(Parameters parameters = {}) : parameters_(parameters)
  {
    if (!std::isfinite(parameters_.detect_thresh) || parameters_.detect_thresh < 0 ||
      parameters_.detect_thresh > 1)
    {
      throw std::invalid_argument("KCF detect_thresh must be finite and in [0, 1]");
    }
  }

  std::string backend() const override {return "kcf";}

  bool initialize(const cv::Mat & bgr, const cv::Rect & box) override
  {
    reset();
    if (bgr.empty() || bgr.type() != CV_8UC3 || !inside(box, bgr.size()) ||
      box.width < 8 || box.height < 8 ||
      // OpenCV pads the ROI to twice each dimension and multiplies integer sizes.
      std::int64_t{box.width} * box.height > std::numeric_limits<int>::max() / 4)
    {
      return false;
    }
    try {
      cv::TrackerKCF::Params native;
      native.detect_thresh = static_cast<float>(parameters_.detect_thresh);
      auto tracker = cv::TrackerKCF::create(native);
      tracker->init(bgr, box);
      // OpenCV KCF trains on its first update, not in init(). Train on the exact
      // initialization image so the first subsequent frame can already move.
      cv::Rect trained = box;
      if (!tracker->update(bgr, trained) || !inside(trained, bgr.size())) {return false;}
      tracker_ = std::move(tracker);
      box_ = trained;
      size_ = bgr.size();
      return true;
    } catch (const cv::Exception &) {
      return false;
    }
  }

  single_tracking::Result update(const cv::Mat & bgr) override
  {
    if (!tracker_ || bgr.empty() || bgr.type() != CV_8UC3 || bgr.size() != size_) {
      reset();
      return {};
    }
    try {
      cv::Rect next = box_;
      if (tracker_->update(bgr, next) && inside(next, size_)) {
        box_ = next;
        return {true, box_, std::nullopt};
      }
    } catch (const cv::Exception &) {
      // An invalid native result must not expose an old box as current tracking.
    }
    reset();
    return {};
  }

  void reset() override {tracker_.release(); box_ = {}; size_ = {};}
  bool initialized() const override {return !tracker_.empty();}

private:
  static bool inside(const cv::Rect & box, cv::Size size)
  {
    return box.x >= 0 && box.y >= 0 && box.width > 0 && box.height > 0 &&
           box.width <= size.width && box.height <= size.height &&
           box.x <= size.width - box.width && box.y <= size.height - box.height;
  }

  Parameters parameters_;
  cv::Ptr<cv::TrackerKCF> tracker_;
  cv::Rect box_;
  cv::Size size_;
};
}  // namespace easyfly::kcf
