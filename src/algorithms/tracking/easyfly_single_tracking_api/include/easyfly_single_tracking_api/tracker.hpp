// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_single_tracking_api/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
//

#pragma once
#include <optional>
#include <string>
#include <opencv2/core.hpp>
namespace easyfly::single_tracking
{
struct Result {bool ok{false}; cv::Rect box; std::optional<double> calibrated_confidence;};
class Tracker
{
public:
  virtual ~Tracker() = default;
  virtual std::string backend() const = 0;
  // Serial synchronous use; image is borrowed only for this call. Pixel ROI.
  virtual bool initialize(const cv::Mat & bgr, const cv::Rect & roi) = 0;
  virtual Result update(const cv::Mat & bgr) = 0;
  virtual void reset() = 0;
  virtual bool initialized() const = 0;
};
}
