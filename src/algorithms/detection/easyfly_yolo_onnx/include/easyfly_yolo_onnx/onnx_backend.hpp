// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_yolo_onnx/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
//

#pragma once
#include <memory>
#include "easyfly_detection_api/detector.hpp"
namespace easyfly::yolo_onnx
{
using easyfly::detection::Detection;
using easyfly::detection::DetectionTiming;
using easyfly::detection::DetectorParams;
using easyfly::detection::DetectorBase;
class OnnxBackend : public DetectorBase
{
public:
  explicit OnnxBackend(const DetectorParams & params);
  ~OnnxBackend() override;
  std::string activeProvider() const override;
protected:
  void detectImpl(const cv::Mat & image, std::vector<Detection> & detections,
    DetectionTiming * timing) override;
  void unletterboxImpl(Detection & detection) override;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace easyfly::yolo_onnx
