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
