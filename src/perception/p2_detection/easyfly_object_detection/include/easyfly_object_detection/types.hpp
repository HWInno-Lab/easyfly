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
#include <string>
#include "easyfly_detection_api/types.hpp"
namespace easyfly_object_detection
{
using easyfly::detection::Detection;
using easyfly::detection::DetectionTiming;
struct DetectorParams : easyfly::detection::DetectorParams
{
  // 节点
  std::string input_topic{"/camera/color/image_raw"};
  std::string output_topic{"/detections"};
  std::string input_qos_reliability{"reliable"};   // reliable | best_effort
  int input_qos_depth{5};                           // bounded KeepLast depth
  bool publish_debug_image{true};
  double performance_log_period_s{10.0};   // <=0 disables periodic metrics
};
}  // namespace easyfly_object_detection
