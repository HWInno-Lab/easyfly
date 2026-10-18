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

// EasyFly P2: NMS 后处理 (自实现, 不依赖 OpenCV DNN)
// 与 Python 基准 (/tmp 基准脚本) 同构: 按类别独立 NMS, IoU 阈值判定
#ifndef EASYFLY_YOLO_ONNX_NMS_HPP_
#define EASYFLY_YOLO_ONNX_NMS_HPP_

#include <algorithm>
#include <vector>

#include "easyfly_detection_api/types.hpp"

namespace easyfly::yolo_onnx
{
using easyfly::detection::Detection;


namespace detail
{
inline float iou_xyxy(const Detection & a, const Detection & b)
{
    const float xx1 = std::max(a.x1, b.x1);
    const float yy1 = std::max(a.y1, b.y1);
    const float xx2 = std::min(a.x2, b.x2);
    const float yy2 = std::min(a.y2, b.y2);
    const float w = std::max(0.0f, xx2 - xx1);
    const float h = std::max(0.0f, yy2 - yy1);
    const float inter = w * h;
    const float area_a = (a.x2 - a.x1) * (a.y2 - a.y1);
    const float area_b = (b.x2 - b.x1) * (b.y2 - b.y1);
    return inter / (area_a + area_b - inter + 1e-9f);
}
}  // namespace detail

/// 对 detections (已过置信度阈值, letterbox 坐标系) 做按类别 NMS, 原地保留
inline std::vector<Detection> nms_by_class(std::vector<Detection> & dets, float nms_thr)
{
    // 按置信度降序
    std::sort(dets.begin(), dets.end(),
              [](const Detection & a, const Detection & b) { return a.confidence > b.confidence; });

    std::vector<char> suppressed(dets.size(), 0);
    std::vector<Detection> out;
    for (size_t i = 0; i < dets.size(); ++i) {
        if (suppressed[i]) continue;
        out.push_back(dets[i]);
        for (size_t j = i + 1; j < dets.size(); ++j) {
            if (suppressed[j]) continue;
            if (dets[j].class_id != dets[i].class_id) continue;  // 按类别独立
            if (detail::iou_xyxy(dets[i], dets[j]) > nms_thr) suppressed[j] = 1;
        }
    }
    return out;
}

}  // namespace easyfly::yolo_onnx

#endif  // EASYFLY_YOLO_ONNX_NMS_HPP_
