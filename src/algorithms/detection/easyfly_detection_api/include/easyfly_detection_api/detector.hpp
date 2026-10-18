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

// EasyFly P2: 检测器基类 (SpireCV Base/Impl 模式)
//
// Base 负责: 参数校验、通用调用流程、输出截断;
// 后端 (OnnxBackend 等) 只实现 detectImpl(), 由运行时工厂按配置选择
// (区别于 SpireCV 的编译宏切换, 换后端无需重新编译)。
#ifndef EASYFLY_DETECTION_API_DETECTOR_BASE_HPP_
#define EASYFLY_DETECTION_API_DETECTOR_BASE_HPP_

#include <string>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <opencv2/opencv.hpp>

#include "easyfly_detection_api/types.hpp"

namespace easyfly::detection
{

class DetectorBase
{
public:
    explicit DetectorBase(const DetectorParams & params) : params_(params) {
      if (params.imgsz < 1 || params.imgsz > 4096 || params.max_detections < 1 ||
          params.max_detections > 4096 || !std::isfinite(params.conf_threshold) ||
          params.conf_threshold < 0 || params.conf_threshold > 1 ||
          !std::isfinite(params.nms_threshold) || params.nms_threshold < 0 || params.nms_threshold > 1)
        throw std::invalid_argument("invalid detector computation parameters");
    }
    virtual ~DetectorBase() = default;

    /// 启动预热: 零图跑一遍推理, 触发 engine/CUDA context 初始化
    /// (借鉴 SpireCV warmUp, 避免第一帧卡顿数秒)
    void warmUp()
    {
        cv::Mat zero_img(params_.imgsz, params_.imgsz, CV_8UC3, cv::Scalar(0, 0, 0));
        std::vector<Detection> ignored;
        detect(zero_img, ignored);
    }

    /// 通用流程: 参数校验 → 后端推理(letterbox 坐标) → NMS → 反 letterbox → 截断
    void detect(const cv::Mat & bgr_img, std::vector<Detection> & detections,
                DetectionTiming * timing = nullptr)
    {
        // The caller owns the sample and may have filled node-level fields
        // (queue/color conversion) before invoking the backend.  Backends
        // only write their own stage fields; do not reset the whole sample.
        if (bgr_img.empty() || bgr_img.type() != CV_8UC3)
          throw std::invalid_argument("detector requires a non-empty BGR8 image");
        detectImpl(bgr_img, detections, timing);

        // 反 letterbox 到原图坐标
        for (auto & d : detections) {
            unletterboxImpl(d);
        }

        // 截断最大目标数
        if (detections.size() > static_cast<size_t>(params_.max_detections)) {
            detections.resize(params_.max_detections);
        }
    }

    const DetectorParams & params() const { return params_; }

    /// Name of the execution provider actually used by the backend.
    /// Backends that do not expose a provider may keep the default value.
    virtual std::string activeProvider() const { return "unknown"; }

protected:
    /// 后端实现: 输入原始 BGR 图, 输出 letterbox 坐标系下、已过 NMS 的检测
    virtual void detectImpl(const cv::Mat & bgr_img,
                            std::vector<Detection> & detections,
                            DetectionTiming * timing) = 0;

    virtual void unletterboxImpl(Detection & d) = 0;

    DetectorParams params_;
};

}  // namespace easyfly::detection

#endif  // EASYFLY_DETECTION_API_DETECTOR_BASE_HPP_
