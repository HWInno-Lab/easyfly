// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_detection_api/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
//

// EasyFly P2: 目标检测公共数据结构
#ifndef EASYFLY_DETECTION_API_TYPES_HPP_
#define EASYFLY_DETECTION_API_TYPES_HPP_

#include <string>
#include <vector>

namespace easyfly::detection
{

/// 检测结果 (原图像素坐标, xyxy)
struct Detection {
    float x1{0}, y1{0}, x2{0}, y2{0};   // 像素坐标
    float confidence{0};                 // 置信度
    int class_id{-1};                    // 类别 id
    std::string class_name;              // 类别名
};

/// One detection pipeline timing sample, measured with steady_clock.
///
/// These values intentionally do not use a ROS timestamp: the values are
/// wall-clock durations and remain meaningful when /clock is paused or when
/// the simulator runs below real time.
struct DetectionTiming {
    double queue_ms{0.0};                 // receive callback -> worker start
    double color_conversion_ms{0.0};      // input rgb8 -> BGR
    double preprocess_ms{0.0};            // letterbox + tensor preparation
    double inference_ms{0.0};             // ONNX Runtime session.Run()
    double postprocess_ms{0.0};           // output parse + confidence/NMS
    double debug_image_ms{0.0};           // optional annotation + conversion
    double end_to_end_ms{0.0};            // receive callback -> final publish
};

/// 检测器参数 (全部来自 config/detection.yaml)
struct DetectorParams {
    // 模型
    std::string backend{"onnx"};             // onnx | tensorrt(预留)
    std::string model_path;                  // ONNX 文件绝对路径
    std::vector<std::string> class_names;    // 类别表 (顺序必须与训练 data.yaml 一致!)
    int imgsz{640};                          // 模型输入边长
    // 阈值
    double conf_threshold{0.35};
    double nms_threshold{0.45};
    // 运行
    bool use_gpu{true};                      // CUDA EP 失败时自动回退 CPU
    int gpu_device_id{0};
    int max_detections{100};

};

}  // namespace easyfly::detection

#endif  // EASYFLY_DETECTION_API_TYPES_HPP_
