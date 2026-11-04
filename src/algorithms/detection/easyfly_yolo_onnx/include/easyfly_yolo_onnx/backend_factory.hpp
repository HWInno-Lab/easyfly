// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_yolo_onnx/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
//

// EasyFly P2: 检测后端运行时工厂
// 配置 params.backend 决定实例化哪个后端 (区别于 SpireCV 编译宏, 换后端不重编译)
#ifndef EASYFLY_YOLO_ONNX_BACKEND_FACTORY_HPP_
#define EASYFLY_YOLO_ONNX_BACKEND_FACTORY_HPP_

#include <memory>
#include <stdexcept>
#include <string>

#include "easyfly_detection_api/detector.hpp"
#include "easyfly_yolo_onnx/onnx_backend.hpp"

namespace easyfly::yolo_onnx
{
using easyfly::detection::Detection;
using easyfly::detection::DetectorParams;
using easyfly::detection::DetectorBase;


inline std::unique_ptr<DetectorBase> createBackend(const DetectorParams & params)
{
    if (params.backend == "onnx") {
        return std::make_unique<OnnxBackend>(params);
    }
    if (params.backend == "tensorrt") {
        // TensorRT 后端预留位 (对应 src/cuda/), 接入时在此分支实例化
        throw std::runtime_error(
            "backend 'tensorrt' 尚未实现 — 当前可用: onnx (留此接口供后续扩展)");
    }
    if (params.backend == "openvino") {
        // OpenVINO 后端预留位 (对应 src/intel/)
        throw std::runtime_error(
            "backend 'openvino' 尚未实现 — 当前可用: onnx (留此接口供后续扩展)");
    }
    throw std::runtime_error("未知 backend: " + params.backend + " (可用: onnx)");
}

}  // namespace easyfly::yolo_onnx

#endif  // EASYFLY_YOLO_ONNX_BACKEND_FACTORY_HPP_
