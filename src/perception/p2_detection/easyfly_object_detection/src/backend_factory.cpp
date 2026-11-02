// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#include "easyfly_object_detection/backend_factory.hpp"
#include "easyfly_backend_registry/metadata.hpp"
#include "easyfly_yolo_onnx/onnx_backend.hpp"
namespace easyfly_object_detection {
namespace b = easyfly::backends;
b::BackendConfig detector_config(const easyfly::detection::DetectorParams & p) {
  return {p.backend, {{"model_path", p.model_path}, {"class_names", p.class_names}, {"imgsz", p.imgsz},
    {"conf_threshold", p.conf_threshold}, {"nms_threshold", p.nms_threshold}, {"use_gpu", p.use_gpu},
    {"gpu_device_id", p.gpu_device_id}, {"max_detections", p.max_detections}}};
}
const DetectorFactory & detector_factory() {
  static const DetectorFactory factory = [] {
    DetectorFactory result;
    result.add("onnx", [](const b::Parameters & input) {
      b::require_backend("detection", "onnx");
      const auto values = b::resolve_parameters("detection", "onnx", input);
      easyfly::detection::DetectorParams p;
      p.backend = "onnx";
      p.model_path = b::parameter<std::string>(values, "model_path");
      p.class_names = b::parameter<std::vector<std::string>>(values, "class_names");
      p.imgsz = b::parameter<int>(values, "imgsz");
      p.conf_threshold = b::parameter<double>(values, "conf_threshold");
      p.nms_threshold = b::parameter<double>(values, "nms_threshold");
      p.use_gpu = b::parameter<bool>(values, "use_gpu");
      p.gpu_device_id = b::parameter<int>(values, "gpu_device_id");
      p.max_detections = b::parameter<int>(values, "max_detections");
      return std::make_unique<easyfly::yolo_onnx::OnnxBackend>(p);
    }, [](const easyfly::backends::Parameters & input) {return easyfly::backends::resolve_parameters("detection", "onnx", input);});
    return result;
  }();
  return factory;
}
}  // namespace easyfly_object_detection
