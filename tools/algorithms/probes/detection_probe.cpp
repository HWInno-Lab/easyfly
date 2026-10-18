// Copyright 2026 EasyFly Project
// SPDX-License-Identifier: Apache-2.0
#include <fstream>
#include <iomanip>
#include <iostream>
#include <yaml-cpp/yaml.h>
#ifdef EASYFLY_LEGACY_DETECTION
#include "easyfly_object_detection/backend_factory.hpp"
namespace detection = easyfly_object_detection;
#else
#include "easyfly_detection_api/types.hpp"
#include "easyfly_yolo_onnx/backend_factory.hpp"
namespace detection = easyfly::detection;
#endif
int main(int argc, char ** argv)
{
  if (argc != 5) return 2;
  cv::setNumThreads(2);
  detection::DetectorParams params;
  params.model_path = argv[1];
  const auto config = YAML::LoadFile(argv[3])["detection"];
  params.class_names = config["class_names"].as<std::vector<std::string>>();
  params.use_gpu = std::string(argv[4]) == "gpu";
#ifdef EASYFLY_LEGACY_DETECTION
  auto backend = detection::createBackend(params);
#else
  auto backend = easyfly::yolo_onnx::createBackend(params);
#endif
  cv::Mat rgb(480, 640, CV_8UC3);
  std::ifstream input(argv[2], std::ios::binary);
  input.read(reinterpret_cast<char *>(rgb.data), 480 * 640 * 3);
  if (input.gcount() != 480 * 640 * 3) return 3;
  cv::Mat bgr; cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
  backend->warmUp();
  std::vector<detection::Detection> output;
  detection::DetectionTiming timing;
  timing.queue_ms = 19; timing.color_conversion_ms = 23;
  timing.debug_image_ms = 29; timing.end_to_end_ms = 31;
  backend->detect(bgr, output, &timing);
  if (timing.queue_ms != 19 || timing.color_conversion_ms != 23 ||
      timing.debug_image_ms != 29 || timing.end_to_end_ms != 31) return 4;
  std::cout << std::setprecision(9) << "RESULT {\"provider\":" << std::quoted(backend->activeProvider())
            << ",\"detections\":[";
  for (std::size_t i = 0; i < output.size(); ++i) {
    const auto & d = output[i];
    if (i) std::cout << ',';
    std::cout << "{\"class_id\":" << d.class_id << ",\"class_name\":" << std::quoted(d.class_name)
              << ",\"confidence\":" << d.confidence << ",\"box\":["
              << d.x1 << ',' << d.y1 << ',' << d.x2 << ',' << d.y2 << "]}";
  }
  std::cout << "],\"caller_timing_preserved\":true}\n";
}
