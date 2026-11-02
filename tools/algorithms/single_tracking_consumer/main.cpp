// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iostream>
#include <stdexcept>
#include <sys/resource.h>
#include <vector>
#include <opencv2/core/ocl.hpp>
#include "easyfly_single_object_tracking/backend_factory.hpp"

int main(int argc, char ** argv)
{
  if (argc != 2) {return 2;}
  const std::string backend = argv[1];
  cv::setNumThreads(2);
  cv::ocl::setUseOpenCL(false);  // This executable measures a declared CPU-only profile.
  auto tracker = easyfly_single_object_tracking::make_tracker({backend, {}});
  if (tracker->backend() != backend) {throw std::runtime_error("wrong installed creator");}
  cv::Mat target(40, 40, CV_8UC3);
  cv::RNG rng(42); rng.fill(target, cv::RNG::UNIFORM, 0, 255);
  auto image = [&target](int step) {
      cv::Mat result(160, 200, CV_8UC3, cv::Scalar(30, 30, 30));
      target.copyTo(result(cv::Rect(60 + step * 2, 50 + step, 40, 40)));
      return result;
    };
  using Clock = std::chrono::steady_clock;
  const auto init_begin = Clock::now();
  if (!tracker->initialize(image(0), {60, 50, 40, 40})) {
    throw std::runtime_error("installed initialization failed");
  }
  const double initialization = std::chrono::duration<double, std::milli>(Clock::now()-init_begin).count();
  std::vector<double> latency;
  double maximum_error = 0, total_ms = 0;
  const auto cpu_begin = std::clock();
  for (int frame = 1; frame <= 40; ++frame) {
    const int step = frame <= 20 ? frame : 40-frame;
    const auto pixels = image(step);
    const auto begin = Clock::now();
    const auto result = tracker->update(pixels);
    const double elapsed = std::chrono::duration<double, std::milli>(Clock::now()-begin).count();
    if (!result.ok || result.calibrated_confidence.has_value()) {
      throw std::runtime_error("installed tracking invalid or invented confidence");
    }
    const double error = std::max(std::abs(result.box.x - (60 + step * 2)),
                                 std::abs(result.box.y - (50 + step)));
    if (error > 6) {throw std::runtime_error("installed translation disagrees with known pixels");}
    maximum_error = std::max(maximum_error, error);
    latency.push_back(elapsed); total_ms += elapsed;
  }
  const double cpu_ms = 1000.0 * (std::clock()-cpu_begin) / CLOCKS_PER_SEC;
  tracker->reset();
  if (tracker->initialized() || tracker->update(image(0)).ok) {
    throw std::runtime_error("installed reset retained tracking");
  }
  std::sort(latency.begin(), latency.end());
  rusage usage{}; getrusage(RUSAGE_SELF, &usage);
  std::cout << "{\"backend\":\"" << backend << "\",\"opencv\":\"" << CV_VERSION
            << "\",\"frames\":40,\"image_width\":200,\"image_height\":160,\"threads\":2,\"opencl\":false,"
            << "\"initialization_ms\":" << initialization << ",\"update_p50_ms\":" << latency[19]
            << ",\"update_p95_ms\":" << latency[37] << ",\"update_mean_ms\":" << total_ms/40
            << ",\"update_only_fps\":" << 40000/total_ms << ",\"cpu_ms\":" << cpu_ms
            << ",\"peak_rss_kib\":" << usage.ru_maxrss << ",\"max_position_error_px\":" << maximum_error
            << ",\"reset_passed\":true}\n";
}
