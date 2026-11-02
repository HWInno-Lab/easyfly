// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <sys/resource.h>
#include <vector>
#if defined(BENCH_sort)
#include "easyfly_multi_object_tracking/adapters/tracker_factory.hpp"
#elif defined(BENCH_astar)
#include "easyfly_global_planner/adapters/planner_factory.hpp"
#elif defined(BENCH_csrt)
#include "easyfly_single_object_tracking/backend_factory.hpp"
#elif defined(BENCH_onnx)
#include <opencv2/imgproc.hpp>
#include "easyfly_object_detection/backend_factory.hpp"
#endif
using Clock = std::chrono::steady_clock;
void require(bool ok) {if (!ok) {throw std::runtime_error("real benchmark result invalid");}}
int main(int argc, char ** argv) {
  if (argc != 3) {return 2;}
  (void)argv;
  const auto setup_begin = Clock::now();
  int count = 0;
  std::string provider;
#if defined(BENCH_sort)
  const std::string backend = "sort", input = "one stationary detection; 640x480; 1ms source increments";
  const int iterations = 500;
  auto factory = easyfly_multi_object_tracking::make_tracker("sort", {{"min_hits", 1}});
  easyfly::tracking::Frame frame;
  frame.image_width = 640; frame.image_height = 480;
  frame.detections.push_back({{100, 100, 140, 180}, .9, 0, "person", {}});
  auto step = [&] {frame.source = {0, static_cast<std::int64_t>(++count)*1000000};
    const auto result = factory->update(frame);
    require(result.status == easyfly::tracking::Status::ok && result.tracks.size() == 1);};
#elif defined(BENCH_astar)
  const std::string backend = "astar", input = "128x128 grid; 0.1m; wall with one gap";
  const int iterations = 40;
  auto factory = easyfly_global_planner::make_planner("astar", {{"max_cells", 16384}, {"unstick_cells", 0}});
  std::vector<std::int8_t> cells(128*128, 0);
  for (int y = 0; y < 128; ++y) {if (y != 64) {cells[y*128+64] = 100;}}
  easyfly::grid_planning::Request request;
  request.grid = {128, 128, .1, 0, 0, cells.data(), cells.size()};
  request.start = {1.1, 1.2}; request.goal = {11.5, 11.6};
  auto step = [&] {++count; require(factory->plan(request, {}).status == easyfly::grid_planning::Status::ok);};
#elif defined(BENCH_csrt)
  const std::string backend = "csrt", input = "128x128 fixed RNG(42) BGR texture; 32x32 ROI";
  const int iterations = 20;
  cv::setNumThreads(2);
  auto factory = easyfly_single_object_tracking::make_tracker({"csrt", {}});
  cv::Mat image(128, 128, CV_8UC3); cv::RNG rng(42); rng.fill(image, cv::RNG::UNIFORM, 0, 255);
  require(factory->initialize(image, {30, 30, 32, 32}));
  auto step = [&] {++count; require(factory->update(image).ok);};
#elif defined(BENCH_onnx)
  const std::string backend = "onnx", input = "fixed m2 RGB frame converted to BGR; 640x480; YOLO11n";
  const int iterations = 12;
  cv::setNumThreads(2);
  std::ifstream file(argv[2], std::ios::binary);
  std::vector<unsigned char> data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  require(data.size() == 640*480*3);
  cv::Mat rgb(480, 640, CV_8UC3, data.data()), image;
  cv::cvtColor(rgb, image, cv::COLOR_RGB2BGR);
  auto factory = easyfly_object_detection::createBackend(easyfly::backends::BackendConfig{"onnx",
    {{"model_path", std::string(argv[1])}, {"use_gpu", false}}});
  factory->warmUp(); provider = factory->activeProvider();
  require(provider == "CPUExecutionProvider");
  std::vector<easyfly::detection::Detection> output;
  auto step = [&] {++count; factory->detect(image, output);};
#endif
  for (int i = 0; i < 3; ++i) {step();}
  const double setup_ms = std::chrono::duration<double, std::milli>(Clock::now()-setup_begin).count();
  const auto wall_begin = Clock::now();
  const auto cpu_begin = std::clock();
  std::vector<double> latency;
  for (int i = 0; i < iterations; ++i) {
    const auto begin = Clock::now(); step();
    latency.push_back(std::chrono::duration<double, std::milli>(Clock::now()-begin).count());
  }
  const double wall = std::chrono::duration<double>(Clock::now()-wall_begin).count();
  const double cpu = static_cast<double>(std::clock()-cpu_begin)/CLOCKS_PER_SEC;
  std::sort(latency.begin(), latency.end());
  struct rusage usage{}; require(getrusage(RUSAGE_SELF, &usage) == 0);
  std::cout << "{\"backend\":\"" << backend << "\",\"input\":\"" << input
    << "\",\"iterations\":" << iterations << ",\"warmup_iterations\":3,\"setup_including_warmup_ms\":" << setup_ms
    << ",\"total_calls_including_warmup\":" << count
    << ",\"wall_seconds\":" << wall << ",\"throughput_per_second\":" << iterations/wall
    << ",\"latency_ms\":{\"p50\":" << latency[latency.size()/2]
    << ",\"p95\":" << latency[static_cast<std::size_t>(std::ceil(latency.size()*.95))-1]
    << ",\"max\":" << latency.back() << "},\"cpu_seconds\":" << cpu
    << ",\"cpu_percent_one_core\":" << cpu/wall*100 << ",\"peak_rss_kib\":" << usage.ru_maxrss
    << ",\"actual_provider\":\"" << provider << "\",\"gpu_requested\":false,"
    << "\"queue_and_drops\":\"not_applicable: synchronous calls\"}\n";
  return 0;
}
