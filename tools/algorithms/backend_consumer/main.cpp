// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#include <iostream>
#include <opencv2/core.hpp>
#include "easyfly_multi_object_tracking/adapters/tracker_factory.hpp"
#include "easyfly_global_planner/adapters/planner_factory.hpp"
#include "easyfly_object_detection/backend_factory.hpp"
#include "easyfly_single_object_tracking/backend_factory.hpp"
void require(bool ok) {if (!ok) throw std::runtime_error("installed backend consumer failed");}
int main(int argc, char ** argv) {
  if (argc != 2) return 2;
  cv::setNumThreads(2);
  auto tracker = easyfly_multi_object_tracking::make_tracker("sort", {{"min_hits", 1}});
  easyfly::tracking::Frame frame; frame.source = {0, 1}; frame.image_width = 100; frame.image_height = 100;
  frame.detections.push_back({{10, 10, 30, 30}, .9, 0, "person", {}});
  const auto tracked = tracker->update(frame);
  require(tracked.status == easyfly::tracking::Status::ok && tracked.tracks.size() == 1);
  tracker->reset(3); frame.source = {3, 1}; require(tracker->update(frame).status == easyfly::tracking::Status::ok);
  std::vector<std::int8_t> cells(100, 0);
  easyfly::grid_planning::Request request;
  request.grid = {10, 10, 1, 0, 0, cells.data(), cells.size()}; request.start = {1.2, 1.3}; request.goal = {8.1, 8.2};
  auto planner = easyfly_global_planner::make_planner("astar", {{"max_cells", 100}, {"unstick_cells", 0}});
  require(planner->plan(request, {}).status == easyfly::grid_planning::Status::ok);
  auto single = easyfly_single_object_tracking::make_tracker({"csrt", {}});
  cv::Mat image(100, 100, CV_8UC3); cv::RNG rng(42); rng.fill(image, cv::RNG::UNIFORM, 0, 255);
  require(single->initialize(image, {20, 20, 30, 30})); require(single->update(image).ok);
  single->reset(); require(!single->initialized());
  auto kcf = easyfly_single_object_tracking::make_tracker({"kcf", {{"detect_thresh", .2}}});
  require(kcf->backend() == "kcf" && kcf->initialize(image, {20, 20, 30, 30}));
  const auto kcf_result = kcf->update(image);
  require(kcf_result.ok && !kcf_result.calibrated_confidence.has_value());
  kcf->reset(); require(!kcf->initialized());
  auto detector = easyfly_object_detection::createBackend(easyfly::backends::BackendConfig{"onnx",
      {{"model_path", std::string(argv[1])}, {"use_gpu", false}}});
  detector->warmUp(); std::vector<easyfly::detection::Detection> output;
  detector->detect(image, output);
  require(detector->activeProvider() == "CPUExecutionProvider");
  std::cout << "SORT/A*/CSRT/KCF/ONNX installed factories passed; actual provider " << detector->activeProvider() << '\n';
}
