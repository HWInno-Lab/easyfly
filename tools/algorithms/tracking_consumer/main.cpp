// Copyright 2026 EasyFly Project
// SPDX-License-Identifier: Apache-2.0
#include <iostream>
#include <stdexcept>
#include "easyfly_multi_object_tracking/adapters/tracker_factory.hpp"
#include "easyfly_multi_object_tracking/sort.hpp"
#include "easyfly_multi_object_tracking/id_consolidator.hpp"

void check(bool value, const char * detail)
{
  if (!value) {throw std::runtime_error(detail);}
}

int main()
{
  try {
    easyfly::sort::Sorter core;
    check(core.update({}).empty(), "core empty input");
    auto backend = easyfly_multi_object_tracking::make_tracker("sort", {});
    easyfly::tracking::Frame frame;
    frame.source = {0, 1};
    frame.image_width = 640;
    frame.image_height = 480;
    frame.detections.push_back({{10, 10, 30, 50}, .9, 0, "person", {}});
    const auto output = backend->update(frame);
    check(output.status == easyfly::tracking::Status::ok && output.tracks.size() == 1,
      "installed SORT adapter");
    const auto first_id = output.tracks[0].raw_id;
    backend->reset(1);
    frame.source = {1, 1};
    const auto reset = backend->update(frame);
    check(reset.status == easyfly::tracking::Status::ok && reset.tracks[0].raw_id > first_id,
      "reset IDs must not alias delayed ROS measurements");
    easyfly_multi_object_tracking::IdConsolidator ids;
    ids.update({7}, {{7, {8, 2, 0, "person"}}}, 0);
    ids.update({}, {}, 1);
    ids.update({12}, {}, 2);
    ids.update({12}, {{12, {8.1, 2, 0, "person"}}}, 2.1);
    check(ids.stableId(12) == 7, "delayed world feedback");
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
