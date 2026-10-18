// Copyright 2026 EasyFly Project
// SPDX-License-Identifier: Apache-2.0
// Compile separately against preserved baseline and migrated headers.
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <string>
#ifdef EASYFLY_LEGACY_TRACKING
#include "easyfly_multi_object_tracking/sort.hpp"
#include "easyfly_multi_object_tracking/id_consolidator.hpp"
namespace core = easyfly_multi_object_tracking;
namespace fusion = easyfly_multi_object_tracking;
#else
#include "easyfly_sort/sort.hpp"
#include "easyfly_track_id_fusion/id_consolidator.hpp"
namespace core = easyfly::sort;
namespace fusion = easyfly::track_id_fusion;
#endif

int main(int argc, char **)
{
  fusion::IdConsolidator identity;
  identity.update({7}, {{7, {8, 2, 0, "person"}}}, 0);
  identity.update({}, {}, 1);
  if (argc > 1) {
    identity.update({12}, {}, 2);
    identity.update({12}, {{12, {8.1, 2, 0, "person"}}}, 2.1);
    std::cout << identity.stableId(12) << '\n';
    return 0;
  }
  identity.update({12}, {{12, {8.1, 2, 0, "person"}}}, 2);
  std::cout << "same_frame_identity " << identity.stableId(12) << '\n';
  core::SortParams params;
  params.iou_threshold = .15;
  params.max_lost_frames = 45;
  params.min_hits = 3;
  params.max_tracks = 5;
  core::Sorter sorter(params);
  std::cout << std::hexfloat;
  for (int frame = 0; frame < 200; ++frame) {
    std::vector<core::DetectionInput> input;
    if (!(frame >= 30 && frame < 40) && !(frame >= 75 && frame < 130)) {
      input.push_back({{10. + frame, 10, 30. + frame, 50}, .9, 0, "person"});
    }
    input.push_back({{500. - frame, 10, 520. - frame, 50}, .8, 0, "person"});
    if (frame >= 20 && frame < 180) {
      input.push_back({{100, 200, 140, 240}, .7, 2, "car"});
    }
    const auto tracks = sorter.update(input);
    std::cout << "frame " << frame << ' ' << tracks.size() << '\n';
    for (const auto & track : tracks) {
      std::cout << track.track_id << ' ' << track.box.x1 << ' ' << track.box.y1 << ' '
                << track.box.x2 << ' ' << track.box.y2 << ' ' << track.box_raw.x1 << ' '
                << track.box_raw.y1 << ' ' << track.box_raw.x2 << ' ' << track.box_raw.y2 << ' '
                << track.confidence << ' ' << track.class_id << ' ' << track.class_name << ' '
                << track.vx << ' ' << track.vy << ' ' << track.hits << ' ' << track.age << ' '
                << track.time_since_update << ' ' << track.confirmed << '\n';
    }
  }
}
