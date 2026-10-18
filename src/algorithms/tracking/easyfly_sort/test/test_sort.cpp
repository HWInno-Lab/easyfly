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

#include <gtest/gtest.h>

#include <set>
#include <string>
#include <vector>

#include "easyfly_sort/sort.hpp"

namespace
{

using easyfly::sort::BBox;
using easyfly::sort::DetectionInput;
using easyfly::sort::SortParams;
using easyfly::sort::Sorter;

DetectionInput make_detection(
  double cx, double cy, double width, double height, int class_id,
  const char * class_name)
{
  return DetectionInput{
    BBox{
      cx - width / 2.0, cy - height / 2.0,
      cx + width / 2.0, cy + height / 2.0},
    0.9, class_id, class_name};
}

SortParams test_params()
{
  SortParams params;
  params.iou_threshold = 0.2;
  params.max_lost_frames = 30;
  params.min_hits = 3;
  return params;
}

struct CrossingScenarioResult
{
  int target1_id{-1};
  int target2_id{-1};
  int distractor_id{-1};
  int target1_id_after_occlusion{-1};
  bool saw_two_confirmed_tracks_during_crossing{false};
};

CrossingScenarioResult run_crossing_scenario()
{
  Sorter sorter(test_params());
  CrossingScenarioResult result;

  for (int frame = 0; frame < 100; ++frame) {
    std::vector<DetectionInput> detections;

    // Target 1 moves left to right and is occluded for frames 50 through 60.
    if (frame < 50 || frame > 60) {
      detections.push_back(
        make_detection(100.0 + 4.0 * frame, 300.0, 40.0, 80.0, 0, "person"));
    }

    // Target 2 moves right to left and crosses target 1.
    detections.push_back(
      make_detection(500.0 - 4.0 * frame, 300.0, 40.0, 80.0, 0, "person"));

    // A different-class distractor appears late in the sequence.
    if (frame >= 80) {
      detections.push_back(
        make_detection(300.0, 100.0, 60.0, 40.0, 2, "car"));
    }

    const auto tracks = sorter.update(detections);
    for (const auto & track : tracks) {
      if (track.class_id == 2) {
        result.distractor_id = track.track_id;
        continue;
      }
      if (track.class_id != 0) {
        continue;
      }

      const double center_x = (track.box_raw.x1 + track.box_raw.x2) / 2.0;
      if (frame < 5 && center_x < 150.0) {
        result.target1_id = track.track_id;
      }
      if (frame < 5 && center_x > 450.0) {
        result.target2_id = track.track_id;
      }
    }

    if (frame >= 40 && frame <= 45) {
      int confirmed_people = 0;
      for (const auto & track : tracks) {
        if (track.class_id == 0 && track.confirmed) {
          ++confirmed_people;
        }
      }
      result.saw_two_confirmed_tracks_during_crossing |= confirmed_people == 2;
    }
  }

  // Continue after the occlusion and retain target 1's identity.
  for (int frame = 0; frame < 5; ++frame) {
    const std::vector<DetectionInput> detections{
      make_detection(500.0 + 4.0 * frame, 300.0, 40.0, 80.0, 0, "person"),
      make_detection(100.0 - 4.0 * frame, 300.0, 40.0, 80.0, 0, "person"),
      make_detection(300.0, 100.0, 60.0, 40.0, 2, "car")};

    for (const auto & track : sorter.update(detections)) {
      if (track.class_id == 0 && track.box_raw.x1 > 400.0) {
        result.target1_id_after_occlusion = track.track_id;
      }
    }
  }

  return result;
}

TEST(Sort, AssignsDistinctIdsToTwoTargets)
{
  const auto result = run_crossing_scenario();
  ASSERT_NE(result.target1_id, -1);
  ASSERT_NE(result.target2_id, -1);
  EXPECT_NE(result.target1_id, result.target2_id);
}

TEST(Sort, MaintainsTwoConfirmedTracksDuringCrossing)
{
  const auto result = run_crossing_scenario();
  EXPECT_TRUE(result.saw_two_confirmed_tracks_during_crossing);
}

TEST(Sort, PreservesIdAcrossTenFrameOcclusion)
{
  const auto result = run_crossing_scenario();
  ASSERT_NE(result.target1_id, -1);
  ASSERT_NE(result.target1_id_after_occlusion, -1);
  EXPECT_EQ(result.target1_id_after_occlusion, result.target1_id);
}

TEST(Sort, GivesDistractorAnIndependentId)
{
  const auto result = run_crossing_scenario();
  ASSERT_NE(result.distractor_id, -1);
  EXPECT_NE(result.distractor_id, result.target1_id);
  EXPECT_NE(result.distractor_id, result.target2_id);
}

TEST(Sort, KeepsDifferentClassesInSeparateTracks)
{
  Sorter sorter(test_params());
  std::set<int> person_ids;
  std::set<int> car_ids;

  for (int frame = 0; frame < 10; ++frame) {
    // Deliberately overlap both detections. Class-aware association must still
    // keep them as independent tracks.
    const std::vector<DetectionInput> detections{
      make_detection(200.0 + frame, 200.0, 40.0, 80.0, 0, "person"),
      make_detection(200.0 + frame, 200.0, 40.0, 80.0, 2, "car")};

    for (const auto & track : sorter.update(detections)) {
      if (!track.confirmed) {
        continue;
      }
      if (track.class_id == 0) {
        person_ids.insert(track.track_id);
      } else if (track.class_id == 2) {
        car_ids.insert(track.track_id);
      }
    }
  }

  ASSERT_EQ(person_ids.size(), 1U);
  ASSERT_EQ(car_ids.size(), 1U);
  EXPECT_NE(*person_ids.begin(), *car_ids.begin());
}

TEST(Sort, EstimatesHorizontalVelocity)
{
  Sorter sorter(test_params());
  bool found_confirmed_target = false;
  double estimated_velocity = 0.0;

  for (int frame = 0; frame < 30; ++frame) {
    const std::vector<DetectionInput> detections{
      make_detection(100.0 + 4.0 * frame, 300.0, 40.0, 80.0, 0, "person")};

    const auto tracks = sorter.update(detections);
    if (frame != 29) {
      continue;
    }
    for (const auto & track : tracks) {
      if (track.class_id == 0 && track.confirmed) {
        found_confirmed_target = true;
        estimated_velocity = track.vx;
        break;
      }
    }
  }

  ASSERT_TRUE(found_confirmed_target);
  EXPECT_GT(estimated_velocity, 2.0);
  EXPECT_LT(estimated_velocity, 6.0);
  EXPECT_NEAR(estimated_velocity, 4.0, 2.0);
}

}  // namespace
