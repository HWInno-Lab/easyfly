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
#include <limits>
#include "easyfly_multi_object_tracking/adapters/tracker_factory.hpp"
#include "easyfly_multi_object_tracking/sort.hpp"  // compatibility consumer
#include "easyfly_multi_object_tracking/id_consolidator.hpp"

namespace tr = easyfly::tracking;
namespace core = easyfly::sort;
namespace platform = easyfly_multi_object_tracking;

namespace
{
tr::Frame frame(std::int64_t stamp = 1)
{
  tr::Frame value;
  value.source = {0, stamp};
  value.image_width = 640;
  value.image_height = 480;
  value.detections.push_back({{100, 100, 140, 180}, .9, 0, "person", {}});
  return value;
}
}  // namespace

TEST(SortAdapter, ExactNativeResultsAcrossMotionOcclusionAndClasses)
{
  core::SortParams params;
  auto adapter = platform::make_tracker("sort", params);
  core::Sorter direct(params);
  for (int step = 0; step < 100; ++step) {
    auto input = frame(step + 1);
    input.detections[0].box.x1 += 2 * step;
    input.detections[0].box.x2 += 2 * step;
    if (step >= 25 && step < 35) {input.detections.clear();}
    if (step >= 60) {input.detections.push_back({{10, 10, 30, 30}, .7, 2, "car", {}});}
    std::vector<core::DetectionInput> native_input;
    for (const auto & d : input.detections) {
      native_input.push_back(
        {{d.box.x1, d.box.y1, d.box.x2, d.box.y2},
          d.confidence, d.class_id, d.class_name});
    }
    const auto expected = direct.update(native_input);
    const auto actual = adapter->update(input);
    ASSERT_EQ(actual.status, tr::Status::ok);
    ASSERT_EQ(actual.velocity_unit, tr::VelocityUnit::pixels_per_frame);
    ASSERT_EQ(actual.tracks.size(), expected.size());
    for (std::size_t i = 0; i < expected.size(); ++i) {
      const auto & a = actual.tracks[i];
      const auto & b = expected[i];
      EXPECT_EQ(a.raw_id, b.track_id);
      EXPECT_DOUBLE_EQ(a.box.x1, b.box.x1);
      EXPECT_DOUBLE_EQ(a.box.y1, b.box.y1);
      EXPECT_DOUBLE_EQ(a.box.x2, b.box.x2);
      EXPECT_DOUBLE_EQ(a.box.y2, b.box.y2);
      EXPECT_DOUBLE_EQ(a.last_observed_box.x1, b.box_raw.x1);
      EXPECT_DOUBLE_EQ(a.last_observed_box.y1, b.box_raw.y1);
      EXPECT_DOUBLE_EQ(a.last_observed_box.x2, b.box_raw.x2);
      EXPECT_DOUBLE_EQ(a.last_observed_box.y2, b.box_raw.y2);
      EXPECT_DOUBLE_EQ(a.vx, b.vx);
      EXPECT_DOUBLE_EQ(a.vy, b.vy);
      EXPECT_DOUBLE_EQ(a.confidence, b.confidence);
      EXPECT_EQ(a.class_id, b.class_id);
      EXPECT_EQ(a.class_name, b.class_name);
      EXPECT_EQ(a.hits, static_cast<std::uint64_t>(b.hits));
      EXPECT_EQ(a.age, static_cast<std::uint64_t>(b.age));
      EXPECT_EQ(a.missed_updates, static_cast<std::uint64_t>(b.time_since_update));
      EXPECT_EQ(a.confirmed, b.confirmed);
    }
  }
}

TEST(SortAdapter, RejectedInputDoesNotConsumeStampOrAdvanceHistory)
{
  auto adapter = platform::make_tracker("sort", {});
  ASSERT_EQ(adapter->update(frame()).status, tr::Status::ok);
  EXPECT_EQ(adapter->update(frame()).status, tr::Status::invalid_input);
  auto bad = frame(2);
  bad.detections[0].confidence = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(adapter->update(bad).status, tr::Status::invalid_input);
  const auto valid = adapter->update(frame(2));
  ASSERT_EQ(valid.status, tr::Status::ok);
  ASSERT_EQ(valid.tracks.size(), 1u);
  EXPECT_EQ(valid.tracks[0].age, 2u);
}

TEST(SortAdapter, ResetAcceptsNewEpochWithoutReusingPublishedRawId)
{
  auto adapter = platform::make_tracker("sort", {});
  const auto before = adapter->update(frame(100));
  ASSERT_EQ(before.tracks.size(), 1u);
  adapter->reset(4);
  EXPECT_EQ(adapter->update(frame(101)).status, tr::Status::invalid_input);
  auto input = frame(1);
  input.source.epoch = 4;
  const auto after = adapter->update(input);
  ASSERT_EQ(after.status, tr::Status::ok);
  ASSERT_EQ(after.tracks.size(), 1u);
  EXPECT_GT(after.tracks[0].raw_id, before.tracks[0].raw_id);
  EXPECT_EQ(after.tracks[0].age, 1u);
}

TEST(SortAdapter, UnknownBackendAndInvalidParamsFailExplicitly)
{
  EXPECT_THROW(platform::make_tracker("bytetrack", {}), std::invalid_argument);
  core::SortParams params;
  params.max_tracks = 0;
  EXPECT_THROW(platform::make_tracker("sort", params), std::invalid_argument);
  params.max_tracks = 100;
  params.iou_threshold = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(platform::make_tracker("sort", params), std::invalid_argument);
  auto adapter = platform::make_tracker("sort", {});
  EXPECT_EQ(adapter->capabilities().backend, "sort");
  EXPECT_EQ(adapter->capabilities().velocity_unit, tr::VelocityUnit::pixels_per_frame);
}

TEST(SortAdapter, LegacyIncludesAreAliasesToIndependentCores)
{
  platform::Sorter old_name;
  const auto result = old_name.update({});
  EXPECT_TRUE(result.empty());
  platform::IdConsolidator identity;
  identity.update({7}, {}, 0);
  EXPECT_EQ(identity.stableId(7), 7);
}
