// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_multi_object_tracking/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
//

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
  auto adapter = platform::make_tracker("sort");
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
  easyfly::backends::Parameters params;
  params["max_tracks"] = 0;
  EXPECT_THROW(platform::make_tracker("sort", params), std::invalid_argument);
  params["max_tracks"] = 100;
  params["iou_threshold"] = std::numeric_limits<double>::quiet_NaN();
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

#include <easyfly_backend_registry/metadata.hpp>
TEST(SortAdapter, EveryRegisteredTrackingBackendHasARealCreator)
{
  for (const auto & entry : easyfly::backends::descriptors) {
    if (std::string(entry.capability) == "multi_tracking") {
      EXPECT_NE(platform::make_tracker(entry.id), nullptr);
    }
  }
}

namespace {
class FixtureTracker final : public tr::Tracker {
public:
  explicit FixtureTracker(double marker) : marker_(marker) {}
  tr::Capabilities capabilities() const override {return {"fixture", {}, tr::VelocityUnit::pixels_per_second};}
  void reset(std::uint64_t epoch) override {epoch_ = epoch;}
  tr::UpdateResult update(const tr::Frame & frame) override {
    tr::UpdateResult result; result.source = frame.source; result.velocity_unit = capabilities().velocity_unit;
    result.status = frame.source.epoch == epoch_ ? tr::Status::ok : tr::Status::invalid_input;
    tr::Track track; track.raw_id = 1LL << 40; track.vx = marker_; result.tracks.push_back(track); return result;
  }
private: double marker_; std::uint64_t epoch_{0};
};
}
TEST(TrackerFactory, SelectedCreatorReceivesOwnParametersAndKeepsNativeUnit) {
  platform::TrackerFactory factory;
  factory.add("fixture", [](const easyfly::backends::Parameters & p) {
    return std::make_unique<FixtureTracker>(easyfly::backends::parameter<double>(p, "marker"));
  });
  auto tracker = platform::make_tracker(easyfly::backends::BackendConfig{"fixture", {{"marker", 12.5}}}, factory);
  tracker->reset(7); auto input = frame(); input.source.epoch = 7;
  const auto result = tracker->update(input);
  EXPECT_EQ(result.status, tr::Status::ok); EXPECT_EQ(result.tracks[0].raw_id, 1LL << 40);
  EXPECT_DOUBLE_EQ(result.tracks[0].vx, 12.5);
  EXPECT_NO_THROW(platform::require_supported_inputs(tracker->capabilities(), false));
  EXPECT_THROW(platform::require_supported_inputs(tracker->capabilities(), true), std::invalid_argument);
  auto caps = tracker->capabilities(); caps.inputs.image = true;
  EXPECT_THROW(platform::require_supported_inputs(caps, false), std::invalid_argument);
  caps.inputs.image = false; caps.inputs.appearance_dimension = 128;
  EXPECT_THROW(platform::require_supported_inputs(caps, false), std::invalid_argument);
  caps.inputs.appearance_dimension = 0; caps.inputs.low_confidence_detections = true;
  EXPECT_THROW(platform::require_supported_inputs(caps, false), std::invalid_argument);
  EXPECT_THROW(platform::make_tracker("fixture"), std::invalid_argument);
  EXPECT_THROW(platform::make_tracker("sort", {{"max_tracks", std::int64_t{1LL << 40}}}), std::invalid_argument);
}

TEST(TrackerBoundary, RejectNarrowingUnitsAndMalformedOutputBeforePublication) {
  auto tracker = platform::make_tracker("sort", {{"min_hits", 1}});
  const auto input = frame(); auto result = tracker->update(input); auto caps = tracker->capabilities();
  ASSERT_TRUE(platform::valid_tracking_output(result, input, caps, true));
  result.tracks[0].raw_id = 1LL << 40;
  EXPECT_FALSE(platform::valid_tracking_output(result, input, caps, true));
  EXPECT_TRUE(platform::valid_tracking_output(result, input, caps, false));
  result.tracks.push_back(result.tracks[0]);
  EXPECT_FALSE(platform::valid_tracking_output(result, input, caps, false));
  result.tracks.pop_back(); result.velocity_unit = tr::VelocityUnit::pixels_per_second;
  EXPECT_FALSE(platform::valid_tracking_output(result, input, caps, false));
  result.velocity_unit = caps.velocity_unit; result.tracks[0].vx = NAN;
  EXPECT_FALSE(platform::valid_tracking_output(result, input, caps, false));
}
