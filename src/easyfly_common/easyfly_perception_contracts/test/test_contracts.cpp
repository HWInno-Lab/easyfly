// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_perception_contracts/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
#include <gtest/gtest.h>
#include "easyfly_perception_contracts/contracts.hpp"
namespace p = easyfly::perception;
static std_msgs::msg::Header header(int seconds)
{std_msgs::msg::Header h; h.stamp.sec = seconds; h.frame_id = "camera_color_optical_frame"; return h;}
TEST(Admission, EpochRestartAndRetiredWriterCannotRebind)
{
  p::Producer first, second; p::SourceGate gate;
  auto a = first.next(header(10), "front", 64, 48);
  EXPECT_EQ(gate.observe(a, "writer-a"), p::SourceGate::accepted);
  EXPECT_EQ(gate.observe(a, "writer-a"), p::SourceGate::rejected);
  first.reset(); auto b = first.next(header(5), "front", 64, 48);
  EXPECT_EQ(gate.observe(b, "writer-a"), p::SourceGate::changed);
  EXPECT_EQ(gate.observe(a, "writer-a"), p::SourceGate::rejected);
  auto c = second.next(header(11), "front", 64, 48);
  EXPECT_EQ(gate.observe(c, "writer-b"), p::SourceGate::changed);
  b.sequence = 100; b.header.stamp.sec = 20;
  EXPECT_EQ(gate.observe(b, "writer-a"), p::SourceGate::rejected);
}
TEST(Identity, Int64RawIdentityIsNeverTruncatedIntoCompatibilityAlias)
{
  p::Producer producer; p::Aliases aliases;
  p::Key k; k.tracker_session_id = producer.session(); k.raw_id = int64_t(INT32_MAX) + 100;
  const auto first = aliases.get(p::identity(k), k.raw_id);
  EXPECT_EQ(first, 0); EXPECT_EQ(k.raw_id, int64_t(INT32_MAX) + 100);
  EXPECT_EQ(aliases.get(p::identity(k), k.raw_id), first);
  ++k.tracker_epoch;
  EXPECT_NE(aliases.get(p::identity(k), k.raw_id), first);
}
TEST(Observation, PredictionCannotPretendToBeAnObservedTrack)
{
  p::Producer detector, tracker;
  easyfly_msgs::msg::ImageTrackArrayV2 m;
  auto h = header(10); auto d = detector.next(h, "front", 64, 48);
  m.context = tracker.next(h, "front", 64, 48); m.upstream = p::reference(d);
  easyfly_msgs::msg::ImageTrackV2 t; t.key.tracker_session_id = tracker.session();
  t.filtered_box_valid = true; t.filtered_box.x2 = t.filtered_box.y2 = 10;
  t.observed_box = t.filtered_box; t.observed_box_valid = true;
  t.observed_this_frame = true; t.missed_updates = 1; m.tracks.push_back(t);
  EXPECT_FALSE(p::valid(m)); m.tracks[0].observed_this_frame = false;
  EXPECT_TRUE(p::valid(m));
  m.tracks[0].image_velocity_valid = true;
  EXPECT_FALSE(p::valid(m)); m.tracks[0].image_velocity_unit = 2;
  EXPECT_TRUE(p::valid(m));
}

TEST(Identity, NearInt32BoundaryCannotExhaustDenseAliases)
{
  p::Aliases aliases;
  EXPECT_EQ(aliases.get("first", int64_t(INT32_MAX) - 1), 0);
  EXPECT_EQ(aliases.get("second", int64_t(INT32_MAX) + 1), 1);
}
