// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_time_sync/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
//

#include <gtest/gtest.h>
#include "easyfly_time_sync/pairing.hpp"
using easyfly_time_sync::Pairer;
using easyfly_time_sync::Options;
using Core = Pairer<int>;
static Core::Entry e(int64_t stamp, double wall = 0, size_t bytes = 4)
{
  return {stamp, wall, bytes, std::make_shared<const int>(42)};
}
TEST(Pairing, SignedDeltaAndOriginalPayload) {
  Core p;
  EXPECT_TRUE(p.add(0, e(1000000000), 1000000000, 0).empty());
  auto pairs = p.add(1, e(1020000000, .02), 1020000000, .02);
  ASSERT_EQ(pairs.size(), 1u); EXPECT_EQ(pairs[0].first.stamp, 1000000000);
  EXPECT_EQ(pairs[0].second.stamp, 1020000000); EXPECT_EQ(*pairs[0].first.payload, 42);
  EXPECT_TRUE(p.valid(1020000000, .02)); EXPECT_EQ(p.bytes(), 0u);
}
TEST(Pairing, OutOfToleranceDropsOlderWithoutInventedStamp) {
  Core p; p.add(0, e(1000000000), 1000000000, 0);
  EXPECT_TRUE(p.add(1, e(1100000000, .1), 1100000000, .1).empty());
  EXPECT_EQ(p.dropped(0), 1u);
  auto pairs = p.add(0, e(1110000000, .11), 1110000000, .11);
  ASSERT_EQ(pairs.size(), 1u); EXPECT_EQ(pairs[0].second.stamp - pairs[0].first.stamp, -10000000);
}
TEST(Pairing, QueueAndMemoryBudgetNonmonotonicInvalid) {
  Core p(Options{.03, .5, 2, 8});
  p.add(0, e(1000000000), 1000000000, 0);
  p.add(0, e(1010000000, .01), 1010000000, .01);
  p.add(0, e(1020000000, .02), 1020000000, .02);
  EXPECT_EQ(p.dropped(0), 1u); EXPECT_EQ(p.queued(0), 2u); EXPECT_EQ(p.bytes(), 8u);
  p.add(0, e(1020000000, .03), 1030000000, .03);
  p.add(0, e(0, .03), 1030000000, .03);
  p.add(0, e(2000000000, .03), 1030000000, .03);
  EXPECT_EQ(p.dropped(0), 4u);
  p.add(1, e(1020000000, .03), 1030000000, .03);
  EXPECT_LE(p.bytes(), 8u); EXPECT_EQ(p.paired(), 1u);
}
TEST(Pairing, SourceAndReceiptExpirePauseAndClockReset) {
  Core p; p.add(0, e(1000000000), 1000000000, 0); p.add(1, e(1000000000), 1000000000, 0);
  p.expire(1000000000, .6); EXPECT_FALSE(p.valid(1000000000, .6));
  EXPECT_TRUE(p.add(0, e(1000000000, .6), 1000000000, .6).empty());
  p.expire(500000000, .7); EXPECT_FALSE(p.valid(500000000, .7));
  p.add(0, e(500000000, .7), 500000000, .7);
  ASSERT_EQ(p.add(1, e(500000000, .7), 500000000, .7).size(), 1u);
  p.expire(1100000000, .71); EXPECT_FALSE(p.valid(1100000000, .71));
  Core q; q.add(0, e(1000000000), 1000000000, 0);
  q.expire(1100000000, .6); EXPECT_EQ(q.queued(0), 0u); EXPECT_EQ(q.dropped(0), 1u);
}
TEST(Pairing, RejectInvalidOptionsAndOversize) {
  EXPECT_THROW(Core(Options{.6, 1, 8, 64}), std::invalid_argument);
  EXPECT_THROW(Core(Options{.2, .1, 8, 64}), std::invalid_argument);
  EXPECT_THROW(Core(Options{.03, .5, 0, 64}), std::invalid_argument);
  Core p(Options{.03, .5, 2, 8}); p.add(0, e(1000000000, 0, 9), 1000000000, 0);
  EXPECT_EQ(p.bytes(), 0u); EXPECT_EQ(p.dropped(0), 1u);
}
