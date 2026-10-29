// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <memory>
#include "scan_queue.hpp"
#include "path_tip.hpp"

TEST(ScanQueue, ProcessingBlackoutRetainsRecentPairedScans) {
    easyfly_fastlio::ScanQueue<std::shared_ptr<int>> queue;
    auto retired = std::make_shared<int>(0);
    queue.push(retired, 10.);
    auto pending = queue.front();  // A scan waiting for its final IMU sample.
    bool pending_valid = true;
    for (int i = 1; i <= 100; ++i) {
        if (queue.push(std::make_shared<int>(i), 10. + .1 * i)) {
            pending_valid = false;
        }
        ASSERT_LE(queue.size(), 2u);
    }
    EXPECT_FALSE(pending_valid);
    EXPECT_EQ(*pending.cloud, 0);  // Existing ownership never becomes dangling.
    EXPECT_EQ(queue.dropped(), 99u);
    EXPECT_EQ(*queue.front().cloud, 99);
    EXPECT_DOUBLE_EQ(queue.front().stamp, 19.9);
    queue.pop_front();
    EXPECT_EQ(*queue.front().cloud, 100);
    EXPECT_DOUBLE_EQ(queue.front().stamp, 20.);
    queue.pop_front();
    EXPECT_TRUE(queue.empty());
}

TEST(ScanQueue, ResetCannotPairANewCloudWithAnOldTimestamp) {
    easyfly_fastlio::ScanQueue<int> queue;
    queue.push(1, 100.); queue.push(2, 100.1);
    queue.clear(); queue.push(3, 1.);
    EXPECT_EQ(queue.size(), 1u);
    EXPECT_EQ(queue.front().cloud, 3);
    EXPECT_DOUBLE_EQ(queue.front().stamp, 1.);
    queue.pop_front(); EXPECT_TRUE(queue.empty());
}

TEST(PathTip, DecimatesHistoryWithoutHoldingAnOldObservationAtTheTip) {
    std::vector<int> poses;
    std::size_t observations = 0;
    for (int source = 1; source <= 105; ++source) {
        easyfly_fastlio::update_path_tip(poses, observations, source);
        ASSERT_FALSE(poses.empty());
        EXPECT_EQ(poses.back(), source);
        EXPECT_EQ(poses.size(), static_cast<std::size_t>((source + 9) / 10));
    }
    EXPECT_EQ(poses.front(), 10);
    EXPECT_EQ(poses[9], 100);
    EXPECT_EQ(poses.back(), 105);
}
