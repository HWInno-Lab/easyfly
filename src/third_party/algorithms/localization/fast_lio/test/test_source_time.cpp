// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "source_time.hpp"

TEST(SourceTime, PreservesNativeMillisecondAcquisitionTimes) {
    for (int64_t ms = 1; ms < 200000; ++ms) {
        EXPECT_EQ(easyfly_fastlio::source_nanoseconds(ms / 1000.0), ms * 1000000);
    }
    EXPECT_EQ(easyfly_fastlio::source_nanoseconds(36.421), 36421000000LL);
}

TEST(SourceTime, RoundsAndNormalizesAtSecondBoundary) {
    EXPECT_EQ(easyfly_fastlio::source_nanoseconds(1.9999999996), 2000000000LL);
    EXPECT_EQ(easyfly_fastlio::source_nanoseconds(1.000000001), 1000000001LL);
    EXPECT_THROW(easyfly_fastlio::source_nanoseconds(NAN), std::invalid_argument);
    EXPECT_THROW(easyfly_fastlio::source_nanoseconds(INFINITY), std::invalid_argument);
    EXPECT_THROW(easyfly_fastlio::source_nanoseconds(-1.), std::invalid_argument);
    EXPECT_THROW(easyfly_fastlio::source_nanoseconds(1e20), std::invalid_argument);
}
