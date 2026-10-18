// Copyright 2026 EasyFly Project
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "easyfly_csrt/track_core.hpp"
TEST(CsrtApi, RealBackendResetTranslationAndUnknownConfidence)
{
  cv::setNumThreads(2);
  cv::Mat frame(160, 200, CV_8UC3, cv::Scalar(30, 30, 30)), patch(40, 40, CV_8UC3);
  cv::RNG rng(42); rng.fill(patch, cv::RNG::UNIFORM, 0, 255);
  patch.copyTo(frame(cv::Rect(60, 50, 40, 40)));
  auto tracker = easyfly::csrt::make_tracker("csrt");
  EXPECT_FALSE(tracker->initialized()); EXPECT_FALSE(tracker->update(frame).ok);
  ASSERT_TRUE(tracker->initialize(frame, {60, 50, 40, 40}));
  EXPECT_TRUE(tracker->initialized());
  cv::Mat next(frame.size(), frame.type(), cv::Scalar(30, 30, 30));
  patch.copyTo(next(cv::Rect(68, 54, 40, 40)));
  const auto output = tracker->update(next);
  ASSERT_TRUE(output.ok); EXPECT_NEAR(output.box.x, 68, 4); EXPECT_NEAR(output.box.y, 54, 4);
  EXPECT_FALSE(output.calibrated_confidence.has_value());
  tracker->reset(); EXPECT_FALSE(tracker->initialized()); EXPECT_FALSE(tracker->update(next).ok);
  ASSERT_TRUE(tracker->initialize(next, {68, 54, 40, 40}));
  EXPECT_EQ(tracker->backend(), "csrt");
}
TEST(CsrtApi, InvalidInitializationAndUnknownBackend)
{
  auto tracker = easyfly::csrt::make_tracker("csrt");
  EXPECT_FALSE(tracker->initialize({}, {0, 0, 20, 20}));
  EXPECT_FALSE(tracker->initialize(cv::Mat(20, 20, CV_8UC3), {15, 15, 20, 20}));
  EXPECT_FALSE(tracker->initialized());
  EXPECT_THROW(easyfly::csrt::make_tracker("kcf"), std::invalid_argument);
}
