// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_kcf/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
#include <gtest/gtest.h>
#include "easyfly_kcf/tracker.hpp"

namespace
{
cv::Mat patch()
{
  cv::Mat result(40, 40, CV_8UC3);
  cv::RNG rng(42);
  rng.fill(result, cv::RNG::UNIFORM, 0, 255);
  return result;
}

cv::Mat image(const cv::Mat & target, int dx = 0, int dy = 0)
{
  cv::Mat result(160, 200, CV_8UC3, cv::Scalar(30, 30, 30));
  target.copyTo(result(cv::Rect(60 + dx, 50 + dy, 40, 40)));
  return result;
}
}

TEST(KcfApi, FirstMovedFrameAndLongTranslationUseOriginalInitializationPixels)
{
  cv::setNumThreads(2);
  const auto target = patch();
  easyfly::kcf::KcfTracker tracker;
  EXPECT_FALSE(tracker.initialized());
  EXPECT_FALSE(tracker.update(image(target)).ok);
  ASSERT_TRUE(tracker.initialize(image(target), {60, 50, 40, 40}));
  EXPECT_EQ(tracker.backend(), "kcf");
  for (int frame = 1; frame <= 20; ++frame) {
    const auto result = tracker.update(image(target, frame * 2, frame));
    ASSERT_TRUE(result.ok) << frame;
    EXPECT_NEAR(result.box.x, 60 + frame * 2, 2) << frame;
    EXPECT_NEAR(result.box.y, 50 + frame, 2) << frame;
    EXPECT_EQ(result.box.size(), cv::Size(40, 40));
    EXPECT_FALSE(result.calibrated_confidence.has_value());
  }
  tracker.reset();
  EXPECT_FALSE(tracker.initialized());
  EXPECT_FALSE(tracker.update(image(target)).ok);
  ASSERT_TRUE(tracker.initialize(image(target, 20, 10), {80, 60, 40, 40}));
  ASSERT_TRUE(tracker.update(image(target, 24, 12)).ok);
}

TEST(KcfApi, ParametersReachNativeTrackerAndLossRequiresExplicitReinitialization)
{
  const auto target = patch();
  const auto first = image(target);
  // Compare the real native implementation at two thresholds, including failure.
  for (const double threshold : {0.1, 1.0}) {
    cv::TrackerKCF::Params params;
    params.detect_thresh = static_cast<float>(threshold);
    auto native = cv::TrackerKCF::create(params);
    native->init(first, {60, 50, 40, 40});
    cv::Rect expected{60, 50, 40, 40};
    ASSERT_TRUE(native->update(first, expected));
    easyfly::kcf::KcfTracker tracker({threshold});
    ASSERT_TRUE(tracker.initialize(first, {60, 50, 40, 40}));
    bool lost = false;
    for (const auto & next : {image(target, 4, 2), cv::Mat(first.size(), first.type(), cv::Scalar(30, 30, 30))}) {
      const bool ok = native->update(next, expected);
      const auto actual = tracker.update(next);
      EXPECT_EQ(actual.ok, ok);
      if (ok) {EXPECT_EQ(actual.box, expected);} else {
        EXPECT_FALSE(tracker.initialized());
        EXPECT_EQ(actual.box, cv::Rect());
        EXPECT_FALSE(tracker.update(first).ok);
        lost = true;
        break;
      }
    }
    EXPECT_TRUE(lost);
    EXPECT_TRUE(tracker.initialize(first, {60, 50, 40, 40}));
  }
}

TEST(KcfApi, InvalidInputAndFailedReinitializeNeverKeepPreviousHistory)
{
  const auto first = image(patch());
  easyfly::kcf::KcfTracker tracker;
  for (const auto & box : {cv::Rect{-1, 0, 40, 40}, cv::Rect{0, 0, 7, 40},
      cv::Rect{190, 0, 40, 40}, cv::Rect{0, 0, 40, 0},
      cv::Rect{std::numeric_limits<int>::max(), 0, 40, 40}})
  {
    ASSERT_TRUE(tracker.initialize(first, {60, 50, 40, 40}));
    EXPECT_FALSE(tracker.initialize(first, box));
    EXPECT_FALSE(tracker.initialized());
  }
  EXPECT_FALSE(tracker.initialize({}, {0, 0, 40, 40}));
  EXPECT_FALSE(tracker.initialize(cv::Mat(160, 200, CV_8UC1), {60, 50, 40, 40}));
  for (const auto & bad : {cv::Mat(), cv::Mat(160, 200, CV_8UC1), cv::Mat(80, 100, CV_8UC3)}) {
    ASSERT_TRUE(tracker.initialize(first, {60, 50, 40, 40}));
    EXPECT_FALSE(tracker.update(bad).ok);
    EXPECT_FALSE(tracker.initialized());
  }
  for (const double bad : {-0.1, 1.1, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()})
  {
    EXPECT_THROW(easyfly::kcf::KcfTracker({bad}), std::invalid_argument);
  }
}
