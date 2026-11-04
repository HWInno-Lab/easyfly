// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_csrt/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
//

#include <gtest/gtest.h>
#include "easyfly_csrt/track_core.hpp"
namespace ef_single = easyfly::csrt;
TEST(SingleCore, ExplicitRoiAndSyntheticTranslation) {
  EXPECT_FALSE(ef_single::roi(-.1, 0, .2, .2));
  EXPECT_FALSE(ef_single::roi(0, 0, 0, .2));

  cv::Mat first(160, 200, CV_8UC3, cv::Scalar(30, 30, 30));
  cv::Mat patch(40, 40, CV_8UC3);

  cv::RNG rng(42);
  rng.fill(patch, cv::RNG::UNIFORM, 0, 255);

  patch.copyTo(first(cv::Rect(60, 50, 40, 40)));

  auto tracker = cv::TrackerCSRT::create();
  cv::Rect roi(60, 50, 40, 40);
  tracker->init(first, roi);

  cv::Mat second(160, 200, CV_8UC3, cv::Scalar(30, 30, 30));

  patch.copyTo(second(cv::Rect(68, 54, 40, 40)));

  ASSERT_TRUE(tracker->update(second, roi));
  EXPECT_NEAR(roi.x, 68, 4);
  EXPECT_NEAR(roi.y, 54, 4);

  cv::Mat lost(160, 200, CV_8UC3, cv::Scalar(30, 30, 30));
  EXPECT_FALSE(tracker->update(lost, roi));
}
