// Copyright 2026 EasyFly Project
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "easyfly_detection_api/detector.hpp"
namespace api = easyfly::detection;
class FakeDetector : public api::DetectorBase
{
public:
  using DetectorBase::DetectorBase;
  void detectImpl(const cv::Mat &, std::vector<api::Detection> & out, api::DetectionTiming * t) override
  {
    out.assign(3, {0, 0, 4, 4, .9f, 0, "test"});
    if (t) {t->preprocess_ms = 2; t->inference_ms = 3; t->postprocess_ms = 5;}
  }
  void unletterboxImpl(api::Detection & d) override {d.x1 += 1;}
};
TEST(DetectionApi, BorrowedTimingCoordinateMappingAndLimit)
{
  api::DetectorParams params; params.max_detections = 2;
  FakeDetector detector(params); api::DetectionTiming t;
  t.queue_ms = 7; t.color_conversion_ms = 11; t.debug_image_ms = 13; t.end_to_end_ms = 17;
  std::vector<api::Detection> out;
  detector.detect(cv::Mat(20, 20, CV_8UC3), out, &t);
  ASSERT_EQ(out.size(), 2u); EXPECT_FLOAT_EQ(out[0].x1, 1);
  EXPECT_DOUBLE_EQ(t.queue_ms, 7); EXPECT_DOUBLE_EQ(t.color_conversion_ms, 11);
  EXPECT_DOUBLE_EQ(t.debug_image_ms, 13); EXPECT_DOUBLE_EQ(t.end_to_end_ms, 17);
  EXPECT_DOUBLE_EQ(t.inference_ms, 3);
}
