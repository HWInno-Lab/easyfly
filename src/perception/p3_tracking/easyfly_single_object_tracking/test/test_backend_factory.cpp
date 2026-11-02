// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "easyfly_single_object_tracking/backend_factory.hpp"
namespace b = easyfly::backends;
namespace s = easyfly::single_tracking;
namespace p = easyfly_single_object_tracking;
class FixtureSingle final : public s::Tracker {
public:
  explicit FixtureSingle(int shift) : shift_(shift) {}
  std::string backend() const override {return "fixture";}
  bool initialize(const cv::Mat &, const cv::Rect & roi) override {box_ = roi; ready_ = true; return true;}
  s::Result update(const cv::Mat &) override {box_.x += shift_; return {ready_, box_, std::nullopt};}
  void reset() override {ready_ = false;}
  bool initialized() const override {return ready_;}
private: int shift_; cv::Rect box_; bool ready_{false};
};
TEST(SingleFactory, SelectedCreatorAndFreshInstancesOwnHistory) {
  p::TrackerFactory factory;
  factory.add("fixture", [](const b::Parameters & values) {return std::make_unique<FixtureSingle>(b::parameter<int>(values, "shift"));});
  b::BackendConfig config{"fixture", {{"shift", 7}}};
  auto first = p::make_tracker(config, factory); auto second = p::make_tracker(config, factory);
  EXPECT_FALSE(second->initialized()); first->initialize({}, {10, 10, 20, 20});
  EXPECT_EQ(first->update({}).box.x, 17); EXPECT_FALSE(second->initialized());
  first->reset(); EXPECT_FALSE(first->initialized());
}
TEST(SingleFactory, RealCsrtAndInvalidSelection) {
  auto tracker = p::make_tracker({"csrt", {}});
  EXPECT_EQ(tracker->backend(), "csrt"); EXPECT_FALSE(tracker->initialized());
  EXPECT_THROW(p::make_tracker({"fixture_missing", {}}), std::invalid_argument);
  EXPECT_THROW(p::make_tracker({"csrt", {{"unknown", 1}}}), std::invalid_argument);
}

TEST(SingleFactory, RealKcfSelectionParameterValidationAndIndependentHistory) {
  auto first = p::make_tracker({"kcf", {{"detect_thresh", .2}}});
  auto second = p::make_tracker({"kcf", {}});
  EXPECT_EQ(first->backend(), "kcf"); EXPECT_FALSE(second->initialized());
  EXPECT_THROW(p::make_tracker({"kcf", {{"detect_thresh", true}}}), std::invalid_argument);
  EXPECT_THROW(p::make_tracker({"kcf", {{"detect_thresh", -1.}}}), std::invalid_argument);
  EXPECT_THROW(p::make_tracker({"kcf", {{"detect_thresh", NAN}}}), std::invalid_argument);
  EXPECT_THROW(p::make_tracker({"kcf", {{"unknown", 1}}}), std::invalid_argument);
  EXPECT_THROW(p::make_tracker({"csrt", {{"detect_thresh", .2}}}), std::invalid_argument);
  cv::Mat frame(160, 200, CV_8UC3); cv::RNG rng(42);
  rng.fill(frame, cv::RNG::UNIFORM, 0, 255);
  ASSERT_TRUE(first->initialize(frame, {60, 50, 40, 40}));
  EXPECT_FALSE(second->initialized());
  EXPECT_TRUE(first->update(frame).ok);
  first->reset(); EXPECT_FALSE(first->initialized());
}

TEST(SingleFactory, ValidateGeometryAndOptionalCalibratedConfidence) {
  s::Result result{true, {10, 10, 20, 20}, .8};
  EXPECT_TRUE(p::valid_result(result, 100, 100));
  result.calibrated_confidence = NAN; EXPECT_FALSE(p::valid_result(result, 100, 100));
  result.calibrated_confidence.reset(); EXPECT_TRUE(p::valid_result(result, 100, 100));
  result.box.x = std::numeric_limits<int>::max(); EXPECT_FALSE(p::valid_result(result, 100, 100));
}
