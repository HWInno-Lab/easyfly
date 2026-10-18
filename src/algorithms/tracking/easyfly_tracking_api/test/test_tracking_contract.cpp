// Copyright 2026 EasyFly Project
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <limits>
#include "easyfly_tracking_api/tracker.hpp"

namespace tr = easyfly::tracking;

namespace
{
tr::Frame frame()
{
  tr::Frame value;
  value.source = {4, 100};
  value.image_width = 4;
  value.image_height = 3;
  value.detections.push_back({{0, 0, 2, 2}, .5, 0, "person", {}});
  return value;
}
}  // namespace

TEST(TrackingContract, EmptyIsValidAndFailureIsNotDefaultSuccess)
{
  auto value = frame();
  value.detections.clear();
  EXPECT_EQ(tr::validate_frame(value), tr::Status::ok);
  EXPECT_EQ(tr::UpdateResult{}.status, tr::Status::backend_error);
}

TEST(TrackingContract, RejectsInvalidIdentityAndDimensions)
{
  auto value = frame();
  value.source.stamp_ns = 0;
  EXPECT_EQ(tr::validate_frame(value), tr::Status::invalid_input);
  value.source.stamp_ns = 100;
  value.image_height = 0;
  EXPECT_EQ(tr::validate_frame(value), tr::Status::invalid_input);
}

TEST(TrackingContract, RejectsMalformedGeometryAndScores)
{
  auto value = frame();
  value.detections[0].box.x2 = 0;
  EXPECT_EQ(tr::validate_frame(value), tr::Status::invalid_input);
  value = frame();
  value.detections[0].confidence = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(tr::validate_frame(value), tr::Status::invalid_input);
  value.detections[0].confidence = 1.01;
  EXPECT_EQ(tr::validate_frame(value), tr::Status::invalid_input);
  value.detections[0].confidence = .5;
  value.detections[0].class_id = -2;
  EXPECT_EQ(tr::validate_frame(value), tr::Status::invalid_input);
  value = frame();
  value.detections[0].box.x1 = -std::numeric_limits<double>::max();
  value.detections[0].box.x2 = std::numeric_limits<double>::max();
  EXPECT_EQ(tr::validate_frame(value), tr::Status::invalid_input);
}

TEST(TrackingContract, DoesNotClipOtherwiseValidBoxes)
{
  auto value = frame();
  value.detections[0].box = {-1, -1, 5, 4};
  value.detections[0].class_id = -1;
  EXPECT_EQ(tr::validate_frame(value), tr::Status::ok);
}

TEST(TrackingContract, ChecksInputRequirementsAndResourceLimits)
{
  auto value = frame();
  tr::Requirements needs;
  needs.low_confidence_detections = true;
  EXPECT_EQ(tr::validate_frame(value, needs), tr::Status::unsupported_input);
  value.includes_low_confidence_detections = true;
  EXPECT_EQ(tr::validate_frame(value, needs), tr::Status::ok);
  tr::Limits limits;
  limits.max_detections = 0;
  EXPECT_EQ(tr::validate_frame(value, needs, limits), tr::Status::resource_limit);
}

TEST(TrackingContract, ValidatesRequiredAppearanceDimensionsAndFiniteValues)
{
  auto value = frame();
  tr::Requirements needs;
  needs.appearance_dimension = 2;
  EXPECT_EQ(tr::validate_frame(value, needs), tr::Status::unsupported_input);
  value.detections[0].appearance = {1, 0};
  EXPECT_EQ(tr::validate_frame(value, needs), tr::Status::ok);
  value.detections[0].appearance[1] = std::numeric_limits<float>::infinity();
  EXPECT_EQ(tr::validate_frame(value, needs), tr::Status::invalid_input);
  tr::Limits limits;
  limits.max_appearance_dimension = 1;
  EXPECT_EQ(tr::validate_frame(value, needs, limits), tr::Status::resource_limit);
}

TEST(TrackingContract, ValidatesPaddedImageAndRejectsTruncatedStorage)
{
  auto value = frame();
  tr::Requirements needs;
  needs.image = true;
  EXPECT_EQ(tr::validate_frame(value, needs), tr::Status::unsupported_input);
  std::vector<std::uint8_t> bytes(48);
  value.image = tr::ImageView{bytes.data(), bytes.size(), 16, 4, 3, tr::PixelFormat::bgr8};
  EXPECT_EQ(tr::validate_frame(value, needs), tr::Status::ok);
  value.image->byte_size = 47;
  EXPECT_EQ(tr::validate_frame(value, needs), tr::Status::invalid_input);
  value.image->byte_size = 48;
  value.image->row_stride = 11;
  EXPECT_EQ(tr::validate_frame(value, needs), tr::Status::invalid_input);
  value.image->row_stride = 16;
  value.image->format = static_cast<tr::PixelFormat>(99);
  EXPECT_EQ(tr::validate_frame(value, needs), tr::Status::invalid_input);
}

TEST(TrackingContract, ChecksImageArithmeticBeforeMultiplication)
{
  auto value = frame();
  const std::uint8_t sentinel = 0;
  value.image = tr::ImageView{
    &sentinel, std::numeric_limits<std::size_t>::max(),
    std::numeric_limits<std::size_t>::max(), 4, 3, tr::PixelFormat::mono8};
  EXPECT_EQ(tr::validate_frame(value), tr::Status::invalid_input);
}

TEST(TrackingContract, SequenceRejectsReplayAndRequiresExplicitEpochReset)
{
  tr::FrameSequence sequence;
  sequence.reset(4);
  EXPECT_EQ(sequence.check({4, 0}), tr::Status::invalid_input);
  EXPECT_TRUE(sequence.commit({4, 100}));
  EXPECT_FALSE(sequence.commit({4, 100}));
  EXPECT_FALSE(sequence.commit({4, 90}));
  EXPECT_EQ(sequence.check({5, 101}), tr::Status::invalid_input);
  // A check alone (e.g. before a failed backend call) must not consume a stamp.
  EXPECT_EQ(sequence.check({4, 101}), tr::Status::ok);
  EXPECT_EQ(sequence.check({4, 101}), tr::Status::ok);
  sequence.reset(5);
  EXPECT_TRUE(sequence.commit({5, 1}));
}
