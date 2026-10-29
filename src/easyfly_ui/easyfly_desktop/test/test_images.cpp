// Copyright 2026 EasyFly Project
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include "easyfly_desktop/image_pipeline.hpp"
namespace ui = easyfly_desktop;
namespace b = easyfly_ui_bridge;
namespace
{
sensor_msgs::msg::Image::SharedPtr image(const std::string & encoding = "rgb8")
{
  auto value = std::make_shared<sensor_msgs::msg::Image>();
  value->header.frame_id = "camera"; value->header.stamp.sec = 10;
  value->width = 2; value->height = 1; value->encoding = encoding;
  value->step = encoding == "mono8" ? 2 : 6;
  value->data = encoding ==
    "mono8" ? std::vector<uint8_t>{11, 22} : std::vector<uint8_t>{1, 2, 3, 4, 5, 6};
  return value;
}
template<typename Predicate>
bool wait_for(Predicate predicate)
{
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (std::chrono::steady_clock::now() < end) {
    if (predicate()) {return true;} std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return predicate();
}
}  // namespace
TEST(Images, FormatsAndOwnedPixels)
{
  auto source = image(); auto rgb = ui::ImagePipeline::convert(*source);
  EXPECT_EQ(rgb.pixelColor(0, 0), QColor(1, 2, 3));
  source->data[0] = 99; EXPECT_EQ(rgb.pixelColor(0, 0), QColor(1, 2, 3));
  EXPECT_EQ(ui::ImagePipeline::convert(*image("bgr8")).pixelColor(0, 0), QColor(3, 2, 1));
  EXPECT_EQ(ui::ImagePipeline::convert(*image("mono8")).pixelColor(1, 0), QColor(22, 22, 22));
}
TEST(Images, RejectMalformedAndUnsupported)
{
  auto source = image(); source->step = 1;
  EXPECT_THROW(ui::ImagePipeline::convert(*source), std::exception);
  source = image(); source->data.clear(); EXPECT_THROW(
    ui::ImagePipeline::convert(
      *source), std::exception);
  source = image("rgba8"); EXPECT_THROW(ui::ImagePipeline::convert(*source), std::exception);
  source = image(); source->width = 1921; EXPECT_THROW(
    ui::ImagePipeline::convert(
      *source), std::exception);
}
TEST(Images, SingleLatestSlotUnderOverload)
{
  ui::ImagePipeline pipeline([](const auto & frame) {
      std::this_thread::sleep_for(std::chrono::milliseconds(40));
      return ui::ImagePipeline::convert(frame);
    });
  pipeline.bind({1, 0, 0, "/front"}); pipeline.submit(image(), b::detail::steady_seconds());
  ASSERT_TRUE(wait_for([&] {return pipeline.stats().active;}));
  for (int i = 1; i <= 10000; ++i) {
    auto frame = image(); frame->header.stamp.nanosec = i;
    pipeline.submit(frame, b::detail::steady_seconds()); ASSERT_LE(pipeline.stats().pending, 1U);
  }
  ASSERT_TRUE(wait_for([&] {return pipeline.latest() != nullptr;}));
  EXPECT_EQ(pipeline.latest()->stamp_ns, 10000010000LL); EXPECT_GT(pipeline.stats().dropped, 0U);
  pipeline.stop(); ASSERT_TRUE(wait_for([&] {return pipeline.stats().stopped;}));
}
TEST(Images, RebindingDiscardsActiveOldResult)
{
  ui::ImagePipeline pipeline([](const auto & frame) {
      std::this_thread::sleep_for(std::chrono::milliseconds(40));
      return ui::ImagePipeline::convert(frame);
    });
  pipeline.bind({1, 0, 0, "/front"}); pipeline.submit(image(), b::detail::steady_seconds());
  ASSERT_TRUE(wait_for([&] {return pipeline.stats().active;}));
  const ui::ImageBinding changed{2, 1, 1, "/down"}; pipeline.bind(changed);
  ASSERT_TRUE(wait_for([&] {return !pipeline.stats().active;})); EXPECT_FALSE(pipeline.latest());
  pipeline.submit(image(), b::detail::steady_seconds());
  ASSERT_TRUE(wait_for([&] {return pipeline.latest() != nullptr;})); EXPECT_TRUE(
    pipeline.latest()->binding == changed);
  pipeline.clear(); EXPECT_FALSE(pipeline.latest());
}
TEST(Images, StopIsNonblockingAndClearsOutput)
{
  ui::ImagePipeline pipeline([](const auto & frame) {
      std::this_thread::sleep_for(std::chrono::milliseconds(150));
      return ui::ImagePipeline::convert(frame);
    });
  pipeline.bind({1, 0, 0, "/front"}); pipeline.submit(image(), b::detail::steady_seconds());
  ASSERT_TRUE(wait_for([&] {return pipeline.stats().active;}));
  const auto start = std::chrono::steady_clock::now(); pipeline.stop();
  EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(100));
  EXPECT_FALSE(pipeline.stats().stopped); pipeline.submit(image(), b::detail::steady_seconds());
  ASSERT_TRUE(wait_for([&] {return pipeline.stats().stopped;}));
  EXPECT_EQ(pipeline.stats().pending, 0U); EXPECT_FALSE(pipeline.latest());
}
