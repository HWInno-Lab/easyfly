// Copyright 2026 EasyFly Project
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <fstream>
#include <iterator>
#include "easyfly_projection/projection.hpp"
namespace p = easyfly::projection;
p::DepthView view(const std::vector<float> & values, int w, int h, int stride)
{
  return {reinterpret_cast<const std::uint8_t *>(values.data()), values.size() * sizeof(float),
    w, h, static_cast<std::size_t>(stride) * sizeof(float)};
}
TEST(Projection, UpperMedianStrictDepthLimitsAndPaddedRows)
{
  std::vector<float> values{.3f, 1, 2, 99, 3, 4, 9.5f, 99};
  // .3f is slightly above the double .3 bound: retain original float/double comparison.
  EXPECT_FLOAT_EQ(p::medianDepth({.5f,.5f,1,1}, view(values,3,2,4)), 2);
  p::Options options{1,4,.1};
  EXPECT_FLOAT_EQ(p::medianDepth({.5f,.5f,1,1}, view(values,3,2,4), options), 3);
}
TEST(Projection, InvalidDepthAndInsufficientRatio)
{
  std::vector<float> values(9, NAN); values[4] = 2;
  EXPECT_FLOAT_EQ(p::medianDepth({.5f,.5f,1,1}, view(values,3,3,3), {.3,9.5,.1}), 2);
  EXPECT_FLOAT_EQ(p::medianDepth({.5f,.5f,1,1}, view(values,3,3,3), {.3,9.5,.2}), -1);
  values[4] = INFINITY;
  EXPECT_FLOAT_EQ(p::medianDepth({.5f,.5f,1,1}, view(values,3,3,3)), -1);
}
TEST(Projection, InvalidLayoutAndNonFiniteBoxAreRejected)
{
  std::vector<float> values(9, 2);
  auto image=view(values,3,3,3); image.byte_count=4;
  EXPECT_FLOAT_EQ(p::medianDepth({.5f,.5f,1,1}, image), -1);
  image=view(values,3,3,3); image.row_stride=0;
  EXPECT_FLOAT_EQ(p::medianDepth({.5f,.5f,1,1}, image), -1);
  EXPECT_FLOAT_EQ(p::medianDepth({NAN,.5f,1,1}, view(values,3,3,3)), -1);
  EXPECT_FLOAT_EQ(p::medianDepth({.5f,.5f,0,0}, view(values,3,3,3)), -1);
}
TEST(Projection, BorrowedUnalignedBytes)
{
  std::vector<std::uint8_t> bytes(4 * 9 + 1);
  const float depth=2;
  for(int i=0;i<9;++i)std::memcpy(bytes.data()+1+i*4,&depth,4);
  EXPECT_FLOAT_EQ(p::medianDepth({.5f,.5f,1,1}, {bytes.data()+1,36,3,3,12}), 2);
}
TEST(Projection, OpticalAxesRightDownForward)
{
  auto xyz=p::pinhole(330,220,2,100,200,320,240);
  EXPECT_DOUBLE_EQ(xyz[0], .2); EXPECT_DOUBLE_EQ(xyz[1], -.2); EXPECT_DOUBLE_EQ(xyz[2], 2);
}
TEST(Projection, CentralDepthRejectsBackgroundMajority)
{
  std::vector<float> values(21 * 21, 8.0f);
  for (int y = 5; y <= 15; ++y) {
    for (int x = 7; x <= 13; ++x) {values[y * 21 + x] = 2.0f;}
  }
  EXPECT_FLOAT_EQ(p::medianDepth({.5f,.5f,1,1}, view(values,21,21,21)), 8);
  EXPECT_FLOAT_EQ(p::centralMedianDepth({.5f,.5f,1,1}, view(values,21,21,21)), 2);
  for (int y = 5; y <= 15; ++y) {
    for (int x = 5; x <= 15; ++x) {values[y * 21 + x] = NAN;}
  }
  // Background outside the central support must not rescue an invalid estimate.
  EXPECT_FLOAT_EQ(p::centralMedianDepth({.5f,.5f,1,1}, view(values,21,21,21)), -1);
}
TEST(Projection, CentralDepthRetainsLayoutAndFiniteBoxRejection)
{
  std::vector<float> values(21 * 21, 2.0f);
  auto image = view(values,21,21,21);
  image.row_stride = 0;
  EXPECT_FLOAT_EQ(p::centralMedianDepth({.5f,.5f,1,1}, image), -1);
  EXPECT_FLOAT_EQ(p::centralMedianDepth({NAN,.5f,1,1}, view(values,21,21,21)), -1);
  EXPECT_FLOAT_EQ(p::centralMedianDepth({.5f,.5f,0,0}, view(values,21,21,21)), -1);
}
TEST(Projection, RecordedImageEdgePersonKeepsForegroundDepth)
{
  std::ifstream input(std::string(EASYFLY_PROJECTION_TEST_DATA_DIR) +
    "/person_edge_depth.bin", std::ios::binary);
  ASSERT_TRUE(input.good());
  const std::vector<std::uint8_t> raw(
    (std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  ASSERT_EQ(raw.size(), 106U * 151U * sizeof(float));
  std::vector<float> image(640 * 480, NAN);
  for (int y = 0; y < 151; ++y) {
    for (int x = 0; x < 106; ++x) {
      const auto offset = static_cast<std::size_t>(y * 106 + x) * 4;
      const std::uint32_t bits = std::uint32_t(raw[offset]) |
        (std::uint32_t(raw[offset + 1]) << 8) |
        (std::uint32_t(raw[offset + 2]) << 16) |
        (std::uint32_t(raw[offset + 3]) << 24);
      std::memcpy(&image[(328 + y) * 640 + x], &bits, sizeof(float));
    }
  }
  // Exact original Gazebo bytes/box; provenance and SHA are beside the fixture.
  // The old whole-box median projects 0.854m outside the native collision AABB.
  // The central estimate projects inside it with the original camera/TF.
  const p::Box box{.08320005238056183f, .8414033651351929f,
    .16362398862838745f, .31226974725723267f};
  EXPECT_FLOAT_EQ(p::medianDepth(box, view(image,640,480,640), {.3,19.5,.1}),
    3.6646006107330322f);
  EXPECT_FLOAT_EQ(p::centralMedianDepth(box, view(image,640,480,640), {.3,19.5,.1}),
    2.5858962535858154f);
}
