// Copyright 2026 EasyFly Project
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
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
