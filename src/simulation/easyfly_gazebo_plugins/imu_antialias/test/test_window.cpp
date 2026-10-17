// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include "easyfly_imu/window.hpp"
using easyfly_imu::Window;

TEST(ImuAntialias, ConstantSixChannelsAndNativeNonzeroNoiseAreRetained)
{
  Window filter;
  const Window::Values constant{.1,.2,9.81,.01,.02,.03};
  for (int i=0;i<16;++i) {
    auto result=filter.Push(i,1000*(i+1),constant);
    ASSERT_TRUE(result);
    for (int axis=0;axis<6;++axis) {EXPECT_NEAR((*result)[axis],constant[axis],1e-12);}
  }
  auto noisy=constant;noisy[0]+=.004;
  auto result=filter.Push(16,17000,noisy);
  ASSERT_TRUE(result);
  EXPECT_NEAR((*result)[0]-.1,.001,1e-12);
}

TEST(ImuAntialias, ContactImpulseSurvivesEveryFourToOneSamplingPhase)
{
  Window filter;
  std::array<double,4> raw{},averaged{};
  double reference=0;
  for (int i=0;i<1024;++i) {
    const double acceleration=(i>=100 && i<500) ? -9.81 : (i==500 ? 400*9.81 : 0);
    auto result=filter.Push(i,1000*(i+1),{0,0,9.81+acceleration,0,0,0});
    ASSERT_TRUE(result);
    reference+=acceleration*.001;
    raw[i%4]+=acceleration*.004;
    averaged[i%4]+=((*result)[2]-9.81)*.004;
  }
  EXPECT_NEAR(reference,0.,1e-10);
  EXPECT_GT(std::abs(raw[0]-raw[1]),10.);
  for (double velocity : averaged) {EXPECT_NEAR(velocity,reference,1e-10);}
}

TEST(ImuAntialias, InvalidSamplesAreNotFilledOrRetimestamped)
{
  Window filter;
  ASSERT_TRUE(filter.Push(0,1000,{0,0,9.81,0,0,0}));
  EXPECT_FALSE(filter.Push(0,2000,{0,0,9.81,0,0,0}));
  EXPECT_FALSE(filter.Push(2,3000,{0,0,9.81,0,0,0}));
  EXPECT_FALSE(filter.Push(1,1000,{0,0,9.81,0,0,0}));
  EXPECT_FALSE(filter.Push(1,2500,{0,0,9.81,0,0,0}));
  EXPECT_NE(filter.error().find("previous_usec=1000"), std::string::npos);
  EXPECT_NE(filter.error().find("received_usec=2500"), std::string::npos);
  EXPECT_FALSE(filter.Push(1,2000,{NAN,0,9.81,0,0,0}));
  // Rejected input did not mutate the valid history.
  EXPECT_TRUE(filter.Push(1,2000,{0,0,9.81,0,0,0}));
}
TEST(ImuAntialias, Native64BitTimestampRemainsContinuousAcrossOriginalOverflowBoundary)
{
  Window filter;
  for (int i=0;i<4000;++i) {
    ASSERT_TRUE(filter.Push(i,2147000000LL+1000LL*i,{0,0,9.81,0,0,0}));
  }
}

#include "easyfly_imu/accel_fifo.hpp"
TEST(ImuAccelFifo, NormalRangeIsUnchangedAtMavlinkFloatPrecision)
{
  for (double value : {-100.,-9.80665,-.01,0.,.01,9.80665,100.}) {
    EXPECT_EQ(easyfly_imu::AccelForFifo(value),static_cast<float>(value));
  }
}
TEST(ImuAccelFifo, SaturationCannotWrapAndTriggersExistingClipDetection)
{
  constexpr float scale=9.80665f/2048.f;
  constexpr float clip_limit=(16.f*9.80665f/scale)*.999f;
  for (double source : {-1e300,-1321.,-200.,200.,1321.,1e300}) {
    const float output=easyfly_imu::AccelForFifo(source);
    for (float sign : {-1.f,1.f}) {
      const float counts=sign*output/scale;
      ASSERT_LE(counts,32767.f);ASSERT_GE(counts,-32768.f);
      const auto native=static_cast<std::int16_t>(counts);
      EXPECT_GE(std::abs(static_cast<float>(native)),clip_limit);
      EXPECT_EQ(std::signbit(counts),std::signbit(static_cast<float>(native)));
    }
  }
}
