// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
namespace easyfly_imu
{
// PX4 1.16.2 SimulatorMavlink accel-0 FIFO: scale=g/2048, int16_t storage,
// range=16g. Unbounded float -> int16 conversion wraps instead of reporting
// clipping. Model ADC saturation before that narrowing; retain the existing
// driver's clipping detection. Symmetric positive capacity also survives FLU
// -> FRD axis sign changes. One float ULP inward prevents rounding to 32768.
inline float AccelForFifo(double measured)
{
  constexpr float scale=9.80665f/2048.f;
  const float limit=std::nextafter(
    static_cast<float>(std::numeric_limits<std::int16_t>::max())*scale,0.f);
  return static_cast<float>(std::clamp(measured,-static_cast<double>(limit),static_cast<double>(limit)));
}
}  // namespace easyfly_imu
