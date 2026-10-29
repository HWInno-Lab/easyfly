// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <algorithm>

namespace easyfly_orbslam3
{
// A correction epoch is necessary but not sufficient: native local optimization
// can move the tracked pose without announcing a global map correction. Reject
// impossible steps before publishing, and require a new process/session to reset.
class PoseContinuity
{
public:
  bool accept(int64_t stamp, const std::array<double, 3> & position,
              const std::array<double, 4> & orientation)
  {
    if (failed_) return false;
    double norm = 0.;
    for (double value : position) if (!std::isfinite(value)) return reject();
    for (double value : orientation) {
      if (!std::isfinite(value)) return reject();
      norm += value * value;
    }
    if (stamp <= 0 || norm < 1e-12 || !std::isfinite(norm)) return reject();
    auto q = orientation;
    for (double & value : q) value /= std::sqrt(norm);
    if (stamp_) {
      const double dt = (stamp - stamp_) * 1e-9;
      if (dt <= 0. || dt > 0.5) return reject();
      double distance = 0., dot = 0.;
      for (size_t i = 0; i < 3; ++i) distance += std::pow(position[i] - position_[i], 2);
      for (size_t i = 0; i < 4; ++i) dot += q[i] * orientation_[i];
      // Platform speed limit is 3 m/s. Allow measurement noise and a 5 m/s
      // envelope, while preserving the separate 0.3 m / 1 m accuracy audits.
      if (std::sqrt(distance) > 0.25 + 5.0 * dt ||
          2.0 * std::acos(std::clamp(std::abs(dot), 0.0, 1.0)) > 0.15 + 3.0 * dt)
        return reject();
    }
    stamp_ = stamp; position_ = position; orientation_ = q;
    return true;
  }
private:
  bool reject() {failed_ = true; return false;}
  bool failed_{false};
  int64_t stamp_{0};
  std::array<double, 3> position_{};
  std::array<double, 4> orientation_{};
};
}  // namespace easyfly_orbslam3
