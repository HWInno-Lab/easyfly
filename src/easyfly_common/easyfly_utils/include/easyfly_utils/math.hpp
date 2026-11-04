// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_utils/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
#ifndef EASYFLY_UTILS__MATH_HPP_
#define EASYFLY_UTILS__MATH_HPP_
#include <algorithm>
#include <cmath>
namespace easyfly_utils
{
// 角度归一化到 [-PI, PI]
inline double wrap_angle(double angle)
{
  angle = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (angle < 0) {angle += 2.0 * M_PI;}
  return angle - M_PI;
}
// 弧度转度
inline double rad2deg(double rad) {return rad * 180.0 / M_PI;}
inline double deg2rad(double deg) {return deg * M_PI / 180.0;}
// 限幅
inline double clamp(double val, double lo, double hi)
{
  return std::max(lo, std::min(hi, val));
}
// 死区
inline double deadzone(double val, double threshold)
{
  return (std::abs(val) < threshold) ? 0.0 : val;
}
// 低通滤波
inline double low_pass(double current, double target, double alpha)
{
  return alpha * target + (1.0 - alpha) * current;
}
}  // namespace easyfly_utils
#endif  // EASYFLY_UTILS__MATH_HPP_
