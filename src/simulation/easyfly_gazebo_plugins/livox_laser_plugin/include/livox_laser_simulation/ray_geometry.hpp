// Copyright 2026 EasyFly Project
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LIVOX_LASER_SIMULATION__RAY_GEOMETRY_HPP_
#define LIVOX_LASER_SIMULATION__RAY_GEOMETRY_HPP_

#include <ignition/math/Vector3.hh>

namespace easyfly_livox
{

// Both initial construction and per-frame updates start at the configured
// minimum range. Sensor-origin distance must add that offset to ray length.
inline ignition::math::Vector3d rayStart(
  const ignition::math::Vector3d & origin,
  const ignition::math::Vector3d & unit_axis, double min_range)
{
  return origin + unit_axis * min_range;
}

// LivoxOdeMultiRayShape overrides Init without running MultiRayShape::Init:
// the base class's private minRange remains zero. Its GetRange therefore
// cannot recover the configured segment-start offset. Use actual RayShape
// length plus the same configured minimum used by rayStart, exactly once.
inline double sensorRange(double ray_length, double min_range)
{
  return min_range + ray_length;
}

}  // namespace easyfly_livox

#endif  // LIVOX_LASER_SIMULATION__RAY_GEOMETRY_HPP_
