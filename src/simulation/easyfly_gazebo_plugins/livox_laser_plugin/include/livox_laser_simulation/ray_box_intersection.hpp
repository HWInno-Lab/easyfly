// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#ifndef LIVOX_LASER_SIMULATION__RAY_BOX_INTERSECTION_HPP_
#define LIVOX_LASER_SIMULATION__RAY_BOX_INTERSECTION_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <gazebo/ode/collision.h>

namespace easyfly_livox
{
// Slab intersection in the box frame. A nearly parallel direction still has
// a finite intersection interval; only an exactly zero component is parallel.
// In particular, never discard an axis without checking its outside slab.
inline std::optional<double> rayBoxDistanceLocal(
  const std::array<double, 3> & start, const std::array<double, 3> & direction,
  const std::array<double, 3> & half_size, double max_distance)
{
  if (!std::isfinite(max_distance) || max_distance <= 0.0) {return std::nullopt;}
  double enter = -std::numeric_limits<double>::infinity();
  double leave = std::numeric_limits<double>::infinity();
  bool moving = false;
  for (int axis = 0; axis < 3; ++axis) {
    const double s = start[axis], d = direction[axis], h = half_size[axis];
    if (!std::isfinite(s) || !std::isfinite(d) || !std::isfinite(h) || h <= 0.0) {
      return std::nullopt;
    }
    if (d == 0.0) {
      if (s < -h || s > h) {return std::nullopt;}
      continue;
    }
    moving = true;
    double first = (-h - s) / d, last = (h - s) / d;
    if (first > last) {std::swap(first, last);}
    enter = std::max(enter, first);
    leave = std::min(leave, last);
    if (enter > leave) {return std::nullopt;}
  }
  // Match a native ray's exit surface when the segment starts inside a box.
  const double distance = enter >= 0.0 ? enter : leave;
  if (!moving || !std::isfinite(distance) || distance < 0.0 || distance > max_distance) {
    return std::nullopt;
  }
  return distance;
}

inline std::optional<double> boxRayDistance(dGeomID ray, dGeomID box)
{
  if (!ray || !box || dGeomGetClass(ray) != dRayClass || dGeomGetClass(box) != dBoxClass) {
    return std::nullopt;
  }
  dVector3 start, direction, size;
  dGeomRayGet(ray, start, direction);
  dGeomBoxGetLengths(box, size);
  const auto * position = dGeomGetPosition(box);
  const auto * rotation = dGeomGetRotation(box);
  std::array<double, 3> local_start{}, local_direction{}, half_size{};
  for (int axis = 0; axis < 3; ++axis) {
    half_size[axis] = size[axis] * 0.5;
    for (int world = 0; world < 3; ++world) {
      local_start[axis] += rotation[4 * world + axis] * (start[world] - position[world]);
      local_direction[axis] += rotation[4 * world + axis] * direction[world];
    }
  }
  return rayBoxDistanceLocal(local_start, local_direction, half_size, dGeomRayGetLength(ray));
}
}  // namespace easyfly_livox
#endif
