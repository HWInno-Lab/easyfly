// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cmath>
#include <map>
#include <utility>
#include "easyfly_semantic_map/ground_plane_tracker.hpp"

namespace easyfly_semantic_map
{

// Evidence only: never removes points from the obstacle cloud. A strict
// 1--3 m annulus must surround the vehicle in every octant. Count independent
// 0.2 m XY cells, not repeated returns. Any inconsistent low return vetoes its
// cell. Ceiling/wall returns above the vehicle-parallel plane cannot increase
// floor support; below-plane outliers count against the 90% confidence gate.
template<class Cloud>
GroundSpatialSupport groundSpatialSupport(
  const Cloud & cloud, const GroundPlaneModel & model, const GroundVehiclePose & vehicle)
{
  GroundSpatialSupport result;
  const double clearance = vehicle.z - model.zAt(vehicle.x, vehicle.y);
  const double norm = std::sqrt(model.normal[0] * model.normal[0] +
    model.normal[1] * model.normal[1] + model.normal[2] * model.normal[2]);
  if (!model.valid() || !std::isfinite(clearance) || clearance < 0.03 ||
    clearance > 0.25 || std::abs(norm - 1.0) > 1e-3 || model.normal[2] < std::cos(0.10) ||
    !std::isfinite(vehicle.x) || !std::isfinite(vehicle.y)) {return result;}
  std::map<std::pair<int, int>, bool> cells;
  for (const auto & point : cloud) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      return GroundSpatialSupport{};
    }
    const double dx = point.x - vehicle.x;
    const double dy = point.y - vehicle.y;
    const double radius_squared = dx * dx + dy * dy;
    if (radius_squared < 1.0 || radius_squared > 9.0) {continue;}
    const double distance = model.signedDistance(point.x, point.y, point.z);
    if (distance / model.normal[2] > clearance) {continue;}
    const auto cell = std::make_pair(
      static_cast<int>(std::floor(dx / 0.2)), static_cast<int>(std::floor(dy / 0.2)));
    const bool inlier = std::abs(distance) <= 0.025;
    const auto inserted = cells.emplace(cell, inlier);
    if (!inserted.second) {inserted.first->second = inserted.first->second && inlier;}
    if (inlier) {++result.points;}
  }
  result.eligible_cells = cells.size();
  constexpr double pi = 3.14159265358979323846;
  for (const auto & entry : cells) {
    if (!entry.second) {continue;}
    ++result.cells;
    const double angle = std::atan2(entry.first.second + 0.5, entry.first.first + 0.5);
    const int sector = static_cast<int>((angle + pi) * 8 / (2 * pi)) % 8;
    ++result.sector_cells[sector];
  }
  return result;
}

}  // namespace easyfly_semantic_map
