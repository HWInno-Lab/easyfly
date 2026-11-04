// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_ground_segmentation/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
//

#pragma once
#include <cstddef>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "easyfly_ground_segmentation/ground_plane_tracker.hpp"
namespace easyfly::ground
{
struct GroundFitParams
{
  double plane_distance_threshold{0.12};
  int plane_min_inliers{50};
  double ground_min_clearance{0.03};
  bool ground_spatial_support_enabled{false};
  double plane_min_inlier_ratio{0.15};
};
struct GroundFitResult
{
  GroundPlaneCandidate candidate;
  bool has_candidate{false};
  std::size_t inlier_count{0};
  std::size_t coefficient_count{0};
};
// Synchronous read-only borrowing of one complete ground-frame cloud.
// Candidate availability is not permission to remove points or publish a ready map.
GroundFitResult fitGroundCandidate(
  const pcl::PointCloud<pcl::PointXYZI> & cloud,
  const GroundVehiclePose & vehicle, const GroundFitParams & params = {});
}  // namespace easyfly::ground
