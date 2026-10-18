// Copyright 2026 EasyFly Project
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
