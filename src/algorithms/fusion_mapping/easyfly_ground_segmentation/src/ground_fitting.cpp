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

#include "easyfly_ground_segmentation/ground_fitting.hpp"
#include "easyfly_ground_segmentation/ground_candidate_selection.hpp"
#include "easyfly_ground_segmentation/ground_spatial_support.hpp"
#include <pcl/segmentation/sac_segmentation.h>
#include <cmath>
namespace easyfly::ground
{
GroundFitResult fitGroundCandidate(
  const pcl::PointCloud<pcl::PointXYZI> & cloud,
  const GroundVehiclePose & vehicle, const GroundFitParams & params)
{
  GroundFitResult result;
  pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr ground_inliers(new pcl::PointIndices);
  if (cloud.size() >= 100) {
    // ---- 本帧地面候选: RANSAC 只提供候选, 不能直接决定剔除 ----
    // A ceiling can dominate an indoor frame. Fit only physically possible
    // below-vehicle samples, while retaining the entire cloud for obstacle
    // classification and for the tracker's original inlier-ratio denominator.
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_fit(new pcl::PointCloud<pcl::PointXYZI>);
    ground_fit->reserve(cloud.size());
    for (const auto & point : cloud.points) {
      if (isGroundFitSample(point.z, vehicle.z, params.ground_min_clearance)) {
        ground_fit->push_back(point);
      }
    }
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setAxis(Eigen::Vector3f(0.0f, 0.0f, 1.0f));
    seg.setEpsAngle(0.1);        // ±6° 内视为水平 (EKF 姿态噪声量级, 平面本身随帧倾斜)
    seg.setDistanceThreshold(params.plane_distance_threshold);
    seg.setMaxIterations(200);
    if (ground_fit->size() >= static_cast<std::size_t>(params.plane_min_inliers)) {
      seg.setInputCloud(ground_fit);
      seg.segment(*ground_inliers, *coeff);
    }

    if (!ground_inliers->indices.empty() && coeff->values.size() >= 4) {
      GroundPlaneCandidate candidate;
      const double coeff_norm = std::sqrt(
        static_cast<double>(coeff->values[0]) * coeff->values[0] +
        static_cast<double>(coeff->values[1]) * coeff->values[1] +
        static_cast<double>(coeff->values[2]) * coeff->values[2]);
      if (std::isfinite(coeff_norm) && coeff_norm >= 1e-9) {
        candidate.model.normal = {
          static_cast<double>(coeff->values[0]) / coeff_norm,
          static_cast<double>(coeff->values[1]) / coeff_norm,
          static_cast<double>(coeff->values[2]) / coeff_norm};
        candidate.model.offset = static_cast<double>(coeff->values[3]) / coeff_norm;
        if (candidate.model.normal[2] < 0.0) {
          for (double & component : candidate.model.normal) {component = -component;}
          candidate.model.offset = -candidate.model.offset;
        }
        candidate.inlier_count = ground_inliers->indices.size();
        candidate.sample_count = cloud.size();
        if (params.ground_spatial_support_enabled &&
          static_cast<double>(candidate.inlier_count) / candidate.sample_count <
          params.plane_min_inlier_ratio)
        {
          // Independently fit a much narrower plane before checking spatial
          // support. The original 0.12 m fit can include low clutter and
          // bias the floor; it cannot certify the 0.025 m support residual.
          pcl::PointIndices strict_inliers;
          pcl::ModelCoefficients strict_coeff;
          seg.setDistanceThreshold(0.02);
          seg.setMaxIterations(300);
          seg.segment(strict_inliers, strict_coeff);
          if (strict_coeff.values.size() >= 4) {
            GroundPlaneCandidate strict_candidate;
            double length = std::sqrt(
              strict_coeff.values[0] * strict_coeff.values[0] +
              strict_coeff.values[1] * strict_coeff.values[1] +
              strict_coeff.values[2] * strict_coeff.values[2]);
            if (std::isfinite(length) && length >= 1e-9) {
              if (strict_coeff.values[2] < 0.0) {length = -length;}
              strict_candidate.model.normal = {strict_coeff.values[0] / length,
                strict_coeff.values[1] / length, strict_coeff.values[2] / length};
              strict_candidate.model.offset = strict_coeff.values[3] / length;
              strict_candidate.inlier_count = strict_inliers.indices.size();
              strict_candidate.sample_count = cloud.size();
              strict_candidate.spatial_support = groundSpatialSupport(
                cloud, strict_candidate.model, vehicle);
              if (strict_candidate.spatial_support.qualified()) {
                candidate = strict_candidate;
                *ground_inliers = strict_inliers;
                *coeff = strict_coeff;
              }
            }
          }
        }
        result.candidate = candidate;
        result.has_candidate = true;
      }
    }
  }
  result.inlier_count = ground_inliers->indices.size();
  result.coefficient_count = coeff->values.size();
  return result;
}
}  // namespace easyfly::ground
