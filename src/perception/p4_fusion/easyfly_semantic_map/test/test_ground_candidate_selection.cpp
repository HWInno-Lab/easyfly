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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <gtest/gtest.h>

#include <limits>

#include "easyfly_semantic_map/ground_candidate_selection.hpp"
#include "easyfly_semantic_map/ground_plane_tracker.hpp"

namespace easyfly_semantic_map
{
namespace
{

GroundPlaneCandidate fitIndoorFrame(std::size_t floor_points, std::size_t ceiling_points)
{
  auto original = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (std::size_t i = 0; i < floor_points + ceiling_points; ++i) {
    original->push_back(
      pcl::PointXYZ(
        static_cast<float>(i % 25) * 0.2f,
        static_cast<float>(i / 25) * 0.2f,
        i < floor_points ? 0.0f : 3.0f));
  }
  auto selected = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (const auto & point : original->points) {
    if (isGroundFitSample(point.z, 1.0, 0.03)) {selected->push_back(point);}
  }
  GroundPlaneCandidate candidate;
  candidate.sample_count = original->size();
  if (selected->size() < 50) {return candidate;}
  pcl::SACSegmentation<pcl::PointXYZ> segmentation;
  segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setAxis(Eigen::Vector3f(0, 0, 1));
  segmentation.setEpsAngle(0.1);
  segmentation.setDistanceThreshold(0.12);
  segmentation.setInputCloud(selected);
  pcl::PointIndices inliers;
  pcl::ModelCoefficients coefficients;
  segmentation.segment(inliers, coefficients);
  candidate.inlier_count = inliers.indices.size();
  if (coefficients.values.size() == 4) {
    candidate.model.normal =
    {coefficients.values[0], coefficients.values[1], coefficients.values[2]};
    candidate.model.offset = coefficients.values[3];
    if (candidate.model.normal[2] < 0) {
      for (auto & component : candidate.model.normal) {
        component = -component;
      }
      candidate.model.offset = -candidate.model.offset;
    }
  }
  return candidate;
}

TEST(GroundCandidateSelection, DominantCeilingDoesNotHideConfidentFloor)
{
  const auto candidate = fitIndoorFrame(200, 800);
  EXPECT_EQ(candidate.inlier_count, 200u);
  EXPECT_EQ(candidate.sample_count, 1000u);
  EXPECT_NEAR(candidate.model.zAt(0, 0), 0.0, 1e-6);
}

TEST(GroundCandidateSelection, SparseFloorKeepsFullFrameConfidenceDenominator)
{
  const auto candidate = fitIndoorFrame(50, 950);
  EXPECT_EQ(candidate.inlier_count, 50u);
  EXPECT_DOUBLE_EQ(static_cast<double>(candidate.inlier_count) / candidate.sample_count, 0.05);
  GroundPlaneTracker::Params params;
  params.min_inlier_ratio = 0.15;
  GroundPlaneTracker tracker(params);
  GroundVehiclePose vehicle;
  vehicle.x = 0; vehicle.y = 0; vehicle.z = 1;
  vehicle.connected = true; vehicle.pose_valid = true; vehicle.state_age_s = 0;
  for (int i = 0; i < 5; ++i) {
    tracker.update(candidate, vehicle);
  }
  EXPECT_FALSE(tracker.initialized());
}

TEST(GroundCandidateSelection, CeilingOnlyDoesNotInventGround)
{
  const auto candidate = fitIndoorFrame(0, 1000);
  EXPECT_EQ(candidate.inlier_count, 0u);
  EXPECT_EQ(candidate.sample_count, 1000u);
}

TEST(GroundCandidateSelection, PhysicalClearanceAndFiniteInputsRemainRequired)
{
  EXPECT_FALSE(isGroundFitSample(1.0, 1.0, 0.03));
  EXPECT_FALSE(isGroundFitSample(2.0, 1.0, 0.03));
  EXPECT_TRUE(isGroundFitSample(0.0, 1.0, 0.03));
  EXPECT_FALSE(isGroundFitSample(std::numeric_limits<double>::quiet_NaN(), 1.0, 0.03));
}

}  // namespace
}  // namespace easyfly_semantic_map
