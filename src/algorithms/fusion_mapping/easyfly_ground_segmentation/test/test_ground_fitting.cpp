// Copyright 2026 EasyFly Project
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "easyfly_ground_segmentation/ground_fitting.hpp"
namespace g = easyfly::ground;
pcl::PointCloud<pcl::PointXYZI> room(bool sparse = false)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  for (int x=-30;x<=30;++x)for(int y=-30;y<=30;++y){
    pcl::PointXYZI p;p.x=x*.1f;p.y=y*.1f;p.z=0;p.intensity=7;
    cloud.push_back(p);
  }
  const auto floor=cloud.size();
  for(std::size_t i=0;i<(sparse?12:1)*floor;++i){
    pcl::PointXYZI p;p.x=(i%61)*.1f-3;p.y=(i/61%61)*.1f-3;p.z=3;p.intensity=9;
    cloud.push_back(p);
  }
  return cloud;
}
g::GroundVehiclePose vehicle()
{
  g::GroundVehiclePose v;v.connected=v.pose_valid=true;v.armed=false;v.state_age_s=0;v.z=.054;
  return v;
}
TEST(GroundFitting, ActualPclCandidateKeepsFullFrameDenominatorAndInput)
{
  const auto cloud=room();const auto before=cloud;
  const auto result=g::fitGroundCandidate(cloud,vehicle());
  ASSERT_TRUE(result.has_candidate);EXPECT_EQ(result.candidate.sample_count,cloud.size());
  EXPECT_EQ(result.inlier_count,3721u);EXPECT_EQ(result.coefficient_count,4u);
  EXPECT_NEAR(result.candidate.model.zAt(0,0),0,1e-6);
  ASSERT_EQ(cloud.size(),before.size());
  for(std::size_t i=0;i<cloud.size();++i){
    EXPECT_EQ(cloud[i].x,before[i].x);EXPECT_EQ(cloud[i].z,before[i].z);EXPECT_EQ(cloud[i].intensity,before[i].intensity);
  }
}
TEST(GroundFitting, StrictSpatialFitRequiresExplicitPolicyAndFiveFrames)
{
  const auto cloud=room(true);g::GroundFitParams params;params.ground_spatial_support_enabled=true;
  const auto result=g::fitGroundCandidate(cloud,vehicle(),params);
  ASSERT_TRUE(result.has_candidate);ASSERT_TRUE(result.candidate.spatial_support.qualified());
  g::GroundPlaneTracker legacy;EXPECT_EQ(legacy.update(result.candidate,vehicle()),g::GroundPlaneUpdate::kRejected);
  g::GroundPlaneTracker::Params tracking;tracking.spatial_support_enabled=true;
  g::GroundPlaneTracker tracker(tracking);
  for(int i=0;i<4;++i)EXPECT_EQ(tracker.update(result.candidate,vehicle()),g::GroundPlaneUpdate::kPending);
  EXPECT_EQ(tracker.update(result.candidate,vehicle()),g::GroundPlaneUpdate::kInitialized);
  auto armed=vehicle();armed.armed=true;
  EXPECT_EQ(tracker.update(result.candidate,armed),g::GroundPlaneUpdate::kFrozen);
}
TEST(GroundFitting, CeilingOnlyAndSparseFramesCannotInventGround)
{
  auto cloud=room();for(auto & point:cloud)point.z=3;
  EXPECT_FALSE(g::fitGroundCandidate(cloud,vehicle()).has_candidate);
  cloud.resize(99);for(auto & point:cloud)point.z=0;
  EXPECT_FALSE(g::fitGroundCandidate(cloud,vehicle()).has_candidate);
}
