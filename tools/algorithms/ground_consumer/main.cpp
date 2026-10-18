// Copyright 2026 EasyFly Project
// SPDX-License-Identifier: Apache-2.0
#include <stdexcept>
#ifdef EASYFLY_CHECK_LEGACY
#include "easyfly_semantic_map/ground_plane_transform.hpp"
#include "easyfly_semantic_map/ground_candidate_selection.hpp"
#include "easyfly_semantic_map/ground_spatial_support.hpp"
namespace g = easyfly_semantic_map;
#else
#include "easyfly_ground_segmentation/ground_plane_transform.hpp"
#include "easyfly_ground_segmentation/ground_candidate_selection.hpp"
#include "easyfly_ground_segmentation/ground_spatial_support.hpp"
namespace g = easyfly::ground;
#endif
#include "easyfly_ground_segmentation/ground_fitting.hpp"
void require(bool condition){if(!condition)throw std::runtime_error("installed ground computation failed");}
int main()
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  for(int x=-20;x<=20;++x)for(int y=-20;y<=20;++y){
    pcl::PointXYZI p;p.x=x*.1f;p.y=y*.1f;p.z=0;p.intensity=2;cloud.push_back(p);
  }
  g::GroundVehiclePose vehicle;vehicle.connected=vehicle.pose_valid=true;
  vehicle.armed=false;vehicle.state_age_s=0;vehicle.z=.054;
  const auto fit=easyfly::ground::fitGroundCandidate(cloud,vehicle);
  require(fit.has_candidate&&fit.candidate.sample_count==cloud.size());
  g::GroundPlaneTracker tracker;
  for(int i=0;i<3;++i)tracker.update(fit.candidate,vehicle);
  require(tracker.initialized()&&!tracker.isGroundPoint(.5,.5,.2,.12));
  vehicle.armed=true;
  require(tracker.update(fit.candidate,vehicle)==g::GroundPlaneUpdate::kFrozen);
  auto basis=Eigen::Isometry3d::Identity();basis.translation().z()=.3;
  require(g::groundPlaneInWorld(tracker.model(),basis).signedDistance(0,0,-.3)==0);
  require(!g::groundBasisContinuous(Eigen::Isometry3d::Identity(),basis));
  require(g::isGroundFitSample(0,1,.03));
  require(g::groundSpatialSupport(cloud,fit.candidate.model,vehicle).points>0);
}
