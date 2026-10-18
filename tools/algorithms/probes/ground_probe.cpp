// Copyright 2026 EasyFly Project
// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#ifdef EASYFLY_LEGACY_GROUND
#include <pcl/segmentation/sac_segmentation.h>
#include "easyfly_semantic_map/ground_candidate_selection.hpp"
#include "easyfly_semantic_map/ground_plane_tracker.hpp"
#include "easyfly_semantic_map/ground_spatial_support.hpp"
#include "easyfly_semantic_map/ground_plane_transform.hpp"
#include "legacy_ground_fitting.hpp"
namespace g = easyfly_semantic_map;
#else
#include "easyfly_ground_segmentation/ground_fitting.hpp"
#include "easyfly_ground_segmentation/ground_candidate_selection.hpp"
#include "easyfly_ground_segmentation/ground_spatial_support.hpp"
#include "easyfly_ground_segmentation/ground_plane_transform.hpp"
namespace g = easyfly::ground;
#endif
void printModel(const g::GroundPlaneModel & model)
{
  for (double n:model.normal)std::cout<<' '<<n;
  std::cout<<' '<<model.offset;
}
int main()
{
  std::cout<<std::setprecision(17);
  g::GroundPlaneTracker::Params tracking;tracking.spatial_support_enabled=true;
  g::GroundPlaneTracker tracker(tracking);
  for(int frame=0;frame<32;++frame){
    pcl::PointCloud<pcl::PointXYZI> cloud;
    const bool sparse=frame%4==1;
    const bool empty=frame%8==7;
    const float floor=frame<16?0.f:.03f;
    if(!empty){
      for(int x=-30;x<=30;++x)for(int y=-30;y<=30;++y){
        if(frame%8==6 && x<0)continue;
        pcl::PointXYZI point;point.x=x*.1f;point.y=y*.1f;point.z=floor+.006f*std::sin(x+y);point.intensity=1;
        cloud.push_back(point);
      }
      const auto count=cloud.size();
      for(std::size_t i=0;i<count*(sparse?12:1);++i){
        pcl::PointXYZI point;point.x=(i%61)*.1f-3;point.y=(i/61%61)*.1f-3;point.z=3;point.intensity=2;
        cloud.push_back(point);
      }
      for(int i=0;i<100;++i){pcl::PointXYZI p;p.x=(i%10)*.05f+.5f;p.y=(i/10)*.05f+.5f;p.z=.2f;p.intensity=3;cloud.push_back(p);}
    }
    g::GroundVehiclePose vehicle;vehicle.connected=vehicle.pose_valid=true;
    vehicle.armed=frame>=8&&frame<20;vehicle.state_age_s=frame==21?2.:0.;vehicle.z=.084;
    if(frame==24)tracker.reset();
    g::GroundFitParams params;params.ground_spatial_support_enabled=true;
    const auto fit=g::fitGroundCandidate(cloud,vehicle,params);
    std::cout<<"fit "<<frame<<' '<<fit.has_candidate<<' '<<fit.inlier_count<<' '<<fit.coefficient_count;
    auto update=g::GroundPlaneUpdate::kRejected;
    if(fit.has_candidate){
      printModel(fit.candidate.model);
      const auto & support=fit.candidate.spatial_support;
      std::cout<<' '<<fit.candidate.inlier_count<<' '<<fit.candidate.sample_count<<' '<<support.points<<' '<<support.cells<<' '<<support.eligible_cells;
      for(auto count:support.sector_cells)std::cout<<' '<<count;
      update=tracker.update(fit.candidate,vehicle);
    }
    std::cout<<' '<<static_cast<int>(update)<<' '<<tracker.initialized();printModel(tracker.model());std::cout<<'\n';
    const auto basis=Eigen::Translation3d(.1,-.2,.3)*Eigen::AngleAxisd(.06,Eigen::Vector3d::UnitY());
    const auto world=g::groundPlaneInWorld(tracker.model(),Eigen::Isometry3d(basis));
    std::cout<<"world "<<frame;printModel(world);
    std::cout<<' '<<g::groundBasisContinuous(Eigen::Isometry3d::Identity(),Eigen::Isometry3d(basis))<<' '<<g::groundBasisStampFresh(10,9.9,1)<<'\n';
  }
  g::GroundPlaneTracker stateful;
  for(int step=0;step<240;++step){
    if(step%41==0)stateful.reset();
    g::GroundVehiclePose v;v.connected=step%17!=0;v.pose_valid=step%19!=0;
    v.armed=step%41>=12&&step%41<30;v.state_age_s=step%23==0?2.:0.;v.z=1.;
    g::GroundPlaneCandidate candidate;candidate.inlier_count=800;candidate.sample_count=1000;
    candidate.model.offset=step%9==0?-.98:-(step%5)*.01;
    if(step%31==0)candidate.model.normal[0]=NAN;
    const auto status=stateful.update(candidate,v);
    std::cout<<"state "<<step<<' '<<static_cast<int>(status)<<' '<<stateful.initialized();printModel(stateful.model());
    std::cout<<' '<<stateful.isGroundPoint(1,2,0,.12)<<' '<<stateful.isGroundPoint(1,2,.2,.12)<<' '<<stateful.heightAboveGround(1,2,1)<<'\n';
  }
}
