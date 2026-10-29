// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#include <easyfly_fuel_core/core.hpp>
#include <exploration_manager/fast_exploration_manager.h>
#include <exploration_manager/expl_data.h>
#include <active_perception/frontier_finder.h>
#include <active_perception/graph_node.h>
#include <path_searching/astar2.h>
#include <plan_env/sdf_map.h>
#include <filesystem>
#include <fstream>
#include <mutex>
namespace easyfly::fuel {
namespace {std::mutex singleton_mutex;bool live=false;}
struct Session::Impl {
 std::shared_ptr<fast_planner::SDFMap> map;
 std::unique_ptr<fast_planner::FastExplorationManager> manager;
 Snapshot geometry;uint64_t updates{0};Options options;
};
Session::Session():Session(Options{}){}
Session::Session(const Options& options){
 if(!options.sensor_offset.allFinite() || options.sensor_offset.norm()>1. ||
    !std::isfinite(options.horizontal_half_angle) || options.horizontal_half_angle<.1 || options.horizontal_half_angle>1.3 ||
    !std::isfinite(options.vertical_half_angle) || options.vertical_half_angle<.1 || options.vertical_half_angle>1.3 ||
    !std::isfinite(options.sensor_range) || options.sensor_range<1. || options.sensor_range>8. ||
    !std::isfinite(options.speed) || options.speed<.1 || options.speed>2.)throw std::runtime_error("FUEL view configuration");
 std::lock_guard<std::mutex> lock(singleton_mutex);if(live)throw std::runtime_error("FUEL singleton per process");
 impl_=std::make_unique<Impl>();impl_->options=options;live=true;
}
Session::~Session(){impl_.reset();std::lock_guard<std::mutex> lock(singleton_mutex);live=false;}
Decision Session::decide(const Snapshot& s,const Vec& p,const Vec& v,double yaw,const std::string& directory){
 if(s.size.minCoeff()<3 || s.size.maxCoeff()>256 || s.size.prod()>500000 ||
    s.raw.size()!=static_cast<size_t>(s.size.prod()) || s.inflated.size()!=s.raw.size() ||
    !s.origin.allFinite() || !s.min_bound.allFinite() || !s.max_bound.allFinite() ||
    (s.min_bound.array()>=s.max_bound.array()).any() || !p.allFinite() || !v.allFinite() ||
    !std::isfinite(yaw) || std::abs(s.resolution-.2)>1e-9)throw std::runtime_error("invalid FUEL snapshot");
 for(size_t i=0;i<s.raw.size();++i){
  if((s.raw[i]!=-1 && s.raw[i]!=0 && s.raw[i]!=100) ||
     (s.inflated[i]!=-1 && s.inflated[i]!=0 && s.inflated[i]!=100) ||
     (s.inflated[i]==0 && s.raw[i]!=0)) throw std::runtime_error("invalid occupancy pair");
 }
 if((s.min_bound.array()<s.origin.array()).any() ||
    (s.max_bound.array()>(s.origin+s.size.cast<double>()*s.resolution).array()+1e-8).any() ||
    (p.array()<s.min_bound.array()).any() || (p.array()>=s.max_bound.array()).any() ||
    v.norm()>2.) throw std::runtime_error("FUEL state/bounds");
 if(!std::filesystem::create_directory(directory))throw std::runtime_error("FUEL job must be fresh");
 if(!impl_->map){
  fuel_support::Config cfg;const auto& options=impl_->options;
  auto set=[&](const std::string& name,const auto& value){cfg.values[name]=value;};
  set("sdf_map/resolution",s.resolution);set("sdf_map/ground_height",s.origin.z());
  set("sdf_map/origin_x",s.origin.x());set("sdf_map/origin_y",s.origin.y());
  const char* axis[]={"x","y","z"};
  for(int i=0;i<3;++i){set(std::string("sdf_map/map_size_")+axis[i],s.size[i]*s.resolution);set(std::string("sdf_map/box_min_")+axis[i],s.min_bound[i]);set(std::string("sdf_map/box_max_")+axis[i],s.max_bound[i]-1e-6);}
  set("sdf_map/obstacles_inflation",0.);set("sdf_map/max_ray_length",options.sensor_range);set("sdf_map/virtual_ceil_height",5.);set("sdf_map/optimistic",false);
  set("frontier/cluster_min",8);set("frontier/cluster_size_xy",1.5);set("frontier/cluster_size_z",.7);
  set("frontier/min_candidate_dist",.35);set("frontier/min_candidate_clearance",.21);
  set("frontier/candidate_dphi",M_PI/6);set("frontier/candidate_rmin",options.camera_viewpoints?1.25:.8);set("frontier/candidate_rmax",options.camera_viewpoints?2.:1.5);set("frontier/candidate_rnum",3);
  set("frontier/down_sample",2);set("frontier/min_visib_num",3);set("frontier/min_view_finish_fraction",.3);
  set("perception_utils/physical_ray_visibility",true);set("perception_utils/sensor_offset_z",options.sensor_offset.z());
  set("perception_utils/sensor_offset_x",options.sensor_offset.x());set("perception_utils/sensor_offset_y",options.sensor_offset.y());
  set("perception_utils/horizontal_omni",!options.camera_viewpoints);set("perception_utils/min_vertical",-.12);set("perception_utils/max_vertical",.90);
  set("perception_utils/top_angle",options.vertical_half_angle);set("perception_utils/left_angle",options.horizontal_half_angle);set("perception_utils/right_angle",options.horizontal_half_angle);set("perception_utils/max_dist",options.sensor_range);set("perception_utils/vis_dist",1.);
  set("exploration/refine_local",true);set("exploration/refined_num",3);set("exploration/refined_radius",4.);set("exploration/top_view_num",4);set("exploration/max_decay",.8);
  set("exploration/tsp_dir",directory);set("exploration/vm",options.speed);set("exploration/am",1.5);set("exploration/yd",1.5);set("exploration/ydd",2.);set("exploration/w_dir",.1);
  set("astar/resolution_astar",.2);set("astar/lambda_heu",1.);set("astar/max_search_time",.05);set("astar/allocate_num",100000);
  std::ofstream(directory+"/native-config.yaml")<<cfg.values;
  impl_->map=std::make_shared<fast_planner::SDFMap>();impl_->map->initMap(cfg);
  impl_->manager=std::make_unique<fast_planner::FastExplorationManager>();impl_->manager->initialize(cfg,impl_->map);impl_->geometry=s;
 }else if(s.size!=impl_->geometry.size || (s.origin-impl_->geometry.origin).norm()>1e-9 ||
          (s.min_bound-impl_->geometry.min_bound).norm()>1e-9 || (s.max_bound-impl_->geometry.max_bound).norm()>1e-9)
   throw std::runtime_error("FUEL geometry/policy changed within session");
 auto& m=*impl_->manager;
 m.ep_->tsp_dir_=directory;
 {std::ofstream par(directory+"/single.par");par<<"PROBLEM_FILE = "<<directory<<"/single.tsp\nGAIN23 = NO\nOUTPUT_TOUR_FILE = "<<directory<<"/single.txt\nRUNS = 1\n";}
 impl_->map->importSnapshot(s.raw,s.inflated);
 Decision out;out.updates=++impl_->updates;
 int code=m.planExploreMotion(p,v,Vec::Zero(),Vec{yaw,0.,0.});
 out.frontiers=m.ed_->frontiers_;out.dormant=m.ed_->dead_frontiers_;out.viewpoints=m.ed_->points_;out.global_tour=m.ed_->global_tour_;out.refined_tour=m.ed_->refined_tour_;
 if(code==fast_planner::NO_FRONTIER){out.reason=out.dormant.empty()?"NO_FRONTIER":"NO_REACHABLE_FRONTIER";return out;}
 if(code!=fast_planner::SUCCEED)throw std::runtime_error("native FUEL planning failed");
 out.goal=m.ed_->next_goal_;out.yaw=m.next_yaw_;
 if(!out.goal.allFinite() || !std::isfinite(out.yaw) || !impl_->map->isInBox(out.goal) ||
    impl_->map->getOccupancy(out.goal)!=fast_planner::SDFMap::FREE || impl_->map->getInflateOccupancy(out.goal)!=0)
    throw std::runtime_error("native FUEL selected unsafe viewpoint");
 // The upstream 1000-cost straight-line estimate is never an executable path.
 auto& astar=*fast_planner::ViewNode::astar_;astar.reset();astar.setResolution(s.resolution);astar.lambda_heu_=1.;
 if(astar.search(p,out.goal)!=fast_planner::Astar::REACH_END){out.reason="SELECTED_FRONTIER_UNREACHABLE";return out;}
 out.native_path=astar.getPath();
 if(impl_->options.camera_viewpoints){
  for(const auto& cluster:out.frontiers){
   const auto cells=m.frontier_finder_->visibleCells(out.goal,out.yaw,cluster);
   for(const auto& cell:cells){if(out.visible_frontier.size()>=256)break;out.visible_frontier.push_back(cell);}
   if(out.visible_frontier.size()>=256)break;
  }
  if(out.visible_frontier.size()<4)throw std::runtime_error("FUEL selected view has insufficient camera-visible frontier");
 }
 out.reason="NEXT_VIEWPOINT";return out;
}
}
