// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#include <easyfly_fuel_core/core.hpp>
#include <easyfly_trajectory_optimizer/observation_grid.hpp>
#include <yaml-cpp/yaml.h>
#include <sys/resource.h>
#include <unistd.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <chrono>

using namespace easyfly::fuel;
static Vec vec(const YAML::Node& n){
  if(!n.IsSequence() || n.size()!=3)throw std::runtime_error("vector layout");
  return {n[0].as<double>(),n[1].as<double>(),n[2].as<double>()};
}
static YAML::Node node(const Vec& p){YAML::Node n;for(int i=0;i<3;++i)n.push_back(p[i]);return n;}
static YAML::Node path(const std::vector<Vec>& p){YAML::Node n(YAML::NodeType::Sequence);for(auto& v:p)n.push_back(node(v));return n;}
static YAML::Node clusters(const std::vector<std::vector<Vec>>& p){YAML::Node n(YAML::NodeType::Sequence);for(auto& v:p)n.push_back(path(v));return n;}
int main(int argc,char** argv){
 try{
  if(argc!=2 && argc!=3)throw std::runtime_error("usage: fuel_worker OWNED_DIRECTORY [CAMERA_CONFIG]");
  const auto root=std::filesystem::canonical(argv[1]);
  rlimit memory{768UL*1024*1024,768UL*1024*1024},cpu{120,120};
  if(setrlimit(RLIMIT_AS,&memory)!=0 || setrlimit(RLIMIT_CPU,&cpu)!=0)throw std::runtime_error("resource limit failed");
  std::ofstream(root/"limits.txt")<<std::ifstream("/proc/self/limits").rdbuf();
  std::ofstream(root/"loaded-maps.txt")<<std::ifstream("/proc/self/maps").rdbuf();
  Options options;
  if(argc==3){
   const auto file=std::filesystem::canonical(argv[2]);
   if(file.parent_path()!=root || file.filename()!="camera-config.yaml")throw std::runtime_error("camera config outside owned directory");
   const auto config=YAML::LoadFile(file);options.camera_viewpoints=true;
   options.sensor_offset=vec(config["sensor_offset"]);
   options.horizontal_half_angle=config["horizontal_half_angle"].as<double>();
   options.vertical_half_angle=config["vertical_half_angle"].as<double>();
   options.sensor_range=config["sensor_range"].as<double>();options.speed=config["speed"].as<double>();
  }
  Session session(options);std::string name;
  while(std::getline(std::cin,name)){
   if(name=="STOP")return 0;
   const auto input=std::filesystem::canonical(name);
   if(input.parent_path()!=root || input.extension()!=".json")throw std::runtime_error("request outside owned directory");
   auto begin=std::chrono::steady_clock::now();
   auto req=YAML::LoadFile(input);Snapshot s;
   s.size=vec(req["size"]).cast<int>();s.origin=vec(req["origin"]);
   s.min_bound=vec(req["min_bound"]);s.max_bound=vec(req["max_bound"]);s.resolution=req["resolution"].as<double>();
   s.raw=req["raw"].as<std::vector<int8_t>>();s.inflated=req["inflated"].as<std::vector<int8_t>>();
   if(options.camera_viewpoints){
    const double radius=req["inflation_radius"].as<double>();
    if(!std::isfinite(radius) || radius<.65 || radius>1.)throw std::runtime_error("body envelope radius");
    easyfly::trajectory::Grid grid;grid.size=s.size;grid.origin=s.origin;grid.resolution=s.resolution;grid.data=s.inflated;
    easyfly::trajectory::reserveUnknown(grid,radius);
    const Vec corner=grid.origin+grid.size.cast<double>()*grid.resolution;
    for(int y=0;y<grid.size.y();++y)for(int x=0;x<grid.size.x();++x){
     const double low_x=grid.origin.x()+x*grid.resolution,low_y=grid.origin.y()+y*grid.resolution;
     if(low_x<grid.origin.x()+radius || low_x+grid.resolution>corner.x()-radius ||
        low_y<grid.origin.y()+radius || low_y+grid.resolution>corner.y()-radius)
      for(int z=0;z<grid.size.z();++z)grid.data[grid.index({x,y,z})]=100;
    }
    s.inflated=std::move(grid.data);
   }
   YAML::Node out;out["request_id"]=req["request_id"].as<std::string>();
   const auto job=root/(input.stem().string()+"-native");
   const auto d=session.decide(s,vec(req["position"]),vec(req["velocity"]),req["yaw"].as<double>(),job);
   out["reason"]=d.reason;out["updates"]=d.updates;
   if(d.reason=="NEXT_VIEWPOINT"){out["goal"]=node(d.goal);out["yaw"]=d.yaw;}
   out["frontiers"]=clusters(d.frontiers);out["dormant"]=clusters(d.dormant);out["viewpoints"]=path(d.viewpoints);
   out["visible_frontier"]=path(d.visible_frontier);
   out["global_tour"]=path(d.global_tour);out["refined_tour"]=path(d.refined_tour);out["native_path"]=path(d.native_path);
   rusage usage{};if(getrusage(RUSAGE_SELF,&usage)!=0)throw std::runtime_error("getrusage failed");
   out["max_rss_kib"]=usage.ru_maxrss;
   out["cpu_total_sec"]=usage.ru_utime.tv_sec+usage.ru_utime.tv_usec/1e6+usage.ru_stime.tv_sec+usage.ru_stime.tv_usec/1e6;
   out["wall_sec"]=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
   auto result=input.string()+".result.yaml",temp=result+".tmp";
   {YAML::Emitter emitter;emitter.SetDoublePrecision(17);emitter<<out;std::ofstream file(temp);file<<emitter.c_str();if(!file)throw std::runtime_error("result write failed");}
   std::filesystem::rename(temp,result);
  }
  return 0;
 }catch(const std::bad_alloc&){std::cerr<<"FUEL_RESOURCE_EXHAUSTED"<<std::endl;return 3;}catch(const std::exception& e){std::cerr<<"FUEL_WORKER_ABORT: "<<e.what()<<std::endl;return 2;}
}
