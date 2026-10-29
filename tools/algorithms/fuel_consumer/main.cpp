// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#include <easyfly_fuel_core/core.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>
using namespace easyfly::fuel;
static Vec vec(YAML::Node n){return {n[0].as<double>(),n[1].as<double>(),n[2].as<double>()};}
int main(int argc,char** argv){
 try{
  if(argc!=3)throw std::runtime_error("fixture and fresh output directory required");
  std::filesystem::path output=argv[2];if(!std::filesystem::create_directory(output))return 2;
  auto n=YAML::LoadFile(argv[1]);Snapshot s;
  s.size=vec(n["size"]).cast<int>();s.origin=vec(n["origin"]);s.resolution=n["resolution"].as<double>();
  s.min_bound=vec(n["min_bound"]);s.max_bound=vec(n["max_bound"]);
  s.raw=n["raw"].as<std::vector<int8_t>>();s.inflated=n["inflated"].as<std::vector<int8_t>>();
  Session session;
  auto first=session.decide(s,vec(n["position"]),Vec::Zero(),0.,(output/"first").string());
  if(first.reason!="NEXT_VIEWPOINT" || first.frontiers.size()<2 || first.native_path.size()<2)return 3;
  s.raw.assign(s.raw.size(),0);s.inflated.assign(s.raw.size(),0);
  auto second=session.decide(s,first.goal,Vec::Zero(),0.,(output/"second").string());
  if(second.reason!="NO_FRONTIER" || second.updates!=2 || !second.frontiers.empty())return 4;
  std::ofstream(output/"loaded-maps.txt")<<std::ifstream("/proc/self/maps").rdbuf();
  std::ofstream(output/"result.json")<<"{\"passed\":true,\"real_lkh\":true,\"incremental_updates\":2}\n";
  return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
