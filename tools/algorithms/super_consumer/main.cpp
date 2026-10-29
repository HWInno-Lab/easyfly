// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#include <easyfly_super_planner/plan.hpp>
#include <fstream>
#include <iostream>
#include <unistd.h>

int main(int argc, char** argv) {
  if (argc != 3) return 2;
  try {
    namespace es = easyfly::superapi;
    const auto request = es::readRequest(argv[1]);
    const auto plan = es::optimize(request, es::Steady::now()+std::chrono::seconds(2));
    es::writePlan(argv[2], plan);
    std::ifstream maps("/proc/self/maps");
    std::string line;
    while(std::getline(maps,line)) {
      if(line.find("libsuper_")!=std::string::npos ||
         line.find("libeasyfly_super_planner.so")!=std::string::npos ||
         line.find("libeasyfly_rog_map.so")!=std::string::npos)
        std::cout << "LOADED " << line << '\n';
    }
    std::cout << "CONSUMER_PASS pid=" << getpid() << " pieces="
              << plan.trajectory.getPieceNum() << " duration="
              << plan.trajectory.getTotalDuration() << '\n';
    return 0;
  } catch(const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
