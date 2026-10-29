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

#include "easyfly_super_planner/plan.hpp"
#include <iostream>
#include <sys/resource.h>
#include <filesystem>
int main(int argc, char ** argv)
{
  if (argc != 3) {return 2;}
  rlimit memory{1024ULL * 1024 * 1024, 1024ULL * 1024 * 1024}; setrlimit(RLIMIT_AS, &memory);
  rlimit cpu{3, 3}; setrlimit(RLIMIT_CPU, &cpu);
  try {
    auto request = easyfly::superapi::readRequest(argv[1]);
    const auto native=std::filesystem::path(argv[1]).string()+".native";
    if(!std::filesystem::create_directory(native)) throw std::runtime_error("native job directory exists");
    std::filesystem::current_path(native);
    auto plan = easyfly::superapi::optimize(
      request,
      easyfly::superapi::Steady::now() + std::chrono::seconds(2));
    easyfly::superapi::writePlan(argv[2], plan);
    std::cout << "SUPER accepted: pieces=" << plan.trajectory.getPieceNum() << " corridors=" <<
      plan.boxes.size()
              << " duration=" << plan.trajectory.getTotalDuration() << " max_speed=" <<
      plan.maximum_speed << '\n';
    rusage usage{};getrusage(RUSAGE_SELF,&usage);
    std::cout << "worker_resource cpu_seconds="
              << usage.ru_utime.tv_sec+usage.ru_stime.tv_sec+
      (usage.ru_utime.tv_usec+usage.ru_stime.tv_usec)*1e-6
              << " peak_rss_kib=" << usage.ru_maxrss << '\n';
    return 0;
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n'; return 1;
  }
}
