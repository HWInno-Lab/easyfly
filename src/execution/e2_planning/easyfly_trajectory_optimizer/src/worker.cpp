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

#include "easyfly_trajectory_optimizer/plan.hpp"
#include <iostream>
#include <sys/resource.h>
int main(int argc, char ** argv)
{
  if (argc != 3) {return 2;}
  rlimit memory{1024ULL * 1024 * 1024, 1024ULL * 1024 * 1024}; setrlimit(RLIMIT_AS, &memory);
  rlimit cpu{3, 3}; setrlimit(RLIMIT_CPU, &cpu);
  try {
    auto request = easyfly::trajectory::readRequest(argv[1]);
    auto plan = easyfly::trajectory::optimize(
      request,
      easyfly::trajectory::Steady::now() + std::chrono::seconds(2));
    easyfly::trajectory::writePlan(argv[2], plan);
    std::cout << "GCOPTER accepted: pieces=" << plan.trajectory.getPieceNum() << " corridors=" <<
      plan.boxes.size()
              << " duration=" << plan.trajectory.getTotalDuration() << " max_speed=" <<
      plan.maximum_speed << '\n';
    return 0;
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n'; return 1;
  }
}
