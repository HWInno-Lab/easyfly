// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_trajectory_optimizer/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
//

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
