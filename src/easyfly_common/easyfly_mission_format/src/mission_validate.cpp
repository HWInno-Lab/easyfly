// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_mission_format/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project

#include <iostream>
#include <iomanip>
#include <string>
#include "easyfly_mission_format/mission_format.hpp"

int main(int argc, char ** argv)
{
  const bool types = argc == 3 && std::string(argv[1]) == "--step-types";
  if (argc != 2 && !types) {
    std::cerr << "usage: mission_validate [--step-types] PATH\n"; return 2;
  }
  const auto result = easyfly_mission_fsm::validateMissionFile(argv[types ? 2 : 1]);
  if (!result.valid()) {std::cerr << result.error << '\n'; return 1;}
  if (types) {
    std::cout << '[';
    for (size_t i = 0; i < result.steps.size(); ++i) {
      if (i) {std::cout << ',';}
      std::cout << std::quoted(result.steps[i].type);
    }
    std::cout << "]\n"; return 0;
  }
  std::cout << "valid: " << result.steps.size() << " steps; live flight admission still required\n";
  return 0;
}
