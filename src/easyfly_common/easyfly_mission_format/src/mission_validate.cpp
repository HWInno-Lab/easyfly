// Copyright 2026 EasyFly Project
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

#include <iostream>
#include "easyfly_mission_format/mission_format.hpp"

int main(int argc, char ** argv)
{
  if (argc != 2) {std::cerr << "usage: mission_validate PATH\n"; return 2;}
  const auto result = easyfly_mission_fsm::validateMissionFile(argv[1]);
  if (!result.valid()) {std::cerr << result.error << '\n'; return 1;}
  std::cout << "valid: " << result.steps.size() << " steps; live flight admission still required\n";
  return 0;
}
