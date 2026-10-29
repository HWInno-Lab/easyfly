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
//
//
#include <cmath>
#include <iostream>
#include "easyfly_utils/geometry.hpp"
int main()
{
  const auto up = easyfly_utils::gps_to_local_enu(30, 120, 110, 30, 120, 100);
  const auto down = easyfly_utils::gps_to_local_enu(30, 120, 90, 30, 120, 100);
  const auto horizontal = easyfly_utils::gps_to_local_enu(30.001, 120.001, 100, 30, 120, 100);
  const auto ned = easyfly_utils::enu_to_ned(up);
  if (std::abs(up.z - 10) > 1e-9 || std::abs(down.z + 10) > 1e-9 ||
    up.x != 0 || up.y != 0 || horizontal.x <= 0 || horizontal.y <= 0 ||
    horizontal.z != 0 || std::abs(ned.z + 10) > 1e-9)
  {
    std::cerr << "GPS local ENU must be east/north/up; NED must be down-positive\n";
    return 1;
  }
  return 0;
}
