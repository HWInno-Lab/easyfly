// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_utils/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
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
