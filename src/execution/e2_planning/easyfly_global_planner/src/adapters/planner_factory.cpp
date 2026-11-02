// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#include "easyfly_global_planner/adapters/planner_factory.hpp"
namespace easyfly_global_planner {
void register_astar(PlannerFactory & factory);
const PlannerFactory & planner_factory() {
  static const PlannerFactory factory = [] {
    PlannerFactory result;
    register_astar(result);
    return result;
  }();
  return factory;
}
}
