// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#pragma once
#include "easyfly_backend_registry/configuration.hpp"
#include "easyfly_grid_planner_api/planner.hpp"
namespace easyfly_global_planner {
using PlannerFactory = easyfly::backends::Factory<easyfly::grid_planning::GridPlanner>;
const PlannerFactory & planner_factory();
inline std::unique_ptr<easyfly::grid_planning::GridPlanner> make_planner(
  const easyfly::backends::BackendConfig & config, const PlannerFactory & factory = planner_factory()) {
  auto result = factory.create(config);
  const auto caps = result->capabilities();
  if (caps.backend != config.backend || !caps.cancellation || !caps.deadline || !caps.expansion_limit)
    throw std::invalid_argument("grid backend must match selection and support cancellation/deadline/expansion limit");
  return result;
}
inline std::unique_ptr<easyfly::grid_planning::GridPlanner> make_planner(
  const std::string & backend, const easyfly::backends::Parameters & parameters = {}) {
  return make_planner(easyfly::backends::BackendConfig{backend, parameters});
}
}  // namespace easyfly_global_planner
