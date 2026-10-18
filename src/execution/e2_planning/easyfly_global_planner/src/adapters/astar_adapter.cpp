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

#include <algorithm>
#include <limits>
#include <stdexcept>
#include "easyfly_astar/astar.hpp"
#include "easyfly_global_planner/adapters/planner_factory.hpp"

namespace easyfly_global_planner
{
namespace gp = easyfly::grid_planning;
class AStarAdapter final : public gp::GridPlanner
{
public:
  explicit AStarAdapter(AStarParams params) : params_(params)
  {
    if (!params_.max_cells || params_.max_cells > static_cast<std::size_t>(
        std::numeric_limits<int>::max()) || params_.unstick_cells < 0 ||
      params_.unstick_cells > 256)
    {
      throw std::invalid_argument("invalid A* parameters");
    }
  }
  gp::Capabilities capabilities() const override {return {"astar", false, false, false};}
  gp::PathResult plan(const gp::Request & request, const gp::SearchControl & control) override
  {
    gp::PathResult result;
    result.map = request.map;
    result.status = gp::validate_layout(request.grid, params_.max_cells);
    if (result.status != gp::Status::ok) {return result;}
    if (!gp::valid_point(request.start) || !gp::valid_point(request.goal)) {
      result.status = gp::Status::invalid_input;
      return result;
    }
    result.status = gp::validate_control(control, capabilities());
    if (result.status != gp::Status::ok) {return result;}
    const auto & g = request.grid;
    const auto inside = [&](gp::Point point) {
        return point.x >= g.origin_x && point.x < g.origin_x + g.width * g.resolution &&
               point.y >= g.origin_y && point.y < g.origin_y + g.height * g.resolution;
      };
    if (!inside(request.start) || !inside(request.goal)) {
      result.status = gp::Status::out_of_bounds;
      return result;
    }
    easyfly::astar::OccupancyGrid2d core;
    core.width = static_cast<int>(g.width);
    core.height = static_cast<int>(g.height);
    core.resolution = g.resolution;
    core.origin_x = g.origin_x;
    core.origin_y = g.origin_y;
    core.data.reserve(g.cell_count);
    for (std::size_t i = 0; i < g.cell_count; ++i) {
      core.data.push_back(gp::occupied_value(g.cells[i], g.occupied_threshold,
          g.unknown_policy) ? 100 : 0);
    }
    const auto output = easyfly::astar::planPath(core, request.start.x, request.start.y,
        request.goal.x, request.goal.y, params_.unstick_cells);
    using Fail = easyfly::astar::AStarResult::Fail;
    switch (output.fail) {
      case Fail::NONE: result.status = output.ok ? gp::Status::ok : gp::Status::backend_error; break;
      case Fail::START_IN_OBSTACLE: result.status = gp::Status::start_in_obstacle; break;
      case Fail::GOAL_IN_OBSTACLE: result.status = gp::Status::goal_in_obstacle; break;
      case Fail::NO_PATH: result.status = gp::Status::no_path; break;
      case Fail::OUT_OF_BOUNDS: result.status = gp::Status::out_of_bounds; break;
    }
    if (result.status == gp::Status::ok) {
      for (const auto & point : output.waypoints) {
        result.waypoints.push_back({point.first, point.second});
      }
    }
    return result;
  }
private:
  AStarParams params_;
};
std::unique_ptr<gp::GridPlanner> make_planner(const std::string & backend, const AStarParams & params)
{
  if (backend == "astar") {return std::make_unique<AStarAdapter>(params);}
  throw std::invalid_argument("unknown grid planner backend '" + backend + "' (available: astar)");
}
}  // namespace easyfly_global_planner
