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

#include <easyfly_astar/astar.hpp>
#include <easyfly_trajectory_optimizer/plan.hpp>
#include <iomanip>
#include <iostream>
int main(int argc, char ** argv)
{
  if (argc != 2) {return 2;}
  namespace et = easyfly::trajectory;
  const auto r = et::readRequest(argv[1]);
  easyfly::astar::OccupancyGrid2d map;
  map.width = r.grid.size.x(); map.height = r.grid.size.y();
  map.origin_x = r.grid.origin.x(); map.origin_y = r.grid.origin.y();
  map.resolution = r.grid.resolution;
  const int z = r.grid.cell(r.start).z();
  for (int y = 0; y < map.height; ++y) {
    for (int x = 0; x < map.width; ++x) {
      map.data.push_back(r.grid.free(et::Cell{x, y, z}) ? 0 : 100);
    }
  }
  auto deadline = et::Steady::now() + std::chrono::seconds(3);
  easyfly::astar::SearchControl control; control.deadline = deadline;
  auto begin = et::Steady::now();
  const auto baseline = easyfly::astar::planPath(
    map, r.start.x(), r.start.y(), r.goal.x(),
    r.goal.y(), 0, control);
  double length = 0., travelled = 0.;
  for (size_t i = 1; i < baseline.waypoints.size(); ++i) {
    length += std::hypot(
      baseline.waypoints[i].first - baseline.waypoints[i - 1].first,
      baseline.waypoints[i].second - baseline.waypoints[i - 1].second);
  }
  bool safe = baseline.ok;
  et::Plan p; p.cost = 0.;
  for (size_t i = 1; i < baseline.waypoints.size(); ++i) {
    et::Vec a{baseline.waypoints[i - 1].first, baseline.waypoints[i - 1].second, r.start.z()};
    et::Vec b{baseline.waypoints[i].first, baseline.waypoints[i].second, r.start.z()};
    a.z() += (r.goal.z() - r.start.z()) * travelled / std::max(length, 1e-9);
    travelled += (b.head<2>() - a.head<2>()).norm();
    b.z() = i + 1 == baseline.waypoints.size() ? r.goal.z() :
      r.start.z() + (r.goal.z() - r.start.z()) * travelled / std::max(length, 1e-9);
    const et::Vec d = b - a, v = i == 1 ? r.velocity : et::Vec::Zero();
    // Standard quintic Hermite timing with conservative allocation; boundary
    // velocity is the exact same measured velocity as the GCOPTER request.
    const double t = std::max(2., 4. * d.norm() / r.max_speed);
    Eigen::Matrix<double, 3, 6> c;
    c.col(5) = a; c.col(4) = v; c.col(3).setZero();
    c.col(2) = 10. * d / std::pow(t, 3) - 6. * v / std::pow(t, 2);
    c.col(1) = -15. * d / std::pow(t, 4) + 8. * v / std::pow(t, 3);
    c.col(0) = 6. * d / std::pow(t, 5) - 3. * v / std::pow(t, 4);
    p.trajectory.emplace_back(t, c);
  }
  const double elapsed =
    std::chrono::duration<double, std::milli>(et::Steady::now() - begin).count();
  std::string reason;
  if (safe) {
    try {
      et::validate(r, p, r.grid, et::Steady::now() + std::chrono::seconds(1));
    } catch (const std::exception & e) {
      safe = false; reason = e.what();
    }
  }
  std::cout << std::setprecision(12) << "{\"astar_geometric_success\":" <<
    (baseline.ok ? "true" : "false")
            << ",\"astar_fail_code\":" << int(baseline.fail) << ",\"astar_search_ms\":" << elapsed
            << ",\"astar_expansions\":" << baseline.expansions << ",\"astar_length_m\":" << length
            << ",\"astar_timed_continuous_validation\":" << (safe ? "true" : "false")
            << ",\"astar_timing_reason\":\"" << reason << "\",\"astar_duration_s\":" <<
    p.trajectory.getTotalDuration()
            << ",\"astar_max_speed\":" << p.maximum_speed
            << ",\"astar_max_acceleration\":" << p.maximum_acceleration
            << ",\"astar_max_jerk\":" << p.maximum_jerk << "}\n";
}
