// Copyright 2026 EasyFly Project
// SPDX-License-Identifier: Apache-2.0
#include <iomanip>
#include <iostream>
#include <limits>
#include <type_traits>
#ifdef EASYFLY_NATIVE_LEGACY
#include "easyfly_semantic_map/grid_inflation.hpp"
#include "easyfly_semantic_map/observed_grid.hpp"
#include "legacy_occupancy_grid.hpp"
namespace geometry = easyfly_semantic_map;
using Plane = easyfly_semantic_map::GroundPlaneModel;
#else
#include "easyfly_occupancy_mapping/grid_inflation.hpp"
#include "easyfly_occupancy_mapping/observed_grid.hpp"
#include "easyfly_occupancy_mapping/occupancy_grid.hpp"
namespace geometry = easyfly::occupancy;
// Test-only structural plane fixture with the protected ground model's numerical semantics.
// The installed occupancy library itself does not depend on that model/package.
struct Plane
{
  std::array<double, 3> normal{{0., 0., 1.}};
  double offset{0.};
  bool valid() const
  {
    const double norm = std::sqrt(normal[0]*normal[0] + normal[1]*normal[1] + normal[2]*normal[2]);
    return std::isfinite(offset) && std::isfinite(normal[0]) && std::isfinite(normal[1]) &&
           std::isfinite(normal[2]) && norm > std::numeric_limits<double>::epsilon() && normal[2] > 0.;
  }
  double zAt(double x, double y) const
  {
    if (!valid() || std::abs(normal[2]) < 1e-9) {return std::numeric_limits<double>::quiet_NaN();}
    return -(normal[0]*x + normal[1]*y + offset) / normal[2];
  }
};
#ifdef EASYFLY_CHECK_LEGACY
#include "easyfly_semantic_map/grid_inflation.hpp"
#include "easyfly_semantic_map/observed_grid.hpp"
static_assert(easyfly_semantic_map::kMinimumGridInflationRadius == geometry::kMinimumGridInflationRadius);
#endif
#endif

struct Point {double x, y, z;};

void dump(const std::vector<int8_t> & grid)
{
  // Injective run-length encoding of every cell, not just occupied-cell counts.
  std::size_t start = 0;
  while (start < grid.size()) {
    std::size_t end = start + 1;
    while (end < grid.size() && grid[end] == grid[start]) {++end;}
    std::cout << static_cast<int>(grid[start]) << ':' << end-start << ',';
    start = end;
  }
  std::cout << '\n';
}

int main()
{
  std::cout << std::setprecision(17);
#ifdef EASYFLY_CHECK_LEGACY
  // The old function template retains its concrete-plane pointer signature.
  using OldFunction = bool (*)(const std::vector<Point> &, std::size_t, std::size_t,
    double, double, double, double, std::vector<int8_t> &,
    const easyfly_semantic_map::GroundPlaneModel *, double, double);
  OldFunction old_function = &easyfly_semantic_map::rasterizeObservedFootprints<std::vector<Point>>;
  std::vector<int8_t> old_cells(4, 0);
  easyfly_semantic_map::GroundPlaneModel old_ground;
  if (!old_function({}, 2, 2, 1., 0., 0., .7, old_cells, &old_ground, 2.9, .05)) {return 1;}
  if (easyfly_semantic_map::normalizeGridInflationRadius(.1) != .7) {return 2;}
#endif
  for (int scenario = 0; scenario < 6; ++scenario) {
    geometry::GridPersistence history;
    Plane ground;
    if (scenario == 2) {ground.normal = {-.2, 0., 1.}; ground.offset = -10.;}
    for (int frame = 0; frame < 90; ++frame) {
      const double width = scenario == 3 && frame >= 45 ? 20.3 : 16.;
      const auto grid = geometry::makeGridGeometry(width, 12., .2);
      std::vector<int8_t> cells(static_cast<std::size_t>(grid.width)*grid.height, 0);
      std::vector<Point> points;
      std::vector<std::array<double, 6>> boxes;
      const double x = -.08*frame + 2.;
      const bool observed = frame < 10 || (frame >= 40 && frame < 55);
      if (observed) {
        if (scenario == 0 || scenario == 3) {boxes.push_back({x-.2, -.3, .5, x+.2, .3, 1.2});}
        if (scenario != 0) {
          for (int i = -20; i <= 20; ++i) {
            points.push_back({i*.1, -3., ground.zAt(i*.1, -3.)+1.});
            points.push_back({-3., i*.1, ground.zAt(-3., i*.1)+1.});
          }
          points.push_back({x, .5, ground.zAt(x, .5)+2.94});
          points.push_back({x+1., .5, ground.zAt(x+1., .5)+3.2});
          points.push_back({-8.5, 0., ground.zAt(-8.5, 0.)+1.});
          points.push_back({1e300, 1e300, 0.});
        }
      }
      if (frame == 60 && scenario == 1) {points.push_back({std::numeric_limits<double>::quiet_NaN(), 0., 1.});}
      const double radius = geometry::normalizeGridInflationRadius(scenario == 5 ? .1 : .75);
      const bool ok = geometry::rasterizeObservedFootprints(
        points, grid.width, grid.height, grid.resolution, grid.origin_x, grid.origin_y,
        radius+.05, cells, &ground, scenario == 2 ? 2.9 : 0., .05);
      // Two complete predicted sections at a subset of frames, added over observed evidence.
      if (frame >= 3 && frame < 8) {
        std::vector<Point> predicted{{100., 100., 100.}};
        for (int step = 1; step <= 2; ++step) {
          for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {predicted.push_back({x+step*.2+dx*.3, dy*.4, 1.});}
          }
        }
        geometry::appendPredictionFootprintBoxes(predicted, 18, 9, boxes);
      }
      if (ok) {
        geometry::rasterizeClusterBoxes(boxes, grid, radius, cells);
        if (scenario == 4 && frame == 12) {history.reset();}
        history.apply(cells, scenario == 5 ? frame % 3 : 20);
      } else {history.reset();}
      std::cout << "F " << scenario << ' ' << frame << ' ' << ok << ' ' << grid.width << ' '
                << grid.height << ' ' << grid.resolution << ' ' << grid.origin_x << ' ' << grid.origin_y << '\n';
      for (const auto & b : boxes) {
        std::cout << 'B'; for (double v : b) {std::cout << ' ' << v;} std::cout << '\n';
      }
      dump(cells);
    }
  }
  std::vector<int8_t> cells(4, 0);
  const std::vector<Point> point{{0., 0., 1.}};
  std::cout << "N " << geometry::rasterizeObservedFootprints(point, 2, 2, 1., 0., 0., .7, cells)
            << ' ' << geometry::rasterizeObservedFootprints(point, 2, 2, 1., 0., 0., .7, cells, nullptr, 2.9, .05)
            << ' ' << geometry::rasterizeObservedFootprints(point, 2, 2, 0., 0., 0., .7, cells) << '\n';
  return 0;
}
