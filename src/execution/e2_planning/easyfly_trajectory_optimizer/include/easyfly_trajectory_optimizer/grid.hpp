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

#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <stdexcept>
#include <vector>

namespace easyfly::trajectory
{
using Vec = Eigen::Vector3d;
using Cell = Eigen::Vector3i;
using Steady = std::chrono::steady_clock;
struct Grid
{
  Cell size{0, 0, 0};
  Vec origin{0, 0, 0};
  double resolution{.2};
  std::vector<int8_t> data;
  bool valid() const
  {
    return size.minCoeff() > 0 && size.maxCoeff() <= 512 && origin.allFinite() &&
           origin.cwiseAbs().maxCoeff() <= 10000 && std::isfinite(resolution) &&
           resolution >= .05 && resolution <= 1. &&
           static_cast<int64_t>(size.x()) * size.y() * size.z() <= 1000000 &&
           data.size() == static_cast<size_t>(size.x()) * size.y() * size.z() &&
           std::all_of(
      data.begin(), data.end(),
      [](auto v) {return v == -1 || v == 0 || v == 100;});
  }
  bool inside(const Cell & c) const
  {
    return (c.array() >= 0).all() && (c.array() < size.array()).all();
  }
  int index(const Cell & c) const {return c.x() + size.x() * (c.y() + size.y() * c.z());}
  Cell cell(int i) const
  {
    return {i % size.x(), (i / size.x()) % size.y(), i / (size.x() * size.y())};
  }
  Cell cell(const Vec & p) const
  {
    if (!p.allFinite() || (p.array() < origin.array()).any() ||
      (p.array() >= (origin + size.cast<double>() * resolution).array()).any())
    {
      return Cell{-1, -1, -1};
    }
    return ((p - origin) / resolution).array().floor().cast<int>();
  }
  Vec center(const Cell & c) const
  {
    return origin + (c.cast<double>().array() + .5).matrix() * resolution;
  }
  bool free(const Cell & c) const {return inside(c) && data[index(c)] == 0;}
  bool free(const Vec & p) const {return p.allFinite() && free(cell(p));}
  bool freeBox(const Cell & lo, const Cell & hi) const
  {
    if (!inside(lo) || !inside(hi) || (lo.array() > hi.array()).any()) {return false;}
    for (int z = lo.z(); z <= hi.z(); ++z) {
      for (int y = lo.y(); y <= hi.y(); ++y) {
        for (int x = lo.x(); x <= hi.x(); ++x) {
          if (!free(Cell{x, y, z})) {return false;}}
      }
    }
    return true;
  }
  bool freeBox(const Vec & lo, const Vec & hi) const
  {
    return lo.allFinite() && hi.allFinite() && freeBox(cell(lo), cell(hi));
  }
};
inline void budget(Steady::time_point deadline)
{
  if (Steady::now() >= deadline) {throw std::runtime_error("planning_deadline");}
}

// 26-connected A*: the complete neighbour bounding box must be free, including
// unknown cells. This prevents diagonal edge/vertex corner cutting.
inline std::vector<Vec> search(
  const Grid & g, const Vec & start, const Vec & goal,
  Steady::time_point deadline, size_t expansion_limit = 500000)
{
  if (!g.valid() || !g.free(start) || !g.free(goal)) {
    throw std::runtime_error("invalid_or_occupied_endpoint");
  }
  const int a = g.index(g.cell(start)), b = g.index(g.cell(goal));
  using Entry = std::pair<double, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
  std::vector<double> costs(g.data.size(), std::numeric_limits<double>::infinity());
  std::vector<int> parent(g.data.size(), -1);
  std::vector<bool> closed(g.data.size(), false);
  costs[a] = 0; queue.emplace(0, a);
  size_t expansions = 0;
  while (!queue.empty()) {
    budget(deadline);
    const int i = queue.top().second; queue.pop();
    if (closed[i]) {continue;}
    closed[i] = true;
    if (++expansions > expansion_limit) {throw std::runtime_error("expansion_limit");}
    if (i == b) {
      std::vector<Vec> path;
      for (int j = b; j >= 0; j = parent[j]) {
        path.push_back(g.center(g.cell(j)));
      }
      std::reverse(path.begin(), path.end());
      path.front() = start;
      if ((path.back() - goal).norm() > 1e-9 || path.size() == 1) {path.push_back(goal);} else {
        path.back() = goal;
      }
      return path;
    }
    const Cell c = g.cell(i);
    for (int z = -1; z <= 1; ++z) {
      for (int y = -1; y <= 1; ++y) {
        for (int x = -1; x <= 1; ++x) {
          if (!x && !y && !z) {continue;}
          const Cell n = c + Cell{x, y, z};
          if (!g.freeBox(Cell(c.cwiseMin(n)), Cell(c.cwiseMax(n)))) {continue;}
          const int j = g.index(n);
          const double cost = costs[i] + std::sqrt(x * x + y * y + z * z) * g.resolution;
          if (cost >= costs[j]) {continue;}
          costs[j] = cost; parent[j] = i;
          queue.emplace(cost + (g.center(n) - goal).norm(), j);
        }
      }
    }
  }
  throw std::runtime_error("no_path");
}

struct Box {Vec low, high;};
inline std::vector<Box> corridors(
  const Grid & g, const std::vector<Vec> & path,
  Steady::time_point deadline)
{
  std::vector<Box> output;
  for (size_t i = 0; i + 1 < path.size(); ) {
    budget(deadline);
    Cell lo = g.cell(path[i]), hi = lo;
    size_t j = i;
    for (size_t k = i + 1; k < path.size(); ++k) {
      const auto low = lo.cwiseMin(g.cell(path[k])).eval();
      const auto high = hi.cwiseMax(g.cell(path[k])).eval();
      if (!g.freeBox(low, high)) {break;}
      lo = low; hi = high; j = k;
    }
    if (j == i) {throw std::runtime_error("corridor_disconnected");}
    // A long path box can already exceed the ordinary 3 m expansion cap.
    // Still allow one free cell beyond each endpoint: measured velocity can
    // initially point away from the goal and requires real braking room.
    const Vec expansion_limit = ((hi - lo).cast<double>().array() + 3.) * g.resolution;
    bool changed = true;
    while (changed) {
      changed = false; budget(deadline);
      for (int axis = 0; axis < 3; ++axis) {
        for (int sign : {-1, 1}) {
          Cell low = lo, high = hi;
          if (sign < 0) {--low[axis];} else {++high[axis];}
          if ((high[axis] - low[axis] + 1) * g.resolution <=
            std::max(3.0, expansion_limit[axis]) + 1e-9 && g.freeBox(low, high))
          {
            lo = low; hi = high; changed = true;
          }
        }
      }
    }
    Vec low = g.origin + lo.cast<double>() * g.resolution;
    Vec high = g.origin + (hi.cast<double>().array() + 1).matrix() * g.resolution;
    Vec path_low = path[i], path_high = path[i];
    for (size_t k = i + 1; k <= j; ++k) {
      path_low = path_low.cwiseMin(path[k]);
      path_high = path_high.cwiseMax(path[k]);
    }
    // Upstream corridor penalties are soft. Reserve physical interior room
    // before optimization; the independent hard collision test still decides
    // admission. Preserve every route point, including measured endpoints.
    const double margin = std::min(.02, .1 * g.resolution);
    for (int axis = 0; axis < 3; ++axis) {
      low[axis] += std::min(margin, .5 * std::max(0., path_low[axis] - low[axis]));
      high[axis] -= std::min(margin, .5 * std::max(0., high[axis] - path_high[axis]));
    }
    output.push_back({low, high});
    if (output.size() > 64) {throw std::runtime_error("corridor_limit");}
    i = j;
  }
  return output;
}
}  // namespace easyfly::trajectory
