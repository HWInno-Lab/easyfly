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

// EasyFly E2: 2D 栅格 A* + 视线剪枝 (M4.2, 纯逻辑 header, 无 ROS 依赖)
//
// 输入: 占据栅格 (来自 /occupancy_grid, 已含 semantic_map 的约 0.7m 安全膨胀)
// 输出: 世界坐标航点序列 (odom 系, 保留真实起点及精确目标, 中间点为格子中心)
//
// 设计:
//   - 8 连通, 移动代价 1/√2, 欧氏启发式 (可采纳, 保证最优)
//   - 起点若落在膨胀障碍内 (贴近障碍悬停), 在小半径内找最近空闲格 "脱困"
//   - 目标点在障碍内 → 直接判不可达 (拒绝语义在节点层)
//   - 视线剪枝 (贪婪 LOS shortcut): 从起点向后找最远可视点, 消除 A* 锯齿
#ifndef EASYFLY_ASTAR_ASTAR_HPP_
#define EASYFLY_ASTAR_ASTAR_HPP_

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>
#include "easyfly_grid_planner_api/planner.hpp"

namespace easyfly::astar
{

struct OccupancyGrid2d
{
  int width{0};            // 列数 (x)
  int height{0};           // 行数 (y)
  double resolution{0.2};  // [m/cell]
  double origin_x{0.0};    // 左下角世界坐标
  double origin_y{0.0};
  std::vector<int8_t> data;  // row-major: idx = row * width + col; >=50 占据

  bool occupied(int row, int col) const
  {
    if (row < 0 || row >= height || col < 0 || col >= width) {return true;}
    const auto index = static_cast<size_t>(row) * width + col;
    return index >= data.size() || data[index] < -1 || data[index] > 100 || data[index] >= 50;
  }

  bool worldOccupied(double wx, double wy) const
  {
    if (!std::isfinite(wx) || !std::isfinite(wy) || !std::isfinite(resolution) ||
      resolution <= 0) {return true;}
    const double cd = std::floor((wx - origin_x) / resolution);
    const double rd = std::floor((wy - origin_y) / resolution);
    if (!(cd >= 0 && cd < width && rd >= 0 && rd < height)) {return true;}
    const int c = static_cast<int>(cd), r = static_cast<int>(rd);
    return occupied(r, c);
  }
};

struct AStarResult
{
  bool ok{false};
  std::size_t expansions{0};
  enum class Fail { NONE, START_IN_OBSTACLE, GOAL_IN_OBSTACLE, NO_PATH,
    OUT_OF_BOUNDS, INVALID_INPUT, RESOURCE_LIMIT, CANCELED, TIMED_OUT,
    BUDGET_EXHAUSTED } fail{Fail::NONE};
  std::vector<std::pair<double, double>> waypoints;  // 世界坐标 (odom)
};

using SearchControl = easyfly::grid_planning::SearchControl;

namespace detail
{
// Cooperative interruption; no partial path escapes planPath on failure.
struct SearchContext
{
  const SearchControl & control;
  std::size_t expansions{0};
  void poll(bool before_expansion = false) const
  {
    using Fail = AStarResult::Fail;
    if (control.cancel_requested && control.cancel_requested->load(std::memory_order_relaxed)) {
      throw Fail::CANCELED;
    }
    if (control.deadline && SearchControl::Clock::now() >= *control.deadline) {
      throw Fail::TIMED_OUT;
    }
    if (before_expansion && control.expansion_limit && expansions >= *control.expansion_limit) {
      throw Fail::BUDGET_EXHAUSTED;
    }
  }
};

inline int cellCol(const OccupancyGrid2d & g, double wx)
{
  return static_cast<int>(std::floor((wx - g.origin_x) / g.resolution));
}
inline int cellRow(const OccupancyGrid2d & g, double wy)
{
  return static_cast<int>(std::floor((wy - g.origin_y) / g.resolution));
}
inline double cellCenterX(const OccupancyGrid2d & g, int col)
{
  return g.origin_x + (col + 0.5) * g.resolution;
}
inline double cellCenterY(const OccupancyGrid2d & g, int row)
{
  return g.origin_y + (row + 0.5) * g.resolution;
}

/// 螺旋找最近空闲格 (半径 limit 格内); 未找到返回 -1
inline int nearestFreeCell(
  const OccupancyGrid2d & g, int row, int col, int limit,
  SearchContext * context = nullptr)
{
  if (!g.occupied(row, col)) {return row * g.width + col;}
  for (int r = 1; r <= limit; ++r) {
    for (int dr = -r; dr <= r; ++dr) {
      for (int dc = -r; dc <= r; ++dc) {
        if (context) {context->poll();}
        if (std::max(std::abs(dr), std::abs(dc)) != r) {
          continue;                                                // 只查环上
        }
        const auto nr = static_cast<std::int64_t>(row) + dr;
        const auto nc = static_cast<std::int64_t>(col) + dc;
        if (nr >= 0 && nr < g.height && nc >= 0 && nc < g.width &&
          !g.occupied(static_cast<int>(nr), static_cast<int>(nc)))
        {
          return static_cast<int>(nr) * g.width + static_cast<int>(nc);
        }
      }
    }
  }
  return -1;
}

/// 两世界坐标连线是否无遮挡 (按半格步长采样)
inline bool lineOfSight(
  const OccupancyGrid2d & g,
  double x1, double y1, double x2, double y2,
  SearchContext * context = nullptr)
{
  const double dist = std::hypot(x2 - x1, y2 - y1);
  const auto count = dist / g.resolution * 2.0;
  if (!std::isfinite(count) ||
    count >= static_cast<double>(std::numeric_limits<std::size_t>::max()))
  {
    return false;
  }
  const auto steps = std::max<std::size_t>(1, static_cast<std::size_t>(count));
  for (std::size_t i = 1; i < steps; ++i) {
    if (context) {context->poll();}
    const double t = static_cast<double>(i) / steps;
    if (g.worldOccupied(x1 + (x2 - x1) * t, y1 + (y2 - y1) * t)) {return false;}
  }
  return true;
}

}  // namespace detail

/// A* 搜索 + 视线剪枝. 输出世界坐标航点 (含起点, 不含起点段则首元素≈起点)。
inline AStarResult planPath(
  const OccupancyGrid2d & g,
  double start_x, double start_y,
  double goal_x, double goal_y,
  int unstick_cells = 5,
  const SearchControl & control = {})
{
  AStarResult res;
  detail::SearchContext context{control};
  try {
    context.poll(true);
    // Validate before integer conversion, allocation or dereferencing cell storage.
    if (g.width <= 0 || g.height <= 0 ||
      static_cast<std::size_t>(g.width) > static_cast<std::size_t>(
        std::numeric_limits<int>::max()) / g.height ||
      g.data.size() != static_cast<std::size_t>(g.width) * g.height ||
      !std::isfinite(g.resolution) || g.resolution <= 0 ||
      !std::isfinite(g.origin_x) || !std::isfinite(g.origin_y) ||
      !std::isfinite(start_x) || !std::isfinite(start_y) ||
      !std::isfinite(goal_x) || !std::isfinite(goal_y) ||
      unstick_cells < 0 || unstick_cells > 256)
    {
      res.fail = AStarResult::Fail::INVALID_INPUT;
      return res;
    }
    if (!std::isfinite(g.origin_x + g.width * g.resolution) ||
      !std::isfinite(g.origin_y + g.height * g.resolution) ||
      g.origin_x + g.width * g.resolution <= g.origin_x ||
      g.origin_y + g.height * g.resolution <= g.origin_y)
    {
      res.fail = AStarResult::Fail::INVALID_INPUT;
      return res;
    }

    // 边界检查
    const double gx_min = g.origin_x, gy_min = g.origin_y;
    const double gx_max = g.origin_x + g.width * g.resolution;
    const double gy_max = g.origin_y + g.height * g.resolution;
    if (start_x < gx_min || start_x >= gx_max || start_y < gy_min || start_y >= gy_max ||
      goal_x < gx_min || goal_x >= gx_max || goal_y < gy_min || goal_y >= gy_max)
    {
      res.fail = AStarResult::Fail::OUT_OF_BOUNDS;
      return res;
    }

    int s_row = detail::cellRow(g, start_y), s_col = detail::cellCol(g, start_x);
    const int g_row = detail::cellRow(g, goal_y), g_col = detail::cellCol(g, goal_x);

    // 起点脱困 (贴近障碍悬停时), 目标在障碍内直接拒绝
    const int s_idx = detail::nearestFreeCell(g, s_row, s_col, unstick_cells, &context);
    if (s_idx < 0) {
      res.fail = AStarResult::Fail::START_IN_OBSTACLE;
      return res;
    }
    if (g.occupied(g_row, g_col)) {
      res.fail = AStarResult::Fail::GOAL_IN_OBSTACLE;
      return res;
    }
    const int g_idx = g_row * g.width + g_col;
    s_row = s_idx / g.width;
    s_col = s_idx % g.width;

    // ---- A* (8 连通) ----
    const int n = g.width * g.height;
    std::vector<double> g_score(n, std::numeric_limits<double>::infinity());
    std::vector<int> came_from(n, -1);
    std::vector<uint8_t> closed(n, 0);
    context.poll();
    using QE = std::pair<double, int>; // (f, idx)
    std::priority_queue<QE, std::vector<QE>, std::greater<QE>> open;

    const double h0 = std::hypot(g_col - s_col, g_row - s_row);
    g_score[s_idx] = 0.0;
    open.push({h0, s_idx});

    const int dr8[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
    const int dc8[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const double dc_cost[8] = {1.41421356, 1.0, 1.41421356, 1.0,
      1.0, 1.41421356, 1.0, 1.41421356};

    bool found = false;
    while (!open.empty()) {
      context.poll();
      const auto [f, cur] = open.top();
      open.pop();
      if (closed[cur]) {continue;}
      context.poll(true);
      ++context.expansions;
      closed[cur] = 1;
      if (cur == g_idx) {
        found = true;
        break;
      }
      const int cr = cur / g.width, cc = cur % g.width;
      for (int d = 0; d < 8; ++d) {
        const int nr = cr + dr8[d], nc = cc + dc8[d];
        if (g.occupied(nr, nc)) {continue;}
        // 禁止穿角: 斜向移动要求两个相邻直向格至少一个空闲
        if (dr8[d] != 0 && dc8[d] != 0) {
          if (g.occupied(cr + dr8[d], cc) && g.occupied(cr, cc + dc8[d])) {continue;}
        }
        const int nidx = nr * g.width + nc;
        const double tentative = g_score[cur] + dc_cost[d];
        if (tentative < g_score[nidx]) {
          g_score[nidx] = tentative;
          came_from[nidx] = cur;
          const double h = std::hypot(g_col - nc, g_row - nr);
          open.push({tentative + h, nidx});
        }
      }
    }
    if (!found) {
      res.expansions = context.expansions;
      res.fail = AStarResult::Fail::NO_PATH;
      return res;
    }

    // 回溯 (格子中心世界坐标)
    std::vector<std::pair<double, double>> raw;
    for (int idx = g_idx; idx != -1; idx = came_from[idx]) {
      context.poll();
      raw.emplace_back(
        detail::cellCenterX(g, idx % g.width),
        detail::cellCenterY(g, idx / g.width));
    }
    for (std::size_t left = 0, right = raw.size(); left < right && left < --right; ++left) {
      context.poll();
      std::swap(raw[left], raw[right]);
    }
    raw[0] = {start_x, start_y}; // 首点用真实起点 (脱困偏移不回传)
    // Keep the free goal-cell center as a safe approach, then include the exact
    // requested endpoint BEFORE LOS pruning. Do not mutate a checked shortcut
    // afterwards. The final within-cell segment stays inside the free goal cell.
    // A same-cell path still needs both endpoints (and a real altitude action).
    if (raw.size() == 1 || raw.back() != std::make_pair(goal_x, goal_y)) {
      context.poll();
      raw.emplace_back(goal_x, goal_y);
    }

    // ---- 视线剪枝 (贪婪) ----
    std::vector<std::pair<double, double>> smooth;
    smooth.push_back(raw.front());
    size_t i = 0;
    while (i + 1 < raw.size()) {
      context.poll();
      size_t j = raw.size() - 1;
      for (; j > i + 1; --j) {
        context.poll();
        if (detail::lineOfSight(
            g, raw[i].first, raw[i].second,
            raw[j].first, raw[j].second, &context))
        {
          break;
        }
      }
      smooth.push_back(raw[j]);
      i = j;
    }

    context.poll();
    res.expansions = context.expansions;
    res.ok = true;
    res.waypoints = std::move(smooth);
    return res;
  } catch (AStarResult::Fail interrupted) {
    res.fail = interrupted;
  } catch (const std::bad_alloc &) {
    res.fail = AStarResult::Fail::RESOURCE_LIMIT;
  } catch (const std::length_error &) {
    res.fail = AStarResult::Fail::RESOURCE_LIMIT;
  }
  res.ok = false;
  res.expansions = context.expansions;
  res.waypoints.clear();
  return res;
}

}  // namespace easyfly::astar

#endif  // EASYFLY_ASTAR_ASTAR_HPP_
