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

// A* 纯逻辑单元测试 (M4.2): 搜索/绕障/剪枝/失败语义
#include <gtest/gtest.h>

#include <cmath>

#include "easyfly_astar/astar.hpp"

using easyfly::astar::AStarResult;
using easyfly::astar::OccupancyGrid2d;

/// 构造空栅格 (w×h 格, 分辨率 res, 原点 (ox,oy))
static OccupancyGrid2d emptyGrid(int w = 100, int h = 100, double res = 0.2,
                                 double ox = -10.0, double oy = -10.0)
{
  OccupancyGrid2d g;
  g.width = w;
  g.height = h;
  g.resolution = res;
  g.origin_x = ox;
  g.origin_y = oy;
  g.data.assign(w * h, 0);
  return g;
}

static void fillRect(OccupancyGrid2d & g, double x1, double y1, double x2,
                     double y2)
{
  const int c1 = std::max(0, static_cast<int>((x1 - g.origin_x) / g.resolution));
  const int c2 = std::min(g.width - 1,
      static_cast<int>((x2 - g.origin_x) / g.resolution));
  const int r1 = std::max(0, static_cast<int>((y1 - g.origin_y) / g.resolution));
  const int r2 = std::min(g.height - 1,
      static_cast<int>((y2 - g.origin_y) / g.resolution));
  for (int r = r1; r <= r2; ++r)
    for (int c = c1; c <= c2; ++c)
      g.data[r * g.width + c] = 100;
}

static double pathLength(const std::vector<std::pair<double, double>> & p)
{
  double len = 0;
  for (size_t i = 1; i < p.size(); ++i)
    len += std::hypot(p[i].first - p[i - 1].first, p[i].second - p[i - 1].second);
  return len;
}

TEST(AStar, EmptyGridDirectPath)
{
  auto g = emptyGrid();
  auto r = planPath(g, -5, 0, 5, 0);
  ASSERT_TRUE(r.ok);
  // 剪枝后应为 直达 2 点 (起点+终点)
  EXPECT_EQ(r.waypoints.size(), 2u);
  EXPECT_NEAR(pathLength(r.waypoints), 10.0, 0.3);
}

TEST(AStar, PreservesExactRequestedGoalInsteadOfCellCenter)
{
  auto g = emptyGrid();
  const double x = 1.957055851817131, y = 0.006087067071348429;
  const auto r = planPath(g, -0.042944148182869, y, x, y);
  ASSERT_TRUE(r.ok);
  ASSERT_GE(r.waypoints.size(), 2u);
  EXPECT_DOUBLE_EQ(r.waypoints.back().first, x);
  EXPECT_DOUBLE_EQ(r.waypoints.back().second, y);
}

TEST(AStar, SameCellRetainsRealStartAndExactGoal)
{
  auto g = emptyGrid();
  const auto r = planPath(g, 0.01, 0.02, 0.18, 0.19);
  ASSERT_TRUE(r.ok);
  ASSERT_EQ(r.waypoints.size(), 2u);
  EXPECT_DOUBLE_EQ(r.waypoints.front().first, 0.01);
  EXPECT_DOUBLE_EQ(r.waypoints.front().second, 0.02);
  EXPECT_DOUBLE_EQ(r.waypoints.back().first, 0.18);
  EXPECT_DOUBLE_EQ(r.waypoints.back().second, 0.19);
}

TEST(AStar, DetoursAroundBlockingBox)
{
  auto g = emptyGrid();
  // 竖墙 (x=0, y∈[-4,4]) 中间不留缝 → 必须绕墙端
  fillRect(g, -0.2, -4.0, 0.2, 4.0);
  auto r = planPath(g, -5, 0, 5, 0);
  ASSERT_TRUE(r.ok);
  EXPECT_GT(pathLength(r.waypoints), 12.0);   // 明显长于直线 10m
  // 每个航点不落在占据格
  for (const auto & [x, y] : r.waypoints) {
    EXPECT_FALSE(g.worldOccupied(x, y)) << x << "," << y;
  }
}

TEST(AStar, GapInWallFound)
{
  auto g = emptyGrid();
  // 竖墙从栅格下缘直通上缘, 仅留 1.2m 缺口 (y∈[-0.6,0.6]);
  // 起终点在 y=-3, 直线被墙挡死且无端点可绕, 唯一通路 = 缺口
  fillRect(g, -0.2, -10.0, 0.2, -0.6);
  fillRect(g, -0.2, 0.6, 0.2, 10.0);
  auto r = planPath(g, -5, -3, 5, -3);
  ASSERT_TRUE(r.ok);
  bool near_gap = false;
  for (const auto & [x, y] : r.waypoints) {
    if (std::abs(x) < 0.6 && std::abs(y) < 1.0) near_gap = true;
  }
  EXPECT_TRUE(near_gap);
  EXPECT_GT(pathLength(r.waypoints), 11.0);  // 直线 10m 被挡, 实际绕行
  EXPECT_LT(pathLength(r.waypoints), 14.0);
}

TEST(AStar, GoalInObstacleRejected)
{
  auto g = emptyGrid();
  fillRect(g, 4.0, -1.0, 6.0, 1.0);
  auto r = planPath(g, -5, 0, 5, 0);  // 目标 (5,0) 在障碍内
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.fail, AStarResult::Fail::GOAL_IN_OBSTACLE);
}

TEST(AStar, FullyEnclosedNoPath)
{
  auto g = emptyGrid();
  // 封闭盒子围住起点
  fillRect(g, -2, -2, 2, -1.8);
  fillRect(g, -2, 1.8, 2, 2);
  fillRect(g, -2, -2, -1.8, 2);
  fillRect(g, 1.8, -2, 2, 2);
  auto r = planPath(g, 0, 0, 5, 0);
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.fail, AStarResult::Fail::NO_PATH);
}

TEST(AStar, StartUnstick)
{
  auto g = emptyGrid();
  fillRect(g, -0.3, -0.3, 0.3, 0.3);  // 起点贴在障碍内 (脱困半径 5 格 = 1m)
  auto r = planPath(g, 0, 0, 5, 0);
  ASSERT_TRUE(r.ok);  // 脱困成功
  EXPECT_FALSE(g.worldOccupied(r.waypoints[1].first, r.waypoints[1].second));
}

TEST(AStar, OutOfBoundsRejected)
{
  auto g = emptyGrid();  // 定义域 x,y ∈ [-10,10]
  auto r = planPath(g, 0, 0, 50, 0);
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.fail, AStarResult::Fail::OUT_OF_BOUNDS);
}

TEST(AStar, SmoothingReducesWaypoints)
{
  auto g = emptyGrid();
  fillRect(g, -0.2, -4.0, 0.2, 4.0);
  auto r = planPath(g, -5, 0, 5, 0);
  ASSERT_TRUE(r.ok);
  EXPECT_LE(r.waypoints.size(), 4u);  // 剪枝后极少航点 (绕端 3~4 个)
}
