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
#ifndef EASYFLY_DESKTOP__RVIZ_GEOMETRY_HPP_
#define EASYFLY_DESKTOP__RVIZ_GEOMETRY_HPP_
#include <cstdint>
#include <string>
#include <vector>
#include <easyfly_msgs/msg/obstacle_info_array.hpp>
#include <easyfly_msgs/msg/landing_guidance_target.hpp>
#include <easyfly_msgs/msg/target_prediction_array.hpp>
#include <easyfly_msgs/msg/classified_obstacle_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
namespace easyfly_desktop
{
namespace spatial
{
constexpr size_t kCloudBytes = 4 * 1024 * 1024;
constexpr size_t kInputPoints = 200000;
constexpr size_t kCloudPoints = 20000;
constexpr size_t kPathInput = 20000;
constexpr size_t kPathPoints = 2000;
constexpr size_t kMapCells = 262144;
constexpr size_t kBoxes = 128;
constexpr int64_t kTtlNs = 2000000000LL;
struct Vertex {float x{0}, y{0}, z{0};};
enum class Primitive {Points, Strip, Segments};
struct Geometry
{
  bool valid{false};
  std::string frame, detail;
  int64_t stamp_ns{0};
  int64_t lifetime_ns{kTtlNs};
  Primitive primitive{Primitive::Points};
  float point_size{0.06F};
  size_t input_count{0}, omitted{0};
  std::vector<Vertex> vertices;
};
Geometry convert(const nav_msgs::msg::Path & message);
Geometry convert(const visualization_msgs::msg::Marker & message);
Geometry convert(const sensor_msgs::msg::PointCloud2 & message);
Geometry convert(const nav_msgs::msg::OccupancyGrid & message);
Geometry convert(const easyfly_msgs::msg::ObstacleInfoArray & message);
Geometry convert(const easyfly_msgs::msg::LandingGuidanceTarget & message);
Geometry convert(const easyfly_msgs::msg::TargetPredictionArray & message);
Geometry convert(const easyfly_msgs::msg::ClassifiedObstacleArray & message);
std::string validity(
  const Geometry & geometry, int64_t ros_now,
  int64_t receipt_steady, int64_t steady_now);
}  // namespace spatial
}  // namespace easyfly_desktop
#endif  // EASYFLY_DESKTOP__RVIZ_GEOMETRY_HPP_
