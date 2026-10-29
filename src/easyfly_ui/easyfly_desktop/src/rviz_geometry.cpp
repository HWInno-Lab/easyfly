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
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include "easyfly_desktop/rviz_geometry.hpp"
#include <easyfly_ui_bridge/special_observations.hpp>
namespace easyfly_desktop
{
namespace spatial
{
namespace
{
Geometry header(const std_msgs::msg::Header & h, Primitive primitive)
{
  Geometry g; g.primitive = primitive;
  if (h.frame_id.empty() || h.frame_id.size() > 128 || h.stamp.sec < 0 ||
    h.stamp.nanosec >= 1000000000U)
  {g.detail = "坐标系/时间戳无效"; return g;}
  g.frame = h.frame_id;
  g.stamp_ns = static_cast<int64_t>(h.stamp.sec) * 1000000000LL + h.stamp.nanosec;
  g.valid = g.stamp_ns > 0;
  if (!g.valid) {g.detail = "零时间戳 / 未就绪";}
  return g;
}
Geometry invalid(Geometry g, const std::string & reason)
{g.valid = false; g.detail = reason; g.vertices.clear(); return g;}
bool finite(double x)
{return std::isfinite(x) && std::abs(x) <= 1000000.0;}
bool point(const geometry_msgs::msg::Point & p)
{return finite(p.x) && finite(p.y) && finite(p.z);}
bool orientation(const geometry_msgs::msg::Quaternion & q)
{
  const double n = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return std::isfinite(n) && std::abs(n - 1.0) <= 0.001;
}
Vertex transform(Vertex p, const geometry_msgs::msg::Pose & pose)
{
  const auto & q = pose.orientation;
  const double tx = 2 * (q.y * p.z - q.z * p.y);
  const double ty = 2 * (q.z * p.x - q.x * p.z);
  const double tz = 2 * (q.x * p.y - q.y * p.x);
  return {static_cast<float>(p.x + q.w * tx + q.y * tz - q.z * ty + pose.position.x),
    static_cast<float>(p.y + q.w * ty + q.z * tx - q.x * tz + pose.position.y),
    static_cast<float>(p.z + q.w * tz + q.x * ty - q.y * tx + pose.position.z)};
}
float read_float(const uint8_t * bytes, bool big)
{
  uint32_t bits = 0;
  for (unsigned i = 0; i < 4; ++i) {
    bits |= uint32_t(bytes[i]) << (8 * (big ? 3 - i : i));
  }
  float result; std::memcpy(&result, &bits, sizeof(result)); return result;
}
}  // namespace
Geometry convert(const nav_msgs::msg::Path & m)
{
  auto h = m.header;
  // FAST-LIO leaves the container stamp at startup; the last pose is the
  // newest observation. Never replace it with receipt time.
  if (!m.poses.empty()) {h.stamp = m.poses.back().header.stamp;}
  auto g = header(h, Primitive::Strip); g.input_count = m.poses.size();
  if (!g.valid) {return g;}
  if (m.poses.size() > kPathInput) {return invalid(g, "轨迹输入超预算");}
  int64_t previous = -1;
  for (const auto & p : m.poses) {
    if ((!p.header.frame_id.empty() && p.header.frame_id != h.frame_id) ||
      p.header.stamp.sec < 0 || p.header.stamp.nanosec >= 1000000000U || !point(p.pose.position))
    {return invalid(g, "轨迹混合坐标系/非法点");}
    const auto stamp = static_cast<int64_t>(p.header.stamp.sec) * 1000000000LL +
      p.header.stamp.nanosec;
    if (stamp <= 0 || stamp < previous || stamp > g.stamp_ns)
    {return invalid(g, "轨迹点时间戳无效");}
    previous = stamp;
  }
  const size_t count = std::min(kPathPoints, m.poses.size());
  for (size_t i = 0; i < count; ++i) {
    const size_t index = count <= 1 ? 0 : i * (m.poses.size() - 1) / (count - 1);
    const auto & p = m.poses[index].pose.position;
    g.vertices.push_back(
      {static_cast<float>(p.x), static_cast<float>(p.y),
        static_cast<float>(p.z)});
  }
  g.omitted = m.poses.size() - count;
  g.detail = "实测轨迹；源时间取末尾观测，不代表当前飞行状态"; return g;
}
Geometry convert(const visualization_msgs::msg::Marker & m)
{
  auto g = header(m.header, Primitive::Strip); g.input_count = m.points.size();
  if (m.action == m.DELETE || m.action == m.DELETEALL) {
    // Deletion is an explicit invalidation, even with a repeated/zero stamp.
    g.valid = false; g.vertices.clear(); g.detail = "规划显示删除"; return g;
  }
  if (!g.valid) {return g;}
  if (m.action != m.ADD || m.type != m.LINE_STRIP || m.frame_locked ||
    !m.text.empty() || !m.mesh_resource.empty() || m.ns.size() > 128 ||
    m.colors.size() > kPathInput ||
    m.points.size() > kPathInput || !point(m.pose.position) || !orientation(m.pose.orientation))
  {return invalid(g, "仅支持有界非锁帧 LINE_STRIP 规划显示");}
  for (const auto & p : m.points) {
    if (!point(p)) {return invalid(g, "规划含非法点");}
  }
  const size_t count = std::min(kPathPoints, m.points.size());
  for (size_t i = 0; i < count; ++i) {
    const size_t index = count <= 1 ? 0 : i * (m.points.size() - 1) / (count - 1);
    const auto & p = m.points[index];
    g.vertices.push_back(
      transform(
        {static_cast<float>(p.x), static_cast<float>(p.y),
          static_cast<float>(p.z)}, m.pose));
  }
  g.omitted = m.points.size() - count;
  g.detail = "规划显示线；不是已接受控制轨迹/会话归属证据"; return g;
}
Geometry convert(const sensor_msgs::msg::PointCloud2 & m)
{
  auto g = header(m.header, Primitive::Points);
  const uint64_t n = uint64_t(m.width) * m.height; g.input_count = n;
  if (!g.valid) {return g;}
  if (m.data.size() > kCloudBytes || n > kInputPoints || m.fields.size() > 32 ||
    m.point_step > 256 || (n && m.point_step < 12) ||
    uint64_t(m.width) * m.point_step > m.row_step ||
    uint64_t(m.row_step) * m.height != m.data.size())
  {return invalid(g, "点云布局/输入超预算");}
  if (!n) {g.detail = "有效空点云"; return g;}
  std::array<uint32_t, 3> offsets{}; std::array<bool, 3> found{};
  const std::array<std::string, 3> names{"x", "y", "z"};
  for (const auto & field : m.fields) {
    if (field.name.size() > 128) {return invalid(g, "点云字段名超预算");}
    for (size_t i = 0; i < 3; ++i) {
      if (field.name != names[i]) {continue;}
      if (found[i] || field.datatype != field.FLOAT32 || field.count != 1 ||
        uint64_t(field.offset) + 4 > m.point_step)
      {return invalid(g, "点云 XYZ 必须为唯一 FLOAT32 字段");}
      offsets[i] = field.offset; found[i] = true;
    }
  }
  if (!found[0] || !found[1] || !found[2]) {return invalid(g, "点云缺 XYZ");}
  const size_t stride = (n + kCloudPoints - 1) / kCloudPoints;
  for (size_t i = 0; i < n; i += stride) {
    const auto start = (i / m.width) * m.row_step + (i % m.width) * m.point_step;
    const auto * bytes = m.data.data() + start;
    const float x = read_float(bytes + offsets[0], m.is_bigendian);
    const float y = read_float(bytes + offsets[1], m.is_bigendian);
    const float z = read_float(bytes + offsets[2], m.is_bigendian);
    if (finite(x) && finite(y) && finite(z)) {g.vertices.push_back({x, y, z});}
  }
  g.omitted = n - g.vertices.size(); g.detail = "几何点云；抽样/无效点省略，非安全占据证据";
  return g;
}
Geometry convert(const nav_msgs::msg::OccupancyGrid & m)
{
  auto g = header(m.header, Primitive::Points);
  const uint64_t n = uint64_t(m.info.width) * m.info.height; g.input_count = n;
  if (!g.valid) {return g;}
  if (n > kMapCells || n != m.data.size() || !std::isfinite(m.info.resolution) ||
    m.info.resolution <= 0 || m.info.resolution > 100 || !point(m.info.origin.position) ||
    !orientation(m.info.origin.orientation))
  {return invalid(g, "占据栅格布局/原点/输入超预算");}
  g.point_size = m.info.resolution;
  for (size_t i = 0; i < n; ++i) {
    const auto value = m.data[i];
    if (value < -1 || value > 100) {return invalid(g, "占据栅格含非法数值");}
    if (value < 50) {continue;}
    if (g.vertices.size() >= kCloudPoints) {return invalid(g, "占据格显示超预算，拒绝截断地图");}
    g.vertices.push_back(
      transform(
        {
          static_cast<float>((i % m.info.width + 0.5) * m.info.resolution),
          static_cast<float>((i / m.info.width + 0.5) * m.info.resolution), 0}, m.info.origin));
  }
  g.detail = "二维占据格中心（值≥50）；含上游膨胀，未知/空闲不绘制"; return g;
}
Geometry convert(const easyfly_msgs::msg::ObstacleInfoArray & m)
{
  auto g = header(m.header, Primitive::Segments); g.input_count = m.obstacles.size();
  if (!g.valid) {return g;}
  if (m.obstacles.size() > kBoxes) {return invalid(g, "障碍数量超预算，拒绝截断");}
  constexpr std::array<unsigned, 24> edges{
    0, 1, 0, 2, 0, 4, 1, 3, 1, 5, 2, 3, 2, 6, 3, 7, 4, 5, 4, 6, 5, 7, 6, 7};
  for (const auto & box : m.obstacles) {
    if (box.header != m.header || box.obstacle_type.size() > 128 ||
      !finite(box.center_x) || !finite(box.center_y) || !finite(box.center_z) ||
      !finite(box.size_x) || !finite(box.size_y) || !finite(box.size_z) ||
      box.size_x <= 0 || box.size_y <= 0 || box.size_z <= 0 ||
      box.size_x > 100 || box.size_y > 100 || box.size_z > 100)
    {return invalid(g, "障碍框/所属坐标系或时间戳无效");}
    std::array<Vertex, 8> corners{};
    for (unsigned i = 0; i < 8; ++i) {
      corners[i] = {box.center_x + (i & 1 ? 0.5F : -0.5F) * box.size_x,
        box.center_y + (i & 2 ? 0.5F : -0.5F) * box.size_y,
        box.center_z + (i & 4 ? 0.5F : -0.5F) * box.size_z};
    }
    for (auto i : edges) {
      g.vertices.push_back(corners[i]);
    }
  }
  g.detail = "几何障碍框；不读取类别推断或改变避障"; return g;
}
Geometry convert(const easyfly_msgs::msg::LandingGuidanceTarget & m)
{
  Geometry g; g.frame = m.header.frame_id;
  g.stamp_ns = easyfly_ui_bridge::detail::stamp_ns(m.header.stamp); g.lifetime_ns = 500000000;
  g.detail = "局部板中心与单位法向（非下降指令）"; g.primitive = Primitive::Segments;
  g.valid = m.valid && easyfly_ui_bridge::special::valid(m, "camera_down_optical_frame");
  if (!g.valid) {g.detail = "局部降落观测无效/源帧不匹配"; return g;}
  const auto & p = m.position; const auto & n = m.plane_normal;
  g.input_count = 1;
  g.vertices = {{static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z)},
    {static_cast<float>(p.x + .5 * n.x), static_cast<float>(p.y + .5 * n.y),
      static_cast<float>(p.z + .5 * n.z)}};
  return g;
}
Geometry convert(const easyfly_msgs::msg::TargetPredictionArray & m)
{
  Geometry g; g.frame = m.header.frame_id;
  g.stamp_ns = easyfly_ui_bridge::detail::stamp_ns(m.header.stamp); g.lifetime_ns = 500000000;
  g.detail = "模型预测十字（非实测；参考时间/horizon见感知表）"; g.primitive = Primitive::Segments;
  g.valid = easyfly_ui_bridge::special::valid(m); g.input_count = m.predictions.size();
  if (!g.valid) {g.detail = "预测来源、ID、数值或数量无效"; return g;}
  for (const auto & p : m.predictions) {
    if (!p.valid) {++g.omitted; continue;}
    if (std::abs(p.position_x) > 1000000 || std::abs(p.position_y) > 1000000 ||
      std::abs(p.position_z) > 1000000) {g.valid = false; g.vertices.clear(); return g;}
    const float x = p.position_x, y = p.position_y, z = p.position_z;
    g.vertices.insert(
      g.vertices.end(), {{x - .1F, y, z}, {x + .1F, y, z},
        {x, y - .1F, z}, {x, y + .1F, z}, {x, y, z - .1F}, {x, y, z + .1F}});
  }
  return g;
}
Geometry convert(const easyfly_msgs::msg::ClassifiedObstacleArray & m)
{
  if (!m.geometry_valid || !easyfly_ui_bridge::special::valid(m)) {
    Geometry invalid; invalid.detail = "类别副本几何无效/超限"; return invalid;
  }
  easyfly_msgs::msg::ObstacleInfoArray copy; copy.header = m.header;
  for (const auto & item : m.obstacles) {
    copy.obstacles.push_back(item.obstacle);
  }
  auto g = convert(copy); g.lifetime_ns = 500000000;
  if (g.valid) {g.detail = "独立类别副本几何（类别有效期见感知表；不改变原障碍）";}
  return g;
}
std::string validity(const Geometry & g, int64_t now, int64_t receipt, int64_t steady)
{
  if (!g.valid) {return g.detail;}
  if (now <= 0) {return "时钟未就绪";}
  if (g.stamp_ns > now && g.stamp_ns - now > 200000000LL) {return "未来源时间";}
  if (now - g.stamp_ns > g.lifetime_ns) {return "源时间过期";}
  if (steady < receipt || steady - receipt > g.lifetime_ns) {return "接收超时 / 重复源不续期";}
  return {};
}
}  // namespace spatial
}  // namespace easyfly_desktop
