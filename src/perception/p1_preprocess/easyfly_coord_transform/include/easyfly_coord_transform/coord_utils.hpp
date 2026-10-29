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
// EasyFly P1: 坐标变换工具 (header-only)
//
// 提供 EasyFly 感知/执行层常用的坐标变换:
//   - rpy ↔ 四元数 (ZYX 惯例, 与 tf2 一致)
//   - 像素 + 深度 → 相机系 3D 点 (针孔模型)
//   - ENU 速度 → FLU 机体系速度
//   - FLU → 光学系 (RDF) 旋转
#ifndef EASYFLY_COORD_TRANSFORM__COORD_UTILS_HPP_
#define EASYFLY_COORD_TRANSFORM__COORD_UTILS_HPP_
#include <array>
#include <cmath>
namespace easyfly_coord_transform
{
/// rpy (ZYX: R = Rz(yaw)·Ry(pitch)·Rx(roll)) → 四元数 (x,y,z,w)
inline std::array<double, 4> rpy_to_quat(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll * 0.5), sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5), sy = std::sin(yaw * 0.5);
  return {
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
    cr * cp * cy + sr * sp * sy};
}
/// 四元数 → rpy (ZYX)
inline std::array<double, 3> quat_to_rpy(double x, double y, double z, double w)
{
  // roll
  const double sinr_cosp = 2.0 * (w * x + y * z);
  const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  // pitch
  const double sinp = 2.0 * (w * y - z * x);
  // yaw
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return {
    std::atan2(sinr_cosp, cosr_cosp),
    std::fabs(sinp) >= 1.0 ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp),
    std::atan2(siny_cosp, cosy_cosp)};
}
/// 像素坐标 + 深度 → 相机光学系 3D 点 (针孔模型)
/// @param u,v 像素坐标 (pix); @param depth 深度 (m, 光轴 z)
/// @param fx,fy,cx,cy 内参
inline std::array<double, 3> pixel_depth_to_point(
  double u, double v, double depth, double fx, double fy, double cx, double cy)
{
  return {
    (u - cx) * depth / fx,
    (v - cy) * depth / fy,
    depth};
}
/// ENU (world) 向量 → FLU (body) 向量, 用 body→world 的 rpy
template<typename V3>
inline V3 enu_to_flu(const V3 & v_world, double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll), sr = std::sin(roll);
  const double cp = std::cos(pitch), sp = std::sin(pitch);
  const double cy = std::cos(yaw), sy = std::sin(yaw);
  // R (body→world) 的转置即 world→body
  const double r00 = cy * cp, r10 = sy * cp, r20 = -sp;
  const double r01 = cy * sp * sr - sy * cr, r11 = sy * sp * sr + cy * cr, r21 = cp * sr;
  const double r02 = cy * sp * cr + sy * sr, r12 = sy * sp * cr - cy * sr, r22 = cp * cr;
  return {
    r00 * v_world[0] + r10 * v_world[1] + r20 * v_world[2],
    r01 * v_world[0] + r11 * v_world[1] + r21 * v_world[2],
    r02 * v_world[0] + r12 * v_world[1] + r22 * v_world[2]};
}
/// FLU (x前 y左 z上) 向量 → 光学 RDF (x右 y下 z前)
/// 即像素/点云光学系向量 → 机体系向量
template<typename V3>
inline V3 flu_to_optical(const V3 & v)
{
  return {-v[1], -v[2], v[0]};
}
/// 光学 RDF 向量 → FLU 机体系
template<typename V3>
inline V3 optical_to_flu(const V3 & v)
{
  return {v[2], -v[0], -v[1]};
}
}  // namespace easyfly_coord_transform
#endif  // EASYFLY_COORD_TRANSFORM__COORD_UTILS_HPP_
