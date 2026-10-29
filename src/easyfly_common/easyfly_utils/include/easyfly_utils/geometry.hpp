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
#ifndef EASYFLY_UTILS__GEOMETRY_HPP_
#define EASYFLY_UTILS__GEOMETRY_HPP_
#include <cmath>
#include <vector>
namespace easyfly_utils
{
struct Vec3
{
  double x = 0.0, y = 0.0, z = 0.0;
  Vec3() = default;
  Vec3(double x, double y, double z)
  : x(x), y(y), z(z) {}
  double norm() const {return std::sqrt(x * x + y * y + z * z);}
  Vec3 operator+(const Vec3 & o) const {return {x + o.x, y + o.y, z + o.z};}
  Vec3 operator-(const Vec3 & o) const {return {x - o.x, y - o.y, z - o.z};}
  Vec3 operator*(double s) const {return {x * s, y * s, z * s};}
};
struct Quaternion
{
  double w = 1.0, x = 0.0, y = 0.0, z = 0.0;
  Quaternion() = default;
  Quaternion(double w, double x, double y, double z)
  : w(w), x(x), y(y), z(z) {}
};
// 欧拉角 (rad)
struct Euler
{
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
};
// 四元数 → 欧拉角
inline Euler quaternion_to_euler(const Quaternion & q)
{
  Euler e;
  double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
  double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  e.roll = std::atan2(sinr_cosp, cosr_cosp);
  double sinp = 2.0 * (q.w * q.y - q.z * q.x);
  if (std::abs(sinp) >= 1.0) {
    e.pitch = std::copysign(M_PI / 2.0, sinp);
  } else {
    e.pitch = std::asin(sinp);
  }
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  e.yaw = std::atan2(siny_cosp, cosy_cosp);
  return e;
}
// ENU → NED 坐标转换
inline Vec3 enu_to_ned(const Vec3 & enu) {return {enu.y, enu.x, -enu.z};}
inline Vec3 ned_to_enu(const Vec3 & ned) {return {ned.y, ned.x, -ned.z};}
// 经纬度 → 平面坐标 (简化，小范围可用)
inline Vec3 gps_to_local_enu(
  double lat, double lon, double alt,
  double ref_lat, double ref_lon, double ref_alt)
{
  const double EARTH_RADIUS = 6378137.0;
  double dlat = (lat - ref_lat) * M_PI / 180.0;
  double dlon = (lon - ref_lon) * M_PI / 180.0;
  double cos_lat = std::cos(ref_lat * M_PI / 180.0);
  return {dlon * EARTH_RADIUS * cos_lat, dlat * EARTH_RADIUS, alt - ref_alt};
}
}  // namespace easyfly_utils
#endif  // EASYFLY_UTILS__GEOMETRY_HPP_
