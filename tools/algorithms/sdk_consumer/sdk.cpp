// Copyright 2026 EasyFly Project
// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <iomanip>
#include <iostream>
#include <livox_lidar_api.h>
#include <librealsense2/rs.h>
#include <librealsense2/rsutil.h>

int main()
{
  LivoxLidarSdkVer version{};
  GetLivoxLidarSdkVer(&version);  // No SDK init, sockets, device enumeration or threads.
  if (version.major != LIVOX_LIDAR_SDK_MAJOR_VERSION ||
      version.minor != LIVOX_LIDAR_SDK_MINOR_VERSION ||
      version.patch != LIVOX_LIDAR_SDK_PATCH_VERSION) {return 1;}
  rs2_error * error = nullptr;
  const int runtime = rs2_get_api_version(&error);
  if (error || runtime != RS2_API_VERSION) {return 2;}
  std::cout << "livox " << version.major << '.' << version.minor << '.' << version.patch
            << " realsense " << runtime << '\n' << std::setprecision(17);
  rs2_intrinsics intrinsics{};
  intrinsics.width = 640; intrinsics.height = 480;
  intrinsics.ppx = 320; intrinsics.ppy = 240;
  intrinsics.fx = 400; intrinsics.fy = 410;
  intrinsics.model = RS2_DISTORTION_NONE;
  rs2_extrinsics transform{};
  transform.rotation[0] = transform.rotation[4] = transform.rotation[8] = 1;
  transform.translation[0] = .25f; transform.translation[1] = -.5f;
  transform.translation[2] = .75f;
  for (int i = 0; i < 512; ++i) {
    const float pixel[2] = {static_cast<float>((i*19)%640), static_cast<float>((i*17)%480)};
    const float depth = .1f + .01f*i;
    float point[3], projected[2], moved[3];
    rs2_deproject_pixel_to_point(point, &intrinsics, pixel, depth);
    rs2_project_point_to_pixel(projected, &intrinsics, point);
    rs2_transform_point_to_point(moved, &transform, point);
    if (std::abs(projected[0]-pixel[0]) > 1e-3f || std::abs(projected[1]-pixel[1]) > 1e-3f ||
        std::abs(point[2]-depth) > 1e-6f) {return 3;}
    for (int j = 0; j < 3; ++j) {
      if (!std::isfinite(moved[j]) || std::abs(moved[j]-point[j]-transform.translation[j]) > 1e-5f) {return 4;}
    }
    std::cout << i << ' ' << point[0] << ' ' << point[1] << ' ' << point[2]
              << ' ' << projected[0] << ' ' << projected[1]
              << ' ' << moved[0] << ' ' << moved[1] << ' ' << moved[2] << '\n';
  }
  float fov[2]; rs2_fov(&intrinsics, fov);
  constexpr double degrees = 180./3.14159265358979323846;
  if (std::abs(fov[0]-2*std::atan(320./400)*degrees) > 1e-3 ||
      std::abs(fov[1]-2*std::atan(240./410)*degrees) > 1e-3) {return 5;}
  std::cout << "fov " << fov[0] << ' ' << fov[1] << '\n';
}
