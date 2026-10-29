// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <iostream>
#include <sophus/se3.hpp>
#include <vikit/pinhole_camera.h>

int main()
{
  vk::PinholeCamera camera(640, 480, 1., 337.3570508552852, 337.3570508552852,
    320.5, 240.5, 0., 0., 0., 0.);
  const Eigen::Vector2d pixel(123.25, 321.75);
  const auto ray = camera.cam2world(pixel);
  const Eigen::Vector3d point = ray * 3.7;
  const auto projected = camera.world2cam(point);
  const Sophus::SE3d transform(Sophus::SO3d::exp(Eigen::Vector3d(.02, -.05, .1)),
    Eigen::Vector3d(.1, .2, .3));
  const auto recovered = transform.inverse() * (transform * (ray * 3.7));
  const double pixel_error = (projected - pixel).norm();
  const double transform_error = (recovered - ray * 3.7).norm();
  std::cout << "pixel_error=" << pixel_error << " rigid_transform_error=" << transform_error << '\n';
  return std::isfinite(pixel_error) && pixel_error < 1e-9 && transform_error < 1e-12 ? 0 : 1;
}
