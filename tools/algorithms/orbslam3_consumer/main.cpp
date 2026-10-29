// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#include <CameraModels/Pinhole.h>
#include <ORBextractor.h>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <iostream>

int main()
{
  ORB_SLAM3::Pinhole camera(std::vector<float>{337.35705F, 337.35705F, 320.5F, 240.5F});
  const cv::Point2f pixel(123.25F, 321.75F);
  const auto ray = camera.unproject(pixel);
  const auto projected = camera.project(cv::Point3f(ray.x * 3.7F, ray.y * 3.7F, ray.z * 3.7F));
  const double error = cv::norm(projected - pixel);
  cv::Mat pattern(480, 640, CV_8UC1);
  cv::RNG random(20261021);
  random.fill(pattern, cv::RNG::UNIFORM, 0, 256);
  cv::GaussianBlur(pattern, pattern, cv::Size(3, 3), .7);
  ORB_SLAM3::ORBextractor extractor(1200, 1.2F, 8, 20, 7);
  std::vector<cv::KeyPoint> keys;
  cv::Mat descriptors;
  std::vector<int> overlap{0, 0};
  extractor(pattern, cv::Mat(), keys, descriptors, overlap);
  std::cout << "projection_error_px=" << error << " real_orb_features=" << keys.size()
            << " descriptor_bytes=" << descriptors.total() * descriptors.elemSize() << '\n';
  return std::isfinite(error) && error < 1e-3 && keys.size() >= 500 &&
         descriptors.rows == static_cast<int>(keys.size()) && descriptors.cols == 32 ? 0 : 1;
}
