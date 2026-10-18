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
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
namespace easyfly::tag_pose
{
inline cv::Ptr<cv::aruco::DetectorParameters> detector_parameters()
{
  auto parameters = cv::aruco::DetectorParameters::create();
  parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
  parameters->cornerRefinementWinSize = 3;
  return parameters;
}
struct Pose {cv::Vec3d t;
  cv::Vec4d q;
  double error{0};
  bool valid{false};
  std::string reason;};

inline cv::Vec4d quaternion(const cv::Mat & rotation)
{
  double q[4] = {};
  double tr = cv::trace(rotation)[0];

  if (tr > 0) {
    double s = 2 * std::sqrt(tr + 1);
    q[3] = s / 4;
    q[0] = (rotation.at<double>(
        2,
        1) - rotation.at<double>(1, 2)) / s;

    q[1] = (rotation.at<double>(0, 2) - rotation.at<double>(2, 0)) / s;

    q[2] = (rotation.at<double>(1, 0) - rotation.at<double>(0, 1)) / s;
  } else {
    int i = 0;
    if (rotation.at<double>(1, 1) > rotation.at<double>(
        i,
        i))
    {
      i = 1;
    }
    if (rotation.at<double>(2, 2) > rotation.at<double>(i, i)) {
      i = 2;
    }
    int j = (i + 1) % 3, k = (i + 2) % 3;
    double s = 2 * std::sqrt(
      1 + rotation.at<double>(
        i,
        i) - rotation.at<double>(
        j, j) - rotation.at<double>(k, k));

    q[i] = s / 4;
    q[j] = (rotation.at<double>(j, i) + rotation.at<double>(i, j)) / s;

    q[k] = (rotation.at<double>(k, i) + rotation.at<double>(i, k)) / s;

    q[3] = (rotation.at<double>(k, j) - rotation.at<double>(j, k)) / s;
  }
  double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);

  return {q[0] / n, q[1] / n, q[2] / n, q[3] / n};
}
inline Pose estimate(
  const std::vector<cv::Point2f> & corners, const cv::Mat & K, const cv::Mat & D,
  double size, double max_error = 2, double ambiguity_margin = .2)
{
  Pose result;
  result.reason = "invalid_geometry";

  if (corners.size() != 4 || !std::isfinite(size) || size <= 0 || size > 10 || K.rows != 3 ||
    K.cols != 3 || K.type() != CV_64F ||
    !std::isfinite(max_error) || max_error <= 0 || !std::isfinite(ambiguity_margin) ||
    ambiguity_margin <= 0 ||
    !std::all_of(
      corners.begin(), corners.end(), [](const auto & p) {
        return std::isfinite(p.x) && std::isfinite(p.y);
      }) ||
    !cv::checkRange(K) || !cv::checkRange(D) ||
    K.at<double>(0, 0) <= 0 || K.at<double>(1, 1) <= 0 ||
    !cv::isContourConvex(corners) || std::abs(cv::contourArea(corners)) < 25)
  {
    return result;
  }
  std::vector<cv::Point3f> object{{static_cast<float>(-size / 2), static_cast<float>(size / 2), 0},
    {static_cast<float>(size / 2), static_cast<float>(size / 2), 0},
    {static_cast<float>(size / 2), static_cast<float>(-size / 2), 0},
    {static_cast<float>(-size / 2), static_cast<float>(-size / 2), 0}};

  std::vector<cv::Mat> rotations, translations;

  cv::solvePnPGeneric(
    object, corners, K, D, rotations, translations, false,
    cv::SOLVEPNP_IPPE_SQUARE);

  struct Candidate {double error;
    cv::Mat r, t;};
  std::vector<Candidate> candidates;

  for (size_t i = 0;
    i < rotations.size();
    ++i)
  {
    if (!cv::checkRange(rotations[i]) || !cv::checkRange(translations[i]) ||
      translations[i].at<double>(2) <= 0)
    {
      continue;
    }
    cv::Mat R;
    cv::Rodrigues(rotations[i], R);
    bool front = true;

    for (const auto & p : object) {
      if ((R.at<double>(
          2,
          0) * p.x + R.at<double>(2, 1) * p.y + translations[i].at<double>(2)) <= 0)
      {
        front = false;
      }
    }
    if (!front) {
      continue;
    }
    std::vector<cv::Point2f> projected;
    cv::projectPoints(
      object, rotations[i], translations[i], K,
      D, projected);

    double e = 0;
    for (int j = 0;
      j < 4;
      ++j)
    {
      e += cv::norm(projected[j] - corners[j]) * cv::norm(projected[j] - corners[j]);
    }
    bool duplicate = false;

    for (const auto & previous : candidates) {
      cv::Mat previous_R;
      cv::Rodrigues(previous.r, previous_R);

      if (cv::norm(previous_R - R) < 1e-6 && cv::norm(previous.t - translations[i]) < 1e-6) {
        duplicate = true;
      }
    }
    if (!duplicate && std::isfinite(e)) {
      candidates.push_back({std::sqrt(e / 4), rotations[i], translations[i]});
    }
  }
  std::sort(
    candidates.begin(), candidates.end(),
    [](auto & a, auto & b) {
      return a.error < b.error;
    });

  if (candidates.empty()) {
    result.reason = "no_positive_depth_pose";
    return result;
  }
  result.error = candidates[0].error;

  if (result.error > max_error) {
    result.reason = "reprojection_error";
    return result;
  }
  if (candidates.size() > 1 && candidates[1].error - candidates[0].error < ambiguity_margin) {
    result.reason = "pose_ambiguous";
    return result;
  }
  cv::Mat R;
  cv::Rodrigues(candidates[0].r, R);
  result.q = quaternion(R);

  result.t = cv::Vec3d(
    candidates[0].t.at<double>(0), candidates[0].t.at<double>(
      1), candidates[0].t.at<double>(2));

  result.valid = true;
  result.reason = "tag36h11_pose";
  return result;
}
}  // namespace easyfly::tag_pose

namespace easyfly::tag_pose
{
struct Detections {std::vector<int> ids; std::vector<std::vector<cv::Point2f>> corners;};
inline Detections detect(const cv::Mat & gray)
{
  Detections result;
  cv::aruco::detectMarkers(gray,
    cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11),
    result.corners, result.ids, detector_parameters());
  return result;
}
}
