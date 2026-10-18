// Copyright 2026 EasyFly Project
// SPDX-License-Identifier: Apache-2.0
#include <stdexcept>
#ifdef EASYFLY_CHECK_LEGACY
#include "tag_core.hpp"
#include "track_core.hpp"
#include "easyfly_object_detection/backend_factory.hpp"
namespace tag_core = ef_tag;
namespace csrt_core = ef_single;
#else
#include "easyfly_tag_pose/tag_core.hpp"
#include "easyfly_csrt/track_core.hpp"
namespace tag_core = easyfly::tag_pose;
namespace csrt_core = easyfly::csrt;
#endif
void require(bool condition) {if (!condition) throw std::runtime_error("installed vision check failed");}
int main()
{
  cv::setNumThreads(2);
  cv::Mat marker; cv::aruco::drawMarker(
    cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11), 7, 160, marker);
  cv::Mat image(240, 240, CV_8U, cv::Scalar(255)); marker.copyTo(image(cv::Rect(40, 40, 160, 160)));
  const auto tags = tag_core::detect(image);
  require(tags.ids.size() == 1 && tags.ids[0] == 7 && tags.corners[0].size() == 4);
  cv::Mat K = (cv::Mat_<double>(3, 3) << 600, 0, 320, 0, 600, 240, 0, 0, 1);
  std::vector<cv::Point3f> object{{-.1, .1, 0}, {.1, .1, 0}, {.1, -.1, 0}, {-.1, -.1, 0}};
  std::vector<cv::Point2f> corners;
  cv::projectPoints(object, cv::Vec3d(2.7, .2, .1), cv::Vec3d(.1, .05, 1), K, cv::Mat(), corners);
  require(tag_core::estimate(corners, K, cv::Mat(), .2).valid);
  cv::Mat frame(160, 200, CV_8UC3, cv::Scalar(30, 30, 30)), patch(40, 40, CV_8UC3);
  cv::RNG rng(42); rng.fill(patch, cv::RNG::UNIFORM, 0, 255);
  patch.copyTo(frame(cv::Rect(60, 50, 40, 40)));
  auto tracker = csrt_core::make_tracker("csrt");
  require(tracker->initialize(frame, {60, 50, 40, 40}));
  const auto tracked = tracker->update(frame);
  require(tracked.ok && !tracked.calibrated_confidence.has_value());
  tracker->reset(); require(!tracker->initialized());
  require(!tracker->update(frame).ok);
}
