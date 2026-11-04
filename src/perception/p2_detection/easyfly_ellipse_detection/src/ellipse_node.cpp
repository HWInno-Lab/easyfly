// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_ellipse_detection/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
//
#include <exception>
#include <iostream>
#include <memory>

#include <easyfly_ellipse/detector.hpp>
#include <easyfly_image_detection_common/node.hpp>
#include <easyfly_msgs/msg/ellipse_observation_array.hpp>

class EllipseNode
  : public easyfly::image_detection::Node<easyfly_msgs::msg::EllipseObservationArray>
{
public:
  EllipseNode()
  : Node("ellipse_detection", "ellipse", true)
  {
    parameters_.min_axis_px = parameter<double>("min_axis_px", 12.0);
    parameters_.max_axis_ratio = parameter<double>("max_axis_ratio", 8.0);
    parameters_.max_radial_error = parameter<double>("max_radial_error", 0.06);
    parameters_.max_area_error = parameter<double>("max_area_error", 0.15);
    parameters_.canny_low = parameter<double>("canny_low", 50.0);
    parameters_.canny_high = parameter<double>("canny_high", 150.0);
    parameters_.max_detections = parameter<int>("max_detections", 32);
    easyfly::ellipse::validate(parameters_);
  }

private:
  void detect(const cv::Mat & image, easyfly_msgs::msg::EllipseObservationArray & output) override
  {
    for (const auto & item : easyfly::ellipse::detect(image, parameters_)) {
      easyfly_msgs::msg::EllipseObservation observation;
      observation.center_x = item.center.x;
      observation.center_y = item.center.y;
      observation.major_axis_px = item.major_axis_px;
      observation.minor_axis_px = item.minor_axis_px;
      observation.angle_rad = item.angle_rad;
      observation.radial_error = item.radial_error;
      observation.score = item.score;
      output.observations.push_back(observation);
    }
  }
  easyfly::ellipse::Parameters parameters_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    rclcpp::spin(std::make_shared<EllipseNode>());
  } catch (const std::exception & error) {
    std::cerr << error.what() << std::endl;
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
