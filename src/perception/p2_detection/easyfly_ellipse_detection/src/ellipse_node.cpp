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
