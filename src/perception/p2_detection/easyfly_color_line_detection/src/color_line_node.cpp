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

#include <easyfly_color_line/detector.hpp>
#include <easyfly_image_detection_common/node.hpp>
#include <easyfly_msgs/msg/color_line_observation_array.hpp>

class ColorLineNode
  : public easyfly::image_detection::Node<easyfly_msgs::msg::ColorLineObservationArray>
{
public:
  ColorLineNode()
  : Node("color_line_detection", "color_line", false)
  {
    parameters_.hue_min = parameter<int>("hue_min", 170);
    parameters_.hue_max = parameter<int>("hue_max", 10);
    parameters_.saturation_min = parameter<int>("saturation_min", 100);
    parameters_.saturation_max = parameter<int>("saturation_max", 255);
    parameters_.value_min = parameter<int>("value_min", 60);
    parameters_.value_max = parameter<int>("value_max", 255);
    parameters_.morphology_kernel = parameter<int>("morphology_kernel", 3);
    parameters_.min_area_px = parameter<double>("min_area_px", 80.0);
    parameters_.min_length_px = parameter<double>("min_length_px", 25.0);
    parameters_.min_aspect_ratio = parameter<double>("min_aspect_ratio", 3.0);
    parameters_.min_fill_ratio = parameter<double>("min_fill_ratio", 0.5);
    parameters_.max_detections = parameter<int>("max_detections", 32);
    easyfly::color_line::validate(parameters_);
  }

private:
  void detect(const cv::Mat & image, easyfly_msgs::msg::ColorLineObservationArray & output) override
  {
    for (const auto & item : easyfly::color_line::detect(image, parameters_)) {
      easyfly_msgs::msg::ColorLineObservation observation;
      observation.center_x = item.center.x;
      observation.center_y = item.center.y;
      observation.start_x = item.start.x;
      observation.start_y = item.start.y;
      observation.end_x = item.end.x;
      observation.end_y = item.end.y;
      observation.angle_rad = item.angle_rad;
      observation.length_px = item.length_px;
      observation.width_px = item.width_px;
      observation.area_px = item.area_px;
      observation.offset_x_normalized = item.offset_x_normalized;
      observation.score = item.score;
      output.observations.push_back(observation);
    }
  }
  easyfly::color_line::Parameters parameters_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    rclcpp::spin(std::make_shared<ColorLineNode>());
  } catch (const std::exception & error) {
    std::cerr << error.what() << std::endl;
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
