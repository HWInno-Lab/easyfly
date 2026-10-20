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
#include <cmath>
#include <iostream>

#include <easyfly_color_line/detector.hpp>
#include <easyfly_ellipse/detector.hpp>
#include <opencv2/imgproc.hpp>

int main()
{
  cv::Mat image(240, 320, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::ellipse(image, {160, 120}, {60, 30}, 0, 0, 360, {255, 255, 255}, -1);
  const auto ellipses = easyfly::ellipse::detect(image);
  image.setTo(cv::Scalar(0, 0, 0));
  cv::rectangle(image, {150, 10}, {170, 230}, {0, 0, 255}, -1);
  const auto lines = easyfly::color_line::detect(image);
  if (ellipses.size() != 1 || lines.size() != 1 ||
    std::abs(ellipses[0].major_axis_px - 120) > 3 ||
    std::abs(lines[0].length_px - 220) > 1)
  {
    std::cerr << "installed detector geometry mismatch" << std::endl;
    return 1;
  }
  std::cout << "ellipse and color line installed targets passed" << std::endl;
  return 0;
}
