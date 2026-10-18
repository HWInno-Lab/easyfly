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

#include <gtest/gtest.h>
#include "plan_env/grid_map.h"

class GridMapDepthProjectionTestPeer
{
public:
  static int project(int rows, int columns, int skip, bool filtered, size_t initial_capacity,
    double * first_depth = nullptr, bool zero_next_pixel = false)
  {
    std::vector<uint16_t> pixels(static_cast<size_t>(rows) * columns, 1000);
    if (zero_next_pixel && pixels.size() > 1) pixels[1] = 0;
    GridMap map;
    map.mp_.skip_pixel_ = skip;
    map.mp_.use_depth_filter_ = filtered;
    map.mp_.depth_filter_margin_ = 0;
    map.mp_.k_depth_scaling_factor_ = 1000;
    map.mp_.fx_ = map.mp_.fy_ = 100;
    map.mp_.cx_ = map.mp_.cy_ = 0;
    map.mp_.depth_filter_mindist_ = .1;
    map.mp_.depth_filter_maxdist_ = 5;
    map.mp_.max_ray_length_ = 5;
    map.md_.has_first_depth_ = true;
    map.md_.depth_image_ = cv::Mat(rows, columns, CV_16UC1, pixels.data());
    map.md_.camera_r_m_ = Eigen::Matrix3d::Identity();
    map.md_.last_camera_r_m_ = Eigen::Matrix3d::Identity();
    map.md_.camera_pos_ = Eigen::Vector3d::Zero();
    map.md_.proj_points_.resize(initial_capacity);
    map.projectDepthImage();
    if (first_depth && map.md_.proj_points_cnt > 0) {
      *first_depth = map.md_.proj_points_[0].z();
    }
    return map.md_.proj_points_cnt;
  }
};

TEST(DepthProjectionMemory, NonDivisibleSamplingUsesCeilingCapacity)
{
  EXPECT_EQ(GridMapDepthProjectionTestPeer::project(480, 640, 3, false, 640 * 480 / 3 / 3), 34240);
}

TEST(DepthProjectionMemory, FilteredLastPixelNeverReadsNextPixel)
{
  EXPECT_EQ(GridMapDepthProjectionTestPeer::project(1, 1, 1, true, 10), 1);
}

TEST(DepthProjectionMemory, NonStandardDimensionsUseActualImageCapacity)
{
  EXPECT_EQ(GridMapDepthProjectionTestPeer::project(481, 641, 2, false, 640 * 480 / 2 / 2), 77361);
}

TEST(DepthProjectionMemory, FilteredStrideExceedingDimensionsRemainsInBounds)
{
  EXPECT_EQ(GridMapDepthProjectionTestPeer::project(1, 1, 3, true, 10), 1);
}

TEST(DepthProjectionMemory, ZeroFollowingPixelDoesNotChangeCurrentDepth)
{
  double first_depth = 0;
  EXPECT_EQ(GridMapDepthProjectionTestPeer::project(1, 2, 1, true, 10,
      &first_depth, true), 2);
  EXPECT_DOUBLE_EQ(first_depth, 1.0);
}
