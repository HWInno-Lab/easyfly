// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cmath>
#include <utility>
#include "common_lib.h"

// UndistortPcl clears its output on the subsequent VIO pass. Retain one owned
// measured scan, then consume it only at that exact post-visual source time.
class AtlasScanSnapshot
{
  PointCloudXYZI cloud_;
  double stamp_ = 0.;
public:
  void remember(const PointCloudXYZI &cloud, double stamp)
  {
    cloud_.clear();
    stamp_ = 0.;
    if (!std::isfinite(stamp) || stamp <= 0. || cloud.size() < 100 || cloud.size() > 200000)
      return;
    cloud_ = cloud;
    stamp_ = stamp;
  }
  PointCloudXYZI take(double stamp)
  {
    PointCloudXYZI result;
    if (stamp == stamp_ && stamp > 0.)
      result.swap(cloud_);
    cloud_.clear();
    stamp_ = 0.;
    return result;
  }
};
