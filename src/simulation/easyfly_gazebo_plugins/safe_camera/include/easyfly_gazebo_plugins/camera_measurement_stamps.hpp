// Copyright 2026 EasyFly Project
// Licensed under the Apache License, Version 2.0.
// You may obtain a copy at http://www.apache.org/licenses/LICENSE-2.0
// Distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND.

#ifndef EASYFLY_GAZEBO_PLUGINS__CAMERA_MEASUREMENT_STAMPS_HPP_
#define EASYFLY_GAZEBO_PLUGINS__CAMERA_MEASUREMENT_STAMPS_HPP_

#include <map>

#include <gazebo/common/Time.hh>

namespace gazebo_plugins
{
namespace detail
{
// Access under the camera's image mutex, including the entire publication group.
class CameraMeasurementStamps
{
public:
  bool duplicate(int camera, const gazebo::common::Time & stamp) const
  {
    const auto found = published_.find(camera);
    return found != published_.end() && found->second == stamp;
  }

  void published(int camera, const gazebo::common::Time & stamp)
  {
    published_[camera] = stamp;
  }

private:
  std::map<int, gazebo::common::Time> published_;
};
}  // namespace detail
}  // namespace gazebo_plugins

#endif  // EASYFLY_GAZEBO_PLUGINS__CAMERA_MEASUREMENT_STAMPS_HPP_
