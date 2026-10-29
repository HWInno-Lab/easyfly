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

#include "easyfly_trajectory_optimizer/observation_grid.hpp"
#include <iostream>
#include <easyfly_msgs/msg/voxel_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <cstring>
#include <deque>
#include <iomanip>
#include <random>
#include <sstream>

namespace et = easyfly::trajectory;
class VoxelMapper : public rclcpp::Node
{
  using Cloud = sensor_msgs::msg::PointCloud2;
  struct Pending {Cloud::ConstSharedPtr cloud; et::Steady::time_point received;};

public:
  VoxelMapper()
  : Node("gcopter_voxel_map")
  {
    rcl_interfaces::msg::ParameterDescriptor d; d.read_only = true;
    auto low = declare_parameter<std::vector<double>>("map_min", {-4., -4., -.2}, d);
    auto high = declare_parameter<std::vector<double>>("map_max", {8., 4., 3.4}, d);
    grid_.resolution = declare_parameter<double>("resolution", .2, d);
    radius_ = declare_parameter<double>("inflation_radius", .65, d);
    if (low.size() != 3 || high.size() != 3 || !std::isfinite(radius_) || radius_ < .35 ||
      radius_ > 1. ||
      !std::isfinite(grid_.resolution) || grid_.resolution < .05 || grid_.resolution > 1.)
    {
      throw std::invalid_argument("map bounds or inflation");
    }
    for (int i = 0; i < 3; ++i) {
      if (!std::isfinite(low[i]) || !std::isfinite(high[i]) || high[i] <= low[i] ||
        std::abs(low[i]) > 10000 || std::abs(high[i]) > 10000 || high[i] - low[i] > 25.6)
      {
        throw std::invalid_argument("map extent");
      }
      grid_.origin[i] = low[i]; grid_.size[i] = std::ceil((high[i] - low[i]) / grid_.resolution);
    }
    if (grid_.size.minCoeff() <= 0 || grid_.size.maxCoeff() > 512 ||
      int64_t(grid_.size.x()) * grid_.size.y() * grid_.size.z() > 1000000)
    {
      throw std::invalid_argument("map cell bound");
    }
    grid_.data.assign(grid_.size.prod(), -1);
    if (!grid_.valid()) {throw std::invalid_argument("map layout");}
    const bool clearing = declare_parameter<bool>("dynamic_clearing", false, d);
    const int clear_scans = declare_parameter<int>("clear_observation_scans", 3, d);
    if (clear_scans < 3 || clear_scans > 20) {
      throw std::invalid_argument("clearing observation bound");
    }
    observations_ = std::make_unique<et::ObservationGrid>(grid_, clearing ? clear_scans : 0);
    rolling_ = declare_parameter<bool>("rolling_xy", false, d);
    auto world_min = declare_parameter<std::vector<double>>("world_xy_min", {-100., -100.}, d);
    auto world_max = declare_parameter<std::vector<double>>("world_xy_max", {100., 100.}, d);
    if (world_min.size() != 2 || world_max.size() != 2) {
      throw std::invalid_argument("rolling fence");
    }
    for (int i = 0; i < 2; ++i) {
      if (rolling_ && (!std::isfinite(world_min[i]) || !std::isfinite(world_max[i]) ||
        world_min[i] < -1000. || world_max[i] > 1000. ||
        world_max[i] - world_min[i] < grid_.size[i] * grid_.resolution ||
        grid_.origin[i] < world_min[i] ||
        grid_.origin[i] + grid_.size[i] * grid_.resolution > world_max[i]))
      {
        throw std::invalid_argument("rolling fence bounds");
      }
      world_min_[i] = world_min[i];world_max_[i] = world_max[i];
    }
    const int extent = std::ceil(radius_ / grid_.resolution) + 1;
    for (int z = -extent; z <= extent; ++z) {
      for (int y = -extent; y <= extent; ++y) {
        for (int x = -extent; x <= extent; ++x) {
          et::Cell delta{x, y, z};
          const et::Vec gap = (delta.cwiseAbs().array() - 1).max(0).cast<double>().matrix() *
            grid_.resolution;
          if (gap.norm() <= radius_ + 1e-9) {inflation_.push_back(delta);}
        }
      }
    }
    std::random_device rd; std::ostringstream id; for (int i = 0; i < 4; ++i) {
      id << std::hex << std::setw(8) << std::setfill('0') << rd();
    }
    session_ = id.str();
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_);
    pub_ = create_publisher<easyfly_msgs::msg::VoxelGrid>(
      "/planning/voxel_grid", rclcpp::QoS(
        1).reliable().transient_local());
    const auto uninflated_topic = declare_parameter<std::string>("uninflated_topic", "", d);
    if (!uninflated_topic.empty()) {
      raw_pub_ = create_publisher<easyfly_msgs::msg::VoxelGrid>(
        uninflated_topic, rclcpp::QoS(1).reliable().transient_local());
    }
    sub_ = create_subscription<Cloud>(
      "/livox/lidar/rays", rclcpp::QoS(2).reliable(),
      [this](Cloud::ConstSharedPtr p) {
        if (p->header.stamp.sec < 0 || p->header.stamp.nanosec >= 1000000000) {return;}
        const auto current = clockSample();
        const auto stamp = rclcpp::Time(
          p->header.stamp,
          get_clock()->get_clock_type()).nanoseconds();
        if (p->header.frame_id != "livox_frame" || stamp <= last_enqueued_ || !fresh(
          stamp,
          current))
        {
          return;
        }
        last_enqueued_ = stamp; if (pending_.size() == 10) {pending_.pop_front();}
        pending_.push_back({p, et::Steady::now()}); process();
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] {process();});
    publish(false);
  }

private:
  bool fresh(int64_t source, int64_t current) const
  {
    return source > 0 && current > 0 && current - source >= -200000000 &&
           current - source <= 1000000000;
  }
  int64_t clockSample()
  {
    const auto current = now().nanoseconds();
    if (current < clock_) {
      RCLCPP_WARN(
        get_logger(), "Voxel clock rollback: previous=%.9f current=%.9f",
        clock_ / 1e9, current / 1e9);
      observations_->reset(); pending_.clear(); ++generation_; last_origin_source_ = 0;
      last_source_ = last_enqueued_ = 0; valid_ = false; publish(false);
    }
    clock_ = current; return current;
  }
  void process()
  {
    if (!rclcpp::ok() || input_latched_) {return;}
    const auto current = clockSample();
    const auto publisher_count = get_publishers_info_by_topic("/livox/lidar/rays").size();
    if (publisher_count != 1) {
      if (valid_) {
        RCLCPP_WARN(get_logger(), "Voxel source publisher count changed: %zu", publisher_count);
        valid_ = false;publish(false);
      }
      return;
    }
    while (!pending_.empty()) {
      auto item = pending_.front(); const auto source = rclcpp::Time(
        item.cloud->header.stamp,
        get_clock()->get_clock_type()).nanoseconds();
      if (!fresh(source, current) || et::Steady::now() - item.received > std::chrono::seconds(1)) {
        pending_.pop_front(); continue;
      }
      Eigen::Isometry3d transform;
      try {
        transform =
          tf2::transformToEigen(
          buffer_->lookupTransform(
            "odom", item.cloud->header.frame_id,
            rclcpp::Time(item.cloud->header.stamp, get_clock()->get_clock_type())));
      } catch (const tf2::TransformException & e) {last_tf_error_ = e.what(); break;}
      pending_.pop_front();
      const auto work_started = et::Steady::now();
      if (!integrate(*item.cloud, transform)) {
        if (input_latched_) {valid_ = false;pending_.clear();publish(false);return;}
        continue;
      }
      last_source_ = source; last_received_ = item.received; last_stamp_ = item.cloud->header.stamp;
      valid_ = true; ++revision_; publish(true);
      last_work_seconds_ = std::chrono::duration<double>(et::Steady::now() - work_started).count();
    }
    if (valid_ &&
      (!fresh(
        last_source_,
        current) || et::Steady::now() - last_received_ > std::chrono::seconds(1) ||
      get_publishers_info_by_topic("/livox/lidar/rays").size() != 1))
    {
      RCLCPP_WARN(
        get_logger(),
        "Voxel input invalid: source_age=%.6f receipt_age=%.6f pending=%zu last_work=%.6f last_tf=%s",
        (current - last_source_) / 1e9,
        std::chrono::duration<double>(et::Steady::now() - last_received_).count(),
        pending_.size(), last_work_seconds_, last_tf_error_.c_str());
      valid_ = false; publish(false);
    }
  }
  bool moveWindow(const et::Vec & sensor, int64_t source)
  {
    if (!sensor.allFinite() || source <= 0) {return false;}
    if (last_origin_source_ && (source <= last_origin_source_ ||
      (sensor - last_origin_).norm() > std::max(1., 10. * (source - last_origin_source_) / 1e9)))
    {
      RCLCPP_ERROR(get_logger(), "Rolling map transform discontinuity; restart required");
      return false;
    }
    et::Cell offset = et::Cell::Zero();
    for (int i = 0; i < 2; ++i) {
      const double width = grid_.size[i] * grid_.resolution;
      if (sensor[i] < world_min_[i] || sensor[i] >= world_max_[i]) {return false;}
      if (std::abs(sensor[i] - (grid_.origin[i] + width * .5)) > width * .2) {
        const int minimum =
          static_cast<int>(std::ceil((world_min_[i] - grid_.origin[i]) / grid_.resolution - 1e-8));
        const int maximum =
          static_cast<int>(std::floor(
            (world_max_[i] - width - grid_.origin[i]) / grid_.resolution +
            1e-8));
        const int desired =
          static_cast<int>(std::floor(
            (sensor[i] - width * .5 - grid_.origin[i]) /
            grid_.resolution));
        offset[i] = std::clamp(desired, minimum, maximum);
      }
    }
    observations_->shift(offset);last_origin_ = sensor;last_origin_source_ = source;return true;
  }
  bool integrate(const Cloud & cloud, const Eigen::Isometry3d & transform)
  {
    if (cloud.is_bigendian || cloud.height != 1 || cloud.width == 0 || cloud.width > 50000 ||
      cloud.point_step != 24 || cloud.row_step != cloud.width * 24 ||
      cloud.data.size() != cloud.row_step || !transform.matrix().allFinite()) {return false;}
    const std::vector<std::string> names{"x", "y", "z", "return_type", "range_min", "range_max"};
    const int offsets[]{0, 4, 8, 12, 16, 20};
    if (cloud.fields.size() != names.size()) {return false;}
    for (size_t i = 0; i < names.size(); ++i) {
      if (cloud.fields[i].name != names[i] || cloud.fields[i].offset != unsigned(offsets[i]) ||
        cloud.fields[i].count != 1 || cloud.fields[i].datatype != (i == 3 ? 2 : 7)) {return false;}}
    struct Ray {et::Vec local; double length, minimum; uint8_t type;};
    std::vector<Ray> measured; measured.reserve(cloud.width);
    for (size_t i = 0; i < cloud.width; ++i) {
      const auto * bytes = cloud.data.data() + i * 24; float values[6]; std::memcpy(
        values, bytes,
        24);
      const uint8_t type = bytes[12]; et::Vec local{values[0], values[1], values[2]};
      const double length = local.norm(), minimum = values[4], maximum = values[5];
      if (type > 1 || !local.allFinite() || !std::isfinite(minimum) || !std::isfinite(maximum) ||
        minimum < .1 || maximum <= minimum || maximum > 100. || length <= minimum ||
        length > maximum + .01 ||
        (type == 0 && std::abs(length - maximum) > .01)) {continue;}
      measured.push_back({local, length, minimum, type});
    }
    if (measured.empty()) {return false;}
    const et::Vec origin = transform.translation(); size_t rays = 0;
    const int64_t source = int64_t(cloud.header.stamp.sec) * 1000000000LL +
      cloud.header.stamp.nanosec;
    if (rolling_ && !moveWindow(origin, source)) {input_latched_ = true;return false;}
    observations_->begin();
    for (const auto & ray:measured) {
      const auto & local = ray.local;
      const auto length = ray.length, minimum = ray.minimum;
      const auto type = ray.type;
      const et::Vec endpoint = transform * local, delta = endpoint - origin;
      // Clip the physical observed ray to the fixed map; no empty-space fill.
      double low = minimum / length, high = 1.;
      const et::Vec corner = grid_.origin + grid_.size.cast<double>() * grid_.resolution;
      bool crosses = true;
      for (int axis = 0; axis < 3; ++axis) {
        if (std::abs(delta[axis]) < 1e-12) {
          if (origin[axis] < grid_.origin[axis] || origin[axis] >= corner[axis]) {
            crosses = false;
          }
        } else {
          double a = (grid_.origin[axis] - origin[axis]) / delta[axis],
            b = (corner[axis] - origin[axis]) / delta[axis];
          if (a > b) {std::swap(a, b);} low = std::max(low, a); high = std::min(high, b);
        }
      }
      if (!crosses || high < low) {continue;}
      const int steps =
        std::max(1, int(std::ceil((high - low) * length / (grid_.resolution * .4))));
      const et::Cell hit = grid_.cell(endpoint);
      for (int j = 0; j <= steps; ++j) {
        const et::Cell c = grid_.cell(origin + delta * (low + (high - low) * double(j) / steps));
        if (!grid_.inside(c) || (type == 1 && c == hit)) {continue;}
        observations_->observeFree(c);
      }
      if (type == 1 && grid_.inside(hit)) {observations_->observeHit(hit);}
      ++rays;
    }
    observations_->commit();
    // The actual sensor mounting cell is occupied by the sensor itself. Only
    // resolve UNKNOWN there; never clear an observed obstacle or a radius.
    const auto sensor_cell = grid_.cell(origin);
    if (rays > 0 && grid_.inside(sensor_cell) && grid_.data[grid_.index(sensor_cell)] == -1) {
      grid_.data[grid_.index(sensor_cell)] = 0;
    }
    return rays > 0;
  }
  void publish(bool valid)
  {
    easyfly_msgs::msg::VoxelGrid msg; msg.header.frame_id = "odom";
    if (valid) {msg.header.stamp = last_stamp_;}
    msg.session_id = session_; msg.generation = generation_; msg.revision = revision_;
    msg.valid = valid;
    msg.resolution = grid_.resolution; msg.inflation_radius = radius_;
    msg.origin.x = grid_.origin.x(); msg.origin.y = grid_.origin.y();
    msg.origin.z = grid_.origin.z();
    msg.size_x = grid_.size.x(); msg.size_y = grid_.size.y(); msg.size_z = grid_.size.z();
    msg.data = valid ? grid_.data : std::vector<int8_t>(grid_.data.size(), -1);
    if (raw_pub_) {auto raw = msg;raw.inflation_radius = 0.;raw_pub_->publish(raw);}
    if (valid) {
      for (size_t i = 0; i < grid_.data.size(); ++i) {
        if (grid_.data[i] == 100) {
          const auto c = grid_.cell(i);
          for (const auto & offset:inflation_) {
            auto n = (c + offset).eval(); if (grid_.inside(n)) {
              msg.data[grid_.index(n)] = 100;
            }
          }
        }
      }
    }
    pub_->publish(msg);
  }
  std::unique_ptr<et::ObservationGrid> observations_;
  bool rolling_{false}, input_latched_{false};
  Eigen::Vector2d world_min_, world_max_;
  et::Vec last_origin_{et::Vec::Zero()};int64_t last_origin_source_{0};
  et::Grid grid_; double radius_; std::vector<et::Cell> inflation_;
  std::string session_, last_tf_error_; uint64_t generation_{0}, revision_{0};
  double last_work_seconds_{0.};
  int64_t clock_{0}, last_source_{0}, last_enqueued_{0}; bool valid_{false};
  builtin_interfaces::msg::Time last_stamp_; et::Steady::time_point last_received_;
  std::deque<Pending> pending_; std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Publisher<easyfly_msgs::msg::VoxelGrid>::SharedPtr pub_, raw_pub_;
  rclcpp::Subscription<Cloud>::SharedPtr sub_; rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<VoxelMapper>();
    rclcpp::spin(node);
  } catch (const rclcpp::exceptions::RCLError & e) {
    // SIGINT/SIGTERM may invalidate the context between the check and a
    // graph query in an already executing callback. Live ROS errors fail.
    if (rclcpp::ok()) {std::cerr << e.what() << '\n'; result = 1;}
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n'; result = 1;
  }
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return result;
}
