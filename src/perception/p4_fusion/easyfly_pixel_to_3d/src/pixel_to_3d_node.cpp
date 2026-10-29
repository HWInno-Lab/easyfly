// Copyright 2026 EasyFly Project
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
// EasyFly M5: 像素转 3D 定位节点 (pixel_to_3d)
//
// 数据流:
//   /tracks (M4, 带 ID 的 bbox, 归一化)
//   + /camera/aligned_depth_to_color/depth/image_raw (M0, 32FC1, 与 color 对齐)
//   + TF 树 (M1/M3)
//     → bbox 区域中位深度采样 (抗噪: 排除无效/边缘混合像素)
//     → 针孔反投影 (内参来自 /camera/color/camera_info, M1 校验过与 YAML 一致)
//     → TF: camera_color_optical_frame → map (或 odom)
//     → /target_positions (easyfly_msgs/Target3DArray)
//     → /target_markers   (visualization_msgs/MarkerArray, RViz 显示: 球+ID+距离)
//
// 深度无效策略: bbox 中心区域采样的有效像素 < min_valid_ratio → position_valid=false
#include <yaml-cpp/yaml.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <algorithm>
#include <cmath>
#include <deque>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <easyfly_msgs/msg/track_info_array.hpp>
#include <easyfly_msgs/msg/target3_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "easyfly_pixel_to_3d/frame_contract.hpp"
#include "easyfly_projection/projection.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
namespace easyfly_pixel_to_3d
{
using easyfly_msgs::msg::Target3D;
using easyfly_msgs::msg::Target3DArray;
class PixelTo3dNode : public rclcpp::Node
{
public:
  explicit PixelTo3dNode(const rclcpp::NodeOptions & options)
  : Node("easyfly_pixel_to_3d", options)
  {
    loadParams();
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      params_.camera_info_topic, 10,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        if (!cam_info_) {
          cam_info_ = msg;
          RCLCPP_INFO(
            get_logger(), "相机内参就绪: %dx%d fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
            msg->width, msg->height, msg->k[0], msg->k[4], msg->k[2], msg->k[5]);
        }
      });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      params_.depth_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        // Q20: 环形缓冲最近 N 帧深度, 供按 tracks 时间戳匹配
        depth_buf_.push_back(msg);
        if (depth_buf_.size() > 10) {depth_buf_.pop_front();}
      });
    tracks_sub_ = create_subscription<easyfly_msgs::msg::TrackInfoArray>(
      params_.input_topic, 10,
      [this](easyfly_msgs::msg::TrackInfoArray::ConstSharedPtr msg) {onTracks(msg);});
    target_pub_ = create_publisher<Target3DArray>(params_.output_topic, 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      params_.marker_topic, 10);
    RCLCPP_INFO(
      get_logger(), "pixel_to_3d 启动: %s + 深度 → %s (输出系: %s)",
      params_.input_topic.c_str(), params_.output_topic.c_str(),
      params_.target_frame.c_str());
  }

private:
  struct NodeParams
  {
    std::string input_topic{"/tracks"};
    std::string output_topic{"/target_positions"};
    std::string marker_topic{"/target_markers"};
    std::string depth_topic{"/camera/aligned_depth_to_color/depth/image_raw"};
    std::string camera_info_topic{"/camera/color/camera_info"};
    std::string camera_optical_frame{"camera_color_optical_frame"};
    std::string target_frame{"odom"};
    double min_depth{0.3};                // 有效深度下限 [m]
    double max_depth{9.5};                // 有效深度上限 [m] (相机 far=10)
    double min_valid_ratio{0.1};          // bbox 内有效像素比例下限
    bool publish_markers{true};
    bool verbose{false};
  };
  void loadParams()
  {
    declare_parameter<std::string>("config_file", "");
    std::string cfg = get_parameter("config_file").as_string();
    if (cfg.empty()) {
      cfg = ament_index_cpp::get_package_share_directory("easyfly_pixel_to_3d") +
        "/config/pixel_to_3d.yaml";
    }
    YAML::Node root = YAML::LoadFile(cfg);
    const auto & p = root["pixel_to_3d"];
    params_.input_topic = p["input_topic"].as<std::string>("/tracks");
    params_.output_topic = p["output_topic"].as<std::string>("/target_positions");
    params_.marker_topic = p["marker_topic"].as<std::string>("/target_markers");
    params_.depth_topic = p["depth_topic"].as<std::string>(
      "/camera/aligned_depth_to_color/depth/image_raw");
    params_.camera_info_topic = p["camera_info_topic"].as<std::string>(
      "/camera/color/camera_info");
    params_.camera_optical_frame = p["camera_optical_frame"].as<std::string>(
      "camera_color_optical_frame");
    params_.target_frame = p["target_frame"].as<std::string>("odom");
    params_.min_depth = p["min_depth"].as<double>(0.3);
    params_.max_depth = p["max_depth"].as<double>(9.5);
    params_.min_valid_ratio = p["min_valid_ratio"].as<double>(0.1);
    params_.publish_markers = p["publish_markers"].as<bool>(true);
    params_.verbose = p["verbose"].as<bool>(false);
  }
  /// bbox 中心区域中位深度 (有效像素不足返回 -1，不回退到背景区域)
  /// Q20: 深度帧由调用方按 tracks 时间戳选定后传入 (而非"最新帧")
  float medianDepth(
    const easyfly_msgs::msg::TrackInfo & t,
    const sensor_msgs::msg::Image & depth) const
  {
    const std::uint16_t endian_probe = 1;
    const bool host_bigendian = *reinterpret_cast<const std::uint8_t *>(&endian_probe) == 0;
    if (depth.encoding != "32FC1" || depth.is_bigendian != host_bigendian ||
      depth.width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
      depth.height > static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {return -1.0f;}
    return easyfly::projection::centralMedianDepth(
      {t.bbox_center_x, t.bbox_center_y, t.bbox_width, t.bbox_height},
      {depth.data.data(), depth.data.size(), static_cast<int>(depth.width),
        static_cast<int>(depth.height), depth.step},
      {params_.min_depth, params_.max_depth, params_.min_valid_ratio});
  }
  void onTracks(const easyfly_msgs::msg::TrackInfoArray::ConstSharedPtr & msg)
  {
    if (!cam_info_) {
      return;                    // 内参未就绪
    }
    // Q20: 按 tracks 时间戳从深度环形缓冲中选最近帧 (原直接用最新帧,
    // 2m/s 飞行下时间差最多一帧 ~70ms → 目标 3D 位置偏差 ~14cm)
    if (depth_buf_.empty()) {return;}
    sensor_msgs::msg::Image::ConstSharedPtr depth_frame;
    double best_dt = std::numeric_limits<double>::max();
    const rclcpp::Time t_tracks(msg->header.stamp);
    for (auto it = depth_buf_.rbegin(); it != depth_buf_.rend(); ++it) {
      const double dt = std::abs((t_tracks - rclcpp::Time((*it)->header.stamp)).seconds());
      if (dt < best_dt) {best_dt = dt; depth_frame = *it;}
      if (dt > 1.0) {
        break;                    // 缓冲按时间序, 超过 1s 的更老帧无意义
      }
    }
    if (!depth_frame || best_dt > 0.2) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "深度帧与 tracks 时间差 %.0fms, 跳过本帧", best_dt * 1000.0);
      return;
    }
    Target3DArray out;
    out.targets.reserve(msg->tracks.size());
    // TF 查询: optical → target_frame
    // 首选 tracks 时间戳 (精确时空对齐); 失败 (含静态 TF stamp=0 引起的
    // TF_OLD_DATA / extrapolation) 则回退 latest —— 静态外参链用 latest 是安全的,
    // 动态链 (odom→base) 的微小延迟对 ~10Hz 目标定位影响可忽略。
    geometry_msgs::msg::TransformStamped tf_optical;
    bool tf_ok = false;
    try {
      tf_optical = tf_buffer_->lookupTransform(
        params_.target_frame, params_.camera_optical_frame,
        msg->header.stamp, rclcpp::Duration::from_seconds(0.02));
      tf_ok = true;
    } catch (const tf2::TransformException & e) {
      try {
        tf_optical = tf_buffer_->lookupTransform(
          params_.target_frame, params_.camera_optical_frame, tf2::TimePointZero);
        tf_ok = true;
      } catch (const tf2::TransformException & e2) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "TF 不可用 (%s), 跳过本帧", e2.what());
        return;
      }
    }
    if (!tf_ok) {return;}
    const double fx = cam_info_->k[0], fy = cam_info_->k[4];
    const double cx = cam_info_->k[2], cy = cam_info_->k[5];
    const int W = static_cast<int>(cam_info_->width);
    const int H = static_cast<int>(cam_info_->height);
    for (const auto & t : msg->tracks) {
      Target3D tgt;
      tgt.track_id = t.track_id;
      tgt.class_name = t.class_name;
      tgt.confidence = t.confidence;
      tgt.pixel_u = static_cast<int32_t>(t.bbox_center_x * W);
      tgt.pixel_v = static_cast<int32_t>(t.bbox_center_y * H);
      const float d = medianDepth(t, *depth_frame);
      tgt.distance = d > 0 ? d : 0.0f;
      if (d > 0) {
        // 反投影 (光学系: x右 y下 z前)
        const auto optical = easyfly::projection::pinhole(
          tgt.pixel_u, tgt.pixel_v, d, fx, fy, cx, cy);
        // TF 变换到目标系
        geometry_msgs::msg::Point p_cam, p_world;
        p_cam.x = optical[0]; p_cam.y = optical[1]; p_cam.z = optical[2];
        tf2::doTransform(p_cam, p_world, tf_optical);
        tgt.position_x = static_cast<float>(p_world.x);
        tgt.position_y = static_cast<float>(p_world.y);
        tgt.position_z = static_cast<float>(p_world.z);
        tgt.position_valid = true;
        if (params_.verbose) {
          RCLCPP_INFO(
            get_logger(), "#%d %s: pixel=(%d,%d) d=%.2f → (%.2f,%.2f,%.2f)",
            t.track_id, t.class_name.c_str(), tgt.pixel_u, tgt.pixel_v,
            d, p_world.x, p_world.y, p_world.z);
        }
      } else {
        tgt.position_valid = false;
      }
      out.targets.push_back(tgt);
    }
    // Positions above are expressed in target_frame. Preserve the tracks
    // timestamp for synchronization while making both message headers
    // describe that same coordinate frame.
    applyTargetFrameContract(out, msg->header, params_.target_frame);
    target_pub_->publish(out);
    if (params_.publish_markers) {publishMarkers(out);}
  }
  void publishMarkers(const Target3DArray & targets)
  {
    visualization_msgs::msg::MarkerArray ma;
    const rclcpp::Time now = this->now();
    // 先删除旧 marker
    visualization_msgs::msg::Marker del;
    del.header.frame_id = params_.target_frame;
    del.header.stamp = now;
    del.ns = "targets";
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(del);
    // 按类别配色
    auto colorFor = [](const std::string & cls) -> std::array<float, 3> {
        if (cls == "person") {
          return {1.0f, 0.2f, 0.2f};  // 红
        }
        if (cls == "car" || cls == "truck" || cls == "bus") {
          return {0.2f, 0.5f, 1.0f};  // 蓝
        }
        return {0.2f, 1.0f, 0.2f};                            // 绿
      };
    for (const auto & t : targets.targets) {
      if (!t.position_valid) {continue;}
      const auto col = colorFor(t.class_name);
      // 球
      visualization_msgs::msg::Marker m;
      m.header.frame_id = params_.target_frame;
      m.header.stamp = now;
      m.ns = "targets";
      m.id = t.track_id;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.pose.position.x = t.position_x;
      m.pose.position.y = t.position_y;
      m.pose.position.z = t.position_z;
      m.pose.orientation.w = 1.0;
      m.scale.x = m.scale.y = m.scale.z = 0.5;
      m.color.r = col[0]; m.color.g = col[1]; m.color.b = col[2]; m.color.a = 0.9;
      m.lifetime = rclcpp::Duration::from_seconds(1.0);
      ma.markers.push_back(m);
      // 文字标签
      visualization_msgs::msg::Marker txt;
      txt = m;
      txt.id = 10000 + t.track_id;
      txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      txt.pose.position.z += 0.45;
      txt.scale.x = txt.scale.y = txt.scale.z = 0.3;
      txt.color.r = txt.color.g = txt.color.b = 1.0; txt.color.a = 1.0;
      std::ostringstream oss;
      oss << "#" << t.track_id << " " << t.class_name << " " << std::fixed
          << std::setprecision(1) << t.distance << "m";
      txt.text = oss.str();
      ma.markers.push_back(txt);
    }
    marker_pub_->publish(ma);
  }
  NodeParams params_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr cam_info_;
  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> depth_buf_;    // Q20: 时间戳匹配用
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<easyfly_msgs::msg::TrackInfoArray>::SharedPtr tracks_sub_;
  rclcpp::Publisher<Target3DArray>::SharedPtr target_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};
}  // namespace easyfly_pixel_to_3d
RCLCPP_COMPONENTS_REGISTER_NODE(easyfly_pixel_to_3d::PixelTo3dNode)
