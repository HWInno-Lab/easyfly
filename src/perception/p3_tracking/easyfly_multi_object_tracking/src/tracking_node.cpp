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

// EasyFly M4: 多目标跟踪节点 (SORT)
//
// 数据流:
//   /detections (easyfly_msgs/DetectionInfoArray, M2 输出, 归一化 bbox)
//     → SORT (像素空间卡尔曼 + 匈牙利关联)
//     → /tracks (easyfly_msgs/TrackInfoArray, 稳定 track_id)
//     → /tracks_debug_image (叠加 track_id 的画框图像, 有订阅才发)
//
// Q19 id 整合 (id_consolidator.hpp): SORT 新建 id 时按世界坐标
// (/target_positions) 关联最近消失的同类别 id → 对外输出稳定 id。
// SORT id 单调递增永不复用, remap 无歧义。
//
// 注意: 检测帧的图像尺寸未知 (消息只带归一化坐标), 由首帧 camera_info
// 或参数 image_width/image_height 获取 (默认 640x480, 与仿真一致)。
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <easyfly_msgs/msg/target3_d_array.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "yaml-cpp/yaml.h"
#include <opencv2/opencv.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <easyfly_msgs/msg/detection_info_array.hpp>
#include <easyfly_msgs/msg/track_info_array.hpp>

#include "easyfly_multi_object_tracking/adapters/tracker_factory.hpp"
#include "easyfly_track_id_fusion/id_consolidator.hpp"

namespace easyfly_multi_object_tracking
{
namespace tr = easyfly::tracking;
using easyfly::track_id_fusion::IdConsolidator;
using easyfly::track_id_fusion::WorldObs;

class TrackingNode : public rclcpp::Node
{
public:
  explicit TrackingNode(const rclcpp::NodeOptions & options)
  : Node("easyfly_multi_object_tracking", options)
  {
    loadParams();

    tracker_ = make_tracker(params_.tracker_backend, sort_params_);

    tracks_pub_ = create_publisher<easyfly_msgs::msg::TrackInfoArray>(
      params_.output_topic, 10);
    // Q19: 原始 SORT id 通道 — pixel_to_3d 消费它, 使 /target_positions
    // 携带 SORT id 供整合器查世界坐标 (/tracks 的稳定 id 是 remap 后的,
    // 新 SORT id 在整合前查不到世界位置 → 永远无法合并的死锁)
    tracks_raw_pub_ = create_publisher<easyfly_msgs::msg::TrackInfoArray>(
      params_.raw_topic, 10);
    debug_pub_ = create_publisher<sensor_msgs::msg::Image>(
      params_.output_topic + "_debug_image", 10);
    if (params_.publish_debug_image) {
      image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        params_.debug_image_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {last_image_ = msg;});
    }

    det_sub_ = create_subscription<easyfly_msgs::msg::DetectionInfoArray>(
      params_.input_topic, 10,
      [this](easyfly_msgs::msg::DetectionInfoArray::ConstSharedPtr msg) {
        onDetections(msg);
      });

    // Q19: 世界观测快照 (id 整合的重关联依据)
    world_sub_ = create_subscription<easyfly_msgs::msg::Target3DArray>(
      params_.world_topic, 10,
      [this](easyfly_msgs::msg::Target3DArray::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lk(world_mtx_);
        latest_world_.reset();
        if (msg->header.frame_id != params_.world_frame || stamp(msg->header.stamp) <= 0 ||
        msg->targets.size() > 1024) {return;}
        std::map<int, bool> ids;
        for (const auto & target : msg->targets) {
          if (!target.position_valid) {continue;}
          if (target.track_id < 0 || !std::isfinite(target.position_x) ||
          !std::isfinite(target.position_y) || !std::isfinite(target.position_z) ||
          target.header.frame_id != msg->header.frame_id ||
          stamp(target.header.stamp) != stamp(msg->header.stamp) ||
          !ids.emplace(target.track_id, true).second) {return;}
        }
        latest_world_ = msg;
      });

    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "~/reset",
      [this](std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
        if (!resetHistory()) {
          response->success = false;
          response->message = "tracking epoch exhausted";
          return;
        }
        easyfly_msgs::msg::TrackInfoArray empty;
        empty.header.stamp = now();
        tracks_raw_pub_->publish(empty);
        tracks_pub_->publish(empty);
        response->success = true;
        response->message = "history cleared; raw IDs remain process-unique";
      });

    RCLCPP_INFO(
      get_logger(),
      "跟踪节点启动: %s → %s (iou=%.2f, max_lost=%d, min_hits=%d, 类别过滤=%s)",
      params_.input_topic.c_str(), params_.output_topic.c_str(),
      sort_params_.iou_threshold, sort_params_.max_lost_frames,
      sort_params_.min_hits,
      params_.target_classes.empty() ? "无(全部跟踪)" :
      std::to_string(params_.target_classes.size()).c_str());
  }

private:
  struct NodeParams
  {
    std::string tracker_backend{"sort"};
    std::string world_frame{"odom"};
    std::string input_topic{"/detections"};
    std::string output_topic{"/tracks"};
    bool publish_debug_image{true};
    std::string debug_image_topic{"/detections_debug_image"};
    int image_width{640};
    int image_height{480};
    std::vector<std::string> target_classes;      // 空 = 全部
    std::string world_topic{"/target_positions"};      // Q19 世界观测
    std::string raw_topic{"/tracks_raw"};      // Q19: 原始 SORT id (供 pixel_to_3d)
    bool cons_enabled{true};
    double cons_radius{3.0};
    double cons_ttl{10.0};
  };

  void loadParams()
  {
    declare_parameter<std::string>("config_file", "");
    std::string cfg = get_parameter("config_file").as_string();
    if (cfg.empty()) {
      cfg = ament_index_cpp::get_package_share_directory("easyfly_multi_object_tracking") +
        "/config/tracking.yaml";
    }

    YAML::Node root = YAML::LoadFile(cfg);
    const auto & t = root["tracking"];

    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    params_.tracker_backend = declare_parameter<std::string>(
      "tracker_backend", t["tracker_backend"].as<std::string>("sort"), descriptor);
    params_.world_frame = t["world_frame"].as<std::string>("odom");
    params_.input_topic = t["input_topic"].as<std::string>("/detections");
    params_.output_topic = t["output_topic"].as<std::string>("/tracks");
    params_.publish_debug_image = t["publish_debug_image"].as<bool>(true);
    params_.debug_image_topic = t["debug_image_topic"].as<std::string>("/detections_debug_image");
    params_.image_width = t["image_width"].as<int>(640);
    params_.image_height = t["image_height"].as<int>(480);
    if (params_.image_width <= 0 || params_.image_height <= 0 || params_.world_frame.empty()) {
      throw std::invalid_argument("invalid tracking image dimensions or world frame");
    }
    if (t["target_classes"]) {
      params_.target_classes = t["target_classes"].as<std::vector<std::string>>();
    }

    sort_params_.iou_threshold = t["iou_threshold"].as<double>(0.3);
    sort_params_.max_lost_frames = t["max_lost_frames"].as<int>(30);
    sort_params_.min_hits = t["min_hits"].as<int>(3);
    sort_params_.max_tracks = t["max_tracks"].as<int>(100);
    sort_params_.dist_gate_scale = t["dist_gate_scale"].as<double>(1.5);      // Q19
    params_.world_topic = t["world_topic"].as<std::string>(params_.world_topic);
    params_.cons_enabled = t["id_consolidation"]["enabled"]
      .as<bool>(params_.cons_enabled);
    params_.cons_radius = t["id_consolidation"]["reacquire_radius"]
      .as<double>(params_.cons_radius);
    params_.cons_ttl = t["id_consolidation"]["ttl"].as<double>(params_.cons_ttl);

    IdConsolidator::Params cp;
    cp.enabled = params_.cons_enabled;
    cp.reacquire_radius = params_.cons_radius;
    cp.ttl = params_.cons_ttl;
    if (!std::isfinite(cp.reacquire_radius) || cp.reacquire_radius <= 0 ||
      !std::isfinite(cp.ttl) || cp.ttl <= 0)
    {
      throw std::invalid_argument("invalid ID consolidation radius or TTL");
    }
    consolidator_ = std::make_unique<IdConsolidator>(cp);
  }

  bool classWanted(const std::string & name) const
  {
    if (params_.target_classes.empty()) {return true;}
    for (const auto & c : params_.target_classes) {
      if (c == name) {return true;}}
    return false;
  }

  static std::int64_t stamp(const builtin_interfaces::msg::Time & value)
  {
    if (value.sec < 0 || value.nanosec >= 1000000000u) {return 0;}
    return static_cast<std::int64_t>(value.sec) * 1000000000 + value.nanosec;
  }

  bool resetHistory()
  {
    if (epoch_ == std::numeric_limits<std::uint64_t>::max()) {return false;}
    tracker_->reset(++epoch_);
    consolidator_->reset();
    class_ids_.clear();
    last_image_.reset();
    std::lock_guard<std::mutex> lock(world_mtx_);
    latest_world_.reset();
    return true;
  }

  void onDetections(const easyfly_msgs::msg::DetectionInfoArray::ConstSharedPtr & msg)
  {
    // 归一化 → 像素
    tr::Frame frame;
    frame.source = {epoch_, stamp(msg->header.stamp)};
    frame.image_width = params_.image_width;
    frame.image_height = params_.image_height;
    if (msg->detections.size() > 1024) {
      publishEmpty(msg->header);
      return;
    }
    frame.detections.reserve(msg->detections.size());
    for (const auto & d : msg->detections) {
      if (!classWanted(d.class_name)) {continue;}
      tr::Detection di;
      di.box = tr::Box{
        (d.bbox_center_x - d.bbox_width / 2) * params_.image_width,
        (d.bbox_center_y - d.bbox_height / 2) * params_.image_height,
        (d.bbox_center_x + d.bbox_width / 2) * params_.image_width,
        (d.bbox_center_y + d.bbox_height / 2) * params_.image_height};
      di.confidence = d.confidence;
      di.class_name = d.class_name;
      // class_id: 用类别名映射 (SORT 内部只要求同类相等)
      di.class_id = classId(d.class_name);
      frame.detections.push_back(di);
    }

    const auto result = tracker_->update(frame);
    if (result.status != tr::Status::ok) {
      if (result.status == tr::Status::backend_error) {resetHistory();}
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "tracking input rejected: %s", result.detail.c_str());
      publishEmpty(msg->header);
      return;
    }
    const auto & tracks = result.tracks;

    // Q19: 世界系 id 整合 — SORT id → 稳定 id
    std::vector<int> sort_ids;
    sort_ids.reserve(tracks.size());
    for (const auto & t : tracks) {sort_ids.push_back(static_cast<int>(t.raw_id));}
    std::map<int, WorldObs> world_by_sort;
    {
      std::lock_guard<std::mutex> lk(world_mtx_);
      if (latest_world_) {
        for (const auto & t : latest_world_->targets) {
          if (!t.position_valid) {continue;}
          world_by_sort[t.track_id] = {t.position_x, t.position_y,
            t.position_z, t.class_name};
        }
      }
    }
    const double now_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    consolidator_->update(sort_ids, world_by_sort, now_s);

    // 原始 SORT id 通道 (remap 前): pixel_to_3d → /target_positions
    // (世界坐标按 SORT id 回流, 下一帧起整合器即可关联新 id)
    easyfly_msgs::msg::TrackInfoArray raw;
    raw.header = msg->header;
    raw.tracks.reserve(tracks.size());
    for (const auto & t : tracks) {
      if (!t.confirmed && t.missed_updates > 0) {continue;}
      easyfly_msgs::msg::TrackInfo ti;
      ti.header = msg->header;
      ti.track_id = static_cast<int>(t.raw_id);
      ti.class_name = t.class_name;
      ti.confidence = t.confidence;
      ti.bbox_center_x = (t.box.x1 + t.box.x2) / 2 / params_.image_width;
      ti.bbox_center_y = (t.box.y1 + t.box.y2) / 2 / params_.image_height;
      ti.bbox_width = (t.box.x2 - t.box.x1) / params_.image_width;
      ti.bbox_height = (t.box.y2 - t.box.y1) / params_.image_height;
      ti.velocity_x = t.vx;
      ti.velocity_y = t.vy;
      raw.tracks.push_back(ti);
    }
    tracks_raw_pub_->publish(raw);

    // 发布 TrackInfoArray (稳定 id)
    easyfly_msgs::msg::TrackInfoArray out;
    out.header = msg->header;
    out.tracks.reserve(tracks.size());
    for (const auto & t : tracks) {
      if (!t.confirmed && t.missed_updates > 0) {
        continue;                                                // 未确认且未命中不发布
      }
      easyfly_msgs::msg::TrackInfo ti;
      ti.header = msg->header;
      ti.track_id = consolidator_->stableId(static_cast<int>(t.raw_id));
      ti.class_name = t.class_name;
      ti.confidence = t.confidence;
      // 滤波框 (归一化)
      ti.bbox_center_x = (t.box.x1 + t.box.x2) / 2 / params_.image_width;
      ti.bbox_center_y = (t.box.y1 + t.box.y2) / 2 / params_.image_height;
      ti.bbox_width = (t.box.x2 - t.box.x1) / params_.image_width;
      ti.bbox_height = (t.box.y2 - t.box.y1) / params_.image_height;
      // 3D 字段 (米) 暂填 0 — M5 pixel_to_3d 接入后扩展
      ti.velocity_x = t.vx;        // 像素/frame (沿用字段, 语义在注释中说明)
      ti.velocity_y = t.vy;
      ti.velocity_z = 0.0f;
      out.tracks.push_back(ti);
    }
    tracks_pub_->publish(out);

    // 调试图像: 在最近一帧检测调试图上叠加 track_id
    if (params_.publish_debug_image && last_image_ &&
      debug_pub_->get_subscription_count() > 0)
    {
      publishDebugImage(tracks, msg->header);
    }
  }

  void publishEmpty(const std_msgs::msg::Header & header)
  {
    easyfly_msgs::msg::TrackInfoArray empty;
    empty.header = header;
    tracks_raw_pub_->publish(empty);
    tracks_pub_->publish(empty);
  }

  void publishDebugImage(
    const std::vector<tr::Track> & tracks,
    const std_msgs::msg::Header & header)
  {
    // rgb8 → cv::Mat (仅画框)
    cv::Mat img(last_image_->height, last_image_->width, CV_8UC3,
      const_cast<uint8_t *>(last_image_->data.data()), last_image_->step);
    for (const auto & t : tracks) {
      if (!t.confirmed) {continue;}
      const cv::Scalar color(0, 255, 255);        // 黄色 = track (区别于检测的绿色)
      cv::rectangle(
        img,
        cv::Point(cvRound(t.box.x1), cvRound(t.box.y1)),
        cv::Point(cvRound(t.box.x2), cvRound(t.box.y2)), color, 2);
      cv::putText(
        img, cv::format(
          "#%d %s",
          consolidator_->stableId(static_cast<int>(t.raw_id)),
          t.class_name.c_str()),
        cv::Point(cvRound(t.box.x1), cvRound(t.box.y1) - 4),
        cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
    }
    // BGR→rgb8 发布
    cv::Mat rgb;
    cv::cvtColor(img, rgb, cv::COLOR_BGR2RGB);
    sensor_msgs::msg::Image msg;
    msg.header = header;
    msg.height = rgb.rows;
    msg.width = rgb.cols;
    msg.encoding = "rgb8";
    msg.is_bigendian = false;
    msg.step = rgb.cols * 3;
    msg.data.assign(rgb.data, rgb.data + rgb.total() * rgb.elemSize());
    debug_pub_->publish(msg);
  }

  int classId(const std::string & name)
  {
    auto it = class_ids_.find(name);
    if (it != class_ids_.end()) {return it->second;}
    const int id = static_cast<int>(class_ids_.size());
    class_ids_[name] = id;
    return id;
  }

  NodeParams params_;
  easyfly::sort::SortParams sort_params_;
  std::unique_ptr<tr::Tracker> tracker_;
  std::uint64_t epoch_{0};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  std::unique_ptr<IdConsolidator> consolidator_;
  std::map<std::string, int> class_ids_;

  std::mutex world_mtx_;
  easyfly_msgs::msg::Target3DArray::ConstSharedPtr latest_world_;
  rclcpp::Subscription<easyfly_msgs::msg::Target3DArray>::SharedPtr world_sub_;
  rclcpp::Publisher<easyfly_msgs::msg::TrackInfoArray>::SharedPtr tracks_raw_pub_;

  rclcpp::Subscription<easyfly_msgs::msg::DetectionInfoArray>::SharedPtr det_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  sensor_msgs::msg::Image::ConstSharedPtr last_image_;
  rclcpp::Publisher<easyfly_msgs::msg::TrackInfoArray>::SharedPtr tracks_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
};

}  // namespace easyfly_multi_object_tracking

RCLCPP_COMPONENTS_REGISTER_NODE(easyfly_multi_object_tracking::TrackingNode)
