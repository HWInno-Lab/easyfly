// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_multi_object_tracking/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
#include "easyfly_perception_contracts/backend_status.hpp"
// Copyright 2026 EasyFly Project
//

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
#include "easyfly_backend_registry/configuration_yaml.hpp"
#include <opencv2/opencv.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <easyfly_msgs/msg/detection_info_array.hpp>
#include <easyfly_msgs/msg/track_info_array.hpp>

#include "easyfly_multi_object_tracking/adapters/tracker_factory.hpp"
#include "easyfly_perception_contracts/contracts.hpp"
#include "easyfly_track_id_fusion/id_consolidator.hpp"

namespace easyfly_multi_object_tracking
{
namespace tr = easyfly::tracking;
namespace pc = easyfly::perception;
using easyfly::track_id_fusion::IdConsolidator;
using easyfly::track_id_fusion::WorldObs;

class TrackingNode : public rclcpp::Node
{
public:
  explicit TrackingNode(const rclcpp::NodeOptions & options)
  : Node("easyfly_multi_object_tracking", options)
  {
    loadParams();

    backend_status_ = std::make_unique<pc::BackendStatus>(*this, "multi_tracking", backend_config_.backend, .5);
    backend_config_ = tracker_factory().resolve(backend_config_);
    tracker_ = make_tracker(backend_config_);
    backend_status_->created(tracker_->capabilities().backend, easyfly::backends::parameters_json(backend_config_.parameters));
    const auto version = declare_parameter<int>("perception_version", 1, pc::frozen());
    if (version != 1 && version != 2) {throw std::invalid_argument("perception_version must be 1 or 2");}
    v2_ = version == 2;
    legacy_publish_ = declare_parameter<bool>("publish_legacy_compatibility", !v2_, pc::frozen());
    require_supported_inputs(tracker_->capabilities(), !v2_);
    camera_ = declare_parameter<std::string>("camera_id", "front", pc::frozen());
    optical_ = declare_parameter<std::string>("optical_frame", "camera_color_optical_frame", pc::frozen());
    v2_tracks_ = create_publisher<easyfly_msgs::msg::ImageTrackArrayV2>("/perception/v2/tracks", 10);
    v2_raw_ = create_publisher<easyfly_msgs::msg::ImageTrackArrayV2>("/perception/v2/tracks_raw", 10);

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

    if (!v2_) {det_sub_ = create_subscription<easyfly_msgs::msg::DetectionInfoArray>(
      params_.input_topic, 10,
      [this](easyfly_msgs::msg::DetectionInfoArray::ConstSharedPtr msg) {
        onDetections(msg);
      });

    }
    if (v2_) {
      v2_det_sub_ = create_subscription<easyfly_msgs::msg::DetectionArrayV2>(
        "/perception/v2/detections", 10, [this](easyfly_msgs::msg::DetectionArrayV2::ConstSharedPtr m,
        const rclcpp::MessageInfo & info) {onV2Detections(m, info);});
      v2_world_sub_ = create_subscription<easyfly_msgs::msg::WorldTargetArrayV2>(
        "/perception/v2/world_targets", 10, [this](easyfly_msgs::msg::WorldTargetArrayV2::ConstSharedPtr m,
        const rclcpp::MessageInfo & info) {onV2World(m, info);});
    }
    // Q19: 世界观测快照 (id 整合的重关联依据)
    if (!v2_) {world_sub_ = create_subscription<easyfly_msgs::msg::Target3DArray>(
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

    }
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
        if (legacy_publish_) {tracks_raw_pub_->publish(empty); tracks_pub_->publish(empty);}
        if (v2_) {publishV2Error(empty.header, pc::Context::NOT_READY, "explicit_reset");}
        response->success = true;
        response->message = "history cleared; raw IDs remain process-unique";
      });

    RCLCPP_INFO(
      get_logger(),
      "跟踪节点启动: %s → %s (backend=%s, 类别过滤=%s)",
      params_.input_topic.c_str(), params_.output_topic.c_str(), params_.tracker_backend.c_str(),
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
    declare_parameter<std::string>("config_file", "", pc::frozen());
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
    if (params_.tracker_backend.empty()) params_.tracker_backend = t["tracker_backend"].as<std::string>("sort");
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

    namespace b = easyfly::backends;
    backend_config_.backend = params_.tracker_backend;
    try {b::require_backend("multi_tracking", backend_config_.backend);}
    catch (const std::invalid_argument &) {throw std::invalid_argument("unknown tracking backend: " + backend_config_.backend);}
    // Reviewed V1 YAML import only. New backends use the dedicated mapping.
    const std::vector<std::string> old_keys{"iou_threshold", "max_lost_frames", "min_hits", "max_tracks", "dist_gate_scale"};
    YAML::Node legacy(YAML::NodeType::Map);
    for (const auto & key : old_keys) if (t[key]) legacy[key] = t[key];
    const auto native = b::parameters_from_yaml(t["backend_parameters"], "multi_tracking", backend_config_.backend);
    if (legacy.size()) {
      if (backend_config_.backend != "sort") throw std::invalid_argument("legacy SORT fields conflict with selected backend");
      backend_config_.parameters = b::parameters_from_yaml(legacy, "multi_tracking", "sort");
      for (const auto & entry : native) {
        auto it = backend_config_.parameters.find(entry.first);
        if (it != backend_config_.parameters.end() && it->second != entry.second)
          throw std::invalid_argument("legacy/new tracking parameter conflict: " + entry.first);
      }
    }
    b::overlay(backend_config_.parameters, native);
    const auto overrides = declare_parameter<std::string>("backend_parameters", "", descriptor);
    if (!overrides.empty()) b::overlay(backend_config_.parameters,
      b::parameters_from_yaml(YAML::Load(overrides), "multi_tracking", backend_config_.backend));
    params_.world_topic = t["world_topic"].as<std::string>(params_.world_topic);
    if (const auto consolidation = t["id_consolidation"]) {
      params_.cons_enabled = consolidation["enabled"].as<bool>(params_.cons_enabled);
      params_.cons_radius = consolidation["reacquire_radius"].as<double>(params_.cons_radius);
      params_.cons_ttl = consolidation["ttl"].as<double>(params_.cons_ttl);
    }

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
    if (v2_) {producer_.reset(); recent_outputs_.clear(); expected_world_rebase_ = true;}
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
    if (v2_) {frame = native_frame_;} else {
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

    }
    const auto input_status = tr::validate_frame(frame, tracker_->capabilities().inputs);
    if (input_status != tr::Status::ok) {publishEmpty(msg->header); return;}
    const auto result = tracker_->update(frame);
    if (!valid_tracking_output(result, frame, tracker_->capabilities(), !v2_)) {
      resetHistory(); publishEmpty(msg->header); return;
    }
    if (result.status != tr::Status::ok) {
      if (result.status == tr::Status::backend_error) {resetHistory();}
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "tracking input rejected: %s", result.detail.c_str());
      publishEmpty(msg->header);
      return;
    }
    const auto & tracks = result.tracks;
    std::map<int64_t, int32_t> aliases;
    for (const auto & t : tracks) {
      pc::Key key; key.tracker_session_id = producer_.session(); key.tracker_epoch = producer_.epoch(); key.raw_id = t.raw_id;
      aliases[t.raw_id] = v2_ ? raw_aliases_.get(pc::identity(key), t.raw_id) : static_cast<int32_t>(t.raw_id);
    }

    // Q19: 世界系 id 整合 — SORT id → 稳定 id
    std::vector<int> sort_ids;
    sort_ids.reserve(tracks.size());
    for (const auto & t : tracks) {sort_ids.push_back(aliases.at(t.raw_id));}
    std::map<int, WorldObs> world_by_sort;
    {
      std::lock_guard<std::mutex> lk(world_mtx_);
      const auto world_age = latest_world_ ? now().nanoseconds() - stamp(latest_world_->header.stamp) : -1;
      const auto receipt_age = std::chrono::duration<double>(std::chrono::steady_clock::now() - native_world_received_).count();
      if (latest_world_ && (!v2_ || (world_age >= -200000000LL && world_age <= 500000000LL && receipt_age <= .5))) {
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
      // A predicted box is not observed in this image. Pairing it with fresh
      // depth can turn background into a plausible but false 3D target.
      // Keep prediction in the stable/display stream below, not in fusion.
      if (t.missed_updates > 0) {continue;}
      easyfly_msgs::msg::TrackInfo ti;
      ti.header = msg->header;
      ti.track_id = aliases.at(t.raw_id);
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
    if (legacy_publish_ && !v2_) {tracks_raw_pub_->publish(raw);}

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
      ti.track_id = consolidator_->stableId(aliases.at(t.raw_id));
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
    if (legacy_publish_ && !v2_) {tracks_pub_->publish(out);}
    if (v2_) {
      easyfly_msgs::msg::ImageTrackArrayV2 value;
      value.context = producer_.next(msg->header, camera_, params_.image_width, params_.image_height);
      value.upstream = input_ref_; value.backend_id = params_.tracker_backend;
      for (const auto & t : tracks) {
        easyfly_msgs::msg::ImageTrackV2 item;
        item.key.tracker_session_id = producer_.session(); item.key.tracker_epoch = producer_.epoch();
        item.key.raw_id = t.raw_id; item.key.stable_id_valid = true;
        item.key.stable_id = consolidator_->stableId(aliases.at(t.raw_id));
        item.key.legacy_id_valid = true; item.key.legacy_raw_id = aliases.at(t.raw_id);
        item.key.legacy_stable_id = stable_aliases_.get(pc::hex(producer_.session()) + ":" +
          std::to_string(producer_.epoch()) + ":" + std::to_string(item.key.stable_id), item.key.stable_id);
        item.filtered_box.x1 = t.box.x1; item.filtered_box.y1 = t.box.y1;
        item.filtered_box.x2 = t.box.x2; item.filtered_box.y2 = t.box.y2;
        item.observed_box.x1 = t.last_observed_box.x1; item.observed_box.y1 = t.last_observed_box.y1;
        item.observed_box.x2 = t.last_observed_box.x2; item.observed_box.y2 = t.last_observed_box.y2;
        item.filtered_box_valid = true; item.observed_box_valid = pc::box_valid(item.observed_box);
        item.observed_this_frame = t.missed_updates == 0; item.confirmed = t.confirmed;
        item.hits = t.hits; item.age = t.age; item.missed_updates = t.missed_updates;
        item.class_id = t.class_id; item.class_name = t.class_name;
        item.confidence_valid = true; item.confidence = t.confidence;
        item.image_velocity_valid = true;
        item.image_velocity_unit = result.velocity_unit == tr::VelocityUnit::pixels_per_frame ?
          item.PIXELS_PER_UPDATE : item.PIXELS_PER_SECOND;
        item.image_velocity_x = t.vx; item.image_velocity_y = t.vy;
        value.tracks.push_back(item);
      }
      if (!pc::valid(value)) {publishV2Error(msg->header, pc::Context::BACKEND_ERROR, "invalid_backend_output"); return;}
      recent_outputs_[value.context.sequence] = {value.context, input_ref_};
      while (recent_outputs_.size() > 64) {recent_outputs_.erase(recent_outputs_.begin());}
      v2_tracks_->publish(value);
      if (legacy_publish_) {tracks_pub_->publish(pc::numeric_snapshot(value, false, true));}
      value.tracks.erase(std::remove_if(value.tracks.begin(), value.tracks.end(),
        [](const auto & t) {return !t.observed_this_frame;}), value.tracks.end());
      v2_raw_->publish(value);
      if (legacy_publish_) {tracks_raw_pub_->publish(pc::numeric_snapshot(value, true));}
    }

    backend_status_->input(true, "", stamp(msg->header.stamp));
    // 调试图像: 在最近一帧检测调试图上叠加 track_id
    if (params_.publish_debug_image && last_image_ &&
      debug_pub_->get_subscription_count() > 0)
    {
      publishDebugImage(tracks, msg->header);
    }
  }

  void publishEmpty(const std_msgs::msg::Header & header)
  {
    backend_status_->input(false, "tracking_input_rejected");
    easyfly_msgs::msg::TrackInfoArray empty;
    empty.header = header;
    if (legacy_publish_) {tracks_raw_pub_->publish(empty); tracks_pub_->publish(empty);}
    if (v2_) {publishV2Error(header, pc::Context::INVALID_INPUT, "tracking_input_rejected");}
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
          consolidator_->stableId(rawAlias(t.raw_id)),
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

  int32_t rawAlias(int64_t id)
  {
    if (!v2_) {return static_cast<int32_t>(id);}
    pc::Key key; key.tracker_session_id = producer_.session(); key.tracker_epoch = producer_.epoch(); key.raw_id = id;
    return raw_aliases_.get(pc::identity(key), id);
  }
  void publishV2Error(const std_msgs::msg::Header & h, uint8_t status, const std::string & reason)
  {
    if (backend_status_) {
      if (status == pc::Context::BACKEND_ERROR) backend_status_->failed(reason);
      else backend_status_->input(false, reason);
    }
    easyfly_msgs::msg::ImageTrackArrayV2 m;
    m.context = producer_.next(h, camera_, params_.image_width, params_.image_height, status, reason);
    m.upstream = input_ref_; m.backend_id = params_.tracker_backend;
    v2_tracks_->publish(m); v2_raw_->publish(m);
  }
  void onV2Detections(easyfly_msgs::msg::DetectionArrayV2::ConstSharedPtr m, const rclcpp::MessageInfo & info)
  {
    auto source = pc::writer(*this, "/perception/v2/detections", info);
    if (source.empty() || m->context.camera_id != camera_ || m->context.header.frame_id != optical_) {
      publishV2Error(m->context.header, pc::Context::INVALID_INPUT, "source_binding_rejected"); return;
    }
    const auto admission = detection_gate_.observe(m->context, source);
    if (admission == pc::SourceGate::rejected) {return;}
    if (admission == pc::SourceGate::changed || int(m->context.image_width) != params_.image_width ||
      int(m->context.image_height) != params_.image_height) {resetHistory();}
    input_ref_ = pc::reference(m->context);
    const auto source_age = now().nanoseconds() - pc::stamp(m->context.header.stamp);
    if (source_age < -200000000LL || source_age > 500000000LL) {publishV2Error(m->context.header, pc::Context::STALE, "stale_detections"); return;}
    if (m->context.status != pc::Context::VALID) {
      publishV2Error(m->context.header, m->context.status, "upstream:" + m->context.reason); return;
    }
    if (!pc::valid(*m)) {publishV2Error(m->context.header, pc::Context::INVALID_INPUT, "invalid_detection_envelope"); return;}
    params_.image_width = m->context.image_width; params_.image_height = m->context.image_height;
    native_frame_ = {}; native_frame_.source = {epoch_, pc::stamp(m->context.header.stamp)};
    native_frame_.image_width = m->context.image_width; native_frame_.image_height = m->context.image_height;
    auto legacy = std::make_shared<easyfly_msgs::msg::DetectionInfoArray>(); legacy->header = m->context.header;
    for (const auto & d : m->detections) {
      if (!d.confidence_valid) {publishV2Error(m->context.header, pc::Context::UNSUPPORTED_INPUT, "unknown_detection_confidence"); return;}
      if (!classWanted(d.class_name)) {continue;}
      tr::Detection input; input.box = {d.box.x1, d.box.y1, d.box.x2, d.box.y2};
      input.confidence = d.confidence; input.class_name = d.class_name; input.class_id = d.class_id;
      native_frame_.detections.push_back(input);
      easyfly_msgs::msg::DetectionInfo item; item.header = legacy->header; item.class_name = d.class_name;
      item.confidence = d.confidence;
      item.bbox_center_x = (d.box.x1+d.box.x2)/2/params_.image_width;
      item.bbox_center_y = (d.box.y1+d.box.y2)/2/params_.image_height;
      item.bbox_width = (d.box.x2-d.box.x1)/params_.image_width;
      item.bbox_height = (d.box.y2-d.box.y1)/params_.image_height; legacy->detections.push_back(item);
    }
    try {onDetections(legacy);} catch (const std::overflow_error & error) {
      publishV2Error(m->context.header, pc::Context::RESOURCE_LIMIT, error.what());
    }
  }
  void onV2World(easyfly_msgs::msg::WorldTargetArrayV2::ConstSharedPtr m, const rclcpp::MessageInfo & info)
  {
    auto source = pc::writer(*this, "/perception/v2/world_targets", info);
    if (source.empty() || m->context.camera_id != camera_ ||
      m->upstream.producer_session_id != producer_.session() || m->upstream.epoch != producer_.epoch()) {return;}
    const auto found = recent_outputs_.find(m->upstream.sequence);
    if (found == recent_outputs_.end() || !pc::ref_matches(m->upstream, found->second.first)) {return;}
    for (const auto & t : m->targets) {if (t.observation != found->second.second) {return;}}
    const auto admission = world_gate_.observe(m->context, source);
    if (admission == pc::SourceGate::rejected) {return;}
    if (admission == pc::SourceGate::changed && !expected_world_rebase_) {resetHistory(); return;}
    expected_world_rebase_ = false;
    if (!pc::valid(*m)) {std::lock_guard<std::mutex> lock(world_mtx_); latest_world_.reset(); return;}
    const auto age = (now() - rclcpp::Time(m->context.header.stamp)).seconds();
    if (age < -.2 || age > .5) {return;}
    easyfly_msgs::msg::Target3DArray numeric;
    try {numeric = pc::numeric_snapshot(*m);} catch (const std::exception &) {return;}
    auto snapshot = std::make_shared<easyfly_msgs::msg::Target3DArray>(numeric);
    std::lock_guard<std::mutex> lock(world_mtx_); latest_world_ = snapshot; native_world_received_ = std::chrono::steady_clock::now();
  }
  bool v2_{false}, legacy_publish_{true}, expected_world_rebase_{true};
  std::string camera_, optical_;
  std::chrono::steady_clock::time_point native_world_received_{};
  tr::Frame native_frame_;
  pc::Producer producer_; pc::SourceGate detection_gate_, world_gate_;
  pc::Aliases raw_aliases_, stable_aliases_; pc::Ref input_ref_;
  std::map<uint64_t, std::pair<pc::Context, pc::Ref>> recent_outputs_;
  rclcpp::Subscription<easyfly_msgs::msg::DetectionArrayV2>::SharedPtr v2_det_sub_;
  rclcpp::Subscription<easyfly_msgs::msg::WorldTargetArrayV2>::SharedPtr v2_world_sub_;
  rclcpp::Publisher<easyfly_msgs::msg::ImageTrackArrayV2>::SharedPtr v2_tracks_, v2_raw_;
  int classId(const std::string & name)
  {
    auto it = class_ids_.find(name);
    if (it != class_ids_.end()) {return it->second;}
    const int id = static_cast<int>(class_ids_.size());
    class_ids_[name] = id;
    return id;
  }

  NodeParams params_;
  std::unique_ptr<pc::BackendStatus> backend_status_;
  easyfly::backends::BackendConfig backend_config_;
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
