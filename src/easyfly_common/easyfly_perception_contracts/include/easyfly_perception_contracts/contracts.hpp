// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <functional>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <easyfly_msgs/msg/detection_array_v2.hpp>
#include <easyfly_msgs/msg/image_track_array_v2.hpp>
#include <easyfly_msgs/msg/world_target_array_v2.hpp>
#include <easyfly_msgs/msg/target_prediction_array_v2.hpp>
#include <easyfly_msgs/msg/target_prediction_array.hpp>
#include <easyfly_msgs/msg/single_track_state_v2.hpp>
#include <easyfly_msgs/msg/single_track_state.hpp>
#include <easyfly_msgs/msg/detection_info_array.hpp>
#include <easyfly_msgs/msg/track_info_array.hpp>
#include <easyfly_msgs/msg/target3_d_array.hpp>

namespace easyfly::perception
{
using Context = easyfly_msgs::msg::PerceptionContextV2;
using Ref = easyfly_msgs::msg::ObservationRefV2;
using Key = easyfly_msgs::msg::TrackKeyV2;
using Uuid = unique_identifier_msgs::msg::UUID;
inline rcl_interfaces::msg::ParameterDescriptor frozen()
{rcl_interfaces::msg::ParameterDescriptor d; d.read_only = true; return d;}
inline int64_t stamp(const builtin_interfaces::msg::Time & t)
{return t.sec < 0 || t.nanosec >= 1000000000u ? -1 : int64_t(t.sec) * 1000000000 + t.nanosec;}
inline bool uuid_valid(const Uuid & u)
{return std::any_of(u.uuid.begin(), u.uuid.end(), [](auto b) {return b != 0;});}
inline Uuid uuid()
{
  Uuid result; std::random_device entropy;
  for (auto & b : result.uuid) {b = static_cast<uint8_t>(entropy());}
  result.uuid[6] = (result.uuid[6] & 15) | 64;
  result.uuid[8] = (result.uuid[8] & 63) | 128;
  return result;
}
inline std::string hex(const Uuid & u)
{
  const char * digits = "0123456789abcdef"; std::string value;
  for (auto b : u.uuid) {value += digits[b >> 4]; value += digits[b & 15];}
  return value;
}
inline std::string identity(const Key & k)
{return hex(k.tracker_session_id) + ":" + std::to_string(k.tracker_epoch) + ":" + std::to_string(k.raw_id);}
inline bool key_valid(const Key & k)
{
  return uuid_valid(k.tracker_session_id) && k.raw_id >= 0 &&
    (!k.stable_id_valid || k.stable_id >= 0) &&
    (!k.legacy_id_valid || (k.legacy_raw_id >= 0 && k.legacy_stable_id >= 0));
}
inline Ref reference(const Context & c)
{Ref r; r.header = c.header; r.producer_session_id = c.producer_session_id; r.epoch = c.epoch; r.sequence = c.sequence; return r;}
inline bool ref_matches(const Ref & r, const Context & c)
{return r.header == c.header && r.producer_session_id == c.producer_session_id && r.epoch == c.epoch && r.sequence == c.sequence;}
inline bool ref_valid(const Ref & r)
{return uuid_valid(r.producer_session_id) && r.sequence > 0 && stamp(r.header.stamp) > 0 && !r.header.frame_id.empty() && r.header.frame_id.size() <= 256;}
inline bool context_valid(const Context & c)
{
  const bool camera = !c.camera_id.empty() && c.camera_id.size() <= 32 &&
    std::all_of(c.camera_id.begin(), c.camera_id.end(), [](unsigned char v) {
      return (v >= 'a' && v <= 'z') || (v >= 'A' && v <= 'Z') || (v >= '0' && v <= '9') || v == '_';});
  return camera && uuid_valid(c.producer_session_id) && c.sequence > 0 &&
    c.status <= Context::STALE && c.reason.size() <= 256 && c.header.frame_id.size() <= 256 &&
    (c.status != Context::VALID || (stamp(c.header.stamp) > 0 && !c.header.frame_id.empty() &&
    c.image_width > 0 && c.image_height > 0 && uint64_t(c.image_width) * c.image_height <= 2073600));
}
inline bool box_valid(const easyfly_msgs::msg::PixelBoxV2 & b)
{return std::isfinite(b.x1) && std::isfinite(b.y1) && std::isfinite(b.x2) && std::isfinite(b.y2) && b.x2 > b.x1 && b.y2 > b.y1 &&
  std::max({std::abs(b.x1), std::abs(b.y1), std::abs(b.x2), std::abs(b.y2)}) <= double(INT32_MAX) / 4;}
inline bool score(bool valid, double value)
{return !valid || (std::isfinite(value) && value >= 0 && value <= 1);}

class Producer
{
public:
  Producer() : session_(uuid()) {}
  void reset()
  {
    if (epoch_ == UINT64_MAX) {throw std::overflow_error("perception epoch exhausted");}
    ++epoch_; sequence_ = 0; last_stamp_ = 0;
  }
  Context next(const std_msgs::msg::Header & h, const std::string & camera, uint32_t w,
    uint32_t height, uint8_t status = Context::VALID, const std::string & reason = "")
  {
    const auto source = stamp(h.stamp);
    if ((source > 0 && source < last_stamp_) || sequence_ == UINT64_MAX) {reset();}
    Context c; c.header = h; c.camera_id = camera; c.image_width = w; c.image_height = height;
    c.producer_session_id = session_; c.epoch = epoch_; c.sequence = ++sequence_;
    c.status = status; c.reason = reason.substr(0, 256); last_stamp_ = std::max<int64_t>(0, source);
    return c;
  }
  const Uuid & session() const {return session_;}
  uint64_t epoch() const {return epoch_;}
private:
  Uuid session_; uint64_t epoch_{0}, sequence_{0}; int64_t last_stamp_{0};
};

// One owner per stream. Never evict retired sessions: exhaustion fails closed.
class SourceGate
{
public:
  enum Result {rejected, accepted, changed};
  Result observe(const Context & c, const std::string & writer = "")
  {
    if (!context_valid(c) || exhausted_) {return rejected;}
    const auto id = hex(c.producer_session_id);
    if (retired_.count(id)) {return rejected;}
    bool transition = !current_.empty() && (id != current_ || c.epoch != epoch_ || writer != writer_);
    if (id == current_ && (c.epoch < epoch_ || (writer != writer_ && !writer_.empty()))) {return rejected;}
    if (id != current_ && !current_.empty()) {
      if (retired_.size() >= 256) {exhausted_ = true; return rejected;}
      retired_.insert(current_);
    }
    if (current_.empty() || transition) {sequence_ = 0; stamp_ = 0;}
    if (c.sequence <= sequence_ || (c.status == Context::VALID && stamp(c.header.stamp) <= stamp_)) {return rejected;}
    current_ = id; epoch_ = c.epoch; writer_ = writer; sequence_ = c.sequence;
    if (c.status == Context::VALID) {stamp_ = stamp(c.header.stamp);}
    return transition ? changed : accepted;
  }
  bool matches(const Key & k) const
  {return hex(k.tracker_session_id) == current_ && k.tracker_epoch == epoch_;}
private:
  std::string current_, writer_; uint64_t epoch_{0}, sequence_{0}; int64_t stamp_{0};
  std::set<std::string> retired_; bool exhausted_{false};
};
inline std::string writer(rclcpp::Node & node, const std::string & topic, const rclcpp::MessageInfo & info)
{
  const auto endpoints = node.get_publishers_info_by_topic(node.get_node_topics_interface()->resolve_topic_name(topic));
  if (endpoints.size() != 1) {return {};}
  const auto & id = endpoints.front().endpoint_gid();
  if (std::memcmp(id.data(), info.get_rmw_message_info().publisher_gid.data, id.size()) != 0) {return {};}
  return std::string(reinterpret_cast<const char *>(id.data()), id.size());
}

class Aliases
{
public:
  int32_t get(const std::string & key, int64_t /* preferred */ = -1)
  {
    const auto found = ids_.find(key); if (found != ids_.end()) {return found->second;}
    if (ids_.size() >= 65536) {throw std::overflow_error("compatibility alias capacity exhausted");}
    // Dense aliases never consume int32 space merely because a backend raw ID is large.
    if (next_ >= INT32_MAX) {throw std::overflow_error("compatibility int32 IDs exhausted");}
    const auto value = static_cast<int32_t>(next_++); ids_[key] = value; return value;
  }
private:
  std::map<std::string, int32_t> ids_; int64_t next_{0};
};

inline bool valid(const easyfly_msgs::msg::DetectionArrayV2 & m)
{
  return context_valid(m.context) && m.context.status == Context::VALID && m.detections.size() <= 100 &&
    std::all_of(m.detections.begin(), m.detections.end(), [](const auto & d) {
      return box_valid(d.box) && d.class_id >= -1 && d.class_name.size() <= 128 && score(d.confidence_valid, d.confidence);});
}
inline bool valid(const easyfly_msgs::msg::ImageTrackArrayV2 & m)
{
  if (!context_valid(m.context) || m.context.status != Context::VALID || !ref_valid(m.upstream) || m.tracks.size() > 100) {return false;}
  std::set<std::string> keys;
  for (const auto & t : m.tracks) {
    if (!key_valid(t.key) || !keys.insert(identity(t.key)).second ||
      t.key.tracker_session_id != m.context.producer_session_id || t.key.tracker_epoch != m.context.epoch ||
      !t.filtered_box_valid || !box_valid(t.filtered_box) ||
      (t.observed_box_valid && !box_valid(t.observed_box)) ||
      (t.observed_this_frame && (!t.observed_box_valid || t.missed_updates != 0)) ||
      t.class_name.size() > 128 || !score(t.confidence_valid, t.confidence) ||
      t.image_velocity_unit > 2 || (t.image_velocity_valid && (t.image_velocity_unit == 0 ||
      !std::isfinite(t.image_velocity_x) || !std::isfinite(t.image_velocity_y)))) {return false;}
  }
  return true;
}
inline bool valid(const easyfly_msgs::msg::WorldTargetArrayV2 & m)
{
  if (!context_valid(m.context) || m.context.status != Context::VALID || !ref_valid(m.upstream) ||
    m.context.header.frame_id != "odom" || !m.transform_exact || stamp(m.depth_stamp) <= 0 ||
    m.context.header.stamp != m.upstream.header.stamp || m.transform_stamp != m.upstream.header.stamp || m.targets.size() > 100) {return false;}
  std::set<std::string> keys; std::set<int32_t> aliases;
  for (const auto & t : m.targets) {
    if (!key_valid(t.key) || !keys.insert(identity(t.key)).second || !ref_valid(t.observation) || t.observation.header != m.upstream.header ||
      t.key.tracker_session_id != m.upstream.producer_session_id || t.key.tracker_epoch != m.upstream.epoch ||
      (t.key.legacy_id_valid && !aliases.insert(t.key.legacy_raw_id).second) ||
      !score(t.confidence_valid, t.confidence) || t.class_name.size() > 128 ||
      (t.position_valid && (!std::isfinite(t.position_x) || !std::isfinite(t.position_y) ||
      !std::isfinite(t.position_z) || !std::isfinite(t.optical_depth_m) || t.optical_depth_m <= 0))) {return false;}
  }
  return true;
}

// V2-native adapters may use a validated legacy-shaped numeric snapshot internally.
// Full keys/context must be retained by the caller; this is never V1->V2 evidence.
inline easyfly_msgs::msg::Target3DArray numeric_snapshot(const easyfly_msgs::msg::WorldTargetArrayV2 & m)
{
  if (!valid(m)) {throw std::invalid_argument("invalid V2 world envelope");}
  easyfly_msgs::msg::Target3DArray result; result.header = m.context.header;
  for (const auto & t : m.targets) {
    if (!t.key.legacy_id_valid) {throw std::invalid_argument("missing producer-owned compatibility alias");}
    const double limit = std::numeric_limits<float>::max();
    if (std::abs(t.position_x) > limit || std::abs(t.position_y) > limit ||
      std::abs(t.position_z) > limit || std::abs(t.optical_depth_m) > limit) {
      throw std::invalid_argument("world coordinate exceeds compatibility view range");
    }
    easyfly_msgs::msg::Target3D d; d.header = result.header; d.track_id = t.key.legacy_raw_id;
    d.class_name = t.class_name; d.confidence = t.confidence_valid ? t.confidence : 0;
    d.position_x = t.position_x; d.position_y = t.position_y; d.position_z = t.position_z;
    d.position_valid = t.position_valid; d.distance = t.optical_depth_m; d.pixel_u = t.pixel_u; d.pixel_v = t.pixel_v;
    result.targets.push_back(d);
  }
  return result;
}
inline easyfly_msgs::msg::TrackInfoArray numeric_snapshot(const easyfly_msgs::msg::ImageTrackArrayV2 & m,
  bool observed_only = false, bool stable = false)
{
  if (!valid(m)) {throw std::invalid_argument("invalid V2 tracks");}
  easyfly_msgs::msg::TrackInfoArray out; out.header = m.context.header;
  for (const auto & t : m.tracks) {
    if (observed_only && !t.observed_this_frame) {continue;}
    if (!t.key.legacy_id_valid) {throw std::invalid_argument("missing producer alias");}
    const auto & b = observed_only ? t.observed_box : t.filtered_box;
    easyfly_msgs::msg::TrackInfo v; v.header = out.header;
    v.track_id = stable ? t.key.legacy_stable_id : t.key.legacy_raw_id;
    v.class_name = t.class_name; v.confidence = t.confidence_valid ? t.confidence : 0;
    v.bbox_center_x = (b.x1 + b.x2) / 2 / m.context.image_width;
    v.bbox_center_y = (b.y1 + b.y2) / 2 / m.context.image_height;
    v.bbox_width = (b.x2 - b.x1) / m.context.image_width;
    v.bbox_height = (b.y2 - b.y1) / m.context.image_height;
    // Legacy velocity has ambiguous units: never copy a V2 velocity into it.
    out.tracks.push_back(v);
  }
  return out;
}

inline easyfly_msgs::msg::DetectionInfoArray numeric_snapshot(const easyfly_msgs::msg::DetectionArrayV2 & m)
{
  if (!valid(m)) {throw std::invalid_argument("invalid native detection");}
  easyfly_msgs::msg::DetectionInfoArray out; out.header = m.context.header;
  for (const auto & t : m.detections) {
    easyfly_msgs::msg::DetectionInfo v; v.header = out.header;
    v.class_name = t.class_name; v.confidence = t.confidence_valid ? t.confidence : 0;
    v.bbox_center_x = (t.box.x1 + t.box.x2) / 2 / m.context.image_width;
    v.bbox_center_y = (t.box.y1 + t.box.y2) / 2 / m.context.image_height;
    v.bbox_width = (t.box.x2 - t.box.x1) / m.context.image_width;
    v.bbox_height = (t.box.y2 - t.box.y1) / m.context.image_height; out.detections.push_back(v);
  }
  return out;
}
inline bool valid(const easyfly_msgs::msg::TargetPredictionArrayV2 & m)
{
  if (!context_valid(m.context) || m.context.status != Context::VALID ||
    m.context.header.frame_id != "odom" || !ref_valid(m.upstream) || m.predictions.size() > 100) {return false;}
  std::set<std::string> keys;
  for (const auto & t : m.predictions) {
    if (!key_valid(t.key) || !keys.insert(identity(t.key)).second || !ref_valid(t.observation) ||
      t.class_name.size() > 128 || t.reason.size() > 256 || !score(t.confidence_valid, t.confidence) ||
      t.header != m.context.header || t.observation_header.frame_id != "odom" || stamp(t.observation_header.stamp) <= 0 ||
      !std::isfinite(t.horizon_seconds) || t.horizon_seconds < 0 || t.horizon_seconds > 10 ||
      !std::isfinite(t.observation_age_seconds) || t.observation_age_seconds < 0 ||
      (t.valid && (!std::isfinite(t.position_x) || !std::isfinite(t.position_y) || !std::isfinite(t.position_z) ||
      !std::isfinite(t.velocity_x) || !std::isfinite(t.velocity_y) || !std::isfinite(t.velocity_z) ||
      !std::isfinite(t.variance_x) || !std::isfinite(t.variance_y) || !std::isfinite(t.variance_z) ||
      std::min({t.variance_x, t.variance_y, t.variance_z}) < 0))) {return false;}
  }
  return true;
}
inline easyfly_msgs::msg::TargetPredictionArray numeric_snapshot(const easyfly_msgs::msg::TargetPredictionArrayV2 & m)
{
  if (!valid(m)) {throw std::invalid_argument("invalid native prediction");}
  easyfly_msgs::msg::TargetPredictionArray out; out.header = m.context.header;
  for (const auto & t : m.predictions) {
    if (!t.key.legacy_id_valid) {throw std::invalid_argument("missing producer alias");}
    easyfly_msgs::msg::TargetPrediction v;
    v.header = t.header; v.observation_header = t.observation_header; v.track_id = t.key.legacy_raw_id;
    v.class_name = t.class_name; v.confidence = t.confidence_valid ? t.confidence : 0;
    v.horizon_seconds = t.horizon_seconds; v.observation_age_seconds = t.observation_age_seconds;
    v.position_x = t.position_x; v.position_y = t.position_y; v.position_z = t.position_z;
    v.velocity_x = t.velocity_x; v.velocity_y = t.velocity_y; v.velocity_z = t.velocity_z;
    v.variance_x = t.variance_x; v.variance_y = t.variance_y; v.variance_z = t.variance_z;
    v.valid = t.valid; v.reason = t.reason; out.predictions.push_back(v);
  }
  return out;
}
inline easyfly_msgs::msg::SingleTrackState numeric_snapshot(const easyfly_msgs::msg::SingleTrackStateV2 & m)
{
  if (!context_valid(m.context) || m.phase.size() > 32 || m.reason.size() > 256) {
    throw std::invalid_argument("invalid native single state");
  }
  easyfly_msgs::msg::SingleTrackState out; out.header = m.context.header;
  out.phase = m.phase; out.reason = m.reason; out.valid = m.valid; out.track_id = -1;
  if (m.valid) {
    easyfly_msgs::msg::ImageTrackArrayV2 frame; frame.context = m.context;
    frame.upstream = reference(m.context); frame.tracks = {m.track};
    out.track = numeric_snapshot(frame).tracks.at(0); out.track_id = out.track.track_id;
  }
  return out;
}

// Owns admission and the native evidence; callbacks must retain full keys while using
// numerical compatibility views. A source transition invalidates active operations first.
class WorldInput
{
public:
  using Message = easyfly_msgs::msg::WorldTargetArrayV2;
  WorldInput(rclcpp::Node & node, std::function<void(Message::ConstSharedPtr)> accept,
    std::function<void(const std::string &)> invalidate,
    const std::string & topic = "/perception/v2/world_targets")
  {
    const auto camera = node.has_parameter("camera_id") ? node.get_parameter("camera_id").as_string() :
      node.declare_parameter<std::string>("camera_id", "front", frozen());
    sub_ = node.create_subscription<Message>(topic, rclcpp::SensorDataQoS(),
      [this, &node, topic, camera, accept, invalidate](Message::ConstSharedPtr m, const rclcpp::MessageInfo & info) {
        const auto source = writer(node, topic, info);
        if (source.empty()) {invalidate("ambiguous_perception_writer"); return;}
        const auto state = gate_.observe(m->context, source);
        if (state == SourceGate::rejected) {return;}
        const std::string scope = hex(m->upstream.producer_session_id) + ":" + std::to_string(m->upstream.epoch);
        if (state == SourceGate::changed || (!scope_.empty() && scope != scope_)) {aliases_.clear(); invalidate("perception_source_changed");}
        scope_ = scope;
        const auto age = node.now().nanoseconds() - stamp(m->context.header.stamp);
        if (!valid(*m) || m->context.camera_id != camera || age < -200000000LL || age > 500000000LL) {
          invalidate("invalid_or_stale_perception_evidence"); return;
        }
        for (const auto & t : m->targets) {
          const auto old = aliases_.find(t.key.legacy_raw_id);
          if ((old != aliases_.end() && old->second != identity(t.key)) || aliases_.size() >= 65536) {
            invalidate("compatibility_alias_rebound_or_exhausted"); return;
          }
          aliases_[t.key.legacy_raw_id] = identity(t.key);
        }
        try {numeric_snapshot(*m);} catch (const std::exception &) {invalidate("unrepresentable_perception_view"); return;}
        accept(m);
      });
  }
private:
  std::map<int32_t, std::string> aliases_;
  SourceGate gate_; std::string scope_; rclcpp::Subscription<Message>::SharedPtr sub_;
};
}  // namespace easyfly::perception
