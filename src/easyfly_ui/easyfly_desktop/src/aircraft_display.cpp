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
#ifdef EASYFLY_ENABLE_RVIZ_EMBED
#include <OgreSceneNode.h>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QResource>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <vector>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/transformation/frame_transformer.hpp>
#include <rviz_rendering/objects/shape.hpp>
#include "rviz_layers.hpp"
static void initialize_aircraft_resources() {Q_INIT_RESOURCE(easyfly_assets);}
namespace easyfly_desktop
{
namespace
{
int64_t steady()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
struct PoseInbox
{
  std::mutex mutex;
  nav_msgs::msg::Odometry::ConstSharedPtr message;
  QString gid;
  int64_t receipt{0}; uint64_t received{0};
};
class Aircraft : public rviz_common::Display, public SpatialDiagnostics
{
public:
  explicit Aircraft(QString topic)
  : topic_(std::move(topic)) {}
  ~Aircraft() override {subscription_.reset(); inbox_.reset(); parts_.clear();}
  RvizLayerSnapshot layer_snapshot() const override
  {
    auto value = status_; value.name = getName(); value.topic = topic_; value.enabled = isEnabled();
    value.input_depth = subscription_ ? subscription_->get_actual_qos().depth() : -1;
    if (inbox_) {
      std::lock_guard<std::mutex> lock(inbox_->mutex); value.received = inbox_->received;
    }
    return value;
  }
  void reset() override
  {
    rviz_common::Display::reset(); current_.reset(); last_stamp_ = 0; previous_clock_ = -1;
    ++status_.epoch; clear("reset");
    if (inbox_) {std::lock_guard<std::mutex> lock(inbox_->mutex); inbox_->message.reset();}
  }
  void fixedFrameChanged() override {reset();}
  void update(float, float) override
  {
    if (!isEnabled() || !inbox_) {return;}
    const auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
    const auto clock = node->now().nanoseconds(), wall = steady();
    if (previous_clock_ >= 0 &&
      (clock < previous_clock_ || clock - previous_clock_ > 10000000000LL))
    {
      reset(); previous_clock_ = clock; clear("clock_jump"); return;
    }
    previous_clock_ = clock;
    if (node->count_publishers(topic_.toStdString()) != 1) {
      current_.reset(); clear("pose_source_missing_or_ambiguous"); return;
    }
    nav_msgs::msg::Odometry::ConstSharedPtr next; QString gid; int64_t receipt;
    {std::lock_guard<std::mutex> lock(inbox_->mutex); next = std::move(inbox_->message);
      gid = inbox_->gid; receipt = inbox_->receipt;}
    if (next) {
      const auto stamp = rclcpp::Time(next->header.stamp).nanoseconds();
      if (gid != status_.source_gid) {
        if (!status_.source_gid.isEmpty()) {
          ++status_.source_changes; ++status_.epoch; clear("pose_source_changed");
        }
        current_.reset(); last_stamp_ = 0; status_.source_gid = gid;
      }
      const auto & p = next->pose.pose.position; const auto & q = next->pose.pose.orientation;
      const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
      if (next->header.frame_id.empty() || next->header.frame_id.size() > 128 ||
        next->child_frame_id != "base_link" || stamp <= 0 ||
        !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        std::hypot(p.x, p.y, p.z) > 1e6 || !std::isfinite(norm) || std::abs(norm - 1) > 1e-3)
      {current_.reset(); clear("invalid_base_pose_or_frame"); return;}
      if (stamp < last_stamp_) {current_.reset(); clear("pose_stamp_rollback"); return;}
      if (stamp == last_stamp_ && current_ && *next != *current_) {
        current_.reset(); clear("same_stamp_pose_conflict"); return;
      }
      if (stamp > last_stamp_) {current_ = next; last_stamp_ = stamp; receipt_ = receipt;}
    }
    if (!current_) {return;}
    status_.received_stamp_ns = last_stamp_; status_.source_frame = QString::fromStdString(
      current_->header.frame_id);
    status_.source_age = (clock - last_stamp_) / 1e9; status_.receipt_age = (wall - receipt_) / 1e9;
    if (status_.source_age < -.02 || status_.source_age > .5 || status_.receipt_age < 0 ||
      status_.receipt_age > .5)
    {
      clear("pose_expired_or_future"); return;
    }
    const rclcpp::Time stamp(last_stamp_, node->get_clock()->get_clock_type());
    Ogre::Vector3 frame_position; Ogre::Quaternion frame_orientation;
    if (!context_->getFrameManager()->getTransform(
        current_->header.frame_id, stamp, frame_position,
        frame_orientation))
    {
      clear("source_pose_transform_unavailable"); return;
    }
    const auto & p = current_->pose.pose.position; const auto & q = current_->pose.pose.orientation;
    const auto base_position = frame_position + frame_orientation * Ogre::Vector3(p.x, p.y, p.z);
    const auto base_orientation = frame_orientation * Ogre::Quaternion(q.w, q.x, q.y, q.z);
    size_t visible = 0, missing = 0;
    for (auto & part : parts_) {
      Ogre::Vector3 position = base_position; Ogre::Quaternion orientation = base_orientation;
      if (part.frame != "base_link") {
        // The authoritative body pose is this original Odometry measurement.
        // Compose the observed body-to-sensor mount, without requiring or using
        // an unrelated global TF estimate of the same vehicle body.
        try {
          geometry_msgs::msg::PoseStamped origin;
          origin.header.frame_id = part.frame.toStdString();
          origin.header.stamp = current_->header.stamp; origin.pose.orientation.w = 1.;
          const auto mount = context_->getFrameManager()->getTransformer()->transform(
            origin, "base_link");
          const auto & t = mount.pose.position; const auto & r = mount.pose.orientation;
          const double length = std::hypot(t.x, t.y, t.z);
          const double norm = std::hypot(std::hypot(r.x, r.y), std::hypot(r.z, r.w));
          if (!std::isfinite(length) || length > 2 || !std::isfinite(norm) ||
            std::abs(norm - 1.) > 1e-3)
          {throw std::runtime_error("Invalid body-to-sensor transform");}
          position = base_position + base_orientation * Ogre::Vector3(t.x, t.y, t.z);
          orientation = base_orientation * Ogre::Quaternion(r.w, r.x, r.y, r.z);
        } catch (const std::exception &) {
          part.shape->getRootNode()->setVisible(false); ++missing; continue;
        }
      }
      part.shape->setPosition(position + orientation * part.position);
      part.shape->setOrientation(orientation * part.orientation);
      part.shape->getRootNode()->setVisible(true); ++visible;
    }
    status_.visible = visible > 0; status_.vertices = visible;
    status_.status = QStringLiteral(
      "只读四旋翼 · %1/%2 部件；%3 个部件缺少原始时刻 TF。"
      "旋翼未模拟转速。").arg(visible).arg(
      parts_.size()).arg(missing);
    setStatus(
      missing ? rviz_common::properties::StatusProperty::Warn :
      rviz_common::properties::StatusProperty::Ok,
      "机模观测", status_.status);
  }

protected:
  void onInitialize() override
  {
    initialize_aircraft_resources(); QFile file(":/easyfly/aircraft/quadrotor.json");
    if (!file.open(QIODevice::ReadOnly) || file.size() > 65536) {
      throw std::runtime_error("Missing bounded aircraft geometry");
    }
    const auto doc = QJsonDocument::fromJson(file.readAll()).object();
    if (doc["schema"] != 1 || doc["body_frame"] != "base_link" || doc["units"] != "metres") {
      throw std::runtime_error("Unsupported aircraft geometry");
    }
    const auto rows = doc["parts"].toArray();
    if (rows.size() < 10 || rows.size() > 64) {
      throw std::runtime_error("Invalid aircraft part count");
    }
    const auto numbers = [](const QJsonValue & value, int count, double limit) {
        const auto a = value.toArray(); std::vector<double> result;
        if (a.size() != count) {throw std::runtime_error("Invalid aircraft vector");}
        for (const auto & x : a) {
          if (!x.isDouble() || !std::isfinite(x.toDouble()) || std::abs(x.toDouble()) > limit) {
            throw std::runtime_error("Invalid aircraft component");
          }
          result.push_back(x.toDouble());
        }
        return result;
      };
    for (const auto & value : rows) {
      const auto row = value.toObject(); const auto frame = row["frame"].toString();
      if (frame != "base_link" && frame != "camera_link" && frame != "livox_frame") {
        throw std::runtime_error("Unknown aircraft frame");
      }
      const auto xyz = numbers(row["xyz"], 3, 2), size = numbers(row["size"], 3, 2);
      const auto rpy = numbers(row["rpy"], 3, 3.141592653589793),
        color = numbers(row["rgba"], 4, 1);
      for (double s : size) {if (s <= 0) {throw std::runtime_error("Invalid aircraft dimension");}}
      for (double c : color) {if (c < 0) {throw std::runtime_error("Invalid aircraft color");}}
      const auto kind = row["kind"].toString();
      auto type = rviz_rendering::Shape::Cube;
      if (kind == "sphere") {type = rviz_rendering::Shape::Sphere;} else if (kind == "cylinder") {
        type = rviz_rendering::Shape::Cylinder;
      } else if (kind != "cube") {throw std::runtime_error("Unknown aircraft primitive");}
      Part part; part.frame = frame; part.position = Ogre::Vector3(xyz[0], xyz[1], xyz[2]);
      part.orientation = Ogre::Quaternion(Ogre::Radian(rpy[2]), Ogre::Vector3::UNIT_Z) *
        Ogre::Quaternion(Ogre::Radian(rpy[1]), Ogre::Vector3::UNIT_Y) *
        Ogre::Quaternion(Ogre::Radian(rpy[0]), Ogre::Vector3::UNIT_X);
      part.shape = std::make_unique<rviz_rendering::Shape>(type, scene_manager_, scene_node_);
      part.shape->setScale(Ogre::Vector3(size[0], size[1], size[2]));
      part.shape->setColor(color[0], color[1], color[2], color[3]);
      parts_.push_back(std::move(part));
    }
    reset();
  }
  void onEnable() override
  {
    inbox_ = std::make_shared<PoseInbox>(); const std::weak_ptr<PoseInbox> weak = inbox_;
    const auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
    subscription_ = node->create_subscription<nav_msgs::msg::Odometry>(
      topic_.toStdString(), rclcpp::QoS(1).best_effort(),
      [weak](nav_msgs::msg::Odometry::ConstSharedPtr message, const rclcpp::MessageInfo & info) {
        if (const auto box = weak.lock()) {
          std::lock_guard<std::mutex> lock(box->mutex); box->message = std::move(
            message); box->receipt = steady(); ++box->received;
          box->gid =
          QString::fromLatin1(
            QByteArray(
              reinterpret_cast<const char *>(info.get_rmw_message_info()
              .publisher_gid.data), RMW_GID_STORAGE_SIZE).toHex());
        }
      });
  }
  void onDisable() override {subscription_.reset(); inbox_.reset(); reset();}

private:
  void clear(const QString & reason)
  {
    for (auto & part : parts_) {part.shape->getRootNode()->setVisible(false);}
    if (status_.visible || status_.last_invalidation != reason) {
      ++status_.invalidations; status_.invalidated_at_ns = steady();
    }
    status_.visible = false; status_.vertices = 0; status_.last_invalidation = reason;
    status_.status = reason;
    setStatus(rviz_common::properties::StatusProperty::Warn, "机模观测", reason);
  }
  struct Part
  {
    QString frame; Ogre::Vector3 position; Ogre::Quaternion orientation;
    std::unique_ptr<rviz_rendering::Shape> shape;
  };
  QString topic_; std::vector<Part> parts_; std::shared_ptr<PoseInbox> inbox_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  nav_msgs::msg::Odometry::ConstSharedPtr current_;
  RvizLayerSnapshot status_; int64_t last_stamp_{0}, receipt_{0}, previous_clock_{-1};
};
}  // namespace
rviz_common::Display * aircraft_display(const QString & topic) {return new Aircraft(topic);}
}  // namespace easyfly_desktop
#endif
