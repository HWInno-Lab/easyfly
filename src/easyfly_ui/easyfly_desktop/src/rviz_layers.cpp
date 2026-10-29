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
#include "rviz_layers.hpp"
#include <QByteArray>
#include <QDebug>
#include <chrono>
#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#ifdef EASYFLY_ENABLE_RVIZ_EMBED
#include <OgreSceneNode.h>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>
#include <rviz_rendering/objects/point_cloud.hpp>
#include "easyfly_desktop/rviz_geometry.hpp"
namespace easyfly_desktop
{
namespace
{
int64_t steady_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
struct Envelope
{
  std::shared_ptr<const spatial::Geometry> geometry;
  int64_t receipt{0};
  QString source_gid;
};
struct Mailbox
{
  std::mutex mutex;
  Envelope pending;
  uint64_t received{0}, replaced{0}, epoch{0};
};
class SpatialDisplay : public rviz_common::Display, public SpatialDiagnostics
{
public:
  SpatialDisplay(SpatialKind kind, QString topic)
  : kind_(kind), topic_(std::move(topic)) {}
  ~SpatialDisplay() override
  {
    stop_worker(); subscription_.reset(); mailbox_.reset(); line_.reset();
    if (cloud_) {scene_node_->detachObject(cloud_.get()); cloud_.reset();}
  }
  RvizLayerSnapshot layer_snapshot() const override
  {
    auto s = snapshot_; s.name = getName(); s.topic = topic_; s.enabled = isEnabled();
    s.input_depth = subscription_ ? static_cast<int>(subscription_->get_actual_qos().depth()) : -1;
    if (!isEnabled()) {s.status = QStringLiteral("已关闭");}
    if (mailbox_) {
      std::lock_guard<std::mutex> lock(mailbox_->mutex);
      s.received = mailbox_->received; s.replaced = mailbox_->replaced;
    }
    return s;
  }
  void reset() override
  {reset_with_reason("rviz_reset");}
  void reset_with_reason(const QString & reason)
  {
    rviz_common::Display::reset(); ++snapshot_.epoch; invalidate(reason); current_ = {};
    high_water_ = 0; previous_ros_ = -1;
    if (mailbox_) {
      std::lock_guard<std::mutex> lock(mailbox_->mutex);
      ++mailbox_->epoch; mailbox_->pending = {};
    }
    snapshot_.source_frame.clear(); snapshot_.source_age = snapshot_.receipt_age = -1;
    snapshot_.status = QStringLiteral("无数据 / 等待新观测"); set_status();
  }
  void fixedFrameChanged() override {reset_with_reason("fixed_frame_changed");}
  void update(float, float) override
  {
    if (!isEnabled() || !mailbox_) {return;}
    const auto raw = context_->getRosNodeAbstraction().lock()->get_raw_node();
    const auto now = raw->now().nanoseconds();
    const bool specialized = kind_ == SpatialKind::Landing || kind_ == SpatialKind::Predictions ||
      kind_ == SpatialKind::Classified;
    if (specialized && raw->count_publishers(topic_.toStdString()) != 1) {
      current_ = {}; invalidate("specialized_source_missing_or_ambiguous");
      snapshot_.status = QStringLiteral("专项源缺失或不唯一；已清图"); set_status(); return;
    }
    if (previous_ros_ >= 0 && (now < previous_ros_ || now - previous_ros_ > 10000000000LL)) {
      reset_with_reason("clock_jump"); previous_ros_ = now;
      snapshot_.status = QStringLiteral("时钟跳变 / 已清图，等待新观测"); set_status(); return;
    }
    previous_ros_ = now;
    Envelope next;
    {
      std::lock_guard<std::mutex> lock(mailbox_->mutex);
      next = std::move(mailbox_->pending); mailbox_->pending = {};
    }
    if (next.geometry) {
      const auto & g = *next.geometry;
      snapshot_.received_stamp_ns = g.stamp_ns;
      if (!next.source_gid.isEmpty() && snapshot_.source_gid != next.source_gid) {
        if (!snapshot_.source_gid.isEmpty()) {
          ++snapshot_.source_changes;
          if (specialized) {
            current_ = {}; high_water_ = 0; invalidate("specialized_source_changed");
          }
        }
        snapshot_.source_gid = next.source_gid;
        qInfo().noquote() << "easyfly_rviz_source topic=" << topic_ <<
          "gid=" << snapshot_.source_gid << "epoch=" << snapshot_.epoch <<
          "steady_ns=" << steady_ns();
      }
      if (!g.valid) {
        current_ = {}; invalidate("invalid_geometry: " + QString::fromStdString(g.detail));
        snapshot_.source_age = snapshot_.receipt_age = -1;
        snapshot_.source_frame = QString::fromStdString(g.frame);
        snapshot_.status = QString::fromStdString(g.detail); set_status(); return;
      }
      if (specialized && current_.geometry && g.stamp_ns <= high_water_) {
        const auto & previous = *current_.geometry;
        const bool identical = g.stamp_ns == high_water_ && g.frame == previous.frame &&
          g.detail == previous.detail && g.vertices.size() == previous.vertices.size() &&
          std::equal(
          g.vertices.begin(), g.vertices.end(), previous.vertices.begin(),
          [](const auto & a, const auto & b) {return a.x == b.x && a.y == b.y && a.z == b.z;});
        if (!identical) {
          current_ = {}; invalidate("specialized_out_of_order_or_conflict");
          snapshot_.status = QStringLiteral("专项数据乱序或同帧冲突；已清图"); set_status(); return;
        }
      }
      if (g.stamp_ns > high_water_) {
        const auto reason = spatial::validity(g, now, next.receipt, steady_ns());
        if (!reason.empty()) {
          current_ = {}; invalidate(QString::fromStdString(reason));
          snapshot_.source_age = snapshot_.receipt_age = -1;
          snapshot_.source_frame = QString::fromStdString(g.frame);
          snapshot_.status = QString::fromStdString(reason); set_status(); return;
        }
        current_ = std::move(next); high_water_ = g.stamp_ns; dirty_ = true;
      }
      // Equal/older source stamps never renew the original receipt deadline.
    }
    if (!current_.geometry) {set_status(); return;}
    const auto & g = *current_.geometry;
    snapshot_.source_frame = QString::fromStdString(g.frame);
    snapshot_.source_age = static_cast<double>(now - g.stamp_ns) / 1e9;
    snapshot_.receipt_age = static_cast<double>(steady_ns() - current_.receipt) / 1e9;
    const auto reason = spatial::validity(g, now, current_.receipt, steady_ns());
    if (!reason.empty()) {
      invalidate(QString::fromStdString(reason));
      snapshot_.status = QString::fromStdString(reason); set_status(); return;
    }
    Ogre::Vector3 position = Ogre::Vector3::ZERO;
    Ogre::Quaternion orientation = Ogre::Quaternion::IDENTITY;
    if (g.frame != fixed_frame_.toStdString() &&
      !context_->getFrameManager()->getTransform(
        g.frame, rclcpp::Time(
          g.stamp_ns,
          raw->get_clock()->get_clock_type()), position, orientation))
    {
      snapshot_.status = QStringLiteral("TF 未就绪：%1 → %2").arg(
        snapshot_.source_frame, fixed_frame_);
      invalidate(snapshot_.status); set_status(); return;
    }
    scene_node_->setPosition(position); scene_node_->setOrientation(orientation);
    if (dirty_) {draw(g); dirty_ = false;}
    snapshot_.status = QStringLiteral(
      "最后收到 / %1 messages received · %2 · 显示%3 / 输入%4，省略%5").arg(
      layer_snapshot().received).arg(QString::fromStdString(g.detail)).arg(g.vertices.size()).arg(
      g.input_count).arg(g.omitted);
    set_status();
  }

protected:
  void onInitialize() override
  {
    line_ = std::make_unique<rviz_rendering::BillboardLine>(scene_manager_, scene_node_);
    line_->setLineWidth(0.035F);
    cloud_ = std::make_unique<rviz_rendering::PointCloud>();
    cloud_->setRenderMode(
      kind_ == SpatialKind::Map ? rviz_rendering::PointCloud::RM_FLAT_SQUARES :
      rviz_rendering::PointCloud::RM_POINTS);
    scene_node_->attachObject(cloud_.get()); reset_with_reason("initialized");
  }
  void onEnable() override
  {
    reset_with_reason("enabled"); mailbox_ = std::make_shared<Mailbox>();
    const auto raw = context_->getRosNodeAbstraction().lock()->get_raw_node();
    // Humble's manager spins its automatic groups on the GUI update thread.
    // Explicitly exclude this bounded geometry group and give it an executor.
    group_ = raw->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    switch (kind_) {
      case SpatialKind::Actual: subscribe<nav_msgs::msg::Path>(raw); break;
      case SpatialKind::Planned: subscribe<visualization_msgs::msg::Marker>(raw); break;
      case SpatialKind::Cloud: subscribe<sensor_msgs::msg::PointCloud2>(raw); break;
      case SpatialKind::Map: subscribe<nav_msgs::msg::OccupancyGrid>(raw); break;
      case SpatialKind::Obstacles: subscribe<easyfly_msgs::msg::ObstacleInfoArray>(raw); break;
      case SpatialKind::Landing: subscribe<easyfly_msgs::msg::LandingGuidanceTarget>(raw); break;
      case SpatialKind::Predictions: subscribe<easyfly_msgs::msg::TargetPredictionArray>(raw);
        break;
      case SpatialKind::Classified: subscribe<easyfly_msgs::msg::ClassifiedObstacleArray>(raw);
        break;
    }
    rclcpp::ExecutorOptions options;
    options.context = raw->get_node_base_interface()->get_context();
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(options);
    executor_->add_callback_group(group_, raw->get_node_base_interface());
    const std::weak_ptr<Mailbox> weak = mailbox_;
    stopping_.store(false);
    worker_ = std::thread(
      [this, executor = executor_, weak] {
        try {
          while (!stopping_.load()) {
            executor->spin_some(std::chrono::milliseconds(10));
            std::unique_lock<std::mutex> lock(worker_mutex_);
            wake_.wait_for(lock, std::chrono::milliseconds(5), [this] {return stopping_.load();});
          }
        } catch (const std::exception & e) {
          if (const auto mailbox = weak.lock()) {
            auto error = std::make_shared<spatial::Geometry>();
            error->detail = std::string("只读观测线程退出：") +
            std::string(e.what()).substr(0, 128);
            std::lock_guard<std::mutex> lock(mailbox->mutex);
            mailbox->pending = {std::move(error), steady_ns(), {}};
          }
        }
      });
  }
  void onDisable() override
  {stop_worker(); subscription_.reset(); mailbox_.reset(); reset_with_reason("disabled");}

private:
  template<typename Message>
  void subscribe(const std::shared_ptr<rclcpp::Node> & node)
  {
    const std::weak_ptr<Mailbox> weak = mailbox_;
    rclcpp::SubscriptionOptions options; options.callback_group = group_;
    subscription_ = node->create_subscription<Message>(
      topic_.toStdString(), rclcpp::QoS(1).best_effort().durability_volatile(),
      [weak](typename Message::ConstSharedPtr message, const rclcpp::MessageInfo & info) {
        const auto mailbox = weak.lock(); if (!mailbox || !message) {return;}
        const auto receipt = steady_ns();
        uint64_t epoch;
        {std::lock_guard<std::mutex> lock(mailbox->mutex); epoch = mailbox->epoch;}
        auto geometry = std::make_shared<const spatial::Geometry>(spatial::convert(*message));
        std::lock_guard<std::mutex> lock(mailbox->mutex);
        ++mailbox->received;
        if (epoch != mailbox->epoch) {++mailbox->replaced; return;}
        if (mailbox->pending.geometry) {++mailbox->replaced;}
        const auto & gid = info.get_rmw_message_info().publisher_gid;
        const auto source = QString::fromLatin1(
          QByteArray(
            reinterpret_cast<const char *>(gid.data), RMW_GID_STORAGE_SIZE).toHex());
        mailbox->pending = {std::move(geometry), receipt, source};
      }, options);
  }
  void stop_worker()
  {
    // A stop flag also handles a thread that has not entered its first spin;
    // cancel() alone can lose that request when a fresh spin starts afterward.
    stopping_.store(true); wake_.notify_all();
    if (executor_) {executor_->cancel();}
    if (worker_.joinable()) {worker_.join();}
    executor_.reset(); group_.reset();
  }
  void clear_geometry()
  {
    if (line_) {line_->clear();}
    if (cloud_) {cloud_->clearAndRemoveAllPoints();}
    snapshot_.visible = false; snapshot_.vertices = 0; dirty_ = true;
  }
  void invalidate(const QString & reason)
  {
    // Repeated stale frames do not flood the log, but a recovery followed by the
    // same fault is a new invalidation. Ordinary redraw is never an invalidation.
    if (snapshot_.visible || snapshot_.last_invalidation != reason) {
      ++snapshot_.invalidations; snapshot_.last_invalidation = reason;
      snapshot_.invalidated_at_ns = steady_ns();
      qInfo().noquote() << "easyfly_rviz_invalidate topic=" << topic_ <<
        "epoch=" << snapshot_.epoch << "gid=" << snapshot_.source_gid <<
        "source_stamp=" << snapshot_.received_stamp_ns <<
        "steady_ns=" << snapshot_.invalidated_at_ns << "reason=" << reason;
    }
    clear_geometry();
  }
  void draw(const spatial::Geometry & g)
  {
    clear_geometry();
    Ogre::ColourValue color(0.09F, 0.41F, 0.72F, 1);
    if (kind_ == SpatialKind::Planned) {color = Ogre::ColourValue(0.87F, 0.32F, 0.07F, 1);}
    if (kind_ == SpatialKind::Obstacles) {color = Ogre::ColourValue(0.79F, 0.13F, 0.20F, 1);}
    if (kind_ == SpatialKind::Map) {color = Ogre::ColourValue(0.39F, 0.30F, 0.65F, 1);}
    if (kind_ == SpatialKind::Cloud) {color = Ogre::ColourValue(0.08F, 0.58F, 0.51F, 1);}
    if (kind_ == SpatialKind::Landing) {color = Ogre::ColourValue(0.12F, 0.55F, 0.22F, 1);}
    if (kind_ == SpatialKind::Predictions) {color = Ogre::ColourValue(0.07F, 0.65F, 0.74F, 1);}
    if (kind_ == SpatialKind::Classified) {color = Ogre::ColourValue(0.65F, 0.32F, 0.66F, 1);}
    if (g.primitive == spatial::Primitive::Points) {
      std::vector<rviz_rendering::PointCloud::Point> points;
      points.reserve(g.vertices.size());
      for (const auto & p : g.vertices) {
        points.push_back({Ogre::Vector3(p.x, p.y, p.z), color});
      }
      cloud_->setDimensions(g.point_size, g.point_size, g.point_size);
      if (!points.empty()) {cloud_->addPoints(points.begin(), points.end());}
    } else if (g.vertices.size() >= 2) {
      const bool segments = g.primitive == spatial::Primitive::Segments;
      line_->setNumLines(segments ? g.vertices.size() / 2 : 1);
      line_->setMaxPointsPerLine(segments ? 2 : g.vertices.size());
      for (size_t i = 0; i < g.vertices.size(); ++i) {
        const auto & p = g.vertices[i]; line_->addPoint(Ogre::Vector3(p.x, p.y, p.z), color);
        if (segments && i % 2) {line_->finishLine();}
      }
    }
    snapshot_.visible = g.primitive == spatial::Primitive::Points ? !g.vertices.empty() :
      g.vertices.size() >= 2;
    snapshot_.vertices = g.vertices.size();
  }
  void set_status()
  {
    setStatus(
      snapshot_.visible ? rviz_common::properties::StatusProperty::Ok :
      rviz_common::properties::StatusProperty::Warn, "只读图层", snapshot_.status);
  }
  SpatialKind kind_;
  QString topic_;
  std::shared_ptr<Mailbox> mailbox_;
  rclcpp::SubscriptionBase::SharedPtr subscription_;
  rclcpp::CallbackGroup::SharedPtr group_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread worker_;
  std::atomic_bool stopping_{true};
  std::mutex worker_mutex_;
  std::condition_variable wake_;
  Envelope current_;
  int64_t high_water_{0}, previous_ros_{-1};
  bool dirty_{true};
  RvizLayerSnapshot snapshot_;
  std::unique_ptr<rviz_rendering::BillboardLine> line_;
  std::unique_ptr<rviz_rendering::PointCloud> cloud_;
};
}  // namespace
rviz_common::Display * spatial_display(SpatialKind kind, const QString & topic)
{return new SpatialDisplay(kind, topic);}
}  // namespace easyfly_desktop
#endif
