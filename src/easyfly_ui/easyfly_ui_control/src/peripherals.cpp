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
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>
#include <variant>
#include <rclcpp/rclcpp.hpp>
#include "easyfly_ui_control/peripherals.hpp"
namespace easyfly_ui_control
{
namespace
{
using Single = easyfly_msgs::srv::ManageSingleTrack;
using Angle = easyfly_msgs::srv::SetGimbalAngle;
using Tracking = easyfly_msgs::srv::SetGimbalTracking;
using Command = std::variant<Single::Request, Angle::Request, Tracking::Request>;
double steady()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
int64_t stamp(const std_msgs::msg::Header & h)
{return int64_t(h.stamp.sec) * 1000000000LL + h.stamp.nanosec;}
bool finite(std::initializer_list<double> values)
{return std::all_of(values.begin(), values.end(), [](double v) {return std::isfinite(v);});}
bool valid(const easyfly_msgs::msg::SingleTrackState & m)
{
  if (m.phase.size() > 32 || m.reason.size() > 512 || m.header.frame_id.size() > 256 ||
    m.track.class_name.size() > 128 ||
    (m.track.trajectory_x.size() > 256 || m.track.trajectory_y.size() > 256 ||
    m.track.trajectory_z.size() > 256) || m.track_id < -1) {return false;}
  if (m.phase != "idle" && m.phase != "initializing" && m.phase != "relocalizing" &&
    m.phase != "tracking" && m.phase != "lost") {return false;}
  if (!m.valid) {return true;}
  const auto & t = m.track;
  return m.phase == "tracking" && m.track_id >= 0 && t.track_id == m.track_id &&
         t.header == m.header &&
         m.header.frame_id == "camera_color_optical_frame" && stamp(m.header) > 0 &&
         finite({t.bbox_center_x, t.bbox_center_y, t.bbox_width, t.bbox_height, t.confidence}) &&
         t.confidence == 0 && t.bbox_width > 0 && t.bbox_height > 0 &&
         t.bbox_center_x >= t.bbox_width / 2 && t.bbox_center_x + t.bbox_width / 2 <= 1 &&
         t.bbox_center_y >= t.bbox_height / 2 && t.bbox_center_y + t.bbox_height / 2 <= 1;
}
bool valid(const easyfly_msgs::msg::GimbalState & m)
{
  return stamp(m.header) > 0 && m.header.frame_id.size() <= 256 && m.gimbal_model.size() <= 128 &&
         finite({m.pitch_angle, m.yaw_angle, m.roll_angle}) && m.tracking_target_id >= -1 &&
         (!m.is_tracking || m.tracking_target_id >= 0);
}
}  // namespace
struct PeripheralControl::Impl
{
  PeripheralConfig config;
  mutable std::mutex mutex;
  std::condition_variable wake;
  PeripheralSnapshot data;
  bool closing{false}, inhibited{true};
  std::optional<Command> queued;
  std::thread worker;
  std::shared_ptr<rclcpp::Context> context;
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  rclcpp::Client<Single>::SharedPtr single;
  rclcpp::Client<Angle>::SharedPtr angle;
  rclcpp::Client<Tracking>::SharedPtr tracking;
  rclcpp::Subscription<easyfly_msgs::msg::SingleTrackState>::SharedPtr single_sub;
  rclcpp::Subscription<easyfly_msgs::msg::GimbalState>::SharedPtr gimbal_sub;
  double sent{0}, clock_receipt{0}, graph_at{0};
  int64_t previous_ros{0}, single_stamp{0}, gimbal_stamp{0};
  explicit Impl(PeripheralConfig options)
  : config(options)
  {
    if (!std::isfinite(config.response_timeout) || config.response_timeout < .05 ||
      config.response_timeout > 30 || (config.commands_enabled && !config.use_sim_time))
    {
      throw std::invalid_argument("peripheral commands require simulation; timeout .05..30s");
    }
    worker = std::thread([this] {run();});
  }
  ~Impl() {stop(); if (worker.joinable()) {worker.join();}}
  void stop() {std::lock_guard<std::mutex> lock(mutex); closing = true; wake.notify_all();}
  template<typename Message>
  void receive(
    std::shared_ptr<const Message> m, std::shared_ptr<const Message> & current,
    int64_t & watermark, double & receipt)
  {
    if (!valid(*m) || stamp(m->header) < watermark || (!current && stamp(m->header) <= watermark)) {
      current.reset(); return;
    }
    if (stamp(m->header) > watermark || !current) {receipt = steady();}
    // Same measurement may acquire an invalid terminal phase, but never renew its age.
    watermark = stamp(m->header); current = std::move(m);
  }
  bool unique_service(const char * endpoint)
  {
    size_t count = 0;
    for (const auto & name : node->get_node_names()) {
      const auto slash = name.find_last_of('/');
      const auto services = node->get_service_names_and_types_by_node(
        name.substr(slash + 1), slash == 0 ? "/" : name.substr(0, slash));
      count += services.count(endpoint);
    }
    return count == 1;
  }
  template<typename Service>
  void dispatch(
    const typename Service::Request & request,
    const typename rclcpp::Client<Service>::SharedPtr & client, const char * endpoint)
  {
    if (!client->service_is_ready() || !unique_service(endpoint)) {
      data.pending = false; data.message = "service unavailable or ambiguous; not sent"; return;
    }
    const auto sequence = data.sequence; sent = steady();
    try {
      client->async_send_request(
        std::make_shared<typename Service::Request>(request),
        [this, sequence](typename rclcpp::Client<Service>::SharedFuture future) {
          std::lock_guard<std::mutex> lock(mutex);
          if (sequence != data.sequence) {return;}
          try {
            const auto reply = future.get();
            const bool late = data.uncertain;
            data.accepted = accepted(*reply); data.reply_received = true;
            data.message = std::string(late ? "late reply: " : "reply: ") +
            reply->message.substr(0, 512) + detail(*reply);
            data.pending = data.uncertain = false;
          } catch (const std::exception &) {
            data.uncertain = true; data.message = "response invalid; outcome unknown, no retry";
          }
        });
      data.message = "one request dispatched; awaiting admission reply";
    } catch (const std::exception &) {
      data.uncertain = true; data.message = "transport exception; outcome unknown, no retry";
    }
  }
  static std::string detail(const Single::Response & r)
  {
    return valid(r.state) ? " · state=" + r.state.phase + " ID=" +
           std::to_string(r.state.track_id) : " · invalid state reply";
  }
  static std::string detail(const Angle::Response &) {return {};}
  static std::string detail(const Tracking::Response &) {return {};}
  static bool accepted(const Single::Response & r) {return r.accepted;}
  static bool accepted(const Angle::Response & r) {return r.success;}
  static bool accepted(const Tracking::Response & r) {return r.success;}
  void tick()
  {
    const double now = steady(); const auto ros = node->now().nanoseconds();
    if (ros < previous_ros || ros <= 0 || ros - previous_ros > 10000000000LL) {
      data.single.reset(); data.gimbal.reset(); single_stamp = gimbal_stamp = 0;
    }
    if (ros != previous_ros) {clock_receipt = now; previous_ros = ros;}
    const bool clock_ok = ros > 0 && now - clock_receipt <= .5;
    const auto fresh = [&](const auto & m, double receipt, const char * topic) {
        return m && clock_ok && stamp(m->header) > 0 &&
               ros - stamp(m->header) >= -200000000LL && ros - stamp(m->header) <= 500000000LL &&
               now - receipt >= 0 && now - receipt <= .5 && node->count_publishers(topic) == 1;
      };
    data.single_fresh = fresh(data.single, data.single_receipt, "/single_track/state");
    data.gimbal_fresh = fresh(data.gimbal, data.gimbal_receipt, "/gimbal/state");
    if (now - graph_at >= .25) {
      data.single_ready = single->service_is_ready() && unique_service("/single_track/manage");
      data.angle_ready = angle->service_is_ready() && unique_service("/gimbal/set_angle");
      data.tracking_ready = tracking->service_is_ready() && unique_service("/gimbal/set_tracking");
      graph_at = now;
    }
    data.ros_now_ns = ros; data.produced_steady = now;
    if (queued) {
      auto command = std::move(*queued); queued.reset();
      const bool query = std::holds_alternative<Single::Request>(command) &&
        std::get<Single::Request>(command).operation == "query";
      if (!query && (inhibited || !clock_ok)) {
        data.pending = false; data.message = "inhibited or clock invalid before dispatch; not sent";
      } else {
        std::visit([this](const auto & r) {send(r);}, command);
      }
    }
    if (data.pending && !queued && sent > 0 && now - sent > config.response_timeout) {
      data.uncertain = true;
      data.message = "reply timeout; outcome unknown, further commands blocked";
    }
  }
  void send(const Single::Request & r) {dispatch<Single>(r, single, "/single_track/manage");}
  void send(const Angle::Request & r) {dispatch<Angle>(r, angle, "/gimbal/set_angle");}
  void send(const Tracking::Request & r) {dispatch<Tracking>(r, tracking, "/gimbal/set_tracking");}
  bool submit(Command command, const std::string & operation)
  {
    std::lock_guard<std::mutex> lock(mutex);
    const bool query = std::holds_alternative<Single::Request>(command) &&
      std::get<Single::Request>(command).operation == "query";
    if ((!query && (!config.commands_enabled || inhibited)) || closing || !data.running ||
      data.pending || data.uncertain || queued) {return false;}
    queued = std::move(command); ++data.sequence; data.operation = operation;
    data.pending = true; data.reply_received = data.accepted = false; sent = 0;
    data.message = "explicit request queued"; wake.notify_all(); return true;
  }
  void run()
  {
    try {
      context = std::make_shared<rclcpp::Context>(); context->init(0, nullptr);
      static std::atomic<uint64_t> ids{0};
      rclcpp::NodeOptions options; options.context(context).use_global_arguments(false);
      options.append_parameter_override("use_sim_time", config.use_sim_time);
      node = std::make_shared<rclcpp::Node>(
        "easyfly_ui_peripherals_" + std::to_string(
          ++ids), options);
      single = node->create_client<Single>("/single_track/manage");
      angle = node->create_client<Angle>("/gimbal/set_angle");
      tracking = node->create_client<Tracking>("/gimbal/set_tracking");
      single_sub = node->create_subscription<easyfly_msgs::msg::SingleTrackState>(
        "/single_track/state", rclcpp::SensorDataQoS().keep_last(1),
        [this](easyfly_msgs::msg::SingleTrackState::ConstSharedPtr m) {
          std::lock_guard<std::mutex> lock(mutex);
          receive<easyfly_msgs::msg::SingleTrackState>(
            m, data.single, single_stamp,
            data.single_receipt);
        });
      gimbal_sub = node->create_subscription<easyfly_msgs::msg::GimbalState>(
        "/gimbal/state", rclcpp::SensorDataQoS().keep_last(1),
        [this](easyfly_msgs::msg::GimbalState::ConstSharedPtr m) {
          std::lock_guard<std::mutex> lock(mutex);
          receive<easyfly_msgs::msg::GimbalState>(
            m, data.gimbal, gimbal_stamp,
            data.gimbal_receipt);
        });
      rclcpp::ExecutorOptions eo; eo.context = context;
      executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(eo);
      executor->add_node(node);
      {std::lock_guard<std::mutex> lock(mutex); data.running = true;}
      for (;; ) {
        executor->spin_some(std::chrono::milliseconds(10));
        std::unique_lock<std::mutex> lock(mutex); if (closing) {break;} tick();
        wake.wait_for(
          lock, std::chrono::milliseconds(20), [this] {
            return closing || queued.has_value();
          });
      }
    } catch (const std::exception & e) {
      std::lock_guard<std::mutex> lock(mutex);
      data.message = std::string("worker stopped: ") + e.what();
    }
    executor.reset(); single_sub.reset(); gimbal_sub.reset();
    single.reset(); angle.reset(); tracking.reset(); node.reset();
    if (context) {context->shutdown("peripheral observation stopped"); context.reset();}
    std::lock_guard<std::mutex> lock(mutex); data.uncertain |= data.pending;
    data.running = false; data.stopped = true; data.single_fresh = data.gimbal_fresh = false;
  }
};
PeripheralControl::PeripheralControl(const PeripheralConfig & config)
: impl_(std::make_unique<Impl>(config)) {}
PeripheralControl::~PeripheralControl() = default;
std::shared_ptr<const PeripheralSnapshot> PeripheralControl::snapshot() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex); return std::make_shared<const PeripheralSnapshot>(
    impl_->data);
}
bool PeripheralControl::single_confirmed(const Single::Request & r)
{
  if (r.operation != "initialize" && r.operation != "reset" && r.operation != "query") {
    return false;
  }
  if (r.operation == "initialize" && (stamp(r.image_header) <= 0 ||
    r.image_header.frame_id != "camera_color_optical_frame" || r.class_name.size() > 128 ||
    !finite({r.roi_x, r.roi_y, r.roi_width, r.roi_height}) || r.roi_x < 0 || r.roi_y < 0 ||
    r.roi_width <= 0 || r.roi_height <= 0 || r.roi_x + r.roi_width > 1 ||
    r.roi_y + r.roi_height > 1)) {return false;}
  return impl_->submit(r, "single_" + r.operation);
}
bool PeripheralControl::angle_confirmed(const Angle::Request & r)
{
  if (!finite({r.pitch_angle, r.yaw_angle, r.roll_angle}) || std::abs(r.pitch_angle) > 90 ||
    std::abs(r.yaw_angle) > 180 || std::abs(r.roll_angle) > 45) {return false;}
  return impl_->submit(r, "gimbal_angle");
}
bool PeripheralControl::tracking_confirmed(const Tracking::Request & r)
{if (r.enabled && r.target_id < 0) {return false;} return impl_->submit(r, "gimbal_tracking");}
void PeripheralControl::inhibit(bool inhibited)
{std::lock_guard<std::mutex> lock(impl_->mutex); impl_->inhibited = inhibited;}
void PeripheralControl::stop() {impl_->stop();}
}  // namespace easyfly_ui_control
