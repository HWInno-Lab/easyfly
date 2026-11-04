// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_perception_contracts/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
// Humble rclpy drops rmw_message_info.publisher_gid. Preserve it at a C++ intake
// boundary; no ROS publications, control operations, Python callbacks or plugins.
#include <atomic>
#include <deque>
#include <mutex>
#include <thread>
#include <pybind11/pybind11.h>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <type_traits>
#include <easyfly_msgs/msg/classified_obstacle_array_v2.hpp>
#include "easyfly_perception_contracts/contracts.hpp"
namespace py = pybind11;
namespace pc = easyfly::perception;
class Intake
{
  struct Event {std::string payload, writer;};
  std::mutex mutex_;
  std::deque<Event> events_;
  std::shared_ptr<rclcpp::Context> context_;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::SubscriptionBase::SharedPtr subscription_;
  std::thread worker_;
  std::atomic<bool> closed_{false};
  template<class Message> void subscribe(const std::string & topic)
  {
    subscription_ = node_->create_subscription<Message>(topic, rclcpp::SensorDataQoS().keep_last(4),
      [this, topic](typename Message::ConstSharedPtr m, const rclcpp::MessageInfo & info) {
        Event event; event.writer = pc::writer(*node_, topic, info);
        if (!event.writer.empty()) {
          rclcpp::SerializedMessage bytes; rclcpp::Serialization<Message> codec;
          codec.serialize_message(m.get(), &bytes);
          const auto & raw = bytes.get_rcl_serialized_message();
          if (raw.buffer_length <= (std::is_same_v<Message, sensor_msgs::msg::Image> ? 8U : 4U) * 1024 * 1024) {
            event.payload.assign(reinterpret_cast<const char *>(raw.buffer), raw.buffer_length);
          }
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (events_.size() == (std::is_same_v<Message, sensor_msgs::msg::Image> ? 1U : 4U)) {events_.pop_front();}
        events_.push_back(std::move(event));
      });
  }
public:
  Intake(const std::string & topic, const std::string & type)
  {
    if (topic.empty() || topic.front() != '/' || topic.size() > 256) {throw std::invalid_argument("absolute intake topic required");}
    context_ = std::make_shared<rclcpp::Context>(); context_->init(0, nullptr);
    static std::atomic<uint64_t> ids{0};
    rclcpp::NodeOptions options; options.context(context_).use_global_arguments(false);
    node_ = std::make_shared<rclcpp::Node>("easyfly_native_intake_" + std::to_string(++ids) + "_" + pc::hex(pc::uuid()).substr(0, 12), options);
    if (type == "DiagnosticArray") {subscribe<diagnostic_msgs::msg::DiagnosticArray>(topic);}
    else if (type == "Image") {subscribe<sensor_msgs::msg::Image>(topic);}
    else if (type == "DetectionArrayV2") {subscribe<easyfly_msgs::msg::DetectionArrayV2>(topic);}
    else if (type == "WorldTargetArrayV2") {subscribe<easyfly_msgs::msg::WorldTargetArrayV2>(topic);}
    else if (type == "ImageTrackArrayV2") {subscribe<easyfly_msgs::msg::ImageTrackArrayV2>(topic);}
    else if (type == "TargetPredictionArrayV2") {subscribe<easyfly_msgs::msg::TargetPredictionArrayV2>(topic);}
    else if (type == "ClassifiedObstacleArrayV2") {subscribe<easyfly_msgs::msg::ClassifiedObstacleArrayV2>(topic);}
    else {throw std::invalid_argument("unsupported native intake type");}
    rclcpp::ExecutorOptions eo; eo.context = context_;
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(eo);
    executor_->add_node(node_);
    worker_ = std::thread([this] {
      try {executor_->spin();} catch (const std::exception &) {closed_ = true;}
    });
  }
  py::object take()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (events_.empty()) {return py::none();}
    auto event = std::move(events_.front()); events_.pop_front();
    return py::make_tuple(py::bytes(event.payload), py::bytes(event.writer));
  }
  void close()
  {
    if (context_ && context_->is_valid()) {context_->shutdown("native intake closed");}
    if (worker_.joinable()) {worker_.join();}
    executor_.reset(); subscription_.reset(); node_.reset(); context_.reset(); closed_ = true;
  }
  ~Intake() {close();}
};
PYBIND11_MODULE(_intake, module)
{
  py::class_<Intake>(module, "Intake").def(py::init<const std::string &, const std::string &>())
    .def("take", &Intake::take).def("close", &Intake::close);
}
