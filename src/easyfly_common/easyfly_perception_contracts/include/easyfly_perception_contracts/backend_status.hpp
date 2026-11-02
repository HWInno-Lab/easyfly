// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <mutex>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include "easyfly_perception_contracts/contracts.hpp"
namespace easyfly::perception {
class BackendStatus {
public:
  BackendStatus(rclcpp::Node & node, std::string role, std::string requested, double max_age = 2.5)
  : node_(node), role_(std::move(role)), requested_(std::move(requested)), session_(hex(uuid())), max_age_(max_age) {
    publisher_ = node_.create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/platform/backends/" + role_, rclcpp::QoS(1).reliable().transient_local());
    timer_ = node_.create_wall_timer(std::chrono::milliseconds(250), [this] {publish();});
  }
  void created(std::string active, std::string parameters, std::string provider = "not_applicable") {
    std::lock_guard<std::mutex> lock(mutex_); active_ = std::move(active); parameters_ = std::move(parameters); provider_ = std::move(provider);
    created_ = true; reason_ = "waiting_for_valid_input"; phase_ = "waiting_input";
  }
  void input(bool valid, std::string reason = "waiting_for_valid_input", std::int64_t source_stamp = -1) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (valid) {input_at_ = std::chrono::steady_clock::now(); input_stamp_ = source_stamp < 0 ? node_.now().nanoseconds() : source_stamp; phase_ = "ready"; reason_ = "valid_input_observed";}
    else {input_at_ = {}; input_stamp_ = 0; phase_ = "waiting_input"; reason_ = std::move(reason);}
  }
  void failed(std::string reason) {std::lock_guard<std::mutex> lock(mutex_); phase_ = "failed"; reason_ = std::move(reason);}
  void publish() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::string phase = phase_, reason = reason_;
    const auto clock = node_.now().nanoseconds();
    if (clock != last_clock_) {last_clock_ = clock; clock_changed_at_ = std::chrono::steady_clock::now();}
    const bool clock_paused = std::chrono::duration<double>(std::chrono::steady_clock::now() - clock_changed_at_).count() > max_age_;
    const auto source_age = (node_.now().nanoseconds() - input_stamp_) / 1e9;
    const auto receipt_age = std::chrono::duration<double>(std::chrono::steady_clock::now() - input_at_).count();
    if (phase == "ready" && (receipt_age > max_age_ || source_age < -.2 || source_age > max_age_ || input_stamp_ <= 0 || clock_paused)) {phase = "waiting_input"; reason = "input_stale_or_clock_changed";}
    diagnostic_msgs::msg::DiagnosticArray out; out.header.stamp = node_.now();
    diagnostic_msgs::msg::DiagnosticStatus status; status.name = role_; status.hardware_id = session_;
    status.level = phase == "ready" ? 0 : phase == "failed" ? 2 : 1; status.message = reason;
    const auto add = [&](const std::string & key, const std::string & value) {diagnostic_msgs::msg::KeyValue item; item.key = key; item.value = value; status.values.push_back(item);};
    add("schema_version", "1"); add("requested_backend", requested_); add("active_backend", active_);
    add("created", created_ ? "true" : "false"); add("phase", phase); add("provider", provider_);
    add("effective_parameters", parameters_); add("sequence", std::to_string(++sequence_));
    out.status.push_back(status); publisher_->publish(out);
  }
private:
  rclcpp::Node & node_; std::string role_, requested_, session_, active_, provider_, parameters_{"{}"};
  std::string phase_{"creating"}, reason_{"creating_backend"}; bool created_{false};
  std::int64_t last_clock_{0}; std::chrono::steady_clock::time_point clock_changed_at_{std::chrono::steady_clock::now()};
  double max_age_; std::uint64_t sequence_{0}; std::int64_t input_stamp_{0};
  std::chrono::steady_clock::time_point input_at_{}; std::mutex mutex_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr publisher_; rclcpp::TimerBase::SharedPtr timer_;
};
}
