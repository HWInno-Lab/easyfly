// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <string>
namespace easyfly_imu
{
class Window
{
public:
  using Values = std::array<double, 6>;
  const std::string & error() const {return error_;}
  std::optional<Values> Push(std::int64_t sequence, std::int64_t usec, const Values & values)
  {
    error_.clear();
    if (sequence < 0 || usec <= 0) {
      error_ = "invalid native sequence/time";
    }
    for (double v : values) {if (!std::isfinite(v)) {error_ = "nonfinite native channel";}}
    if (initialized_ && (last_sequence_ == std::numeric_limits<std::int64_t>::max() ||
      sequence != last_sequence_ + 1 || usec <= last_usec_ || usec - last_usec_ != 1000))
    {
      error_ = "native sequence/time discontinuity";
    }
    if (!error_.empty()) {
      error_ += "; previous_seq=" + std::to_string(last_sequence_) +
        " previous_usec=" + std::to_string(last_usec_) +
        " received_seq=" + std::to_string(sequence) + " received_usec=" + std::to_string(usec);
      return std::nullopt;
    }
    initialized_ = true;
    last_sequence_ = sequence; last_usec_ = usec;
    samples_.push_back(values);
    for (int i = 0; i < 6; ++i) {sum_[i] += values[i];}
    if (samples_.size() > 4) {
      for (int i = 0; i < 6; ++i) {sum_[i] -= samples_.front()[i];}
      samples_.pop_front();
    }
    Values result;
    for (int i = 0; i < 6; ++i) {result[i] = sum_[i] / samples_.size();}
    return result;
  }
private:
  bool initialized_{false};
  std::int64_t last_sequence_{0}, last_usec_{0};
  std::deque<Values> samples_;
  Values sum_{};
  std::string error_;
};
}  // namespace easyfly_imu
