// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_time_sync/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
//

#pragma once
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <utility>

namespace easyfly_time_sync
{
struct Options
{
  double tolerance{.03}, max_age{.5};
  size_t queue_size{8}, max_bytes{64 * 1024 * 1024};
};
template<class T>
class Pairer
{
public:
  struct Entry {int64_t stamp; double received; size_t bytes; std::shared_ptr<const T> payload;};
  struct Pair {Entry first, second;};
  explicit Pairer(Options options = {})
  : options_(options)
  {
    if (!std::isfinite(options.tolerance) || options.tolerance < 0 || options.tolerance > .5 ||
      !std::isfinite(options.max_age) || options.max_age < .05 || options.max_age > 10 ||
      options.tolerance > options.max_age || options.queue_size < 1 || options.queue_size > 32 ||
      options.max_bytes < 1 || options.max_bytes > 128 * 1024 * 1024)
    {
      throw std::invalid_argument("invalid pairing budgets");
    }
  }
  std::vector<Pair> add(size_t side, Entry entry, int64_t now, double wall)
  {
    if (side > 1) {throw std::invalid_argument("pair side");}
    expire(now, wall);
    if (!clock_ready_ || entry.stamp <= 0 || entry.stamp <= last_stamp_[side] ||
      !fresh(entry, now, wall) || !entry.payload || entry.bytes > options_.max_bytes)
    {
      ++dropped_[side]; reason_ = "invalid_stale_or_nonmonotonic_input"; return {};
    }
    last_stamp_[side] = entry.stamp;
    while (queue_[side].size() >= options_.queue_size) {drop(side, "queue_full");}
    while (queued_bytes_ + entry.bytes > options_.max_bytes) {
      size_t oldest = queue_[0].empty() ? 1 : queue_[1].empty() ? 0 :
        queue_[0].front().received <= queue_[1].front().received ? 0 : 1;
      drop(oldest, "memory_budget");
    }
    queued_bytes_ += entry.bytes; queue_[side].push_back(std::move(entry));
    std::vector<Pair> pairs;
    while (!queue_[0].empty() && !queue_[1].empty()) {
      const auto difference = (queue_[1].front().stamp - queue_[0].front().stamp) / 1e9;
      if (std::abs(difference) <= options_.tolerance + 1e-9) {
        auto a = queue_[0].front(), b = queue_[1].front();
        queued_bytes_ -= a.bytes + b.bytes; queue_[0].pop_front(); queue_[1].pop_front();
        last_pair_ = {a.stamp, b.stamp}; last_received_ = std::min(a.received, b.received);
        ++paired_; reason_ = "paired"; pairs.push_back({std::move(a), std::move(b)});
      } else {drop(difference > 0 ? 0 : 1, "outside_tolerance");}
    }
    return pairs;
  }
  void reject(size_t side, const std::string & reason) {++dropped_.at(side); reason_ = reason;}
  void expire(int64_t now, double wall)
  {
    if (!clock_initialized_) {last_clock_ = now; clock_progress_ = wall; clock_initialized_ = true;}
    if (now < last_clock_) {
      for (size_t side = 0; side < 2; ++side) {
        while (!queue_[side].empty()) {
          drop(side, "clock_reset");
        }
      }
      last_stamp_ = {}; last_pair_ = {}; reason_ = "clock_reset"; clock_progress_ = wall;
    } else if (now > last_clock_) {clock_progress_ = wall;}
    last_clock_ = now;
    clock_ready_ = now > 0 && wall - clock_progress_ <= options_.max_age;
    for (size_t side = 0; side < 2; ++side) {
      while (!queue_[side].empty() &&
        (!clock_ready_ || !fresh(queue_[side].front(), now, wall)))
      {
        drop(side, "expired_or_clock_paused");
      }
    }
    if (!clock_ready_) {
      reason_ = "clock_not_ready_or_paused";
    } else if (last_pair_[0] && !valid(now, wall)) {reason_ = "pair_stale";}
  }
  bool valid(int64_t now, double wall) const
  {
    if (!clock_ready_ || !last_pair_[0]) {return false;}
    for (auto stamp : last_pair_) {
      if ((now - stamp) / 1e9 < -.2 || (now - stamp) / 1e9 > options_.max_age) {
        return false;
      }
    }
    return wall - last_received_ <= options_.max_age;
  }
  uint64_t paired() const {return paired_;}
  uint64_t dropped(size_t side) const {return dropped_.at(side);}
  size_t queued(size_t side) const {return queue_.at(side).size();}
  size_t bytes() const {return queued_bytes_;}
  const std::string & reason() const {return reason_;}

private:
  bool fresh(const Entry & entry, int64_t now, double wall) const
  {
    const double age = (now - entry.stamp) / 1e9;
    return age >= -.2 && age <= options_.max_age && wall >= entry.received &&
           wall - entry.received <= options_.max_age;
  }
  void drop(size_t side, const std::string & reason)
  {
    queued_bytes_ -= queue_[side].front().bytes; queue_[side].pop_front(); ++dropped_[side];
    reason_ = reason;
  }
  Options options_;
  std::array<std::deque<Entry>, 2> queue_;
  std::array<int64_t, 2> last_stamp_{}, last_pair_{};
  std::array<uint64_t, 2> dropped_{};
  uint64_t paired_{0}; size_t queued_bytes_{0};
  int64_t last_clock_{0}; double clock_progress_{0}, last_received_{0};
  bool clock_initialized_{false}, clock_ready_{false};
  std::string reason_{"not_received"};
};
}  // namespace easyfly_time_sync
