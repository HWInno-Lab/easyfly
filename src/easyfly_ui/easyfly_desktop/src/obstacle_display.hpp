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
#include <QPainter>
#ifndef OBSTACLE_DISPLAY_HPP_
#define OBSTACLE_DISPLAY_HPP_
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>
namespace easyfly_desktop
{
// Presentation only: never persists a missing detection or changes source evidence.
struct DisplayObstacle
{
  double left{0}, bottom{0}, right{0}, top{0};
  bool dynamic{false};
  std::string type;
};
class ObstacleDisplay
{
public:
  static constexpr double duration = .12;
  void clear()
  {previous_.clear(); transitions_.clear(); identity_.clear(); stamp_ = 0; started_ = 0;}
  void accept(
    std::vector<DisplayObstacle> current, const std::string & identity,
    int64_t stamp, double now)
  {
    if (identity == identity_ && stamp == stamp_) {return;}
    const bool adjacent = identity == identity_ && stamp > stamp_ &&
      stamp - stamp_ <= 1000000000LL;
    const auto displayed = at(now);
    std::vector<std::vector<size_t>> candidates(current.size());
    std::vector<size_t> previous_uses(previous_.size(), 0);
    std::vector<size_t> current_related(current.size(), 0), previous_related(previous_.size(), 0);
    if (adjacent) {
      for (size_t i = 0; i < current.size(); ++i) {
        for (size_t j = 0; j < previous_.size(); ++j) {
          // Broad overlap catches split/merge even when only one fragment
          // passes the tighter jitter gate. Ambiguous geometry never eases.
          if (related(previous_[j], current[i])) {
            ++current_related[i]; ++previous_related[j];
          }
          if (small_change(previous_[j], current[i])) {
            candidates[i].push_back(j); ++previous_uses[j];
          }
        }
      }
    }
    transitions_.clear(); transitions_.reserve(current.size());
    for (size_t i = 0; i < current.size(); ++i) {
      auto from = current[i];
      if (candidates[i].size() == 1 && previous_uses[candidates[i][0]] == 1 &&
        current_related[i] == 1 && previous_related[candidates[i][0]] == 1)
      {
        from = displayed[candidates[i][0]];
      }
      transitions_.push_back({std::move(from), current[i]});
    }
    previous_ = std::move(current); identity_ = identity; stamp_ = stamp; started_ = now;
  }
  std::vector<DisplayObstacle> at(double now) const
  {
    const double t = std::clamp((now - started_) / duration, 0., 1.);
    const double eased = t * t * (3 - 2 * t);
    std::vector<DisplayObstacle> result; result.reserve(transitions_.size());
    for (const auto & transition : transitions_) {
      auto box = transition.second;
      box.left = blend(transition.first.left, box.left, eased);
      box.bottom = blend(transition.first.bottom, box.bottom, eased);
      box.right = blend(transition.first.right, box.right, eased);
      box.top = blend(transition.first.top, box.top, eased);
      result.push_back(std::move(box));
    }
    return result;
  }
  bool active(double now) const
  {
    if (now >= started_ + duration) {return false;}
    return std::any_of(
      transitions_.begin(), transitions_.end(), [](const auto & t) {
        return t.first.left != t.second.left || t.first.bottom != t.second.bottom ||
        t.first.right != t.second.right || t.first.top != t.second.top;
      });
  }

private:
  static double blend(double from, double to, double t) {return from + (to - from) * t;}
  static bool related(const DisplayObstacle & a, const DisplayObstacle & b)
  {
    const double smaller_area = std::min(
      (a.right - a.left) * (a.top - a.bottom),
      (b.right - b.left) * (b.top - b.bottom));
    const double intersection = std::max(
      0., std::min(a.right, b.right) -
      std::max(a.left, b.left)) * std::max(
      0., std::min(a.top, b.top) -
      std::max(a.bottom, b.bottom));
    return smaller_area > 0 && intersection > .5 * smaller_area;
  }
  static bool small_change(const DisplayObstacle & a, const DisplayObstacle & b)
  {
    if (a.dynamic || b.dynamic || a.type != b.type) {return false;}
    const double aw = a.right - a.left, ah = a.top - a.bottom;
    const double bw = b.right - b.left, bh = b.top - b.bottom;
    if (std::min({aw, ah, bw, bh}) <= 0) {return false;}
    if (std::abs(aw - bw) > .2 * std::min(aw, bw) ||
      std::abs(ah - bh) > .2 * std::min(ah, bh)) {return false;}
    if (std::hypot(
        (a.left + a.right - b.left - b.right) / 2,
        (a.bottom + a.top - b.bottom - b.top) / 2) > .10 ||
      std::max(
        {std::abs(a.left - b.left), std::abs(a.right - b.right),
          std::abs(a.bottom - b.bottom), std::abs(a.top - b.top)}) > .15) {return false;}
    const double intersection = std::max(
      0., std::min(a.right, b.right) -
      std::max(a.left, b.left)) * std::max(
      0., std::min(a.top, b.top) -
      std::max(a.bottom, b.bottom));
    return intersection / (aw * ah + bw * bh - intersection) >= .75;
  }
  std::vector<DisplayObstacle> previous_;
  std::vector<std::pair<DisplayObstacle, DisplayObstacle>> transitions_;
  std::string identity_;
  int64_t stamp_{0};
  double started_{0};
};
}  // namespace easyfly_desktop
#endif  // OBSTACLE_DISPLAY_HPP_
