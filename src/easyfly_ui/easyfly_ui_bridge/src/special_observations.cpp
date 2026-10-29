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
#include <cmath>
#include <set>
#include <utility>
#include "easyfly_ui_bridge/special_observations.hpp"
namespace easyfly_ui_bridge
{
namespace
{
bool finite(std::initializer_list<double> values)
{return std::all_of(values.begin(), values.end(), [](double v) {return std::isfinite(v);});}
bool header(const std_msgs::msg::Header & h, const std::string & frame)
{return !frame.empty() && h.frame_id == frame && detail::stamp_ns(h.stamp) > 0;}
bool pose(const geometry_msgs::msg::Pose & p)
{
  const auto & q = p.orientation;
  return finite({p.position.x, p.position.y, p.position.z, q.x, q.y, q.z, q.w}) &&
         std::abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1) <= .001 &&
         p.position.z > 0;
}
bool uuid(const std::string & value)
{
  return value.size() == 32 && value != std::string(32, '0') &&
         std::all_of(
    value.begin(), value.end(), [](char c) {
      return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
    });
}
bool upstream(const easyfly_msgs::msg::TagObservationArray & m) {return m.valid;}
bool upstream(const easyfly_msgs::msg::LandingTarget & m) {return m.valid;}
bool upstream(const easyfly_msgs::msg::LandingGuidanceTarget & m) {return m.valid;}
bool upstream(const easyfly_msgs::msg::MissionStepFeedback & m) {return m.active;}
bool upstream(const easyfly_msgs::msg::TargetPredictionArray &) {return true;}
bool upstream(const easyfly_msgs::msg::ClassifiedObstacleArray & m) {return m.geometry_valid;}
template<typename Message>
void store(
  std::shared_ptr<const Message> m, Observation<Message> & current,
  int64_t & watermark, const char * topic, const std::string & optical, double now, uint64_t epoch)
{
  if (!m) {return;}
  const bool valid = special::valid(*m, optical);
  const auto ns = detail::stamp_ns(m->header.stamp);
  if (valid && current.value && ns == watermark && *m == *current.value) {return;}
  current = {}; auto & e = current.evidence;
  e.endpoint = topic; e.time_epoch = epoch; e.received_steady = e.first_source_steady = now;
  if (!valid || ns <= watermark) {
    e.quality = Quality::Invalid;
    e.reason = "invalid/budgeted, duplicate conflict or out-of-order specialized observation";
    return;
  }
  watermark = ns; current.value = std::move(m); e.quality = Quality::ValidAtObservation;
}
template<typename Message>
Observation<Message> age(Observation<Message> current, const ClockState & clock, double now)
{
  if (!current.value) {return current;}
  auto & e = current.evidence; e.receipt_age = now - e.first_source_steady;
  e.source_age = (clock.now_ns - detail::stamp_ns(current.value->header.stamp)) / 1e9;
  e.source_age_available = clock.now_ns > 0;
  e.quality = e.time_epoch != clock.epoch ? Quality::ClockReset :
    detail::source_quality(current.value->header, clock, e.receipt_age, .5, false);
  if (e.quality == Quality::ValidAtObservation && !upstream(*current.value)) {
    e.quality = Quality::Unknown;
    e.reason = "upstream invalid/inactive; no valid geometry or permission";
  }
  return current;
}
}  // namespace
namespace special
{
bool valid(const easyfly_msgs::msg::TagObservationArray & m, const std::string & optical)
{
  if (!header(m.header, optical) || m.observations.size() > 64 || m.reason.size() > 512) {
    return false;
  }
  std::set<int32_t> ids;
  for (const auto & tag : m.observations) {
    if (tag.header != m.header || tag.tag_id < 0 || !ids.insert(tag.tag_id).second ||
      tag.reason.size() > 512) {return false;}
    if (tag.valid && (!pose(tag.pose) ||
      !finite({tag.tag_size_m, tag.reprojection_error_px, tag.range_m}) ||
      tag.tag_size_m <= 0 || tag.tag_size_m > 10 || tag.reprojection_error_px < 0 ||
      tag.range_m <= 0)) {return false;}
  }
  return true;
}
bool valid(const easyfly_msgs::msg::LandingTarget & m, const std::string & optical)
{
  if (!header(m.header, optical) || m.reason.size() > 512 || m.tag_id < -1) {return false;}
  return !m.valid || (m.tag_id >= 0 && pose(m.pose) &&
         finite({m.tag_size_m, m.yaw_rad, m.optical_depth_m, m.reprojection_error_px}) &&
         m.tag_size_m > 0 && m.tag_size_m <= 10 && m.reprojection_error_px >= 0 &&
         m.optical_depth_m > 0 && std::abs(m.optical_depth_m - m.pose.position.z) < .001);
}
bool valid(const easyfly_msgs::msg::LandingGuidanceTarget & m, const std::string & optical)
{
  if (!header(m.header, "odom") || m.source_header.frame_id.size() > 256 ||
    m.reason.size() > 512 || m.tag_id < -1) {return false;}
  if (!m.valid) {return true;}
  const auto & p = m.position; const auto & n = m.plane_normal;
  return header(m.source_header, optical) && m.source_header.stamp == m.header.stamp &&
         m.tag_id >= 0 && finite({p.x, p.y, p.z, n.x, n.y, n.z, m.tag_size_m}) &&
         m.tag_size_m > 0 && m.tag_size_m <= 10 &&
         std::abs(n.x * n.x + n.y * n.y + n.z * n.z - 1) < .001;
}
bool valid(const easyfly_msgs::msg::MissionStepFeedback & m, const std::string &)
{
  if (!header(m.header, "odom") || !uuid(m.mission_goal_id) || !uuid(m.child_goal_id) ||
    m.mission_goal_id == m.child_goal_id || m.step_index >= 256 || m.phase.size() > 128 ||
    !std::isfinite(m.feedback_age_sec) || m.feedback_age_sec < 0 || m.feedback_age_sec > .5 ||
    (m.step_type != "precision_land" && m.step_type != "explore")) {return false;}
  if (!m.active) {return true;}
  if (m.step_type == "precision_land") {
    return !m.target_valid || (finite({m.horizontal_error_m, m.height_above_target_m}) &&
           m.horizontal_error_m >= 0 && m.height_above_target_m >= 0);
  }
  return finite({m.coverage, m.next_viewpoint.x, m.next_viewpoint.y, m.next_viewpoint.z}) &&
         m.coverage >= 0 && m.coverage <= 1;
}
bool valid(const easyfly_msgs::msg::TargetPredictionArray & m, const std::string &)
{
  if (!header(m.header, "odom") || m.predictions.size() > 100) {return false;}
  std::set<int32_t> ids;
  for (const auto & p : m.predictions) {
    if (p.header != m.header || p.track_id < 0 || !ids.insert(p.track_id).second ||
      p.class_name.size() > 128 || p.reason.size() > 512 ||
      p.observation_header.frame_id.size() > 256) {return false;}
    if (!p.valid) {continue;}
    const double age = (detail::stamp_ns(p.header.stamp) -
      detail::stamp_ns(p.observation_header.stamp)) / 1e9;
    if (!header(p.observation_header, "odom") || age < 0 || age > 5 ||
      !finite(
        {p.confidence, p.horizon_seconds, p.observation_age_seconds,
          p.position_x, p.position_y, p.position_z, p.velocity_x, p.velocity_y, p.velocity_z,
          p.variance_x, p.variance_y, p.variance_z}) || p.confidence < 0 || p.confidence > 1 ||
      p.horizon_seconds < 0 || p.horizon_seconds > 2 ||
      std::abs(age - p.observation_age_seconds) > .001 || p.variance_x < 0 ||
      p.variance_y < 0 || p.variance_z < 0) {return false;}
  }
  return true;
}
bool valid(const easyfly_msgs::msg::ClassifiedObstacleArray & m, const std::string &)
{
  if (!header(m.header, "odom") || m.obstacles.size() > 128 || m.reason.size() > 512) {
    return false;
  }
  std::set<int32_t> ids;
  for (const auto & c : m.obstacles) {
    const auto & o = c.obstacle;
    if (o.obstacle_type.size() > 128 || o.header.frame_id.size() > 256) {return false;}
    if (!m.geometry_valid) {continue;}
    if (o.header != m.header || o.obstacle_id < 0 || !ids.insert(o.obstacle_id).second ||
      !finite(
        {o.center_x, o.center_y, o.center_z,
          o.size_x, o.size_y, o.size_z, o.velocity_x, o.velocity_y, o.velocity_z}) ||
      o.size_x <= 0 || o.size_y <= 0 || o.size_z <= 0) {return false;}
  }
  return true;
}
bool class_current(const easyfly_msgs::msg::ClassifiedObstacle & c, int64_t now_ns)
{
  const auto observed = detail::stamp_ns(c.observed_at), expires = detail::stamp_ns(c.expires_at);
  return c.association_valid && c.target_id >= 0 && std::isfinite(c.confidence) &&
         c.confidence >= 0 && c.confidence <= 1 && observed > 0 && observed <= now_ns &&
         now_ns < expires && expires - observed <= 5000000000LL &&
         !c.obstacle.obstacle_type.empty() && c.obstacle.obstacle_type != "unknown";
}
}  // namespace special
SpecialObservationBuffer::SpecialObservationBuffer(std::string optical)
: optical_(std::move(optical)) {}
void SpecialObservationBuffer::clear() {data_ = {}; watermarks_ = {};}
#define RECEIVE(Type, member, index, topic) \
  void SpecialObservationBuffer::receive( \
    easyfly_msgs::msg::Type::ConstSharedPtr m, double now, \
    uint64_t epoch) \
  {store(std::move(m), data_.member, watermarks_[index], topic, optical_, now, epoch);}
RECEIVE(TagObservationArray, tags, 0, "/tag_observations")
RECEIVE(LandingTarget, landing, 1, "/landing_target")
RECEIVE(LandingGuidanceTarget, guidance, 2, "/landing/guidance_target")
RECEIVE(MissionStepFeedback, step_feedback, 3, "/mission/step_feedback")
RECEIVE(TargetPredictionArray, predictions, 4, "/target_predictions")
RECEIVE(ClassifiedObstacleArray, classified, 5, "/obstacles_classified")
#undef RECEIVE
SpecialObservations SpecialObservationBuffer::snapshot(const ClockState & clock, double now) const
{
  SpecialObservations s;
  s.tags = age(data_.tags, clock, now); s.landing = age(data_.landing, clock, now);
  s.guidance = age(data_.guidance, clock, now); s.step_feedback = age(
    data_.step_feedback, clock,
    now);
  s.predictions = age(data_.predictions, clock, now); s.classified = age(
    data_.classified, clock,
    now);
  return s;
}
}  // namespace easyfly_ui_bridge
