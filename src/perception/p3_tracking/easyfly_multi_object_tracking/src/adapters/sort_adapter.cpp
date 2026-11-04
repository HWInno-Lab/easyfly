// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_multi_object_tracking/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
#include <easyfly_backend_registry/metadata.hpp>
// Copyright 2026 EasyFly Project
//

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "easyfly_multi_object_tracking/adapters/tracker_factory.hpp"
#include "easyfly_sort/sort.hpp"

namespace easyfly_multi_object_tracking
{
namespace tr = easyfly::tracking;
namespace core = easyfly::sort;

class SortAdapter final : public tr::Tracker
{
public:
  explicit SortAdapter(core::SortParams params)
  : params_(params)
  {
    if (!std::isfinite(params.iou_threshold) || params.iou_threshold < 0 ||
      params.iou_threshold > 1 || !std::isfinite(params.dist_gate_scale) ||
      params.dist_gate_scale < 0 || params.max_lost_frames < 0 ||
      params.min_hits < 1 || params.max_tracks < 1)
    {
      throw std::invalid_argument("invalid SORT parameters");
    }
    reset(0);
  }

  tr::Capabilities capabilities() const override
  {
    return {"sort", {}, tr::VelocityUnit::pixels_per_frame};
  }

  void reset(std::uint64_t epoch) override
  {
    // ROS TrackInfo has int32 IDs and no epoch. Never reuse an emitted raw ID.
    auto fresh = std::make_unique<core::Sorter>(params_);
    sorter_ = std::move(fresh);
    offset_ = next_id_;
    epoch_ = epoch;
    sequence_.reset(epoch);
  }

  tr::UpdateResult update(const tr::Frame & frame) override
  {
    tr::UpdateResult result;
    result.source = frame.source;
    result.velocity_unit = tr::VelocityUnit::pixels_per_frame;
    result.status = tr::validate_frame(frame);
    if (result.status == tr::Status::ok) {result.status = sequence_.check(frame.source);}
    if (result.status != tr::Status::ok) {
      result.detail = "invalid, replayed, out-of-epoch or over-limit tracking input";
      return result;
    }
    constexpr auto max_id = std::numeric_limits<std::int32_t>::max();
    if (next_id_ > max_id ||
      frame.detections.size() > static_cast<std::size_t>(max_id - next_id_))
    {
      result.status = tr::Status::resource_limit;
      result.detail = "int32 track ID space exhausted";
      return result;
    }
    std::vector<core::DetectionInput> input;
    input.reserve(frame.detections.size());
    for (const auto & detection : frame.detections) {
      input.push_back(
        {{detection.box.x1, detection.box.y1, detection.box.x2, detection.box.y2},
          detection.confidence, detection.class_id, detection.class_name});
    }
    const auto tracks = sorter_->update(input);
    for (const auto & track : tracks) {
      const auto id = offset_ + track.track_id;
      next_id_ = std::max(next_id_, id + 1);
    }
    for (const auto & track : tracks) {
      tr::Track output;
      output.raw_id = offset_ + track.track_id;
      output.box = {track.box.x1, track.box.y1, track.box.x2, track.box.y2};
      output.last_observed_box = {
        track.box_raw.x1, track.box_raw.y1, track.box_raw.x2, track.box_raw.y2};
      output.confidence = track.confidence;
      output.class_id = track.class_id;
      output.class_name = track.class_name;
      output.vx = track.vx;
      output.vy = track.vy;
      output.hits = track.hits;
      output.age = track.age;
      output.missed_updates = track.time_since_update;
      output.confirmed = track.confirmed;
      if (output.raw_id < 0 || output.raw_id > max_id || !tr::valid_box(output.box) ||
        !std::isfinite(output.vx) || !std::isfinite(output.vy))
      {
        result.status = tr::Status::backend_error;
        result.detail = "non-finite or unrepresentable SORT output; history cleared";
        result.tracks.clear();
        reset(epoch_);
        return result;
      }
      result.tracks.push_back(std::move(output));
    }
    sequence_.commit(frame.source);
    return result;
  }

private:
  core::SortParams params_;
  std::unique_ptr<core::Sorter> sorter_;
  tr::FrameSequence sequence_;
  std::uint64_t epoch_{0};
  std::int64_t offset_{0}, next_id_{0};
};

easyfly::backends::Parameters normalize_sort(const easyfly::backends::Parameters & input) {
  namespace b = easyfly::backends;
  core::SortParams params;
  b::Parameters values{{"iou_threshold", params.iou_threshold}, {"max_lost_frames", params.max_lost_frames},
        {"min_hits", params.min_hits}, {"max_tracks", params.max_tracks}, {"dist_gate_scale", params.dist_gate_scale}};
  for (const auto & item : input) values[item.first] = item.second;
  values = b::resolve_parameters("multi_tracking", "sort", values);
  return values;
}
void register_sort(TrackerFactory & result)
{
    result.add("sort", [](const easyfly::backends::Parameters & input) {
      namespace b = easyfly::backends;
      b::require_backend("multi_tracking", "sort");
      // Preserve the plain C++ creator's historical defaults; profiles override them.
      core::SortParams params;
      const auto values = normalize_sort(input);
      params.iou_threshold = b::parameter<double>(values, "iou_threshold");
      params.max_lost_frames = b::parameter<int>(values, "max_lost_frames");
      params.min_hits = b::parameter<int>(values, "min_hits");
      params.max_tracks = b::parameter<int>(values, "max_tracks");
      params.dist_gate_scale = b::parameter<double>(values, "dist_gate_scale");
      return std::make_unique<SortAdapter>(params);
    }, normalize_sort);
}
}  // namespace easyfly_multi_object_tracking
