// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_multi_object_tracking/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
#pragma once
#include <set>
#include "easyfly_backend_registry/configuration.hpp"
#include "easyfly_tracking_api/tracker.hpp"
namespace easyfly_multi_object_tracking {
using TrackerFactory = easyfly::backends::Factory<easyfly::tracking::Tracker>;
const TrackerFactory & tracker_factory();
inline std::unique_ptr<easyfly::tracking::Tracker> make_tracker(
  const easyfly::backends::BackendConfig & config, const TrackerFactory & factory = tracker_factory()) {
  auto result = factory.create(config);
  if (result->capabilities().backend != config.backend) throw std::runtime_error("tracking creator identity mismatch");
  return result;
}
inline std::unique_ptr<easyfly::tracking::Tracker> make_tracker(
  const std::string & backend, const easyfly::backends::Parameters & parameters = {}) {
  return make_tracker(easyfly::backends::BackendConfig{backend, parameters});
}
inline void require_supported_inputs(const easyfly::tracking::Capabilities & caps, bool legacy_output) {
  if (caps.inputs.image || caps.inputs.appearance_dimension || caps.inputs.low_confidence_detections)
    throw std::invalid_argument("tracking node does not supply required image/appearance/low-confidence inputs");
  using Unit = easyfly::tracking::VelocityUnit;
  if ((caps.velocity_unit != Unit::pixels_per_frame && caps.velocity_unit != Unit::pixels_per_second) ||
      (legacy_output && caps.velocity_unit != Unit::pixels_per_frame))
    throw std::invalid_argument("unsupported tracking velocity unit for selected wire version");
}
// Wire adaptation cannot narrow an arbitrary backend's int64 ID or reinterpret its velocity.
inline bool valid_tracking_output(const easyfly::tracking::UpdateResult & result,
  const easyfly::tracking::Frame & input, const easyfly::tracking::Capabilities & caps, bool legacy) {
  namespace tr = easyfly::tracking;
  if (result.source.epoch != input.source.epoch || result.source.stamp_ns != input.source.stamp_ns ||
      result.velocity_unit != caps.velocity_unit || result.tracks.size() > (legacy ? 1024u : 100u)) return false;
  std::set<std::int64_t> ids;
  const auto box_ok = [](const tr::Box & box) {
    const double bound = std::numeric_limits<std::int32_t>::max() / 4.;
    return tr::valid_box(box) && std::abs(box.x1) <= bound && std::abs(box.y1) <= bound &&
      std::abs(box.x2) <= bound && std::abs(box.y2) <= bound;
  };
  for (const auto & track : result.tracks) {
    if (track.raw_id < 0 || (legacy && track.raw_id > std::numeric_limits<std::int32_t>::max()) ||
        !ids.insert(track.raw_id).second || !box_ok(track.box) ||
        (track.missed_updates == 0 && !box_ok(track.last_observed_box)) ||
        !std::isfinite(track.vx) || !std::isfinite(track.vy) || !std::isfinite(track.confidence) ||
        track.confidence < 0 || track.confidence > 1 || track.class_id < -1 || track.class_name.size() > 128) return false;
  }
  return true;
}
}  // namespace easyfly_multi_object_tracking
