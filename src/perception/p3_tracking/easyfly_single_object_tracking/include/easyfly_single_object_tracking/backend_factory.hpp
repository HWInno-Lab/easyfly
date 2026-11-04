// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_single_object_tracking/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
#pragma once
#include <cmath>
#include "easyfly_backend_registry/configuration.hpp"
#include "easyfly_single_tracking_api/tracker.hpp"
namespace easyfly_single_object_tracking {
using TrackerFactory = easyfly::backends::Factory<easyfly::single_tracking::Tracker>;
const TrackerFactory & tracker_factory();
inline std::unique_ptr<easyfly::single_tracking::Tracker> make_tracker(
  const easyfly::backends::BackendConfig & config, const TrackerFactory & factory = tracker_factory()) {
  auto result = factory.create(config);
  if (result->backend() != config.backend) throw std::invalid_argument("single tracker creator identity mismatch");
  return result;
}
inline bool valid_result(const easyfly::single_tracking::Result & result, int width, int height) {
  const auto & box = result.box;
  return result.ok && box.x >= 0 && box.y >= 0 && box.width > 0 && box.height > 0 &&
    box.width <= width && box.height <= height && box.x <= width - box.width && box.y <= height - box.height &&
    (!result.calibrated_confidence || (std::isfinite(*result.calibrated_confidence) &&
      *result.calibrated_confidence >= 0 && *result.calibrated_confidence <= 1));
}
}
