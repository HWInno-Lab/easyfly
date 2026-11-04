// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_single_object_tracking/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
#include "easyfly_single_object_tracking/backend_factory.hpp"
#include "easyfly_backend_registry/metadata.hpp"
#include "easyfly_csrt/track_core.hpp"
#include "easyfly_kcf/tracker.hpp"
namespace easyfly_single_object_tracking {
const TrackerFactory & tracker_factory() {
  static const TrackerFactory factory = [] {
    TrackerFactory result;
    result.add("csrt", [](const easyfly::backends::Parameters & input) {
      easyfly::backends::require_backend("single_tracking", "csrt");
      easyfly::backends::resolve_parameters("single_tracking", "csrt", input);
      return std::make_unique<easyfly::csrt::CsrtTracker>();
    }, [](const easyfly::backends::Parameters & input) {return easyfly::backends::resolve_parameters("single_tracking", "csrt", input);});
    result.add("kcf", [](const easyfly::backends::Parameters & input) {
      easyfly::backends::require_backend("single_tracking", "kcf");
      const auto parameters = easyfly::backends::resolve_parameters("single_tracking", "kcf", input);
      return std::make_unique<easyfly::kcf::KcfTracker>(easyfly::kcf::Parameters{
        easyfly::backends::parameter<double>(parameters, "detect_thresh")});
    }, [](const easyfly::backends::Parameters & input) {return easyfly::backends::resolve_parameters("single_tracking", "kcf", input);});
    return result;
  }();
  return factory;
}
}
