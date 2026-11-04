// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_multi_object_tracking/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
#include "easyfly_multi_object_tracking/adapters/tracker_factory.hpp"
namespace easyfly_multi_object_tracking {
void register_sort(TrackerFactory & factory);
const TrackerFactory & tracker_factory() {
  static const TrackerFactory factory = [] {
    TrackerFactory result;
    register_sort(result);
    return result;
  }();
  return factory;
}
}
