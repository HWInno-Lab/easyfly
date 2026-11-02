// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
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
