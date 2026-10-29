// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#include <tbb/task_group.h>
#include "easyfly_transport/compat.hpp"
int easyfly_witness_main(int, char **);
int main(int argc, char **argv)
{
  if (!easyfly_transport::ValidateLibrary()) return 10;
  // Retire an actual Connection on a real task worker. This increments the
  // cumulative deferred count; Drain must leave no outstanding work. Keep the
  // reaper open for the ordinary native observer lifecycle that follows.
  tbb::task_group group;
  group.run([] { auto connection = easyfly_transport::MakeConnection(); });
  group.wait();
  easyfly_transport::Drain();
  const auto stats = easyfly_transport::GetStatistics();
  if (!stats.deferred || stats.live || stats.pending) return 11;
  return easyfly_witness_main(argc, argv);
}
