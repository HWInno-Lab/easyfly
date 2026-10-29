// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#ifndef EASYFLY_FASTLIO_PATH_TIP_HPP_
#define EASYFLY_FASTLIO_PATH_TIP_HPP_
#include <cstddef>
#include <vector>
namespace easyfly_fastlio {
// Preserve the existing one-in-ten history density, but always expose the most
// recent real observation at the tip. No timer fabricates a new source stamp.
template<class Pose>
void update_path_tip(std::vector<Pose>& poses, std::size_t& observations, const Pose& pose) {
    if (poses.empty() || observations % 10 == 0) { poses.push_back(pose); }
    else { poses.back() = pose; }
    ++observations;
}
}  // namespace easyfly_fastlio
#endif
