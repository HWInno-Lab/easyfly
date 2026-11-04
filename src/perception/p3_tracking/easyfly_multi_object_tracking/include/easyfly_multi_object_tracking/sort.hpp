// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_multi_object_tracking/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
// Copyright 2026 EasyFly Project
//

#pragma once
// Compatibility aliases only; the implementation is in src/algorithms/tracking/.
#include "easyfly_sort/sort.hpp"
#include "easyfly_multi_object_tracking/kalman.hpp"
#include "easyfly_multi_object_tracking/hungarian.hpp"

namespace easyfly_multi_object_tracking
{
using easyfly::sort::DetectionInput;
using easyfly::sort::TrackOutput;
using easyfly::sort::SortParams;
using easyfly::sort::Sorter;
}  // namespace easyfly_multi_object_tracking
