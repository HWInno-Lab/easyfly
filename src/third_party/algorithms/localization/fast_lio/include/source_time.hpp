// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace easyfly_fastlio {
inline int64_t source_nanoseconds(double seconds) {
    const double ns = seconds * 1e9;
    if (!std::isfinite(ns) || ns < 0 ||
        ns >= static_cast<double>(std::numeric_limits<int64_t>::max())) {
        throw std::invalid_argument("invalid source time");
    }
    return std::llround(ns);
}
}
