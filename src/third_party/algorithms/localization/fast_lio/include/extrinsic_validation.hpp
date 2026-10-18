#pragma once

#include <cmath>
#include <string>
#include <vector>

namespace fast_lio
{

// The Eigen initializers used by FAST-LIO index these arrays directly.  Keep
// the validation separate from the node constructor so it can be regression
// tested without starting ROS or a sensor pipeline.
inline bool validate_extrinsics(
    const std::vector<double> &extrinsic_t,
    const std::vector<double> &extrinsic_r,
    std::string *error = nullptr)
{
    auto fail = [error](const std::string &message) {
        if (error != nullptr) {
            *error = message;
        }
        return false;
    };

    if (extrinsic_t.size() != 3U) {
        return fail("mapping.extrinsic_T must contain exactly 3 values (got " +
                    std::to_string(extrinsic_t.size()) + ")");
    }
    if (extrinsic_r.size() != 9U) {
        return fail("mapping.extrinsic_R must contain exactly 9 values (got " +
                    std::to_string(extrinsic_r.size()) + ")");
    }

    for (const double value : extrinsic_t) {
        if (!std::isfinite(value)) {
            return fail("mapping.extrinsic_T contains a non-finite value");
        }
    }
    for (const double value : extrinsic_r) {
        if (!std::isfinite(value)) {
            return fail("mapping.extrinsic_R contains a non-finite value");
        }
    }

    if (error != nullptr) {
        error->clear();
    }
    return true;
}

}  // namespace fast_lio
