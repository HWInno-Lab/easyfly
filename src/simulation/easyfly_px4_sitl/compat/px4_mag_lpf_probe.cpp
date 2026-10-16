// Copyright 2026 EasyFly Project
// SPDX-License-Identifier: Apache-2.0
// Runs the checked production update expression with PX4's actual AlphaFilter
// and matrix implementation. No simulator, flight command or sensor substitution.
#include <mathlib/math/filter/AlphaFilter.hpp>
#include <matrix/matrix/math.hpp>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

using matrix::Vector3f;
struct Sample { uint64_t time_us; Vector3f mag; };
struct Previous { uint64_t timestamp_sample; };

class Probe {
public:
    Vector3f push(const Sample &mag_sample) {
        if (!initialized) {
            _mag_lpf.reset(mag_sample.mag);
            initialized = true;
        } else {
#include EASYFLY_MAG_UPDATE_HEADER
        }
        aid_src.timestamp_sample = mag_sample.time_us;
        return _mag_lpf.getState();
    }
private:
    AlphaFilter<Vector3f> _mag_lpf{0.010f, 0.09f};
    Previous aid_src{};
    bool initialized{false};
};

int main(int argc, char **argv) {
    std::cout << std::setprecision(10);
    if (argc == 2 && std::string(argv[1]) == "--step") {
        // Same physical step, sampled at 10ms and 70ms. At 140ms the response
        // must reflect 140ms of elapsed time, regardless of arrival count.
        std::cout << "{";
        bool first = true;
        for (uint64_t dt : {10000UL, 70000UL}) {
            Probe probe;
            probe.push({1000000, Vector3f{0.f, 0.f, 0.f}});
            Vector3f result;
            for (uint64_t t = dt; t <= 140000; t += dt) {
                result = probe.push({1000000 + t, Vector3f{1.f, 0.f, 0.f}});
            }
            if (!first) { std::cout << ","; }
            first = false;
            std::cout << "\"" << dt << "\":" << result(0);
        }
        std::cout << "}\n";
        return 0;
    }
    Probe probe;
    std::string line;
    uint64_t previous = 0;
    while (std::getline(std::cin, line)) {
        std::istringstream row(line);
        uint64_t stamp;
        float x, y, z;
        if (!(row >> stamp >> x >> y >> z) || (previous && stamp <= previous)) {
            std::cerr << "invalid or non-increasing recorded sample\n";
            return 2;
        }
        previous = stamp;
        const Vector3f value = probe.push({stamp, Vector3f{x, y, z}});
        std::cout << stamp << ' ' << value(0) << ' ' << value(1) << ' ' << value(2) << '\n';
    }
    return previous ? 0 : 2;
}
