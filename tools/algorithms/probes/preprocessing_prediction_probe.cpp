// Copyright 2026 EasyFly Project
// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <tuple>
#include <vector>
#ifdef EASYFLY_LEGACY_PREPROCESSING_PREDICTION
#include "easyfly_target_prediction/predictor.hpp"
#include "easyfly_pointcloud_filter/filter_core.hpp"
namespace model = easyfly_target_prediction;
namespace filters = easyfly_pointcloud_filter;
#else
#include "easyfly_motion_prediction/predictor.hpp"
#include "easyfly_pointcloud_filters/filter_core.hpp"
namespace model = easyfly::motion_prediction;
namespace filters = easyfly::pointcloud_filters;
#endif
constexpr std::int64_t S = 1000000000;
void predictions(model::Predictor & predictor, std::int64_t reference, std::int64_t steady)
{
  const auto output = predictor.predict(reference, steady);
  std::cout << "prediction " << predictor.size() << ' ' << output.size();
  for (const auto & p : output) {
    const auto & o = p.observation;
    std::cout << ' ' << o.id << ' ' << o.stamp_ns << ' ' << o.frame << ' ' << o.class_name
              << ' ' << o.confidence << ' ' << o.position_valid;
    for (auto v : o.position) {std::cout << ' ' << v;}
    for (auto v : p.position) {std::cout << ' ' << v;}
    for (auto v : p.velocity) {std::cout << ' ' << v;}
    std::cout << ' ' << p.age << ' ' << p.horizon << ' ' << p.variance << ' ' << p.valid << ' ' << p.reason;
  }
  std::cout << '\n';
}
void points(const char * name, std::vector<filters::Pt> output)
{
  // Only the complete retained point set is contractual, not hash-map traversal order.
  std::sort(output.begin(), output.end(), [](const auto & a, const auto & b) {
      return std::tie(a.x, a.y, a.z, a.intensity) < std::tie(b.x, b.y, b.z, b.intensity);
    });
  std::cout << name << ' ' << output.size();
  for (const auto & p : output) {std::cout << ' ' << p.x << ' ' << p.y << ' ' << p.z << ' ' << p.intensity;}
  std::cout << '\n';
}
int main()
{
  std::cout << std::hexfloat;
  model::Predictor predictor;
  std::int64_t reference = 100 * S, steady = 10 * S, previous_stamp = 0;
  for (int i = 0; i < 320; ++i) {
    if (i < 260 || i >= 280) {reference += S / 20;}
    steady += S / 20;
    if (i == 160) {reference -= 20 * S;}
    if (i == 220) {steady -= 3 * S;}
    model::Frame frame;
    frame.stamp_ns = reference;
    for (int id = 1; id <= 3; ++id) {
      model::Observation observation;
      observation.id = id;
      observation.stamp_ns = reference;
      observation.class_name = i % 45 == 7 ? "car" : "person";
      observation.position_valid = true;
      observation.confidence = .9;
      observation.position = {i * .01 + id, id * .02, 1.};
      frame.observations.push_back(observation);
    }
    switch (i % 45) {
      case 4: frame.observations[0].position[0] += 30; break;
      case 10: frame.observations.clear(); break;
      case 14: frame.observations.pop_back(); break;
      case 19: frame.frame = "map"; break;
      case 23: frame.stamp_ns += S; break;
      case 27: frame.stamp_ns -= S; break;
      case 31: frame.stamp_ns = previous_stamp; break;
      case 35: frame.observations[1].id = frame.observations[0].id; break;
      case 39: frame.well_formed = false; break;
      default: break;
    }
    for (auto & o : frame.observations) {o.stamp_ns = frame.stamp_ns;}
    std::cout << "observe " << i << ' ' << reference << ' ' << steady << ' '
              << predictor.observe(frame, reference, steady) << '\n';
    previous_stamp = frame.stamp_ns;
    predictions(predictor, reference, steady);
    predictions(predictor, reference + S / 100, steady + S / 100);
  }
  std::uint32_t seed = 211;
  for (int scenario = 0; scenario < 60; ++scenario) {
    std::vector<filters::Pt> input;
    for (int i = 0; i < 1500; ++i) {
      const auto coordinate = [&seed]() {
          seed = seed * 1664525u + 1013904223u;
          return (static_cast<int>(seed % 6000) - 3000) / 100.f;
        };
      filters::Pt p{coordinate(), coordinate(), coordinate() / 3, static_cast<float>(i % 255)};
      input.push_back(p);
      if (i % 5 == 0) {p.intensity = 1000; input.push_back(p);}
    }
    const float voxel = scenario % 4 == 0 ? 0.f : scenario % 4 == 1 ? .1f : scenario % 4 == 2 ? .5f : 1.f;
    std::cout << "cloud " << scenario << '\n';
    points("range", filters::rangeFilter(input, .3f, 25.f));
    points("voxel", filters::voxelDownsample(input, voxel));
    points("chain", filters::filterCloud(input, .3f, 25.f, voxel));
  }
}
