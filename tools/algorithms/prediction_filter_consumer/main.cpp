// Copyright 2026 EasyFly Project
// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <stdexcept>
#include <type_traits>
#include "easyfly_motion_prediction/predictor.hpp"
#include "easyfly_pointcloud_filters/filter_core.hpp"
#ifdef EASYFLY_CHECK_LEGACY
#include "easyfly_target_prediction/predictor.hpp"
#include "easyfly_pointcloud_filter/filter_core.hpp"
namespace prediction = easyfly_target_prediction;
namespace filters = easyfly_pointcloud_filter;
static_assert(std::is_same<prediction::Predictor, easyfly::motion_prediction::Predictor>::value);
static_assert(std::is_same<filters::Pt, easyfly::pointcloud_filters::Pt>::value);
#else
namespace prediction = easyfly::motion_prediction;
namespace filters = easyfly::pointcloud_filters;
#endif
void require(bool value) {if (!value) {throw std::runtime_error("installed algorithm check failed");}}
int main()
{
  constexpr std::int64_t S = 1000000000;
  prediction::Predictor predictor;
  prediction::Frame frame;
  frame.stamp_ns = 100 * S;
  prediction::Observation observation;
  observation.id = 7; observation.stamp_ns = frame.stamp_ns; observation.class_name = "person";
  observation.confidence = .9; observation.position_valid = true;
  observation.position = {1, 2, 3}; frame.observations.push_back(observation);
  require(predictor.observe(frame, 100 * S, 9 * S));
  require(!predictor.predict(100 * S, 9 * S).at(0).valid);
  frame.stamp_ns += S / 10;frame.observations[0].stamp_ns = frame.stamp_ns;
  frame.observations[0].position[0] += .1;
  require(predictor.observe(frame, frame.stamp_ns, 9 * S + S / 10));
  const auto output = predictor.predict(100 * S + S / 5, 9 * S + S / 5);
  require(output.size() == 1 && output[0].valid && output[0].observation.id == 7);
  require(std::abs(output[0].position[0] - 1.7) < 1e-9);
  require(std::abs(output[0].velocity[0] - 1) < 1e-9);
  require(output[0].observation.stamp_ns == frame.stamp_ns);
  require(!predictor.observe(frame, frame.stamp_ns, 9 * S + S / 5));
  require(predictor.predict(frame.stamp_ns, 9 * S + S / 5).empty());
  std::vector<filters::Pt> points = {{1.1f, .1f, .1f, 11}, {1.2f, .2f, .2f, 22},
    {-1.1f, -.1f, -.1f, 33}, {30, 0, 0, 44}, {.1f, 0, 0, 55}};
  const auto cloud = filters::filterCloud(points, .3f, 25, 1);
  require(cloud.size() == 2);
  bool positive = false, negative = false;
  for (const auto & p : cloud) {
    if (p.x > 0) {positive = p.x == points[0].x && p.y == points[0].y && p.intensity == 11;}
    else {negative = p.x == points[2].x && p.z == points[2].z && p.intensity == 33;}
  }
  require(positive && negative);
}
