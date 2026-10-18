// Copyright 2026 EasyFly Project
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>
#include <limits>
#include "easyfly_motion_prediction/predictor.hpp"
using easyfly::motion_prediction::Frame;
using easyfly::motion_prediction::Observation;
using easyfly::motion_prediction::Options;
using easyfly::motion_prediction::Predictor;
static constexpr int64_t S = 1000000000LL;
static Frame frame(int64_t stamp, double x = 0, int id = 1)
{
  Observation o;o.id = id;o.stamp_ns = stamp;o.class_name = "person";o.confidence = .9;
  o.position_valid = true;o.position = {x, 0, 0};
  return Frame{stamp, "odom", true, {o}};
}
TEST(Predictor, TwoRealObservationsAndReferenceTimeHorizon) {
  Predictor p;
  ASSERT_TRUE(p.observe(frame(S), S, S));
  EXPECT_FALSE(p.predict(S, S).at(0).valid);
  ASSERT_TRUE(p.observe(frame(S + S / 10, .1), S + S / 10, S + S / 10));
  auto output = p.predict(S + S / 5, S + S / 5).at(0);
  EXPECT_TRUE(output.valid);EXPECT_NEAR(output.velocity[0], 1, 1e-9);
  EXPECT_NEAR(output.age, .1, 1e-9);EXPECT_DOUBLE_EQ(output.horizon, .5);
  EXPECT_NEAR(output.position[0], .7, 1e-9);
  EXPECT_NEAR(output.variance, .04 + .6 * .6, 1e-9);
  EXPECT_EQ(output.observation.stamp_ns, S + S / 10);
}
TEST(Predictor, InvalidWholeFrameClearsCache) {
  for (int kind = 0; kind < 14; ++kind) {
    Predictor p;ASSERT_TRUE(p.observe(frame(S), S, S));
    auto bad = frame(S + S / 10, .1);
    switch (kind) {
      case 0: bad.frame = "map";break;
      case 1: bad.stamp_ns = 0;break;
      case 2: bad.observations[0].stamp_ns++;break;
      case 3: bad.observations[0].frame = "map";break;
      case 4: bad.observations[0].id = -1;break;
      case 5: bad.observations.push_back(bad.observations[0]);break;
      case 6: bad.observations[0].position_valid = false;break;
      case 7: bad.observations[0].confidence = std::numeric_limits<double>::quiet_NaN();break;
      case 8: bad.observations[0].confidence = 1.1;break;
      case 9: bad.observations[0].confidence = .1;break;
      case 10: bad.observations[0].position[0] = std::numeric_limits<double>::infinity();break;
      case 11: bad.observations[0].class_name = std::string(129, 'x');break;
      case 12: bad.well_formed = false;break;
      case 13: bad.stamp_ns = S + S / 2;bad.observations[0].stamp_ns = bad.stamp_ns;break;
    }
    EXPECT_FALSE(p.observe(bad, S + S / 10, S + S / 10)) << kind;EXPECT_EQ(p.size(), 0u);
  }
}
TEST(Predictor, DuplicateOutOfOrderAndStaleInputsCannotRestartOldVelocity) {
  Predictor p;p.observe(frame(S), S, S);p.observe(frame(S + S / 10, .1), S + S / 10, S + S / 10);
  EXPECT_FALSE(p.observe(frame(S + S / 10, .1), S + S / 10, S + S / 10));
  EXPECT_FALSE(p.observe(frame(S), S + S / 5, S + S / 5));
  EXPECT_FALSE(p.observe(frame(S + S / 5), 2 * S, 2 * S));
  EXPECT_TRUE(p.observe(frame(2 * S), 2 * S, 2 * S));
  EXPECT_FALSE(p.predict(2 * S, 2 * S).at(0).valid);
}
TEST(Predictor, EmptyMissingAndClassChangeBreakIdentityHistory) {
  Predictor p;p.observe(frame(S), S, S);p.observe(frame(S + S / 10, .1), S + S / 10, S + S / 10);
  Frame empty{S + S / 5, "odom", true, {}};
  ASSERT_TRUE(p.observe(empty, S + S / 5, S + S / 5));EXPECT_TRUE(
    p.predict(
      S + S / 5,
      S + S / 5).empty());
  p.observe(frame(S + 3 * S / 10, .3), S + 3 * S / 10, S + 3 * S / 10);
  EXPECT_FALSE(p.predict(S + 3 * S / 10, S + 3 * S / 10).at(0).valid);
  p.observe(frame(S + 4 * S / 10, .4, 2), S + 4 * S / 10, S + 4 * S / 10);EXPECT_EQ(p.size(), 1u);
  auto changed = frame(S + S / 2, .5, 2);changed.observations[0].class_name = "car";
  p.observe(changed, S + S / 2, S + S / 2);
  EXPECT_FALSE(p.predict(S + S / 2, S + S / 2).at(0).valid);
}
TEST(Predictor, MotionBoundsRequireNewBaselineAndFreshObservation) {
  Predictor p;p.observe(frame(S), S, S);
  p.observe(frame(S + S / 10, 2), S + S / 10, S + S / 10);
  EXPECT_EQ(p.predict(S + S / 10, S + S / 10).at(0).reason, "motion_discontinuity");
  p.observe(frame(S + S / 5, 4), S + S / 5, S + S / 5);
  EXPECT_FALSE(p.predict(S + S / 5, S + S / 5).at(0).valid);
  p.observe(frame(S + 3 * S / 10, 4.1), S + 3 * S / 10, S + 3 * S / 10);
  EXPECT_TRUE(p.predict(S + 3 * S / 10, S + 3 * S / 10).at(0).valid);
  Options options;options.max_jump = .05;Predictor jump(options);
  jump.observe(frame(S), S, S);jump.observe(frame(S + S / 10, .1), S + S / 10, S + S / 10);
  EXPECT_FALSE(jump.predict(S + S / 10, S + S / 10).at(0).valid);
  Options diagonal_options;diagonal_options.max_speed = 1;
  Predictor diagonal(diagonal_options);diagonal.observe(frame(S), S, S);
  auto diagonal_frame = frame(S + S / 10, .08);diagonal_frame.observations[0].position[1] = .08;
  diagonal.observe(diagonal_frame, S + S / 10, S + S / 10);
  EXPECT_FALSE(diagonal.predict(S + S / 10, S + S / 10).at(0).valid);  // norm, not axis caps
}
TEST(Predictor, SourceReceiptAndClockProgressExpirationAreIndependent) {
  Predictor p;p.observe(frame(S), S, S);p.observe(frame(S + S / 10, .1), S + S / 10, S + S / 10);
  auto expired = p.predict(S + S / 10, S + 7 * S / 10);  // paused ROS clock, advancing steady clock
  EXPECT_TRUE(expired.empty());EXPECT_EQ(p.size(), 0u);
  p.observe(frame(2 * S), 2 * S, 2 * S);p.observe(
    frame(
      2 * S + S / 10,
      .1), 2 * S + S / 10, 2 * S + S / 10);
  EXPECT_FALSE(p.predict(3 * S, 2 * S + S / 5).at(0).valid);  // source expiration
  EXPECT_TRUE(p.predict(3 * S, 2 * S + S / 5).empty());
  p.observe(frame(3 * S + S / 10), 3 * S + S / 10, 3 * S);
  p.observe(frame(3 * S + S / 5, .1), 3 * S + S / 5, 3 * S + S / 10);
  auto receipt_expired = p.predict(3 * S + 3 * S / 10, 3 * S + 7 * S / 10);
  ASSERT_EQ(receipt_expired.size(), 1u);  // ROS progresses, but independent receipt age expired
  EXPECT_FALSE(receipt_expired[0].valid);
  p.observe(frame(4 * S), 4 * S, 4 * S);p.observe(
    frame(
      4 * S + S / 10,
      .1), 4 * S + S / 10, 4 * S + S / 10);
  EXPECT_TRUE(p.predict(2 * S, 5 * S).empty());  // ROS rollback clears identity
  p.observe(frame(2 * S + S / 10, .2), 2 * S + S / 10, 5 * S + S / 10);
  EXPECT_FALSE(p.predict(2 * S + S / 10, 5 * S + S / 10).at(0).valid);
}
TEST(Predictor, CacheAndInputBoundsAndOptions) {
  Options o;o.max_tracks = 1;Predictor p(o);auto excessive = frame(S);
  auto other = excessive.observations[0];other.id = 2;excessive.observations.push_back(other);
  EXPECT_FALSE(p.observe(excessive, S, S));EXPECT_EQ(p.size(), 0u);
  for (double horizon : {-.1, 2.1, std::numeric_limits<double>::quiet_NaN()}) {
    o.horizon = horizon;EXPECT_THROW(Predictor predictor(o), std::invalid_argument);
  }
  o.horizon = 0;Predictor zero(o);zero.observe(frame(S), S, S);
  zero.observe(frame(S + S / 10, .1), S + S / 10, S + S / 10);
  EXPECT_NEAR(zero.predict(S + S / 5, S + S / 5).at(0).position[0], .2, 1e-9);
}

TEST(PredictorMigration, DistinctClockOriginsAndPredictionsDoNotRefreshObservations)
{
  Predictor p;
  ASSERT_TRUE(p.observe(frame(100 * S), 100 * S, 9 * S));
  for (int i = 1; i <= 3; ++i) {
    const auto output = p.predict(100 * S + i * S / 100, 9 * S + i * S / 100);
    ASSERT_EQ(output.size(), 1u);
    EXPECT_FALSE(output[0].valid);
    EXPECT_EQ(output[0].observation.stamp_ns, 100 * S);
  }
  ASSERT_TRUE(p.observe(frame(100 * S + S / 10, .1), 100 * S + S / 10, 9 * S + S / 10));
  const auto output = p.predict(100 * S + S / 5, 9 * S + S / 5);
  ASSERT_EQ(output.size(), 1u);
  EXPECT_TRUE(output[0].valid);
  EXPECT_NEAR(output[0].velocity[0], 1, 1e-9);
  EXPECT_NEAR(output[0].position[0], .7, 1e-9);
  const auto expired = p.predict(100 * S + 3 * S / 10, 9 * S + 8 * S / 10);
  ASSERT_EQ(expired.size(), 1u);
  EXPECT_FALSE(expired[0].valid);
  EXPECT_EQ(expired[0].reason, "expired");
  EXPECT_TRUE(p.predict(100 * S + 3 * S / 10, 9 * S + 8 * S / 10).empty());
}

TEST(PredictorMigration, SteadyRollbackRequiresTwoNewObservations)
{
  Predictor p;
  ASSERT_TRUE(p.observe(frame(S), S, 10 * S));
  ASSERT_TRUE(p.observe(frame(S + S / 10, .1), S + S / 10, 10 * S + S / 10));
  EXPECT_TRUE(p.predict(S + S / 5, 10 * S + S / 5).at(0).valid);
  EXPECT_TRUE(p.predict(S + 3 * S / 10, 9 * S).empty());
  ASSERT_TRUE(p.observe(frame(S + 4 * S / 10, .2), S + 4 * S / 10, 9 * S + S / 10));
  EXPECT_FALSE(p.predict(S + 4 * S / 10, 9 * S + S / 10).at(0).valid);
  ASSERT_TRUE(p.observe(frame(S + S / 2, .3), S + S / 2, 9 * S + S / 5));
  EXPECT_TRUE(p.predict(S + S / 2, 9 * S + S / 5).at(0).valid);
}
