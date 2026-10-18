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

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "easyfly_obstacle_geometry/dynamic_obstacle_tracker.hpp"

namespace easyfly::obstacle_geometry
{
namespace
{

DynamicObstacleObservation box(const double x, const double y = 0.0, const double z = 1.0)
{
  DynamicObstacleObservation observation;
  observation.center_x = x;
  observation.center_y = y;
  observation.center_z = z;
  observation.size_x = 0.6;
  observation.size_y = 0.4;
  observation.size_z = 1.2;
  return observation;
}

DynamicObstacleObservation sparseHuman(
  const double x, const double z, const double sx, const double sy, const double sz)
{
  DynamicObstacleObservation observation;
  observation.center_x = x;
  observation.center_y = -1.0;
  observation.center_z = z;
  observation.size_x = sx;
  observation.size_y = sy;
  observation.size_z = sz;
  return observation;
}

DynamicObstacleTrackerConfig testConfig()
{
  DynamicObstacleTrackerConfig config;
  config.association_gate = 1.5;
  config.association_cost_margin = 0.01;
  config.min_dt = 0.01;
  config.max_dt = 1.0;
  config.speed_deadband = 0.20;
  config.max_speed = 3.0;
  config.min_direction_cosine = 0.60;
  config.min_confirmations = 2;
  config.prediction_horizon = 1.0;
  config.prediction_steps = 8;
  config.max_prediction_points = 64;
  return config;
}

TEST(DynamicObstacleTrackerTest, StaticPillarJitterStaysStatic)
{
  DynamicObstacleTracker tracker(testConfig());
  for (const auto sample : std::vector<std::pair<double, double>>{
        {0.0, 0.00}, {0.2, 0.04}, {0.4, -0.03}, {0.6, 0.02}, {0.8, -0.01}})
  {
    const auto frame = tracker.update(sample.first, {box(sample.second)});
    ASSERT_EQ(frame.estimates.size(), 1u);
    EXPECT_FALSE(frame.estimates[0].is_dynamic);
    EXPECT_DOUBLE_EQ(frame.estimates[0].velocity_x, 0.0);
    EXPECT_TRUE(tracker.predict({box(sample.second)}, frame.estimates).empty());
  }
}

TEST(DynamicObstacleTrackerTest, ConstantVelocityNeedsConsistentFramesAndPredicts)
{
  DynamicObstacleTracker tracker(testConfig());
  auto frame = tracker.update(0.0, {box(0.0)});
  EXPECT_FALSE(frame.estimates[0].is_dynamic);
  frame = tracker.update(0.2, {box(0.2)});
  EXPECT_FALSE(frame.estimates[0].is_dynamic);
  frame = tracker.update(0.4, {box(0.4)});
  EXPECT_TRUE(frame.estimates[0].is_dynamic);
  EXPECT_NEAR(frame.estimates[0].velocity_x, 1.0, 1e-9);

  const auto predicted = tracker.predict({box(0.4)}, frame.estimates);
  ASSERT_FALSE(predicted.empty());
  EXPECT_GT(predicted.back().x, 0.4);
  for (const auto & point : predicted) {
    EXPECT_TRUE(std::isfinite(point.x));
    EXPECT_TRUE(std::isfinite(point.y));
    EXPECT_TRUE(std::isfinite(point.z));
  }
}

TEST(DynamicObstacleTrackerTest, SparseHumanPlanarMotionSurvivesZSliceJitter)
{
  auto config = testConfig();
  config.association_z_gate = 1.0;
  config.association_xy_size_ratio = 6.0;
  config.association_z_size_ratio = 8.0;
  config.min_displacement = 0.15;
  DynamicObstacleTracker tracker(config);
  tracker.update(0.0, {sparseHuman(0.0, 1.0, 0.50, 0.35, 0.15)});
  tracker.update(0.2, {sparseHuman(0.25, 1.35, 0.70, 0.45, 0.50)});
  const auto frame = tracker.update(
    0.4, {sparseHuman(0.50, 0.80, 0.45, 0.30, 0.20)});
  ASSERT_EQ(frame.estimates.size(), 1u);
  EXPECT_TRUE(frame.estimates[0].is_dynamic);
  EXPECT_NEAR(frame.estimates[0].velocity_x, 1.25, 1e-9);
  EXPECT_DOUBLE_EQ(frame.estimates[0].velocity_z, 0.0);
  const auto predicted = tracker.predict(
    {sparseHuman(0.50, 0.80, 0.45, 0.30, 0.20)}, frame.estimates);
  EXPECT_FALSE(predicted.empty());
  EXPECT_GT(predicted.size(), 0u);
}

TEST(DynamicObstacleTrackerTest, R24SplitMergeDoesNotConfirmAtShortWindow)
{
  auto config = testConfig();
  config.max_speed = 5.0;
  DynamicObstacleTracker tracker(config);
  const auto sample = [](
    const double x, const double y, const double z,
    const double sx, const double sy, const double sz) {
      DynamicObstacleObservation observation;
      observation.center_x = x;
      observation.center_y = y;
      observation.center_z = z;
      observation.size_x = sx;
      observation.size_y = sy;
      observation.size_z = sz;
      return observation;
    };

  tracker.update(49.2, {sample(10.9431, -6.3035, 1.9126, 0.6864, 0.8658, 0.3826)});
  auto frame = tracker.update(
    49.3, {sample(10.6984, -6.0379, 1.2897, 1.6481, 1.6711, 2.2853)});
  ASSERT_EQ(frame.estimates.size(), 1u);
  EXPECT_FALSE(frame.estimates[0].is_dynamic);
  frame = tracker.update(
    49.4, {sample(10.5586, -5.9620, 1.3389, 1.3649, 1.6993, 2.1842)});
  ASSERT_EQ(frame.estimates.size(), 1u);
  // The 49.2 -> 49.3 centroid shift is smaller than the projected footprint
  // growth, and the later short interval is below the bounded 0.4 s window.
  EXPECT_FALSE(frame.estimates[0].is_dynamic);
  frame = tracker.update(
    49.5, {sample(10.072832, -5.926814, 0.833495, 0.390272, 1.135212, 1.638006)});
  ASSERT_EQ(frame.estimates.size(), 1u);
  EXPECT_FALSE(frame.estimates[0].is_dynamic);
}

TEST(DynamicObstacleTrackerTest, OversizedEnvironmentClusterStaysStaticAndNoPrediction)
{
  auto config = testConfig();
  config.max_dynamic_bbox_size_x = 2.5;
  config.max_dynamic_bbox_size_y = 2.5;
  config.max_dynamic_bbox_size_z = 2.5;
  DynamicObstacleTracker tracker(config);
  const auto large = [](const double x) {
      auto observation = box(x);
      observation.size_x = 7.0;
      observation.size_y = 2.0;
      observation.size_z = 1.0;
      return observation;
    };
  tracker.update(0.0, {large(0.0)});
  tracker.update(0.2, {large(0.2)});
  const auto frame = tracker.update(0.4, {large(0.4)});
  ASSERT_EQ(frame.estimates.size(), 1u);
  EXPECT_FALSE(frame.estimates[0].is_dynamic);
  EXPECT_EQ(frame.stats.rejected_by_bbox, 1u);
  EXPECT_TRUE(tracker.predict({large(0.4)}, frame.estimates).empty());
  EXPECT_FALSE(tracker.predictionEnvelope(large(0.4), 0.5).valid);
}

TEST(DynamicObstacleTrackerTest, InvalidDynamicBoxLimitFallsBackToBoundedDefault)
{
  auto config = testConfig();
  config.max_dynamic_bbox_size_x = std::numeric_limits<double>::quiet_NaN();
  config.max_dynamic_bbox_size_y = -1.0;
  config.max_dynamic_bbox_size_z = std::numeric_limits<double>::infinity();
  DynamicObstacleTracker tracker(config);
  EXPECT_DOUBLE_EQ(tracker.config().max_dynamic_bbox_size_x, 2.5);
  EXPECT_DOUBLE_EQ(tracker.config().max_dynamic_bbox_size_y, 2.5);
  EXPECT_DOUBLE_EQ(tracker.config().max_dynamic_bbox_size_z, 2.5);
}

TEST(DynamicObstacleTrackerTest, FrameDiagnosticsAreBoundedAndExplainAssociation)
{
  DynamicObstacleTracker tracker(testConfig());
  auto frame = tracker.update(0.0, {box(0.0)});
  EXPECT_EQ(frame.stats.observations, 1u);
  EXPECT_EQ(frame.stats.new_tracks, 1u);
  frame = tracker.update(0.2, {box(0.25)});
  EXPECT_EQ(frame.stats.observations, 1u);
  EXPECT_EQ(frame.stats.matched, 1u);
  EXPECT_EQ(frame.stats.new_tracks, 0u);
  frame = tracker.update(0.4, {box(0.50)});
  EXPECT_EQ(frame.stats.confirmed_dynamic, 1u);
  EXPECT_LT(frame.stats.observations, DynamicObstacleTracker::kHardMaxObservations + 1);
}

TEST(DynamicObstacleTrackerTest, StaticPillarHeightSliceJitterStaysStatic)
{
  DynamicObstacleTracker tracker(testConfig());
  const std::vector<std::pair<double, DynamicObstacleObservation>> samples{
    {0.0, sparseHuman(0.00, 1.0, 0.9, 0.9, 0.15)},
    {0.2, sparseHuman(0.04, 1.4, 0.8, 1.0, 0.40)},
    {0.4, sparseHuman(-0.03, 0.8, 1.0, 0.85, 0.20)},
    {0.6, sparseHuman(0.02, 1.2, 0.9, 0.9, 0.30)}};
  for (const auto & sample : samples) {
    const auto frame = tracker.update(sample.first, {sample.second});
    ASSERT_EQ(frame.estimates.size(), 1u);
    EXPECT_FALSE(frame.estimates[0].is_dynamic);
    EXPECT_TRUE(tracker.predict({sample.second}, frame.estimates).empty());
  }
}

TEST(DynamicObstacleTrackerTest, DirectionReversalDropsPrediction)
{
  DynamicObstacleTracker tracker(testConfig());
  tracker.update(0.0, {box(0.0)});
  tracker.update(0.2, {box(0.2)});
  auto frame = tracker.update(0.4, {box(0.4)});
  ASSERT_TRUE(frame.estimates[0].is_dynamic);

  frame = tracker.update(0.6, {box(0.2)});
  EXPECT_FALSE(frame.estimates[0].is_dynamic);
  EXPECT_DOUBLE_EQ(frame.estimates[0].velocity_x, 0.0);
  EXPECT_TRUE(tracker.predict({box(0.2)}, frame.estimates).empty());
}

TEST(DynamicObstacleTrackerTest, ContinuousMotionKeepsPredictionAfterTwoSeconds)
{
  DynamicObstacleTracker tracker(testConfig());
  DynamicObstacleFrame frame;
  for (std::size_t index = 0; index <= 10; ++index) {
    const double stamp = 0.2 * static_cast<double>(index);
    frame = tracker.update(stamp, {box(stamp)});
    ASSERT_EQ(frame.estimates.size(), 1u);
    if (index >= 2) {
      EXPECT_TRUE(frame.estimates[0].is_dynamic);
    }
  }
  EXPECT_TRUE(frame.estimates[0].is_dynamic);
  EXPECT_NEAR(frame.estimates[0].velocity_x, 1.0, 1e-9);
}

TEST(DynamicObstacleTrackerTest, SlowTurnUsesNetWindowDirection)
{
  DynamicObstacleTracker tracker(testConfig());
  DynamicObstacleFrame frame;
  constexpr double kPi = 3.14159265358979323846;
  bool saw_direction_rejection = false;
  double x = 0.0;
  double y = 0.0;
  for (std::size_t index = 0; index <= 10; ++index) {
    if (index > 0) {
      const double angle =
        (static_cast<double>(index) - 1.0) * (20.0 * kPi / 180.0);
      x += 0.1 * std::cos(angle);
      y += 0.1 * std::sin(angle);
    }
    frame = tracker.update(0.1 * static_cast<double>(index), {box(x, y)});
    ASSERT_EQ(frame.estimates.size(), 1u);
    saw_direction_rejection = saw_direction_rejection ||
      frame.stats.rejected_by_direction > 0;
  }
  EXPECT_FALSE(frame.estimates[0].is_dynamic);
  EXPECT_TRUE(saw_direction_rejection);
}

TEST(DynamicObstacleTrackerTest, NegativeDirectionThresholdFallsBackToSafeDefault)
{
  auto config = testConfig();
  config.min_direction_cosine = -1.0;
  DynamicObstacleTracker tracker(config);
  EXPECT_DOUBLE_EQ(tracker.config().min_direction_cosine, 0.60);

  tracker.update(0.0, {box(0.0)});
  tracker.update(0.2, {box(0.2)});
  auto frame = tracker.update(0.4, {box(0.4)});
  ASSERT_TRUE(frame.estimates[0].is_dynamic);
  frame = tracker.update(0.6, {box(0.2)});
  EXPECT_FALSE(frame.estimates[0].is_dynamic);
  EXPECT_TRUE(tracker.predict({box(0.2)}, frame.estimates).empty());
}

TEST(DynamicObstacleTrackerTest, MissedFrameDoesNotPublishStaleSweep)
{
  DynamicObstacleTracker tracker(testConfig());
  tracker.update(0.0, {box(0.0)});
  tracker.update(0.2, {box(0.2)});
  auto frame = tracker.update(0.4, {box(0.4)});
  ASSERT_TRUE(frame.estimates[0].is_dynamic);

  frame = tracker.update(0.6, {});
  EXPECT_TRUE(frame.estimates.empty());
  EXPECT_TRUE(tracker.predict({}, frame.estimates).empty());

  // Reacquisition starts a new confidence run; it cannot inherit the old
  // velocity through an occlusion.
  frame = tracker.update(0.8, {box(0.8)});
  ASSERT_EQ(frame.estimates.size(), 1u);
  EXPECT_FALSE(frame.estimates[0].is_dynamic);
}

TEST(DynamicObstacleTrackerTest, ExpiredMotionEvidenceNeedsFreshWindow)
{
  auto config = testConfig();
  config.max_dt = 1.5;
  DynamicObstacleTracker tracker(config);
  tracker.update(0.0, {box(0.0)});
  tracker.update(0.2, {box(0.2)});
  auto frame = tracker.update(0.4, {box(0.4)});
  ASSERT_TRUE(frame.estimates[0].is_dynamic);

  // The association-valid 1.1 s gap is longer than the prediction evidence
  // window, so it must not inherit the prior run.
  frame = tracker.update(1.5, {box(1.5)});
  ASSERT_EQ(frame.estimates.size(), 1u);
  EXPECT_FALSE(frame.estimates[0].is_dynamic);
  frame = tracker.update(1.7, {box(1.7)});
  EXPECT_FALSE(frame.estimates[0].is_dynamic);
  frame = tracker.update(1.9, {box(1.9)});
  EXPECT_TRUE(frame.estimates[0].is_dynamic);
}

TEST(DynamicObstacleTrackerTest, TwoDormantFramesDoNotPublishStalePrediction)
{
  DynamicObstacleTracker tracker(testConfig());
  tracker.update(0.0, {box(0.0)});
  tracker.update(0.2, {box(0.2)});
  auto frame = tracker.update(0.4, {box(0.4)});
  ASSERT_TRUE(frame.estimates[0].is_dynamic);
  const auto old_id = frame.estimates[0].track_id;
  EXPECT_TRUE(tracker.update(0.6, {}).estimates.empty());
  EXPECT_TRUE(tracker.update(0.8, {}).estimates.empty());
  frame = tracker.update(1.0, {box(1.0)});
  ASSERT_EQ(frame.estimates.size(), 1u);
  EXPECT_FALSE(frame.estimates[0].is_dynamic);
  EXPECT_NE(frame.estimates[0].track_id, old_id);
  EXPECT_TRUE(tracker.predict({box(1.0)}, frame.estimates).empty());
}

TEST(DynamicObstacleTrackerTest, InvalidAndOverspeedSamplesFailClosed)
{
  DynamicObstacleTracker tracker(testConfig());
  auto invalid = box(0.0);
  invalid.center_y = std::numeric_limits<double>::quiet_NaN();
  auto frame = tracker.update(0.0, {invalid});
  EXPECT_FALSE(frame.valid);
  ASSERT_EQ(frame.estimates.size(), 1u);
  EXPECT_FALSE(frame.estimates[0].is_dynamic);

  tracker.update(0.0, {box(0.0)});
  frame = tracker.update(0.2, {box(1.0)});  // 5m/s > configured 3m/s
  ASSERT_EQ(frame.estimates.size(), 1u);
  EXPECT_FALSE(frame.estimates[0].is_dynamic);
  EXPECT_DOUBLE_EQ(frame.estimates[0].velocity_x, 0.0);
  EXPECT_TRUE(tracker.predict({box(1.0)}, frame.estimates).empty());
}

TEST(DynamicObstacleTrackerTest, PredictionPointBudgetIsGlobalAndFinite)
{
  auto config = testConfig();
  config.max_prediction_points = 10;
  DynamicObstacleTracker tracker(config);
  tracker.update(0.0, {box(0.0), box(10.0)});
  tracker.update(0.2, {box(0.2), box(10.2)});
  auto frame = tracker.update(0.4, {box(0.4), box(10.4)});
  ASSERT_EQ(frame.estimates.size(), 2u);
  ASSERT_TRUE(frame.estimates[0].is_dynamic);
  ASSERT_TRUE(frame.estimates[1].is_dynamic);

  const auto predicted = tracker.predict({box(0.4), box(10.4)}, frame.estimates);
  // A nine-point footprint is indivisible: a budget of ten emits one full
  // step, never a partial footprint.
  EXPECT_EQ(predicted.size(), 9u);
  EXPECT_EQ(predicted.size() % 9u, 0u);
  for (const auto & point : predicted) {
    EXPECT_TRUE(std::isfinite(point.x));
    EXPECT_TRUE(std::isfinite(point.y));
    EXPECT_TRUE(std::isfinite(point.z));
  }
}

TEST(DynamicObstacleTrackerTest, TooSmallBudgetEmitsNoPartialFootprint)
{
  auto config = testConfig();
  config.max_prediction_points = 4;
  DynamicObstacleTracker tracker(config);
  tracker.update(0.0, {box(0.0)});
  tracker.update(0.2, {box(0.2)});
  const auto frame = tracker.update(0.4, {box(0.4)});
  ASSERT_TRUE(frame.estimates[0].is_dynamic);
  EXPECT_TRUE(tracker.predict({box(0.4)}, frame.estimates).empty());
}

TEST(DynamicObstacleTrackerTest, TrackIdsStayWithCurrentObservationOrder)
{
  DynamicObstacleTracker tracker(testConfig());
  tracker.update(0.0, {box(0.0), box(5.0)});
  tracker.update(0.2, {box(0.2), box(5.2)});
  auto frame = tracker.update(0.4, {box(0.4), box(5.4)});
  ASSERT_EQ(frame.estimates.size(), 2u);
  EXPECT_EQ(frame.estimates[0].track_id, 0u);
  EXPECT_EQ(frame.estimates[1].track_id, 1u);

  // The current observation order changes, but each result retains the
  // corresponding track identity rather than receiving a frame index.
  frame = tracker.update(0.6, {box(5.6), box(0.6)});
  ASSERT_EQ(frame.estimates.size(), 2u);
  EXPECT_EQ(frame.estimates[0].track_id, 1u);
  EXPECT_EQ(frame.estimates[1].track_id, 0u);
}

TEST(DynamicObstacleTrackerTest, OutputMatchingIsOneToOneAndIdsAreChecked)
{
  const std::vector<DynamicObstacleObservation> current{box(0.0), box(5.0)};
  const std::vector<DynamicObstacleObservation> output{box(0.1), box(5.1)};
  std::vector<DynamicObstacleEstimate> estimates(2);
  estimates[0].valid = true;
  estimates[0].track_id = 17;
  estimates[1].valid = true;
  estimates[1].track_id = 23;
  const auto matches = matchCurrentDynamicObservations(output, current, estimates, 1.0);
  ASSERT_EQ(matches.size(), 2u);
  EXPECT_EQ(matches[0], 0);
  EXPECT_EQ(matches[1], 1);

  std::int32_t message_id = -1;
  EXPECT_TRUE(tryStableObstacleId(23u, message_id));
  EXPECT_EQ(message_id, 23);
  EXPECT_FALSE(
    tryStableObstacleId(
      static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max()) + 1u,
      message_id));
}

TEST(DynamicObstacleTrackerTest, GlobalAssociationIsNotOutputOrderGreedy)
{
  const std::vector<DynamicObstacleObservation> output{box(0.0), box(0.75)};
  const std::vector<DynamicObstacleObservation> current{box(0.3), box(-0.5)};
  std::vector<DynamicObstacleEstimate> estimates(2);
  estimates[0].valid = true;
  estimates[1].valid = true;
  const auto matches = matchCurrentDynamicObservations(
    output, current, estimates, 1.0, 0.10);
  ASSERT_EQ(matches.size(), 2u);
  // The globally optimal one-to-one assignment is output0→current1 and
  // output1→current0. A nearest-first output-order walk would consume
  // current0 for output0 and leave output1 unmatched.
  EXPECT_EQ(matches[0], 1);
  EXPECT_EQ(matches[1], 0);
}

TEST(DynamicObstacleTrackerTest, CloseShapeAlternativesFailSafeAsAmbiguous)
{
  const std::vector<DynamicObstacleObservation> output{box(0.0)};
  const std::vector<DynamicObstacleObservation> current{box(0.02), box(0.04)};
  std::vector<DynamicObstacleEstimate> estimates(2);
  estimates[0].valid = true;
  estimates[1].valid = true;
  const auto matches = matchCurrentDynamicObservations(
    output, current, estimates, 1.0, 0.10);
  ASSERT_EQ(matches.size(), 1u);
  EXPECT_EQ(matches[0], -1);
}

TEST(DynamicObstacleTrackerTest, StableAndFallbackIdsAreUnique)
{
  const std::vector<std::int32_t> fallback{0, 2, 3, 4};
  const std::vector<std::int32_t> stable{1, 1, 7, 0};
  const std::vector<bool> valid{true, true, false, true};
  const auto ids = assignUniqueObstacleIds(fallback, stable, valid);
  ASSERT_EQ(ids.size(), 4u);
  EXPECT_EQ(ids[0], 1);
  EXPECT_NE(ids[1], ids[0]);
  EXPECT_EQ(ids[2], 3);
  EXPECT_EQ(ids[3], 0);
  float finite = 0.0F;
  EXPECT_FALSE(tryFiniteFloat(std::numeric_limits<double>::max(), finite));
  EXPECT_TRUE(tryFiniteFloat(1.25, finite));
  EXPECT_FLOAT_EQ(finite, 1.25F);
}

TEST(DynamicObstacleTrackerTest, TightBudgetStartsPredictionNearCurrentObject)
{
  auto config = testConfig();
  config.max_prediction_points = 9;
  DynamicObstacleTracker tracker(config);
  tracker.update(0.0, {box(0.0)});
  tracker.update(0.2, {box(0.2)});
  const auto frame = tracker.update(0.4, {box(0.4)});
  ASSERT_TRUE(frame.estimates[0].is_dynamic);
  const auto predicted = tracker.predict({box(0.4)}, frame.estimates);
  ASSERT_EQ(predicted.size(), 9u);
  EXPECT_GT(predicted.front().x, 0.4);
  EXPECT_LT(predicted.front().x, 0.8);
}

TEST(DynamicObstacleTrackerTest, EmptyOrTimestampResetDropsVelocityAndStableId)
{
  DynamicObstacleTracker tracker(testConfig());
  tracker.update(0.0, {box(0.0)});
  tracker.update(0.2, {box(0.2)});
  auto frame = tracker.update(0.4, {box(0.4)});
  ASSERT_TRUE(frame.estimates[0].is_dynamic);
  const auto old_id = frame.estimates[0].track_id;

  frame = tracker.update(0.6, {});
  EXPECT_TRUE(frame.valid);
  EXPECT_TRUE(frame.estimates.empty());
  frame = tracker.update(0.8, {box(0.8)});
  ASSERT_EQ(frame.estimates.size(), 1u);
  EXPECT_FALSE(frame.estimates[0].is_dynamic);
  const auto fresh_id = frame.estimates[0].track_id;
  EXPECT_NE(fresh_id, old_id);

  frame = tracker.update(0.7, {box(0.7)});
  ASSERT_EQ(frame.estimates.size(), 1u);
  EXPECT_FALSE(frame.estimates[0].is_dynamic);
  EXPECT_NE(frame.estimates[0].track_id, fresh_id);
}

TEST(DynamicObstacleTrackerTest, InternalAssociationUsesGlobalOneToOneGeometry)
{
  DynamicObstacleTracker tracker(testConfig());
  tracker.update(0.0, {box(0.0), box(0.75)});
  const auto frame = tracker.update(0.2, {box(0.3), box(-0.5)});
  ASSERT_EQ(frame.estimates.size(), 2u);
  // The global assignment preserves each object's track despite the input
  // order/near-crossing geometry; a nearest-first greedy walk would swap or
  // lose one of these IDs.
  EXPECT_EQ(frame.estimates[0].track_id, 1u);
  EXPECT_EQ(frame.estimates[1].track_id, 0u);
}

TEST(DynamicObstacleTrackerTest, AssociationAcceptsTwoPreviousToOneCurrent)
{
  const std::vector<DynamicObstacleObservation> previous{box(0.0), box(5.0)};
  const std::vector<DynamicObstacleObservation> current{box(0.2)};
  std::vector<DynamicObstacleEstimate> prior_estimates(1);
  prior_estimates[0].valid = true;
  const auto direct_matches = matchCurrentDynamicObservations(
    previous, current, prior_estimates, 1.5);
  ASSERT_EQ(direct_matches.size(), 2u);
  EXPECT_EQ(direct_matches[0], 0);
  EXPECT_EQ(direct_matches[1], -1);

  DynamicObstacleTracker tracker(testConfig());
  tracker.update(0.0, {box(0.0), box(5.0)});
  const auto frame = tracker.update(0.2, {box(0.2)});
  ASSERT_EQ(frame.estimates.size(), 1u);
  EXPECT_EQ(frame.estimates[0].track_id, 0u);
  EXPECT_TRUE(frame.estimates[0].stable_id_valid);
}

TEST(DynamicObstacleTrackerTest, AssociationAcceptsOnePreviousToTwoCurrent)
{
  DynamicObstacleTracker tracker(testConfig());
  tracker.update(0.0, {box(0.0)});
  const auto frame = tracker.update(0.2, {box(0.2), box(5.0)});
  ASSERT_EQ(frame.estimates.size(), 2u);
  EXPECT_EQ(frame.estimates[0].track_id, 0u);
  EXPECT_EQ(frame.estimates[1].track_id, 1u);
  EXPECT_TRUE(frame.estimates[0].stable_id_valid);
  EXPECT_TRUE(frame.estimates[1].stable_id_valid);
}

TEST(DynamicObstacleTrackerTest, ShortObservationLossStartsFreshTrack)
{
  DynamicObstacleTracker tracker(testConfig());
  tracker.update(0.0, {box(0.0)});
  tracker.update(0.2, {box(0.2)});
  auto frame = tracker.update(0.4, {box(0.4)});
  ASSERT_TRUE(frame.estimates[0].is_dynamic);
  const auto old_id = frame.estimates[0].track_id;

  frame = tracker.update(0.5, {});
  EXPECT_TRUE(frame.valid);
  EXPECT_TRUE(frame.estimates.empty());
  frame = tracker.update(0.6, {box(0.6)});
  ASSERT_EQ(frame.estimates.size(), 1u);
  EXPECT_FALSE(frame.estimates[0].is_dynamic);
  EXPECT_NE(frame.estimates[0].track_id, old_id);
}

TEST(DynamicObstacleTrackerTest, TightBudgetPrioritizesNearestCompleteFootprint)
{
  auto config = testConfig();
  config.max_prediction_points = 9;
  DynamicObstacleTracker tracker(config);
  tracker.update(0.0, {box(10.0), box(1.0)});
  tracker.update(0.2, {box(10.2), box(1.2)});
  const auto frame = tracker.update(0.4, {box(10.4), box(1.4)});
  ASSERT_TRUE(frame.estimates[0].is_dynamic);
  ASSERT_TRUE(frame.estimates[1].is_dynamic);
  const auto predicted = tracker.predict(
    {box(10.4), box(1.4)}, frame.estimates, 0.0, 0.0, 1.0);
  ASSERT_EQ(predicted.size(), 9u);
  EXPECT_LT(predicted.front().x, 2.0);
  EXPECT_EQ(predicted.size() % 9u, 0u);
}

TEST(DynamicObstacleTrackerTest, SmallDynamicBoxGetsGrowingBoundedEnvelope)
{
  auto config = testConfig();
  config.max_prediction_points = 20;
  config.prediction_min_half_x = 0.25;
  config.prediction_min_half_y = 0.30;
  config.prediction_uncertainty_growth = 0.20;
  config.prediction_max_uncertainty = 0.40;
  DynamicObstacleTracker tracker(config);
  const auto small = [](const double x) {
      auto observation = box(x, 0.0, 1.0);
      observation.size_x = 0.15;
      observation.size_y = 0.12;
      observation.size_z = 0.07;
      return observation;
    };
  tracker.update(0.0, {small(0.0)});
  tracker.update(0.2, {small(0.2)});
  const auto frame = tracker.update(0.4, {small(0.4)});
  ASSERT_TRUE(frame.estimates[0].is_dynamic);
  const auto at_zero = tracker.predictionEnvelope(small(0.4), 0.0);
  const auto at_one = tracker.predictionEnvelope(small(0.4), 1.0);
  const auto at_late = tracker.predictionEnvelope(small(0.4), 10.0);
  ASSERT_TRUE(at_zero.valid);
  ASSERT_TRUE(at_one.valid);
  ASSERT_TRUE(at_late.valid);
  EXPECT_DOUBLE_EQ(at_zero.half_x, 0.25);
  EXPECT_DOUBLE_EQ(at_zero.half_y, 0.30);
  EXPECT_GT(at_one.half_x, at_zero.half_x);
  EXPECT_GT(at_one.half_y, at_zero.half_y);
  EXPECT_DOUBLE_EQ(at_late.half_x, 0.65);
  EXPECT_DOUBLE_EQ(at_late.half_y, 0.70);
  const auto predicted = tracker.predict({small(0.4)}, frame.estimates);
  EXPECT_EQ(predicted.size(), 18u);
  EXPECT_EQ(predicted.size() % DynamicObstacleTracker::kPredictionFootprintPoints, 0u);
  for (const auto & point : predicted) {
    EXPECT_TRUE(std::isfinite(point.x));
    EXPECT_TRUE(std::isfinite(point.y));
    EXPECT_TRUE(std::isfinite(point.z));
  }
}

TEST(DynamicObstacleTrackerTest, InvalidEnvelopeParametersUseSafeDefaults)
{
  auto config = testConfig();
  config.prediction_min_half_x = -1.0;
  config.prediction_min_half_y = std::numeric_limits<double>::quiet_NaN();
  config.prediction_uncertainty_growth = -0.1;
  config.prediction_max_uncertainty = std::numeric_limits<double>::infinity();
  DynamicObstacleTracker tracker(config);
  EXPECT_DOUBLE_EQ(tracker.config().prediction_min_half_x, 0.20);
  EXPECT_DOUBLE_EQ(tracker.config().prediction_min_half_y, 0.20);
  EXPECT_DOUBLE_EQ(tracker.config().prediction_uncertainty_growth, 0.10);
  EXPECT_DOUBLE_EQ(tracker.config().prediction_max_uncertainty, 0.50);
  auto invalid = box(0.0);
  invalid.size_x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(tracker.predictionEnvelope(invalid, 0.0).valid);
}

TEST(DynamicObstacleTrackerTest, StableIdAllocationStopsBeforeMessageWrap)
{
  std::size_t next_id = static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max());
  bool exhausted = false;
  std::size_t assigned = 0;
  EXPECT_TRUE(allocateStableObstacleId(next_id, exhausted, assigned));
  EXPECT_EQ(assigned, static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max()));
  EXPECT_TRUE(exhausted);
  EXPECT_FALSE(allocateStableObstacleId(next_id, exhausted, assigned));
}

TEST(DynamicObstacleTrackerTest, FloatBoxAndTotalBudgetAreFailSafe)
{
  float cx = 0.0F;
  float cy = 0.0F;
  float cz = 0.0F;
  float sx = 0.0F;
  float sy = 0.0F;
  float sz = 0.0F;
  EXPECT_TRUE(
    tryFiniteObstacleBox(
      1.0, 2.0, 3.0, 0.4, 0.5, 1.0,
      cx, cy, cz, sx, sy, sz));
  EXPECT_FALSE(
    tryFiniteObstacleBox(
      std::numeric_limits<double>::max(), 2.0, 3.0, 0.4, 0.5, 1.0,
      cx, cy, cz, sx, sy, sz));
  EXPECT_FALSE(
    tryFiniteObstacleBox(
      1.0, 2.0, 3.0, -0.1, 0.5, 1.0,
      cx, cy, cz, sx, sy, sz));
  EXPECT_EQ(boundedCompletePredictionPointCount(100, 200, 90), 90u);
  EXPECT_EQ(boundedCompletePredictionPointCount(198, 200, 9), 0u);
  EXPECT_EQ(boundedCompletePredictionPointCount(100, 200, 10), 0u);
  EXPECT_EQ(boundedCompletePredictionPointCount(100, 200, 100, 5), 100u);
}

}  // namespace
}  // namespace easyfly::obstacle_geometry
