#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "plan_env/odom_freshness.h"
#include "plan_env/trusted_cloud_persistence.h"

namespace
{
constexpr std::int64_t kSecond = 1000000000LL;
constexpr std::int64_t kFreshness = kSecond;
constexpr std::int64_t kFutureTolerance = 200000000LL;
constexpr std::int64_t kClockJumpLimit = 10 * kSecond;

plan_env::OdomFreshnessResult evaluate(
  const plan_env::OdomFreshnessSample & sample,
  const std::int64_t now_ns)
{
  return plan_env::evaluateOdomFreshness(
    sample, now_ns, kFreshness, kFutureTolerance, kClockJumpLimit);
}
}  // namespace

TEST(OdomFreshness, ValidSampleIsFreshUsingStampAndReceiveClock)
{
  const plan_env::OdomFreshnessSample sample{10 * kSecond, 10 * kSecond};
  EXPECT_EQ(evaluate(sample, 10 * kSecond), plan_env::OdomFreshnessResult::kFresh);
  EXPECT_EQ(evaluate(sample, 11 * kSecond), plan_env::OdomFreshnessResult::kFresh);
}

TEST(OdomFreshness, StoppedStreamBecomesStaleAndCannotKeepCloudAlive)
{
  plan_env::OdomFreshnessSample sample{10 * kSecond, 10 * kSecond};
  std::vector<char> occupied(1, 0);
  std::vector<std::uint16_t> age(1, 0);
  plan_env::markTrustedCloudVoxel(occupied, age, 0);

  // Clouds may continue to arrive, but once the odom receive clock is past
  // the one-second bound, the trusted layer is cleared rather than aged back
  // into a usable state.
  EXPECT_EQ(evaluate(sample, 12 * kSecond), plan_env::OdomFreshnessResult::kStale);
  plan_env::clearTrustedCloudStorage(occupied, age);
  EXPECT_EQ(occupied[0], 0);
  EXPECT_EQ(age[0], 0);
}

TEST(OdomFreshness, BackwardAndLargeForwardClockJumpsFailClosed)
{
  const plan_env::OdomFreshnessSample sample{10 * kSecond, 10 * kSecond};
  EXPECT_EQ(evaluate(sample, 9 * kSecond), plan_env::OdomFreshnessResult::kClockJump);
  EXPECT_EQ(
    evaluate(sample, 21 * kSecond), plan_env::OdomFreshnessResult::kClockJump);
}

TEST(OdomFreshness, NewValidSampleRecoversOnlyFromClearedTrustedState)
{
  const plan_env::OdomFreshnessSample old_sample{10 * kSecond, 10 * kSecond};
  std::vector<char> occupied(2, 0);
  std::vector<std::uint16_t> age(2, 0);
  plan_env::markTrustedCloudVoxel(occupied, age, 1);

  ASSERT_EQ(evaluate(old_sample, 12 * kSecond), plan_env::OdomFreshnessResult::kStale);
  plan_env::clearTrustedCloudStorage(occupied, age);

  const plan_env::OdomFreshnessSample new_sample{13 * kSecond, 13 * kSecond};
  EXPECT_EQ(evaluate(new_sample, 13 * kSecond), plan_env::OdomFreshnessResult::kFresh);
  EXPECT_EQ(occupied[1], 0);
  plan_env::markTrustedCloudVoxel(occupied, age, 0);
  EXPECT_EQ(occupied[0], 1);
  EXPECT_EQ(occupied[1], 0);
}

TEST(OdomFreshness, InvalidAndTooFarFutureSamplesAreRejected)
{
  EXPECT_EQ(
    evaluate(plan_env::OdomFreshnessSample{}, 10 * kSecond),
    plan_env::OdomFreshnessResult::kInvalid);
  const plan_env::OdomFreshnessSample future{12 * kSecond, 10 * kSecond};
  EXPECT_EQ(evaluate(future, 10 * kSecond), plan_env::OdomFreshnessResult::kClockJump);
}
