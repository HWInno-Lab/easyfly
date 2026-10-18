#ifndef PLAN_ENV_ODOM_FRESHNESS_H
#define PLAN_ENV_ODOM_FRESHNESS_H

#include <cstdint>

namespace plan_env
{

// The message stamp and the node-clock receive time are intentionally kept
// as separate samples.  A valid message stamp alone cannot prove that the
// odometry stream is still running when the ROS clock advances.
struct OdomFreshnessSample
{
  std::int64_t stamp_ns{0};
  std::int64_t receive_ns{0};
};

enum class OdomFreshnessResult
{
  kFresh,
  kStale,
  kClockJump,
  kInvalid,
};

inline OdomFreshnessResult evaluateOdomFreshness(
  const OdomFreshnessSample & sample,
  const std::int64_t now_ns,
  const std::int64_t max_age_ns,
  const std::int64_t future_tolerance_ns,
  const std::int64_t forward_jump_limit_ns)
{
  if (now_ns <= 0 || sample.stamp_ns <= 0 || sample.receive_ns <= 0 ||
    max_age_ns <= 0 || future_tolerance_ns < 0 || forward_jump_limit_ns <= 0)
  {
    return OdomFreshnessResult::kInvalid;
  }

  // A backward clock reset is distinguishable from an ordinary stale stream.
  if (now_ns < sample.receive_ns) {
    return OdomFreshnessResult::kClockJump;
  }

  const auto receive_age = static_cast<std::uint64_t>(now_ns) -
    static_cast<std::uint64_t>(sample.receive_ns);
  if (receive_age > static_cast<std::uint64_t>(forward_jump_limit_ns)) {
    return OdomFreshnessResult::kClockJump;
  }

  if (sample.stamp_ns > now_ns) {
    const auto future_age = static_cast<std::uint64_t>(sample.stamp_ns) -
      static_cast<std::uint64_t>(now_ns);
    if (future_age > static_cast<std::uint64_t>(future_tolerance_ns)) {
      return OdomFreshnessResult::kClockJump;
    }
  } else {
    const auto stamp_age = static_cast<std::uint64_t>(now_ns) -
      static_cast<std::uint64_t>(sample.stamp_ns);
    if (stamp_age > static_cast<std::uint64_t>(max_age_ns)) {
      return OdomFreshnessResult::kStale;
    }
  }

  if (receive_age > static_cast<std::uint64_t>(max_age_ns)) {
    return OdomFreshnessResult::kStale;
  }
  return OdomFreshnessResult::kFresh;
}

}  // namespace plan_env

#endif  // PLAN_ENV_ODOM_FRESHNESS_H
