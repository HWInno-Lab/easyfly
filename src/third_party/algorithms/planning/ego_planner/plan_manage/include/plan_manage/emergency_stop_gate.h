#ifndef EGO_PLANNER_EMERGENCY_STOP_GATE_H_
#define EGO_PLANNER_EMERGENCY_STOP_GATE_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>

namespace ego_planner
{

// Small, ROS-free latch for the safety transition owned by EGOReplanFSM.
// A request made while already stopped is idempotent and does not cause a
// second stop trajectory or a repeated state-transition log.  The pending
// bit is consumed exactly once by the FSM timer.
class EmergencyStopGate
{
public:
  enum class RecoveryAction
  {
    kHold,
    kWaitForTarget,
    kResumePlanning
  };

  enum class RequestResult
  {
    kAlreadyStopped,
    kArmed
  };

  RequestResult request(bool already_stopped) noexcept
  {
    if (already_stopped) {
      return RequestResult::kAlreadyStopped;
    }
    stop_pending_ = true;
    return RequestResult::kArmed;
  }

  bool consume(bool stopped) noexcept
  {
    if (!stopped || !stop_pending_) {
      return false;
    }
    stop_pending_ = false;
    return true;
  }

  bool pending() const noexcept {return stop_pending_;}

  // A failed stop-trajectory construction/publication must be retried while
  // the FSM remains in EMERGENCY_STOP.  This is intentionally separate from
  // request(): a retry is not a new safety transition and must not replace
  // the target-generation snapshot owned by the recovery gate.
  void retry() noexcept {stop_pending_ = true;}

  // These two predicates are deliberately kept next to the latch so that
  // collision callbacks cannot accidentally plan or commit EXEC_TRAJ after
  // the safety state has been entered.
  bool shouldReturnFromCollisionCallback(bool stopped) const noexcept
  {
    return !canTransitionToReplan(stopped);
  }

  bool canCommitExecution(bool stopped) const noexcept
  {
    return !stopped;
  }

  // A remote trajectory can finish validation and collision checking after a
  // safety stop has already been entered.  In that state the broadcast must
  // never reopen planning; the stop trajectory remains authoritative.
  bool canTransitionToReplan(bool stopped) const noexcept
  {
    return !stopped;
  }

  // Decide what the FSM should do after the stop trajectory has settled.
  // Keeping this predicate ROS-free makes the terminal-goal case explicit:
  // a stopped vehicle that is already within the old goal tolerance must not
  // regenerate that completed goal forever.  A fresh manual goal will then
  // arrive in WAIT_TARGET and take the normal GEN_NEW_TRAJ path.
  static RecoveryAction recoveryAction(
    bool stopped, bool have_target, bool fresh_cloud,
    double goal_distance, double no_replan_threshold) noexcept
  {
    if (!stopped || !fresh_cloud) {
      return RecoveryAction::kHold;
    }
    if (have_target && std::isfinite(goal_distance) &&
        std::isfinite(no_replan_threshold) && no_replan_threshold > 0.0 &&
        goal_distance < no_replan_threshold) {
      return RecoveryAction::kWaitForTarget;
    }
    return RecoveryAction::kResumePlanning;
  }

private:
  bool stop_pending_{false};
};

// ROS-free stability gate for leaving EMERGENCY_STOP.  The FSM owns the ROS
// observations (odom/cloud freshness and current ROS time); this helper only
// makes the safety contract explicit and testable:
//
//   1. a stop trajectory must have been published successfully;
//   2. finite low velocity, fresh odom and fresh trusted cloud must hold for a
//      bounded number of consecutive samples and a minimum wall of time;
//   3. a ROS-clock reset/jump clears the evidence and requires a new stop
//      publication before recovery can proceed.
//
// The target generation captured at stop entry is kept here as well.  A
// terminal old goal may be cleared only if no newer goal was accepted while
// the vehicle was stopped.
class EmergencyRecoveryGate
{
public:
  struct Params
  {
    int stable_cycles{5};
    double hold_seconds{0.5};
    double max_speed{0.1};
    double max_clock_jump{1.0};
    double ack_timeout{0.5};
    double ack_retry_period{0.5};
    double ack_retry_max_period{5.0};
  };

  EmergencyRecoveryGate()
  {
    configure(Params{});
  }

  explicit EmergencyRecoveryGate(const Params & params)
  {
    configure(params);
  }

  void configure(const Params & params) noexcept
  {
    params_.stable_cycles =
      params.stable_cycles >= 1 && params.stable_cycles <= 100 ? params.stable_cycles : 5;
    params_.hold_seconds =
      finitePositiveBounded(params.hold_seconds, 0.5, 0.01, 30.0);
    params_.max_speed =
      finitePositiveBounded(params.max_speed, 0.1, 1e-4, 10.0);
    params_.max_clock_jump =
      finitePositiveBounded(params.max_clock_jump, 1.0, 0.05, 60.0);
    params_.ack_timeout =
      finitePositiveBounded(params.ack_timeout, 0.5, 0.05, 5.0);
    params_.ack_retry_period =
      finitePositiveBounded(params.ack_retry_period, 0.5, 0.05, 5.0);
    params_.ack_retry_max_period =
      finitePositiveBounded(params.ack_retry_max_period, 5.0, 0.05, 30.0);
    if (params_.ack_retry_max_period < params_.ack_retry_period) {
      params_.ack_retry_max_period = params_.ack_retry_period;
    }
    reset();
  }

  const Params & params() const noexcept {return params_;}

  void beginStop(
    const std::uint64_t target_generation,
    const std::uint64_t activation_sequence_baseline = 0) noexcept
  {
    target_generation_snapshot_ = target_generation;
    have_generation_snapshot_ = true;
    activation_sequence_baseline_ = activation_sequence_baseline;
    have_expected_stop_traj_id_ = false;
    stop_published_ = false;
    downstream_acknowledged_ = false;
    stable_cycles_seen_ = 0;
    stop_time_ = quietNaN();
    stable_start_time_ = quietNaN();
    last_time_ = quietNaN();
    last_stop_attempt_time_ = quietNaN();
    last_ack_time_ = quietNaN();
    last_ack_stamp_ = quietNaN();
    last_ack_sequence_ = 0;
    attempt_publisher_key_.clear();
    attempt_last_activation_sequence_ = 0;
    attempt_id_ = nextAttemptId(attempt_id_);
    stop_retry_count_ = 0;
  }

  // Returns true only for a finite, successfully published stop trajectory
  // with a usable clock sample.  A failed publication deliberately leaves the
  // gate requiring another publication.
  bool markStopPublished(
    const bool success, const double now,
    const std::uint32_t expected_stop_traj_id = 1) noexcept
  {
    const bool was_published = stop_published_;
    stop_published_ = success && std::isfinite(now) && now >= 0.0 &&
      expected_stop_traj_id != 0;
    have_expected_stop_traj_id_ = stop_published_;
    expected_stop_traj_id_ = expected_stop_traj_id;
    downstream_acknowledged_ = false;
    stable_cycles_seen_ = 0;
    stable_start_time_ = quietNaN();
    last_time_ = stop_published_ ? now : quietNaN();
    stop_time_ = stop_published_ ? now : quietNaN();
    last_stop_attempt_time_ = std::isfinite(now) && now >= 0.0 ? now : quietNaN();
    last_ack_time_ = quietNaN();
    last_ack_stamp_ = quietNaN();
    last_ack_sequence_ = 0;
    // A retry is a new ACK attempt. A restarted publisher may bind only after
    // this reset and only with the graph-verified new endpoint GID.
    attempt_publisher_key_.clear();
    attempt_last_activation_sequence_ = 0;
    attempt_id_ = nextAttemptId(attempt_id_);
    if (stop_retry_count_ < kMaxRetryCount) {
      ++stop_retry_count_;
    }
    if (!was_published && stop_retry_count_ == 0) {
      stop_retry_count_ = 1;
    }
    return stop_published_;
  }

  bool needsStopPublication() const noexcept {return !stop_published_;}

  // A ROS publish call only proves local serialization.  The FSM uses this
  // predicate to retry until a new, matching downstream PositionCommand has
  // arrived, and to retry again if that stream disappears during recovery.
  bool stopPublicationDue(const double now) const noexcept
  {
    if (!std::isfinite(now) || now < 0.0) return false;
    if (!std::isfinite(last_stop_attempt_time_)) return true;
    if (now < last_stop_attempt_time_) return true;
    if (!stop_published_ || !downstream_acknowledged_) {
      return now - last_stop_attempt_time_ >= retryDelay();
    }
    // TrajectoryAck is a one-shot validation/activation receipt, not a
    // heartbeat.  Once this attempt is acknowledged, do not re-publish the
    // same stop merely because the receipt has aged.  The FSM separately
    // checks that the acknowledged publisher endpoint is still alive.
    return false;
  }

  bool acknowledgeStop(
    const std::uint32_t trajectory_id, const std::uint64_t sequence,
    const bool ack_source_valid, const std::string & publisher_key,
    const double message_stamp, const double receive_time) noexcept
  {
    if (!stop_published_ || !have_expected_stop_traj_id_ ||
      !ack_source_valid || trajectory_id != expected_stop_traj_id_ ||
      publisher_key.empty() || sequence == 0 ||
      sequence <= activation_sequence_baseline_ || !std::isfinite(message_stamp) ||
      !std::isfinite(receive_time) || receive_time < 0.0 ||
      message_stamp <= 0.0 || message_stamp < stop_time_ ||
      message_stamp > receive_time ||
      receive_time - message_stamp > params_.ack_timeout ||
      (std::isfinite(last_ack_stamp_) && message_stamp <= last_ack_stamp_))
    {
      return false;
    }
    // A single stop publication/attempt is bound to one endpoint GID. A
    // changed GID (including a restarted traj_server whose sequence starts
    // at one again) can bind only after markStopPublished starts a new
    // attempt. Within one GID activation_sequence is strictly monotonic.
    if (!attempt_publisher_key_.empty() && publisher_key != attempt_publisher_key_) {
      return false;
    }
    if (!attempt_publisher_key_.empty() && sequence <= attempt_last_activation_sequence_) {
      return false;
    }
    const auto global_sequence = publisher_sequences_.find(publisher_key);
    if (global_sequence != publisher_sequences_.end() &&
      sequence <= global_sequence->second)
    {
      return false;
    }
    // Keep a small replay ledger across publisher restarts and recovery
    // cycles. Refuse to grow it without bound; the safe result is to reject
    // a new endpoint until the process is restarted under a fresh graph.
    if (global_sequence == publisher_sequences_.end() && publisher_sequences_.size() >= 16) {
      return false;
    }
    attempt_publisher_key_ = publisher_key;
    attempt_last_activation_sequence_ = sequence;
    publisher_sequences_[publisher_key] = sequence;
    last_ack_sequence_ = sequence;
    last_ack_stamp_ = message_stamp;
    last_ack_time_ = receive_time;
    downstream_acknowledged_ = true;
    return true;
  }

  // Compatibility overload for older ROS-free tests. Production callbacks
  // must pass the endpoint GID key explicitly through the overload above.
  bool acknowledgeStop(
    const std::uint32_t trajectory_id, const std::uint64_t sequence,
    const bool ack_source_valid, const double message_stamp,
    const double receive_time) noexcept
  {
    return acknowledgeStop(
      trajectory_id, sequence, ack_source_valid, "legacy-test-publisher",
      message_stamp, receive_time);
  }

  bool downstreamAcknowledged() const noexcept {return downstream_acknowledged_;}

  bool acknowledgedPublisherMatches(const std::string & publisher_key) const noexcept
  {
    return downstream_acknowledged_ && !attempt_publisher_key_.empty() &&
           publisher_key == attempt_publisher_key_;
  }

  // The dedicated ACK topic is accepted only when discovery reports exactly
  // one publisher, its endpoint GID matches the message metadata, the graph
  // identifies the configured traj_server node FQN and type, and the message
  // carries the same source FQN. This is a trusted ROS graph boundary: it
  // prevents accidental remaps/multiple publishers, but is not an
  // anti-spoofing mechanism; hostile deployments require DDS Security.
  static bool endpointFqnMatches(
    const std::string & node_name, const std::string & node_namespace,
    const std::string & expected_fqn)
  {
    if (node_name.empty() || expected_fqn.empty()) {
      return false;
    }
    const auto normalize_fqn = [](const std::string & value) {
        if (value.empty()) {
          return std::string();
        }
        std::string normalized = value.front() == '/' ? value : "/" + value;
        while (normalized.size() > 1 && normalized.back() == '/') {
          normalized.pop_back();
        }
        return normalized;
      };
    std::string normalized_namespace = node_namespace.empty() ? "/" : node_namespace;
    if (normalized_namespace.front() != '/') {
      normalized_namespace = "/" + normalized_namespace;
    }
    while (normalized_namespace.size() > 1 && normalized_namespace.back() == '/') {
      normalized_namespace.pop_back();
    }
    const std::string actual_fqn = normalized_namespace == "/" ?
      "/" + node_name : normalized_namespace + "/" + node_name;
    return normalize_fqn(actual_fqn) == normalize_fqn(expected_fqn);
  }

  static bool validTrajectoryAckSource(
    const bool unique_publisher, const bool gid_matches,
    const std::string & endpoint_node_name,
    const std::string & endpoint_node_namespace,
    const std::string & expected_node_fqn,
    const std::string & endpoint_topic_type,
    const std::string & source_node_fqn,
    const std::string & expected_topic_type = "traj_utils/msg/TrajectoryAck")
  {
    const std::string normalized_expected = expected_node_fqn.empty() ? std::string() :
      (expected_node_fqn.front() == '/' ? expected_node_fqn : "/" + expected_node_fqn);
    return unique_publisher && gid_matches &&
           endpointFqnMatches(
             endpoint_node_name, endpoint_node_namespace, normalized_expected) &&
           endpoint_topic_type == expected_topic_type &&
           source_node_fqn == normalized_expected;
  }

  // Kept only for existing ROS-free callers; production uses the FQN
  // overload above so a bare node name cannot satisfy the source contract.
  static bool validTrajectoryAckSource(
    const bool unique_publisher, const bool gid_matches,
    const bool node_matches, const bool type_matches,
    const bool source_field_matches) noexcept
  {
    return unique_publisher && gid_matches && node_matches && type_matches &&
           source_field_matches;
  }

  // A newer target must not inherit stable evidence collected for the old
  // target.  Keep the stop publication/ACK contract intact while restarting
  // the low-speed hold from this generation's first stable sample.
  void resetStableEvidence() noexcept
  {
    stable_cycles_seen_ = 0;
    stable_start_time_ = quietNaN();
  }

  // Observe one FSM sample.  False means hold EMERGENCY_STOP; true means the
  // bounded stability contract has been met.  Clock rollback or an excessive
  // forward jump invalidates all consecutive evidence and also forces a new
  // stop publication.
  bool observe(
    const double now, const bool odom_fresh, const bool cloud_fresh,
    const double speed, const bool ack_source_alive = true) noexcept
  {
    if (!stop_published_ || !std::isfinite(now) || now < 0.0) {
      return false;
    }
    if (!std::isfinite(last_time_) || now < last_time_ ||
      now - last_time_ > params_.max_clock_jump)
    {
      resetEvidence(true);
      return false;
    }
    last_time_ = now;

    if (!downstream_acknowledged_ || !ack_source_alive ||
      !odom_fresh || !cloud_fresh || !std::isfinite(speed) ||
      speed > params_.max_speed)
    {
      // A source loss is different from an ordinary transient sensor sample:
      // the one-shot ACK can no longer prove a live executor.  Force a new
      // stop publication/attempt before any future recovery evidence counts.
      if (downstream_acknowledged_ && !ack_source_alive) {
        resetEvidence(true);
      }
      stable_cycles_seen_ = 0;
      stable_start_time_ = quietNaN();
      return false;
    }
    if (stable_cycles_seen_ == 0) {
      stable_start_time_ = now;
    }
    ++stable_cycles_seen_;
    const bool cycles_ok = stable_cycles_seen_ >= params_.stable_cycles;
    const bool hold_ok = std::isfinite(stable_start_time_) &&
      now >= stable_start_time_ && now - stable_start_time_ >= params_.hold_seconds;
    return cycles_ok && hold_ok;
  }

  void reset() noexcept
  {
    stop_published_ = false;
    have_expected_stop_traj_id_ = false;
    downstream_acknowledged_ = false;
    have_generation_snapshot_ = false;
    target_generation_snapshot_ = 0;
    stable_cycles_seen_ = 0;
    stop_time_ = quietNaN();
    stable_start_time_ = quietNaN();
    last_time_ = quietNaN();
    last_stop_attempt_time_ = quietNaN();
    last_ack_time_ = quietNaN();
    last_ack_stamp_ = quietNaN();
    last_ack_sequence_ = 0;
    attempt_publisher_key_.clear();
    attempt_last_activation_sequence_ = 0;
    stop_retry_count_ = 0;
  }

  bool targetUnchanged(const std::uint64_t current_generation) const noexcept
  {
    return have_generation_snapshot_ &&
      current_generation == target_generation_snapshot_;
  }

  static bool shouldClearReachedTarget(
    const std::uint64_t stop_generation,
    const std::uint64_t current_generation,
    const bool have_target,
    const double goal_distance,
    const double no_replan_threshold) noexcept
  {
    return stop_generation == current_generation && have_target &&
      std::isfinite(goal_distance) && std::isfinite(no_replan_threshold) &&
      no_replan_threshold > 0.0 && goal_distance < no_replan_threshold;
  }

  int stableCyclesSeen() const noexcept {return stable_cycles_seen_;}

  static bool canRecoverOdomDepthTimeout(
    const bool latched, const bool odom_fresh, const bool cloud_fresh,
    const bool have_target, const std::uint64_t stop_generation,
    const std::uint64_t current_generation) noexcept
  {
    return !latched || (odom_fresh && cloud_fresh && have_target &&
      current_generation != stop_generation);
  }

private:
  static double quietNaN() noexcept
  {
    return std::numeric_limits<double>::quiet_NaN();
  }

  static double finitePositiveBounded(
    const double value, const double fallback, const double minimum,
    const double maximum) noexcept
  {
    if (!std::isfinite(value) || value < minimum || value > maximum) return fallback;
    return value;
  }

  void resetEvidence(const bool require_republish) noexcept
  {
    stable_cycles_seen_ = 0;
    stable_start_time_ = quietNaN();
    last_time_ = quietNaN();
    if (require_republish) {
      stop_published_ = false;
      stop_time_ = quietNaN();
      have_expected_stop_traj_id_ = false;
      downstream_acknowledged_ = false;
      last_ack_time_ = quietNaN();
      last_ack_stamp_ = quietNaN();
      last_ack_sequence_ = 0;
      attempt_publisher_key_.clear();
      attempt_last_activation_sequence_ = 0;
      attempt_id_ = nextAttemptId(attempt_id_);
      stop_retry_count_ = 0;
    }
  }

  static std::uint64_t nextAttemptId(const std::uint64_t current) noexcept
  {
    return current == std::numeric_limits<std::uint64_t>::max() ? 1 : current + 1;
  }

  double retryDelay() const noexcept
  {
    double delay = params_.ack_retry_period;
    const int exponent = std::min(std::max(stop_retry_count_ - 1, 0), 6);
    for (int i = 0; i < exponent; ++i) {
      delay = std::min(params_.ack_retry_max_period, delay * 2.0);
    }
    return std::min(params_.ack_retry_max_period, delay);
  }

  static constexpr int kMaxRetryCount = 7;

  Params params_;
  bool stop_published_{false};
  bool have_expected_stop_traj_id_{false};
  bool downstream_acknowledged_{false};
  bool have_generation_snapshot_{false};
  std::uint64_t target_generation_snapshot_{0};
  std::uint64_t activation_sequence_baseline_{0};
  std::uint64_t last_ack_sequence_{0};
  std::uint64_t attempt_last_activation_sequence_{0};
  std::uint64_t attempt_id_{0};
  int stop_retry_count_{0};
  std::uint32_t expected_stop_traj_id_{0};
  int stable_cycles_seen_{0};
  double stop_time_{std::numeric_limits<double>::quiet_NaN()};
  double stable_start_time_{std::numeric_limits<double>::quiet_NaN()};
  double last_time_{std::numeric_limits<double>::quiet_NaN()};
  double last_stop_attempt_time_{std::numeric_limits<double>::quiet_NaN()};
  double last_ack_time_{std::numeric_limits<double>::quiet_NaN()};
  double last_ack_stamp_{std::numeric_limits<double>::quiet_NaN()};
  std::string attempt_publisher_key_;
  std::unordered_map<std::string, std::uint64_t> publisher_sequences_;
};

}  // namespace ego_planner

#endif  // EGO_PLANNER_EMERGENCY_STOP_GATE_H_
