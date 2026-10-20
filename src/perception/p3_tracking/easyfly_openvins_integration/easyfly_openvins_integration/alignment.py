# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Bounded, session-pinned, observation-only alignment to a source-time reference."""

from collections import deque
from dataclasses import dataclass
import math

import numpy as np

from .gate import Gate
from .geometry import (
    angle_between, covariance, imu_to_base, interpolate, multiply, quaternion,
    rotation, vector, yaw, yaw_quaternion)


@dataclass(frozen=True)
class AlignmentLimits:
    freshness: float = .5
    reference_gap: float = .15
    reference_samples: int = 200
    pending_samples: int = 16
    initialization_samples: int = 5
    initialization_span: float = .2
    initialization_position_error: float = .15
    initialization_angle_error: float = .15
    position_error: float = 1.
    angle_error: float = .35


@dataclass
class Pose:
    stamp: int
    position: np.ndarray
    orientation: np.ndarray
    receipt: float
    covariance: np.ndarray = None

    def __post_init__(self):
        if type(self.stamp) is not int or self.stamp <= 0 or not math.isfinite(self.receipt):
            raise ValueError('invalid source stamp/receipt')
        self.position = vector(self.position, 3)
        self.orientation = quaternion(self.orientation)
        if self.covariance is not None:
            self.covariance = covariance(self.covariance)


class Alignment:
    def __init__(self, translation, orientation_ib, limits=AlignmentLimits()):
        self.translation = vector(translation, 3)
        self.orientation_ib = quaternion(orientation_ib)
        self.limits = limits
        self.clock = 0
        self.session_id = ''
        self.generation = 0
        self.failure = None
        self.reason = ''
        self.source_state = Gate.WAITING_INPUT
        self.source_pose_stamp = self.source_status_stamp = 0
        self.source_received = self.health_received = float('-inf')
        self.health_stamp = 0
        self.health_valid = False
        self.references = deque(maxlen=limits.reference_samples)
        self.pending = deque()
        self.initial = []
        self.last_pose_seen = self.last_output_stamp = 0
        self.last_output_receipt = float('-inf')
        self.transform = None
        self.pairs = self.rejected = 0
        self.last_rejection = ''

    def fail(self, reason, reset=True):
        if self.failure is None:
            self.failure = Gate.RESET_REQUIRED if reset else Gate.FAILED
            self.reason = reason
            self.generation += 1
            self.rejected += len(self.pending)
            self.pending.clear()
        self.last_rejection = reason

    def update_clock(self, stamp):
        if self.clock and stamp < self.clock and (self.last_pose_seen or self.references):
            self.fail('alignment_clock_rollback')
        self.clock = stamp

    def fresh(self, stamp, receipt, now):
        return self.clock > 0 and 0 <= now-receipt <= self.limits.freshness and \
            -.2e9 <= self.clock-stamp <= self.limits.freshness*1e9

    def confirm(self, session, generation, state, pose_stamp, status_stamp, now):
        if self.failure is not None:
            return
        if not session or len(session) > 128 or generation < 0 or state not in range(6):
            self.fail('invalid_alignment_source_identity', reset=False)
            return
        if self.session_id and (session != self.session_id or generation != self.generation):
            self.fail('localization_session_or_generation_changed')
            return
        self.session_id = session
        if not self.source_status_stamp:
            self.generation = generation
        if state in (Gate.RESET_REQUIRED, Gate.FAILED):
            self.fail('localization_upstream_failed', reset=state == Gate.RESET_REQUIRED)
            return
        if ((status_stamp < self.source_status_stamp and (self.last_pose_seen or self.references))
                or pose_stamp < self.source_pose_stamp):
            self.fail('localization_status_time_rollback')
            return
        # Repeated status with a frozen source clock must not renew freshness.
        if status_stamp != self.source_status_stamp:
            self.source_received = now
        self.source_state, self.source_pose_stamp = state, pose_stamp
        self.source_status_stamp = status_stamp

    def health(self, valid, stamp, now):
        if stamp != self.health_stamp:
            self.health_received = now
        self.health_stamp, self.health_valid = stamp, valid

    def reference(self, pose):
        if self.failure is not None:
            return
        if self.references:
            previous = self.references[-1]
            if pose.stamp == previous.stamp and np.array_equal(
                    pose.position, previous.position) and angle_between(
                        pose.orientation, previous.orientation) < 1e-7:
                return  # Identical republish does not extend the receipt lifetime.
            if pose.stamp <= previous.stamp:
                self.fail('reference_time_rollback_or_conflict')
                return
            if pose.stamp-previous.stamp > self.limits.reference_gap*1e9:
                self.fail('reference_source_gap')
                return
        self.references.append(pose)

    def pose(self, pose):
        if self.failure is not None:
            self.rejected += 1
            return
        if pose.covariance is None or pose.stamp <= self.last_pose_seen:
            self.rejected += 1
            self.fail('alignment_pose_time_or_covariance', reset=False)
            return
        self.last_pose_seen = pose.stamp
        self.pending.append(pose)
        if len(self.pending) > self.limits.pending_samples:
            self.fail('alignment_pose_queue_overflow')

    def paired_reference(self, stamp, now):
        for first, second in zip(self.references, list(self.references)[1:]):
            if first.stamp <= stamp <= second.stamp:
                if not self.fresh(first.stamp, first.receipt, now) or not self.fresh(
                        second.stamp, second.receipt, now):
                    return None
                fraction = (stamp-first.stamp)/(second.stamp-first.stamp)
                return interpolate(first.position, first.orientation,
                                   second.position, second.orientation, fraction)
        return None  # Never extrapolate or substitute the latest reference.

    def inputs_ready(self, now):
        return self.source_state == Gate.ESTIMATING and self.health_valid and \
            self.fresh(self.source_status_stamp, self.source_received, now) and \
            self.fresh(self.source_pose_stamp, self.source_received, now) and \
            self.fresh(self.health_stamp, self.health_received, now)

    def drain(self, now):
        outputs = []
        if self.failure is not None:
            return outputs
        while self.pending and not self.fresh(
                self.pending[0].stamp, self.pending[0].receipt, now):
            # Startup may precede the first status/reference. Drop expired data;
            # no output can be admitted until all sources are fresh together.
            self.pending.popleft()
            self.rejected += 1
            self.last_rejection = 'alignment_pending_pose_expired'
        if not self.inputs_ready(now):
            return outputs
        while self.pending and self.pending[0].stamp < self.source_pose_stamp:
            self.pending.popleft()
            self.rejected += 1
            self.last_rejection = 'pose_not_confirmed_by_status'
        if not self.pending or self.pending[0].stamp != self.source_pose_stamp:
            return outputs
        pose = self.pending[0]
        reference = self.paired_reference(pose.stamp, now)
        if reference is None:
            return outputs
        self.pending.popleft()
        p, q, cov = imu_to_base(
            pose.position, pose.orientation, pose.covariance,
            self.translation, self.orientation_ib)
        base = Pose(pose.stamp, p, q, pose.receipt, cov)
        rp, rq = reference
        self.pairs += 1
        if self.transform is None:
            self.initial.append((pose.stamp, p, q, rp, rq))
            # Initialization is bounded even if the source rate is unusually high.
            if len(self.initial) > 100:
                self.fail('alignment_initialization_window_overflow')
                return []
            if len(self.initial) >= self.limits.initialization_samples and \
                    self.initial[-1][0]-self.initial[0][0] >= \
                    self.limits.initialization_span*1e9:
                angles = [yaw(refq)-yaw(bq) for _, _, bq, _, refq in self.initial]
                angle = math.atan2(sum(math.sin(a) for a in angles),
                                   sum(math.cos(a) for a in angles))
                aq = yaw_quaternion(angle)
                r = rotation(aq)
                t = np.mean([refp-r@bp for _, bp, _, refp, _ in self.initial], axis=0)
                if any(np.linalg.norm(t+r@bp-refp) >
                       self.limits.initialization_position_error or angle_between(
                           multiply(aq, bq), refq) > self.limits.initialization_angle_error
                       for _, bp, bq, refp, refq in self.initial):
                    self.fail('initial_alignment_inconsistent')
                    return []
                self.transform = (t, aq)  # Frozen for this entire localization session.
                self.initial.clear()
        aligned = None
        if self.transform is not None:
            t, aq = self.transform
            ap, aori = t+rotation(aq)@p, multiply(aq, q)
            if np.linalg.norm(ap-rp) > self.limits.position_error or \
                    angle_between(aori, rq) > self.limits.angle_error:
                self.fail('aligned_reference_discontinuity')
                return []
            # Alignment/reference uncertainty is unknown: do not emit a fabricated covariance.
            aligned = Pose(pose.stamp, ap, aori, pose.receipt)
            self.last_output_stamp, self.last_output_receipt = pose.stamp, pose.receipt
        outputs.append((base, aligned))
        return outputs

    def status(self, now):
        if self.failure is not None:
            return self.failure, self.reason
        if not self.session_id or not self.references:
            return Gate.WAITING_INPUT, 'waiting_for_localization_and_source_time_reference'
        if self.source_state == Gate.WAITING_INPUT:
            return Gate.WAITING_INPUT, 'waiting_for_localization_inputs'
        if self.source_state == Gate.INITIALIZING:
            return Gate.INITIALIZING, 'waiting_for_localization_initialization'
        if not self.inputs_ready(now):
            return Gate.STALE, 'localization_or_reference_health_stale'
        if self.transform is None:
            return Gate.INITIALIZING, 'collecting_consistent_yaw_translation_pairs'
        if not self.fresh(self.last_output_stamp, self.last_output_receipt, now):
            return Gate.STALE, 'aligned_output_stale'
        if not self.fresh(self.references[-1].stamp, self.references[-1].receipt, now):
            return Gate.STALE, 'reference_stream_stale'
        return Gate.ESTIMATING, 'aligned_pose_observation_only_no_map_or_flight_admission'
