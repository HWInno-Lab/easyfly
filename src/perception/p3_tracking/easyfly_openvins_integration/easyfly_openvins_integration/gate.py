# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Bounded source-time admission. Payload ownership stays with the ROS wrapper."""

from collections import deque
from dataclasses import dataclass


@dataclass(frozen=True)
class Limits:
    max_imu_gap: float = .025
    max_camera_gap: float = .25
    receipt_timeout: float = .5
    pose_timeout: float = .5
    max_source_age: float = .5
    future_tolerance: float = .2
    max_pending: int = 4


class Gate:
    WAITING_INPUT, INITIALIZING, ESTIMATING, STALE, RESET_REQUIRED, FAILED = range(6)

    def __init__(self, limits=Limits()):
        self.limits = limits
        self.clock = 0
        self.failure = None
        self.generation = 0
        self.reason = ''
        self.failure_detail = ''
        self.last_rejection = ''
        self.imu_stamp = 0
        self.camera_seen = 0
        self.camera_stamp = 0
        self.pose_stamp = 0
        self.imu_receipt = self.camera_receipt = self.pose_receipt = float('-inf')
        self.imu_forwarded = self.imu_rejected = 0
        self.camera_forwarded = self.camera_rejected = self.pose_rejected = 0
        self.imus = deque(maxlen=1000)
        self.infos = deque(maxlen=64)
        self.pending = deque()

    def fail(self, reason, reset=True):
        if self.failure is None:
            self.failure = self.RESET_REQUIRED if reset else self.FAILED
            self.generation += 1
            self.reason = reason
            self.camera_rejected += sum(p[1] is not None for p in self.pending)
            self.pending.clear()
        self.last_rejection = reason

    def clock_update(self, stamp):
        if self.clock and stamp < self.clock and (self.imu_forwarded or self.camera_seen):
            self.fail('clock_rollback')
        self.clock = stamp

    def time_valid(self, stamp):
        if self.failure is not None or not self.clock:
            return False
        if stamp <= 0 or stamp > self.clock + self.limits.future_tolerance * 1e9 or \
                stamp < self.clock - self.limits.max_source_age * 1e9:
            self.failure_detail = (f'clock_ns={self.clock} incoming_ns={stamp} '
                                   f'source_minus_clock_ns={stamp-self.clock}')
            self.fail('invalid_source_time')
            return False
        return True

    def info(self, stamp, now):
        self.infos.append(stamp)
        return self.drain(now)

    def imu(self, stamp, now):
        if not self.time_valid(stamp):
            self.imu_rejected += 1
            return False, []
        if self.imu_stamp and (stamp <= self.imu_stamp or
                               stamp - self.imu_stamp > self.limits.max_imu_gap * 1e9):
            self.imu_rejected += 1
            self.failure_detail = (f'previous_imu_ns={self.imu_stamp} incoming_imu_ns={stamp} '
                                   f'delta_ns={stamp-self.imu_stamp}')
            self.fail('imu_time_discontinuity')
            return False, []
        previous = self.imu_stamp, self.imu_receipt, self.imu_forwarded
        self.imu_stamp, self.imu_receipt = stamp, now
        self.imus.append(stamp)
        self.imu_forwarded += 1
        images = self.drain(now)
        if self.failure is not None:
            self.imu_stamp, self.imu_receipt, self.imu_forwarded = previous
            self.imu_rejected += 1
            return False, []
        return True, images

    def camera(self, stamp, payload, now):
        if not self.time_valid(stamp):
            self.camera_rejected += 1
            return []
        if stamp < self.camera_seen:
            self.camera_rejected += 1
            self.fail('camera_time_rollback_or_late_duplicate')
            return []
        if stamp == self.camera_seen:
            match = next((p for p in self.pending if p[0] == stamp), None)
            if match is None:
                self.camera_rejected += 1
                self.fail('late_camera_duplicate')
            else:
                self.camera_rejected += 1 + (match[1] is not None)
                match[1] = None  # Reject the entire ambiguous group before forwarding.
                self.last_rejection = 'ambiguous_camera_stamp'
            return self.drain(now)
        if self.camera_seen and stamp - self.camera_seen > self.limits.max_camera_gap * 1e9:
            self.camera_rejected += 1
            self.fail('camera_time_gap')
            return []
        self.camera_seen = stamp
        self.pending.append([stamp, payload, now])
        if len(self.pending) > self.limits.max_pending:
            self.failure_detail = self.pending_detail(now)
            self.fail('camera_queue_overflow')
            return []
        return self.drain(now)

    def pending_detail(self, now):
        return (f'clock_ns={self.clock} imu_ns={self.imu_stamp} '
                f'imu_oldest_ns={self.imus[0] if self.imus else 0} '
                f'info_latest_ns={self.infos[-1] if self.infos else 0} '
                'pending(stamp_ns,has_payload,has_info,receipt_age_s)=' +
                repr([(p[0], p[1] is not None, p[0] in self.infos,
                       round(now - p[2], 6)) for p in self.pending]))

    def drain(self, now):
        result = []
        before = self.camera_forwarded, self.camera_stamp, self.camera_receipt
        if self.failure is not None:
            return result
        # A strictly newer camera stamp seals the previous group. Last frame stays pending.
        while self.pending and self.pending[0][0] < self.camera_seen:
            stamp, payload, received = self.pending[0]
            if payload is None:
                self.pending.popleft()
                continue
            if stamp not in self.infos or not self.imus or self.imu_stamp < stamp:
                break
            self.pending.popleft()
            if self.imus[0] > stamp:
                self.camera_rejected += 1
                self.last_rejection = 'camera_before_imu_history'
                continue
            if now - received > self.limits.receipt_timeout:
                self.camera_rejected += 1 + len(result)
                self.fail('pending_camera_expired')
                self.camera_forwarded, self.camera_stamp, self.camera_receipt = before
                return []
            if self.camera_stamp and stamp - self.camera_stamp > self.limits.max_camera_gap * 1e9:
                self.camera_rejected += 1 + len(result)
                self.fail('forwarded_camera_gap')
                self.camera_forwarded, self.camera_stamp, self.camera_receipt = before
                return []
            result.append(payload)
            self.camera_stamp, self.camera_receipt = stamp, received
            self.camera_forwarded += 1
        return result

    def fresh_inputs(self, now):
        return self.camera_stamp > 0 and self.imu_stamp > 0 and \
            now - self.camera_receipt <= self.limits.receipt_timeout and \
            now - self.imu_receipt <= self.limits.receipt_timeout and \
            self.clock - self.camera_stamp <= self.limits.max_source_age * 1e9 and \
            self.clock - self.imu_stamp <= self.limits.max_source_age * 1e9

    def pose(self, stamp, now):
        if self.failure is not None or not self.fresh_inputs(now):
            self.pose_rejected += 1
            return False
        if stamp < self.pose_stamp or stamp > self.camera_stamp + 1000:
            self.pose_rejected += 1
            self.fail('unexpected_estimator_time')
            return False
        if stamp == self.pose_stamp or stamp <= 0 or \
                self.clock - stamp > self.limits.max_source_age * 1e9:
            self.pose_rejected += 1
            return False
        self.pose_stamp, self.pose_receipt = stamp, now
        return True

    def status(self, now):
        # Backpressure leaves later images in DDS. Missing dependencies must still
        # expire on the steady timer, including when no more inputs arrive.
        if self.pending and self.pending[0][0] < self.camera_seen and \
                now - self.pending[0][2] > self.limits.receipt_timeout:
            self.failure_detail = self.pending_detail(now)
            self.fail('pending_camera_expired')
        if self.failure is not None:
            return self.failure, self.reason
        if not self.clock or not self.camera_stamp or not self.imu_stamp:
            return self.WAITING_INPUT, 'waiting_for_clock_and_bracketed_camera_imu'
        if not self.fresh_inputs(now):
            return self.STALE, 'input_stale'
        if not self.pose_stamp:
            return self.INITIALIZING, 'waiting_for_estimator_output'
        if now - self.pose_receipt > self.limits.pose_timeout or \
                self.clock - self.pose_stamp > self.limits.max_source_age * 1e9:
            return self.STALE, 'output_stale'
        return self.ESTIMATING, 'fresh_output_tracking_quality_unavailable'
