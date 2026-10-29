# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Session-pinned navigation admission over confirmed, source-stamped body poses."""
from collections import deque
import numpy as np

from .gate import Gate
from .geometry import angle_between


class Navigation:
    def __init__(self):
        self.clock = 0
        self.session = ''
        self.generation = 0
        self.failure = ''
        self.status_stamp = self.pose_stamp = self.last_output = self.epoch = 0
        self.status_receipt = self.health_receipt = float('-inf')
        self.last_output_receipt = float('-inf')
        self.health_stamp = 0
        self.state = Gate.WAITING_INPUT
        self.pending = deque(maxlen=32)
        self.history = deque(maxlen=32)

    def fail(self, reason):
        if not self.failure:
            self.failure = reason
            self.pending.clear()
            self.history.clear()

    def update_clock(self, stamp):
        if self.clock and stamp < self.clock and self.session:
            self.fail('navigation_clock_rollback')
        self.clock = stamp

    def fresh(self, stamp, receipt, now):
        return self.clock > 0 and stamp > 0 and 0 <= now-receipt <= .5 and \
            -.2e9 <= self.clock-stamp <= .5e9

    def confirm(self, session, generation, state, pose_stamp, stamp, now):
        if not session and not self.session and state in (Gate.WAITING_INPUT, Gate.INITIALIZING) \
                and pose_stamp == 0:
            return  # Initial transient-local status may precede the guard UUID.
        if not session or len(session) > 128 or generation < 0 or state not in range(6):
            self.fail('invalid_navigation_identity')
            return
        if self.session and (session != self.session or generation != self.generation):
            self.fail('navigation_session_changed')
            return
        if stamp < self.status_stamp or pose_stamp < self.pose_stamp:
            self.fail('navigation_status_rollback')
            return
        if not self.session:
            self.session, self.generation = session, generation
        if state >= Gate.RESET_REQUIRED:
            self.fail('alignment_failed')
        if stamp != self.status_stamp:
            self.status_receipt = now
        self.state, self.status_stamp, self.pose_stamp = state, stamp, pose_stamp

    def health(self, stamp, now):
        if stamp != self.health_stamp:
            self.health_receipt = now
        self.health_stamp = stamp

    def pose(self, pose):
        if self.failure:
            return
        previous = self.pending[-1] if self.pending else (
            self.history[-1] if self.history else None)
        if previous is not None:
            if pose.stamp <= previous.stamp:
                self.fail('navigation_pose_rollback_or_duplicate')
                return
            dt = (pose.stamp-previous.stamp)/1e9
            if dt > .5 or np.linalg.norm(pose.position-previous.position) > 20*dt+.1 or \
                    angle_between(pose.orientation, previous.orientation) > 10*dt+.1:
                self.fail('navigation_pose_jump_or_gap')
                return
        if len(self.pending) == self.pending.maxlen:
            if not self.last_output:
                self.pending.popleft()  # Bounded startup before visual readiness.
            else:
                self.fail('navigation_pending_overflow')
                return
        self.pending.append(pose)

    def drain(self, now, health_valid):
        if self.failure or self.state != Gate.ESTIMATING or not health_valid or not \
                self.fresh(self.status_stamp, self.status_receipt, now) or not \
                self.fresh(self.health_stamp, self.health_receipt, now):
            return []
        while self.pending and self.pending[0].stamp < self.pose_stamp:
            self.history.append(self.pending.popleft())
        if not self.pending or self.pending[0].stamp != self.pose_stamp:
            return []
        pose = self.pending.popleft()
        if not self.fresh(pose.stamp, pose.receipt, now):
            return []
        self.history.append(pose)
        # Derive average velocity over a bounded measured interval, never from FCU position.
        earlier = [p for p in self.history if .1e9 <= pose.stamp-p.stamp <= .25e9]
        velocity = None
        if earlier:
            first = earlier[0]
            velocity = (pose.position-first.position)/((pose.stamp-first.stamp)/1e9)
            if not np.isfinite(velocity).all() or np.linalg.norm(velocity) > 20:
                self.fail('invalid_navigation_velocity')
                return []
        self.last_output = pose.stamp
        self.last_output_receipt = pose.receipt
        if not self.epoch:
            self.epoch = pose.stamp
        return [(pose, velocity)]

    def status(self, now, health_valid):
        if self.failure:
            return Gate.RESET_REQUIRED, self.failure
        if not self.last_output:
            return Gate.INITIALIZING, 'waiting_for_confirmed_aligned_navigation_pose'
        if self.state != Gate.ESTIMATING or not health_valid or not \
                self.fresh(self.status_stamp, self.status_receipt, now) or not \
                self.fresh(self.health_stamp, self.health_receipt, now) or not \
                self.history or not self.fresh(self.last_output, self.last_output_receipt, now):
            return Gate.STALE, 'navigation_input_stale'
        return Gate.ESTIMATING, 'source_stamped_local_navigation_ready'
