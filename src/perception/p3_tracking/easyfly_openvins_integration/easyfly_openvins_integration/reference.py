# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Pair FCU position and attitude by their own source stamps, without cached attitude."""

from collections import deque

import numpy as np

from .alignment import Pose
from .geometry import angle_between, interpolate


class ReferencePairer:
    def __init__(self, engine):
        self.engine = engine
        self.attitudes = deque(maxlen=engine.limits.reference_samples)
        self.positions = deque()
        self.last_position = None

    def add_position(self, sample):
        if self.engine.failure is not None:
            return
        if self.last_position is not None:
            previous = self.last_position
            if sample.stamp == previous.stamp and np.array_equal(
                    sample.position, previous.position):
                return
            if sample.stamp <= previous.stamp:
                self.engine.fail('reference_position_time_rollback_or_conflict')
                return
            if sample.stamp-previous.stamp > self.engine.limits.reference_gap*1e9:
                self.engine.fail('reference_position_source_gap')
                return
        self.last_position = sample
        self.positions.append(sample)
        if len(self.positions) > self.engine.limits.pending_samples:
            self.engine.fail('reference_position_queue_overflow')
            self.positions.clear()

    def add_attitude(self, sample):
        if self.engine.failure is not None:
            return
        if self.attitudes:
            previous = self.attitudes[-1]
            if sample.stamp == previous.stamp and angle_between(
                    sample.orientation, previous.orientation) < 1e-7:
                return
            if sample.stamp <= previous.stamp:
                self.engine.fail('reference_attitude_time_rollback_or_conflict')
                return
            if sample.stamp-previous.stamp > self.engine.limits.reference_gap*1e9:
                self.engine.fail('reference_attitude_source_gap')
                return
        self.attitudes.append(sample)

    def drain(self, now):
        if self.engine.failure is not None:
            self.positions.clear()
            return
        while self.positions:
            position = self.positions[0]
            if not self.engine.fresh(position.stamp, position.receipt, now):
                self.positions.popleft()
                self.engine.last_rejection = 'reference_position_expired'
                continue
            if not self.attitudes:
                return
            if self.attitudes[0].stamp > position.stamp:
                self.positions.popleft()
                self.engine.last_rejection = 'position_before_attitude_history'
                continue
            if self.attitudes[-1].stamp < position.stamp:
                return
            matched = False
            for first, second in zip(self.attitudes, list(self.attitudes)[1:]):
                if first.stamp <= position.stamp <= second.stamp:
                    if not self.engine.fresh(first.stamp, first.receipt, now) or \
                            not self.engine.fresh(second.stamp, second.receipt, now):
                        return
                    fraction = (position.stamp-first.stamp)/(second.stamp-first.stamp)
                    _, q = interpolate([0, 0, 0], first.orientation,
                                       [0, 0, 0], second.orientation, fraction)
                    self.engine.reference(Pose(
                        position.stamp, position.position, q,
                        min(position.receipt, first.receipt, second.receipt)))
                    matched = True
                    break
            if not matched:
                return
            self.positions.popleft()
