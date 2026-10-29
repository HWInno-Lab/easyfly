# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Frozen gauges, bounded original measurements and hysteretic source selection.

This policy never consults an FCU reference after activation. It never generates
an extrapolated pose, a new timestamp or a reset counter to conceal a jump.
"""
from collections import deque
from dataclasses import dataclass, field
import math
import uuid

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


@dataclass(frozen=True)
class Sample:
    stamp: int
    receipt: float
    p: np.ndarray
    q: np.ndarray

    @classmethod
    def make(cls, stamp, receipt, p, q):
        p, q = np.asarray(p, dtype=float), np.asarray(q, dtype=float)
        if (not isinstance(stamp, int) or stamp <= 0 or p.shape != (3,) or
                q.shape != (4,) or not np.isfinite(np.r_[p, q, receipt]).all() or
                abs(float(q @ q) - 1.) > 1e-3 or np.max(np.abs(p)) > 10000):
            raise ValueError('invalid finite metric pose, quaternion or time')
        p, q = p.copy(), q / np.linalg.norm(q)
        p.flags.writeable = q.flags.writeable = False
        return cls(stamp, receipt, p, q)


def angle(a, b):
    return float((Rotation.from_quat(a).inv() * Rotation.from_quat(b)).magnitude())


def at(history, stamp, gap=.16):
    """Interpolate strictly inside an original measurement bracket, never extrapolate."""
    for sample in reversed(history):
        if sample.stamp == stamp:
            return sample
    for a, b in zip(history, list(history)[1:]):
        if a.stamp < stamp < b.stamp and b.stamp - a.stamp <= gap * 1e9:
            t = (stamp - a.stamp) / (b.stamp - a.stamp)
            q = Slerp([0., 1.], Rotation.from_quat([a.q, b.q]))([t]).as_quat()[0]
            return Sample.make(stamp, min(a.receipt, b.receipt), (1-t)*a.p+t*b.p, q)
    return None


@dataclass
class Source:
    samples: deque = field(default_factory=lambda: deque(maxlen=256))
    identity: object = None
    failure: str = ''
    qualified: bool = False
    since: int = 0
    last_agreement: int = 0
    rejected_stale: int = 0


class Policy:
    names = ('fast_lio', 'openvins')
    # Expire before the native OpenVINS receiver's 0.5 s pause budget. Leaving
    # the decision until 0.35 simulated seconds can exceed that wall deadline
    # when a native simulation runs below real time.
    source_age = .25
    wall_age = .35
    qualification = 2.
    residence = 3.
    max_solo = 5.

    def __init__(self):
        self.session = uuid.uuid4().hex
        self.sources = {name: Source() for name in self.names}
        self.clock = 0
        self.failure = ''
        self.transforms = {}
        self.active = False
        self.selected = None
        self.switched = 0
        self.last_output = 0
        self.last_pair = 0
        self.switches = []
        self.reason = 'explicit_ground_activation_required'
        self.agreement = None
        self.activation = None

    def fail(self, reason):
        if not self.failure:
            self.failure = reason
        self.selected = None

    def clock_update(self, stamp):
        if self.clock and stamp < self.clock:
            self.fail('simulation_clock_rollback')
        self.clock = stamp

    def fresh(self, sample, now):
        return (sample is not None and -.02e9 <= self.clock-sample.stamp <= self.source_age*1e9
                and 0 <= now-sample.receipt <= self.wall_age)

    def identity(self, name, identity):
        source = self.sources[name]
        if source.identity is not None and source.identity != identity:
            source.failure = 'source_identity_changed_restart_on_ground_required'
            source.identity_change = dict(before=source.identity, after=identity)
            source.qualified = False
        elif identity is not None:
            source.identity = identity

    def ingest(self, name, sample):
        source = self.sources[name]
        if source.failure or self.failure:
            return False
        # Ordered transport backlog after a short native pause is discarded,
        # never refreshed or re-stamped. A fresh rollback is a reset, not recovery.
        if not -.02e9 <= self.clock-sample.stamp <= self.source_age*1e9:
            source.rejected_stale += 1
            return False
        if source.samples:
            old = source.samples[-1]
            if sample.stamp == old.stamp and np.array_equal(sample.p, old.p) and angle(sample.q, old.q) < 1e-8:
                return False
            dt = (sample.stamp-old.stamp)/1e9
            if dt <= 0:
                source.failure = 'fresh_source_time_rollback_or_conflict'
            elif dt > 2. or np.linalg.norm(sample.p-old.p) > .08+3.*dt or angle(sample.q, old.q) > .05+3.*dt:
                source.failure = 'source_pose_discontinuity_or_unobservable_gap'
            if source.failure:
                source.qualified = False
                return False
        source.samples.append(sample)
        return True

    def aligned(self, name, sample):
        rotation, translation = self.transforms[name]
        return Sample.make(sample.stamp, sample.receipt,
                           rotation.apply(sample.p)+translation,
                           (rotation * Rotation.from_quat(sample.q)).as_quat())

    def activate(self, references, now, ground_since, healthy):
        if self.active or self.failure or len(self.transforms):
            raise ValueError('new ground session required; activation is one-shot')
        if not ground_since or self.clock-ground_since < 2e9:
            raise ValueError('two seconds of fresh settled disarmed FCU evidence required')
        transforms, evidence = {}, {}
        for name, source in self.sources.items():
            if (source.failure or source.identity is None or not healthy.get(name) or
                    not source.samples or not self.fresh(source.samples[-1], now)):
                raise ValueError('source not healthy: '+name)
            pairs = [(s, at(references, s.stamp)) for s in source.samples
                     if max(ground_since, self.clock-2.5e9) <= s.stamp <= self.clock]
            pairs = [(a, b) for a, b in pairs if b is not None]
            if len(pairs) < 12 or pairs[-1][0].stamp-pairs[0][0].stamp < 1.8e9:
                raise ValueError('insufficient source-stamped ground alignment: '+name)
            yaws = [(Rotation.from_quat(b.q)*Rotation.from_quat(a.q).inv()).as_euler('xyz')[2]
                    for a, b in pairs]
            yaw = math.atan2(np.sin(yaws).mean(), np.cos(yaws).mean())
            rot = Rotation.from_euler('z', yaw)
            translation = np.mean([b.p-rot.apply(a.p) for a, b in pairs], axis=0)
            position = max(float(np.linalg.norm(rot.apply(a.p)+translation-b.p)) for a, b in pairs)
            orientation = max(angle((rot*Rotation.from_quat(a.q)).as_quat(), b.q) for a, b in pairs)
            motion = max(float(np.linalg.norm(b.p-pairs[0][1].p)) for _, b in pairs)
            if position > .08 or orientation > .08 or motion > .06:
                raise ValueError('ground gauge fit inconsistent or moving: '+name)
            transforms[name] = rot, translation
            evidence[name] = dict(samples=len(pairs), first_ns=pairs[0][0].stamp,
                                  last_ns=pairs[-1][0].stamp, yaw=yaw,
                                  translation=translation.tolist(), max_position_m=position,
                                  max_angle_rad=orientation, reference_motion_m=motion)
        self.transforms = transforms
        self.activation = dict(clock_ns=self.clock, frozen_gauges=evidence,
                               covariance_available=False, reference_used_after_activation=False)
        self.active = True
        # Even calibrated sources must complete a live agreement window.
        self.reason = 'qualifying_independent_sources'
        return self.activation

    def step(self, now, healthy):
        if self.failure or not self.active:
            return None
        good = {}
        for name, source in self.sources.items():
            good[name] = (not source.failure and bool(healthy.get(name)) and
                          bool(source.samples) and self.fresh(source.samples[-1], now))
            if not good[name]:
                source.qualified = False
                source.since = 0
        if all(good.values()):
            stamp = min(s.samples[-1].stamp for s in self.sources.values())
            samples = [at(self.sources[name].samples, stamp) for name in self.names]
            if all(s is not None for s in samples) and stamp > self.last_pair:
                a, b = [self.aligned(name, s) for name, s in zip(self.names, samples)]
                distance, rotation = float(np.linalg.norm(a.p-b.p)), angle(a.q, b.q)
                self.agreement = dict(stamp_ns=stamp, position_m=distance, angle_rad=rotation)
                if distance > .20 or rotation > .12:
                    self.fail('independent_sources_disagree_new_ground_session_required')
                    return None
                for source in self.sources.values():
                    if stamp-source.last_agreement > .20e9:
                        source.since = stamp
                    elif not source.since:
                        source.since = stamp
                    source.last_agreement = stamp
                    if stamp-source.since >= self.qualification*1e9:
                        source.qualified = True
                self.last_pair = stamp
        for name, source in self.sources.items():
            if self.clock-source.last_agreement > self.max_solo*1e9:
                source.qualified = False
                source.since = 0
        available = [n for n in self.names if good[n] and self.sources[n].qualified]
        old = self.selected
        if self.selected not in available:
            self.selected = available[0] if available else None
        elif (self.selected != self.names[0] and self.names[0] in available and
              self.clock-self.switched >= self.residence*1e9):
            self.selected = self.names[0]
        if old != self.selected:
            self.switched = self.clock
            self.switches.append(dict(stamp_ns=self.clock, previous=old, selected=self.selected))
            self.switches = self.switches[-64:]
        self.reason = 'healthy' if self.selected else 'no_qualified_source_stop_and_cancel_owned_task'
        if not self.selected:
            return None
        sample = self.aligned(self.selected, self.sources[self.selected].samples[-1])
        if sample.stamp <= self.last_output:
            return None
        self.last_output = sample.stamp
        return sample

    def status(self):
        return dict(session=self.session, active=self.active, selected=self.selected,
                    healthy=bool(self.selected) and not bool(self.failure), failure=self.failure,
                    reason=self.failure or self.reason, clock_ns=self.clock,
                    output_stamp_ns=self.last_output, agreement=self.agreement,
                    covariance_available=False, alignment=self.activation, switches=self.switches,
                    sources={n: dict(qualified=s.qualified, failure=s.failure,
                                     identity_change=getattr(s, 'identity_change', None),
                                     latest_ns=s.samples[-1].stamp if s.samples else 0,
                                     last_agreement=s.last_agreement, rejected_stale=s.rejected_stale)
                             for n, s in self.sources.items()})
