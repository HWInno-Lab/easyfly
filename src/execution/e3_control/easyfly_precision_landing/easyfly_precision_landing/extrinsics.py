# Copyright 2026 EasyFly Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Bounded original-time camera mounts; no latest-TF fallback or extrapolation."""

from collections import deque
from dataclasses import dataclass
import math

from easyfly_precision_landing.geometry import apply, finite, rotation


@dataclass(frozen=True)
class Mount:
    """One measured body-to-optical transform and its first steady receipt."""

    stamp_ns: int
    receipt: float
    translation: tuple
    quaternion: tuple


def interpolate_rotation(a, b, fraction):
    """Slerp unit xyzw rotations along their shortest geodesic."""
    rotation(a)
    rotation(b)
    dot = sum(x * y for x, y in zip(a, b))
    if dot < 0:
        b, dot = tuple(-x for x in b), -dot
    dot = min(1., dot)
    if dot > .9995:
        q = tuple(x + fraction * (y - x) for x, y in zip(a, b))
    else:
        angle = math.acos(dot)
        left = math.sin((1 - fraction) * angle) / math.sin(angle)
        right = math.sin(fraction * angle) / math.sin(angle)
        q = tuple(left * x + right * y for x, y in zip(a, b))
    norm = math.sqrt(sum(x * x for x in q))
    return tuple(x / norm for x in q)


class MountHistory:
    """Pin one publisher and require fresh bracketing encoder observations."""

    def __init__(self, optical_frame, max_age=.3, pair_gap=.1):
        if (not optical_frame or not math.isfinite(max_age)
                or not math.isfinite(pair_gap) or not 0 < pair_gap <= max_age <= .3):
            raise ValueError('invalid dynamic mount limits')
        self.frame = optical_frame
        self.max_age = max_age
        self.pair_gap = pair_gap
        self.samples = deque(maxlen=64)
        self.gid = None
        self.failure = ''

    def invalidate(self, reason):
        """Require a new configured session after an identity/discontinuity failure."""
        self.samples.clear()
        self.failure = reason
        raise ValueError(reason)

    def receive(self, sample, parent, child, gid, clock_ns, now):
        """Admit a distinct original measurement; duplicates retain first receipt."""
        if self.failure:
            raise ValueError(self.failure)
        if (parent != 'base_link' or child != self.frame or not gid
                or type(sample.stamp_ns) is not int or sample.stamp_ns <= 0
                or not finite((sample.receipt, now))
                or len(sample.translation) != 3 or not finite(sample.translation)
                or math.hypot(*sample.translation) > .5):
            self.invalidate('invalid_dynamic_mount_frame_or_geometry')
        try:
            rotation(sample.quaternion)
        except ValueError:
            self.invalidate('invalid_dynamic_mount_rotation')
        if self.gid is not None and gid != self.gid:
            self.invalidate('dynamic_mount_source_changed')
        self.gid = gid
        if (not -.02 <= (clock_ns - sample.stamp_ns) * 1e-9 <= self.max_age
                or not 0 <= now - sample.receipt <= self.max_age):
            return False
        if self.samples:
            last = self.samples[-1]
            if sample.stamp_ns == last.stamp_ns:
                if (sample.translation != last.translation
                        or sample.quaternion != last.quaternion):
                    self.invalidate('conflicting_dynamic_mount_stamp')
                return False
            if sample.stamp_ns < last.stamp_ns:
                self.invalidate('dynamic_mount_stamp_rollback')
            dt = (sample.stamp_ns - last.stamp_ns) * 1e-9
            dot = min(1., abs(sum(x * y for x, y in zip(
                sample.quaternion, last.quaternion))))
            if (math.dist(sample.translation, last.translation) > .002 + .3 * dt
                    or 2 * math.acos(dot) > .002 + 1.5 * dt):
                self.invalidate('dynamic_mount_discontinuity')
            if dt > self.pair_gap:
                self.samples.clear()
        self.samples.append(sample)
        return True

    def at(self, stamp_ns, clock_ns, now):
        """Return an exact/interpolated mount only inside a fresh source bracket."""
        if self.failure:
            raise ValueError(self.failure)
        if not 0 < stamp_ns <= clock_ns + 20000000:
            raise ValueError('invalid_dynamic_mount_query_time')
        usable = [s for s in self.samples
                  if 0 <= now - s.receipt <= self.max_age
                  and -.02 <= (clock_ns - s.stamp_ns) * 1e-9 <= self.max_age]
        for sample in usable:
            if sample.stamp_ns == stamp_ns:
                return sample.translation, sample.quaternion
        for first, last in zip(usable, usable[1:]):
            gap = (last.stamp_ns - first.stamp_ns) * 1e-9
            if first.stamp_ns < stamp_ns < last.stamp_ns and gap <= self.pair_gap:
                fraction = (stamp_ns - first.stamp_ns) / (last.stamp_ns - first.stamp_ns)
                position = tuple(a + fraction * (b - a) for a, b in zip(
                    first.translation, last.translation))
                return position, interpolate_rotation(first.quaternion, last.quaternion, fraction)
        raise ValueError('dynamic_mount_original_time_bracket_unavailable')


def measured_mount(angle, pivot, offset, axis, zero):
    """Compose calibrated pivot/axis/optical geometry with measured joint angle."""
    if (not math.isfinite(angle) or abs(angle) > .35
            or any(len(v) != 3 or not finite(v) for v in (pivot, offset, axis))
            or math.hypot(*pivot) > .3 or math.hypot(*offset) > .2
            or abs(sum(x*x for x in axis) - 1) > 1e-6):
        raise ValueError('invalid_encoder_or_explicit_mount_calibration')
    rotation(zero)
    s, c = math.sin(angle / 2), math.cos(angle / 2)
    x, y, z = (v*s for v in axis)
    a, b, d, e = zero
    quaternion = (c*a+x*e+y*d-z*b, c*b-x*d+y*e+z*a,
                  c*d+x*b-y*a+z*e, c*e-x*a-y*b-z*d)
    delta = apply(rotation((x, y, z, c)), offset)
    return tuple(a+b for a, b in zip(pivot, delta)), quaternion
