# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_gimbal_control/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project
"""Device abstraction and deterministic rate-limited simulation."""

from abc import ABC, abstractmethod
import math


class GimbalDevice(ABC):
    """Minimal backend contract; acceptance is not physical completion."""

    @abstractmethod
    def command(self, angles):
        """Accept or reject a bounded angle tuple (pitch, yaw, roll)."""

    @abstractmethod
    def stop(self):
        """Stop pending motion without returning to neutral."""

    @abstractmethod
    def update(self, elapsed):
        """Advance device observation and return current angles."""


class SimulatedGimbal(GimbalDevice):
    """Each axis approaches its target at a bounded degree/second rate."""

    def __init__(self, limits, rate):
        """Validate startup bounds and initialize neutral angles."""
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError("rate must be finite and positive")
        if len(limits) != 3 or any(
            not math.isfinite(lo)
            or not math.isfinite(hi)
            or lo >= hi
            or not lo <= 0 <= hi
            for lo, hi in limits
        ):
            raise ValueError(
                "three finite increasing limits must include zero"
            )
        self.limits = limits
        self.rate = rate
        self.angles = (0.0, 0.0, 0.0)
        self.target = self.angles

    def command(self, angles):
        """Reject invalid or out-of-range inputs without clamping."""
        if len(angles) != 3 or any(
            not math.isfinite(value) or not lo <= value <= hi
            for value, (lo, hi) in zip(angles, self.limits)
        ):
            return False
        self.target = tuple(angles)
        return True

    def stop(self):
        """Discard the old target."""
        self.target = self.angles

    def update(self, elapsed):
        """Integrate only a finite nonnegative interval."""
        if not math.isfinite(elapsed) or elapsed < 0:
            raise ValueError("invalid update interval")
        step = elapsed * self.rate
        self.angles = tuple(
            current + max(-step, min(step, target - current))
            for current, target in zip(self.angles, self.target)
        )
        return self.angles


def target_angles(delta, attitude):
    """Transform odom ENU vector through inverse Rz(yaw)Ry(pitch)Rx(roll)."""
    if not all(math.isfinite(v) for v in (*delta, *attitude)):
        raise ValueError("nonfinite target geometry")
    x, y, z = delta
    roll, pitch, yaw = attitude
    x, y = math.cos(yaw) * x + math.sin(yaw) * y, (
        -math.sin(yaw) * x + math.cos(yaw) * y
    )
    x, z = math.cos(pitch) * x - math.sin(pitch) * z, (
        math.sin(pitch) * x + math.cos(pitch) * z
    )
    y, z = math.cos(roll) * y + math.sin(roll) * z, (
        -math.sin(roll) * y + math.cos(roll) * z
    )
    if math.sqrt(x * x + y * y + z * z) < 1e-6:
        raise ValueError("coincident target")
    return (
        math.degrees(math.atan2(z, math.hypot(x, y))),
        math.degrees(math.atan2(y, x)),
        0.0,
    )
