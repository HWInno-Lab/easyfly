# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_gimbal_control/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project
"""Device bounds and full orientation geometry regression."""
import math
from types import SimpleNamespace

import pytest

from easyfly_gimbal_control.model import SimulatedGimbal, target_angles
from easyfly_gimbal_control.node import GimbalNode


def test_rate_limit_and_stop():
    """Commands are accepted without teleporting; stop discards old motion."""
    device = SimulatedGimbal([(-90, 90)] * 3, 10)
    assert device.command((20, -20, 0))
    assert device.angles == (0, 0, 0)
    assert device.update(0.5) == (5, -5, 0)
    device.stop()
    assert device.update(10) == (5, -5, 0)


@pytest.mark.parametrize(
    "angles", [(91, 0, 0), (0, math.nan, 0), (0, 0, math.inf), (0, 0)]
)
def test_invalid_command_retains_target(angles):
    """Invalid requests must not alter a previous accepted target."""
    device = SimulatedGimbal([(-90, 90)] * 3, 10)
    device.command((10, 0, 0))
    assert not device.command(angles)
    assert device.target == (10, 0, 0)


@pytest.mark.parametrize("rate", [0, -1, math.inf, math.nan])
def test_invalid_rate(rate):
    """Startup rejects invalid rate constraints."""
    with pytest.raises(ValueError):
        SimulatedGimbal([(-90, 90)] * 3, rate)


def test_limits_require_finite_neutral_range():
    """Bounds are increasing and include the initial neutral state."""
    for bounds in [[(1, 2)] * 3, [(-1, math.inf)] * 3, [(1, 1)] * 3]:
        with pytest.raises(ValueError):
            SimulatedGimbal(bounds, 10)


def test_geometry_uses_all_attitude_axes():
    """Verify yaw, pitch and roll inverse rotation with known vectors."""
    assert target_angles((1, 1, 0), (0, 0, 0)) == pytest.approx((0, 45, 0))
    assert target_angles((0, 1, 0), (0, 0, math.pi / 2)) == pytest.approx(
        (0, 0, 0)
    )
    assert target_angles((1, 0, 0), (0, math.pi / 4, 0)) == pytest.approx(
        (45, 0, 0)
    )
    assert target_angles((0, 0, 1), (math.pi / 2, 0, 0)) == pytest.approx(
        (0, 90, 0)
    )
    with pytest.raises(ValueError):
        target_angles((0, 0, 0), (0, 0, 0))


@pytest.mark.parametrize(
    "frame, stamp, ros, receipt, expected",
    [
        ("odom", 10.0, 10.1, 19.9, True),
        ("map", 10.0, 10.1, 19.9, False),
        ("odom", 0.0, 10.1, 19.9, False),
        ("odom", 10.0, 0.0, 19.9, False),
        ("odom", 10.0, 9.9, 19.9, False),
        ("odom", 10.0, 11.0, 19.9, False),
        ("odom", 10.0, 10.0, 19.0, False),
        ("odom", 10.0, 10.0, 20.1, False),
    ],
)
def test_dual_clock_source_evidence(
    monkeypatch, frame, stamp, ros, receipt, expected
):
    """Frozen ROS time never masks receipt expiry or invalid source time."""
    monkeypatch.setattr(
        "easyfly_gimbal_control.node.time.monotonic", lambda: 20
    )
    fake = SimpleNamespace(
        observation_timeout=0.5,
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=int(ros * 1e9))
        ),
    )
    header = SimpleNamespace(
        frame_id=frame,
        stamp=SimpleNamespace(
            sec=int(stamp), nanosec=int((stamp - int(stamp)) * 1e9)
        ),
    )
    assert GimbalNode.fresh(fake, header, receipt) == expected
