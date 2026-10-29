# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Fault/session/measurement tests, not substitute algorithm execution evidence."""
import numpy as np

from easyfly_openvins_integration.alignment import Pose
from easyfly_openvins_integration.gate import Gate
from easyfly_openvins_integration.navigation import Navigation


def sample(seconds, receipt=1., x=0.):
    return Pose(round(seconds*1e9), [x, 0., 1.], [0., 0., 0., 1.], receipt)


def confirm(gate, seconds=10., now=1., session='one', state=Gate.ESTIMATING, generation=0):
    stamp = round(seconds*1e9)
    gate.update_clock(stamp)
    gate.health(stamp, now)
    gate.confirm(session, generation, state, stamp, stamp, now)


def test_requires_exact_status_confirmation_and_preserves_pose_source_stamp():
    gate = Navigation()
    gate.pose(sample(10.))
    assert gate.drain(1., True) == []
    confirm(gate, 9.95)
    assert gate.drain(1., True) == []
    confirm(gate)
    result = gate.drain(1., True)
    assert len(result) == 1 and result[0][0].stamp == 10_000_000_000
    assert result[0][1] is None
    assert gate.epoch == gate.last_output == 10_000_000_000
    assert gate.status(1., True)[0] == Gate.ESTIMATING


def test_velocity_comes_from_measured_positions_and_bounded_source_interval():
    gate = Navigation()
    for index in range(5):
        seconds, now = 10+index*.05, 1+index*.05
        gate.pose(sample(seconds, now, x=index*.025))
        confirm(gate, seconds, now)
        pose, velocity = gate.drain(now, True)[0]
    np.testing.assert_allclose(velocity, [.5, 0., 0.], atol=1e-12)
    assert pose.stamp == 10_200_000_000


def test_stopped_clock_or_repeated_messages_do_not_renew_lifetime():
    gate = Navigation()
    gate.pose(sample(10.))
    confirm(gate)
    assert gate.drain(1., True)
    confirm(gate, now=1.6)
    assert gate.status(1.6, True)[0] == Gate.STALE
    assert gate.drain(1.6, True) == []
    assert gate.status(1.1, False)[0] == Gate.STALE


def test_newer_unconfirmed_history_does_not_keep_old_output_fresh():
    gate = Navigation()
    gate.pose(sample(10.))
    confirm(gate)
    gate.drain(1., True)
    gate.pose(sample(10.1, 1.6))
    confirm(gate, 10.15, 1.6)
    assert gate.drain(1.6, True) == []
    assert gate.status(1.6, True)[0] == Gate.STALE


def test_session_generation_and_time_faults_latch_without_recovery():
    for fault in ['session', 'generation', 'clock', 'duplicate', 'jump']:
        gate = Navigation()
        gate.pose(sample(10.))
        confirm(gate)
        gate.drain(1., True)
        if fault == 'session':
            confirm(gate, session='two')
        elif fault == 'generation':
            confirm(gate, generation=1)
        elif fault == 'clock':
            gate.update_clock(9_000_000_000)
        elif fault == 'duplicate':
            gate.pose(sample(10.))
        else:
            gate.pose(sample(10.05, x=5.))
        assert gate.failure
        confirm(gate, 11., 2.)
        assert gate.drain(2., True) == []
        assert gate.status(2., True)[0] == Gate.RESET_REQUIRED
