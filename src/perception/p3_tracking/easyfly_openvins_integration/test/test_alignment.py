# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Independent JPL perturbations, reference-time pairing and session failure cases."""

import math

import numpy as np
import pytest

from easyfly_openvins_integration.alignment import Alignment, Pose
from easyfly_openvins_integration.gate import Gate
from easyfly_openvins_integration.geometry import (
    angle_between, imu_to_base, interpolate, multiply, rotation, yaw_quaternion)


T = np.array([.011, .02329, -.12412])
QI = [0., 0., 0., 1.]


def test_lever_arm_rotates_and_is_not_added_in_world_axes():
    p, q, _ = imu_to_base([1, 2, 3], yaw_quaternion(math.pi/2), np.eye(6), T, QI)
    np.testing.assert_allclose(p, [1-T[1], 2+T[0], 3+T[2]], atol=1e-12)
    assert angle_between(q, yaw_quaternion(math.pi/2)) < 1e-7


def test_upstream_jpl_covariance_matches_independent_numerical_perturbations():
    q = np.array([.2, -.3, .1, math.sqrt(.86)])
    p = np.array([.7, .2, 1.])
    rng = np.random.default_rng(4)
    a = rng.normal(size=(6, 6))
    cov = a@a.T*.001
    bp, bq, transformed = imu_to_base(p, q, cov, T, QI)
    jacobian = np.zeros((6, 6))
    step = 1e-6
    for i in range(6):
        dx = np.zeros(6)
        dx[i] = step
        # JPLQuat::update / quat_multiply: left JPL multiplication. Its vector
        # cross-product has the opposite sign from Hamilton multiplication.
        d = dx[3:]/2
        qjpl = np.r_[q[3]*d + q[:3] - np.cross(d, q[:3]), q[3]-d@q[:3]]
        qjpl /= np.linalg.norm(qjpl)
        pos = p+dx[:3]+rotation(qjpl)@T
        dr = rotation(qjpl)@rotation(bq).T
        theta = np.array([dr[2, 1]-dr[1, 2], dr[0, 2]-dr[2, 0],
                          dr[1, 0]-dr[0, 1]])/2
        jacobian[:, i] = np.r_[pos-bp, theta]/step
    np.testing.assert_allclose(transformed, jacobian@cov@jacobian.T, rtol=2e-5, atol=1e-8)
    assert np.linalg.eigvalsh(transformed).min() >= -1e-10
    # Even a zero lever arm needs orientation covariance changed from I to G axes.
    _, _, no_arm = imu_to_base(p, q, cov, [0, 0, 0], QI)
    np.testing.assert_allclose(no_arm[3:, 3:], rotation(q)@cov[3:, 3:]@rotation(q).T)


def test_interpolation_handles_quaternion_sign_and_refuses_extrapolation():
    p, q = interpolate([0, 0, 0], QI, [2, 0, 0], -yaw_quaternion(math.pi/2), .5)
    np.testing.assert_allclose(p, [1, 0, 0])
    assert angle_between(q, yaw_quaternion(math.pi/4)) < 1e-7
    with pytest.raises(ValueError):
        interpolate([0, 0, 0], QI, [1, 0, 0], QI, 1.01)


def engine():
    return Alignment(T, QI)


def feed(a, t, wall, ref_shift=None):
    stamp = round(t*1e9)
    a.update_clock(stamp+20_000_000)
    a.health(True, stamp, wall)
    q = yaw_quaternion(.1*t)
    p = np.array([.3*t, .2*t, 1.])
    bp, bq, _ = imu_to_base(p, q, np.eye(6)*.01, T, QI)
    aq, at = yaw_quaternion(.5), np.array([1., -2., .3])
    if ref_shift is not None:
        at += ref_shift
    for offset in (-10_000_000, 10_000_000):
        # Equal physical endpoints isolate the time-pairing logic in this fixture.
        a.reference(Pose(stamp+offset, at+rotation(aq)@bp, multiply(aq, bq), wall))
    a.pose(Pose(stamp, p, q, wall, np.eye(6)*.01))
    a.confirm('session-a', 0, Gate.ESTIMATING, stamp, stamp+20_000_000, wall)
    return a.drain(wall)


def ready():
    a = engine()
    output = []
    for i in range(6):
        output = feed(a, 1+i*.08, 10+i*.08)
    assert a.status(10.4)[0] == Gate.ESTIMATING
    return a, output


def test_alignment_freezes_yaw_translation_and_preserves_measurement_stamp():
    a, output = ready()
    base, aligned = output[0]
    assert base.stamp == aligned.stamp == 1_400_000_000
    assert base.covariance is not None and aligned.covariance is None
    np.testing.assert_allclose(a.transform[0], [1., -2., .3], atol=1e-12)
    assert angle_between(a.transform[1], yaw_quaternion(.5)) < 1e-7
    frozen = a.transform[0].copy()
    feed(a, 1.48, 10.48, np.array([.1, 0, 0]))
    np.testing.assert_array_equal(a.transform[0], frozen)


def test_pairing_waits_for_both_source_time_endpoints():
    a = engine()
    a.update_clock(1_100_000_000)
    a.reference(Pose(1_000_000_000, [0, 0, 0], QI, 10.))
    a.health(True, 1_100_000_000, 10.)
    a.pose(Pose(1_050_000_000, [0, 0, 0], QI, 10., np.eye(6)))
    a.confirm('session-a', 0, Gate.ESTIMATING, 1_050_000_000, 1_100_000_000, 10.)
    assert not a.drain(10.)
    assert a.paired_reference(1_050_000_000, 10.) is None
    a.reference(Pose(1_100_000_000, [2, 0, 0], QI, 10.))
    np.testing.assert_allclose(a.paired_reference(1_050_000_000, 10.)[0], [1, 0, 0])
    assert a.drain(10.)


@pytest.mark.parametrize('fault', ['uuid', 'generation', 'clock', 'reference', 'jump'])
def test_faults_latch_and_never_realign_on_a_new_sample(fault):
    a, _ = ready()
    if fault == 'uuid':
        a.confirm('session-b', 0, Gate.ESTIMATING, 1_400_000_000, 1_420_000_000, 10.4)
    elif fault == 'generation':
        a.confirm('session-a', 1, Gate.ESTIMATING, 1_400_000_000, 1_420_000_000, 10.4)
    elif fault == 'clock':
        a.update_clock(1_000_000_000)
    elif fault == 'reference':
        a.reference(Pose(1_300_000_000, [0, 0, 0], QI, 10.4))
    else:
        assert not feed(a, 1.48, 10.48, np.array([5., 0, 0]))
    assert a.status(10.5)[0] == Gate.RESET_REQUIRED
    assert not feed(a, 1.56, 10.56)
    assert a.status(10.56)[0] == Gate.RESET_REQUIRED


def test_pause_reference_loss_health_failure_and_old_confirmations_do_not_admit():
    a, _ = ready()
    assert a.status(11.)[0] == Gate.STALE  # ROS clock did not move.
    a.health(False, 1_420_000_000, 10.41)
    assert a.status(10.41)[0] == Gate.STALE
    a.health(True, 1_420_000_000, 10.42)
    a.update_clock(2_100_000_000)
    assert a.status(10.42)[0] == Gate.STALE


def test_conflicting_reference_duplicate_is_not_a_new_measurement():
    a, _ = ready()
    previous = a.references[-1]
    a.reference(Pose(previous.stamp, previous.position, -previous.orientation, 99.))
    assert a.references[-1].receipt == previous.receipt
    a.reference(Pose(previous.stamp, previous.position+[.1, 0, 0], QI, 10.4))
    assert a.status(10.4)[0] == Gate.RESET_REQUIRED


def test_queues_and_invalid_covariance_are_bounded():
    a = engine()
    for i in range(17):
        a.pose(Pose(1_000_000_000+i, [0, 0, 0], QI, 10., np.eye(6)))
    assert a.failure == Gate.RESET_REQUIRED and not a.pending
    with pytest.raises(ValueError):
        Pose(1, [0, 0, 0], QI, 0., -np.eye(6))
    with pytest.raises(ValueError):
        Pose(1, [0, 0, 0], [0, 0, 0, 0], 0., np.eye(6))


def test_reference_position_uses_interpolated_source_attitude_not_cached_pose_attitude():
    from easyfly_openvins_integration.reference import ReferencePairer
    a = engine()
    a.update_clock(1_100_000_000)
    pairer = ReferencePairer(a)
    pairer.add_position(Pose(1_050_000_000, [1., 2., 3.], QI, 10.))
    pairer.add_attitude(Pose(1_000_000_000, [0., 0., 0.], QI, 10.))
    pairer.drain(10.)
    assert not a.references
    pairer.add_attitude(Pose(1_100_000_000, [0., 0., 0.], yaw_quaternion(math.pi/2), 10.))
    pairer.drain(10.)
    assert a.references[-1].stamp == 1_050_000_000
    assert angle_between(a.references[-1].orientation, yaw_quaternion(math.pi/4)) < 1e-7


def test_reference_attitude_conflict_latches_and_never_fills_a_missing_interval():
    from easyfly_openvins_integration.reference import ReferencePairer
    a = engine()
    pairer = ReferencePairer(a)
    pairer.add_attitude(Pose(1_000_000_000, [0., 0., 0.], QI, 10.))
    pairer.add_attitude(Pose(1_000_000_000, [0., 0., 0.], QI, 20.))
    assert pairer.attitudes[-1].receipt == 10.
    pairer.add_attitude(Pose(1_000_000_000, [0., 0., 0.], yaw_quaternion(.2), 10.))
    assert a.failure == Gate.RESET_REQUIRED
    pairer.drain(10.)
    assert not a.references


def test_reference_attitude_gap_and_position_queues_are_bounded():
    from easyfly_openvins_integration.reference import ReferencePairer
    a = engine()
    pairer = ReferencePairer(a)
    pairer.add_attitude(Pose(1_000_000_000, [0., 0., 0.], QI, 10.))
    pairer.add_attitude(Pose(1_200_000_000, [0., 0., 0.], QI, 10.))
    assert a.failure == Gate.RESET_REQUIRED
    a = engine()
    pairer = ReferencePairer(a)
    for i in range(17):
        pairer.add_position(Pose(1_000_000_000+i, [0., 0., 0.], QI, 10.))
    assert a.failure == Gate.RESET_REQUIRED and not pairer.positions
