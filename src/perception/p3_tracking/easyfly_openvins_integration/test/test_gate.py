# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Temporal admission regressions; these are not algorithm accuracy tests."""

from easyfly_openvins_integration.gate import Gate


def ready_gate():
    gate = Gate()
    gate.clock_update(1_150_000_000)
    for i in range(31):
        accepted, _ = gate.imu(1_000_000_000 + i * 5_000_000, .0)
        assert accepted
    return gate


def frame(gate, stamp, payload=None):
    gate.info(stamp, .0)
    return gate.camera(stamp, payload if payload is not None else object(), .0)


def test_reject_entire_duplicate_group_and_preserve_payload_ownership():
    gate = ready_gate()
    a, b = object(), object()
    assert frame(gate, 1_020_000_000, a) == []
    assert frame(gate, 1_053_000_000) == [a]
    frame(gate, 1_053_000_000)
    frame(gate, 1_053_000_000)
    assert frame(gate, 1_087_000_000, b) == []
    assert frame(gate, 1_120_000_000) == [b]
    assert gate.camera_rejected == 3
    assert gate.camera_forwarded == 2
    assert len(gate.pending) == 1  # EOF never forces the last group through.
    assert gate.pose(1_087_000_000, .01)
    assert gate.status(.02)[0] == Gate.ESTIMATING
    assert gate.status(.8)[0] == Gate.STALE  # Wall timeout despite paused ROS time.


def test_late_duplicate_cannot_be_retracted_and_requires_whole_session_restart():
    gate = ready_gate()
    frame(gate, 1_020_000_000)
    frame(gate, 1_053_000_000)
    frame(gate, 1_020_000_000)
    assert gate.status(.0)[0] == Gate.RESET_REQUIRED
    assert gate.generation == 1 and not gate.pending
    gate.clock_update(2_000_000_000)
    assert not gate.imu(2_000_000_000, .1)[0]
    assert not gate.pose(1_020_000_000, .1)
    assert gate.generation == 1


def test_imu_discontinuity_clock_rollback_and_missing_calibration_are_bounded():
    gate = ready_gate()
    assert not gate.imu(1_150_000_000, .0)[0]
    assert gate.reason == 'imu_time_discontinuity'
    gate = ready_gate()
    gate.clock_update(1_149_999_999)
    assert gate.status(.0)[0] == Gate.RESET_REQUIRED
    gate = ready_gate()
    for stamp in [1_000_000_000, 1_030_000_000, 1_060_000_000, 1_090_000_000, 1_120_000_000]:
        gate.camera(stamp, object(), .0)  # No CameraInfo: never forward partial input.
    assert gate.reason == 'camera_queue_overflow'
    assert len(gate.pending) == 0 and gate.camera_forwarded == 0
    assert '1000000000, True, False' in gate.failure_detail
    assert 'imu_ns=1150000000' in gate.failure_detail


def test_future_measurement_and_output_rollback_never_become_ready():
    gate = ready_gate()
    assert not gate.imu(2_000_000_000, .0)[0]
    assert gate.reason == 'invalid_source_time'
    gate = ready_gate()
    frame(gate, 1_020_000_000)
    frame(gate, 1_053_000_000)
    frame(gate, 1_087_000_000)
    assert gate.pose(1_053_000_000, .0)
    assert not gate.pose(1_020_000_000, .0)
    assert gate.status(.0)[0] == Gate.RESET_REQUIRED


def test_clock_bootstrap_is_only_allowed_before_measurement_history():
    gate = Gate()
    gate.clock_update(1_000_000_000)
    gate.clock_update(20_000_000)
    assert gate.failure is None
    assert gate.imu(20_000_000, .0)[0]
    gate.clock_update(19_000_000)
    assert gate.status(.0)[0] == Gate.RESET_REQUIRED


def test_expired_pending_camera_does_not_count_unpublished_imu_as_forwarded():
    gate = ready_gate()
    gate.camera(1_020_000_000, object(), .0)
    gate.camera(1_053_000_000, object(), .0)
    gate.infos.append(1_020_000_000)
    accepted, images = gate.imu(1_155_000_000, .6)
    assert not accepted and not images
    assert gate.imu_forwarded == 31 and gate.imu_stamp == 1_150_000_000
    assert gate.camera_forwarded == 0 and gate.camera_rejected == 2
    assert gate.status(.6)[0] == Gate.RESET_REQUIRED


def test_missing_dependencies_expire_on_steady_status_without_new_input():
    gate = ready_gate()
    gate.camera(1_020_000_000, object(), .0)
    gate.camera(1_053_000_000, object(), .0)
    assert gate.status(.4)[0] == Gate.WAITING_INPUT
    assert gate.status(.6)[0] == Gate.RESET_REQUIRED
    assert gate.reason == 'pending_camera_expired'
    assert not gate.pending and gate.camera_forwarded == 0
