# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
from easyfly_fast_livo2_integration.guard import Admission


def ready():
    e = Admission()
    e.update_clock(10 ** 9)
    for k in ("camera", "imu", "lidar"):
        assert e.accept(k, 10 ** 9, 1.0)
    return e


def test_three_sensors_required():
    e = Admission()
    e.update_clock(10 ** 9)
    assert e.accept("camera", 10 ** 9, 1.0)
    assert not e.ready(1.0)
    assert ready().ready(1.0)


def test_independent_wall_and_source_expiration():
    e = ready()
    assert not e.ready(1.501)
    e = ready()
    e.update_clock(1500000001)
    assert not e.ready(1.0)


def test_duplicate_latches():
    e = ready()
    assert not e.accept("camera", 10 ** 9, 1.0)
    assert e.failure == "camera_rollback_or_duplicate"
    assert not e.accept("imu", 1000000001, 1.0)


def test_clock_rollback_latches():
    e = ready()
    e.update_clock(999999999)
    e.update_clock(1000000001)
    assert e.failure == "clock_rollback"
    assert not e.ready(1.0)


def test_future_and_stale_inputs_are_rejected():
    e = Admission()
    e.update_clock(10 ** 9)
    assert not e.accept("imu", 1300000000, 1.0)
    assert not e.accept("lidar", 499999999, 1.0)
    assert e.forwarded["imu"] == 0 and e.rejected["imu"] == 1


def test_new_sensor_does_not_refresh_another():
    e = ready()
    e.update_clock(1400000000)
    assert e.accept("camera", 1400000000, 1.4)
    e.update_clock(1600000000)
    assert not e.ready(1.6)


def test_freshness_boundaries():
    e = ready()
    assert e.fresh(500000000, 1.0, 1.5)
    assert not e.fresh(499999999, 1.0, 1.5)
    assert not e.fresh(1000000000, 1.0, 0.9)


def test_first_failure_is_preserved():
    e = ready()
    e.fail("first")
    e.fail("second")
    assert e.failure == "first"


def fixture_receiver(monkeypatch):
    from collections import OrderedDict
    from types import SimpleNamespace
    from easyfly_fast_livo2_integration import guard

    e = ready()
    messages = []
    pubs = {
        k: SimpleNamespace(publish=lambda m: messages.append(m))
        for k in ("camera", "imu", "lidar")
    }
    receiver = SimpleNamespace(
        engine=e,
        calibration=True,
        clock_update=lambda: None,
        pubs=pubs,
        camera_stamps=OrderedDict(),
    )
    monkeypatch.setattr(guard.time, "monotonic", lambda: 1.0)
    return receiver, messages


def test_live_sensor_is_not_buffered_behind_expired_required_sensor(monkeypatch):
    from easyfly_fast_livo2_integration.guard import FastLivoGuard
    from sensor_msgs.msg import Imu

    receiver, messages = fixture_receiver(monkeypatch)
    m = Imu()
    m.header.frame_id = "livox_imu_frame"
    for i in range(1, 101):
        stamp = 1_000_000_000 + i * 5_000_000
        receiver.engine.update_clock(stamp)
        m.header.stamp.sec, m.header.stamp.nanosec = divmod(stamp, 10 ** 9)
        FastLivoGuard.sensor(receiver, "imu", m)
    before = len(messages)
    for i in range(101, 1101):
        stamp = 1_000_000_000 + i * 5_000_000
        receiver.engine.update_clock(stamp)
        m.header.stamp.sec, m.header.stamp.nanosec = divmod(stamp, 10 ** 9)
        FastLivoGuard.sensor(receiver, "imu", m)
    assert len(messages) == before and before > 0
    assert not receiver.engine.failure


def test_imu_source_gap_latches_before_forwarding():
    e = ready()
    e.update_clock(1_025_000_000)
    assert not e.accept("imu", 1_025_000_000, 1.0)
    assert e.failure == "imu_source_gap" and e.rejected["imu"] == 1
    assert not e.accept("camera", 1_025_000_000, 1.0)


def test_forwarded_observation_is_original_object(monkeypatch):
    from easyfly_fast_livo2_integration.guard import FastLivoGuard
    from sensor_msgs.msg import Imu

    receiver, messages = fixture_receiver(monkeypatch)
    m = Imu()
    m.header.frame_id = "livox_imu_frame"
    m.header.stamp.sec = 1
    m.header.stamp.nanosec = 1
    m.linear_acceleration.z = 9.81
    FastLivoGuard.sensor(receiver, "imu", m)
    assert len(messages) == 1 and messages[0] is m
    assert m.header.stamp.nanosec == 1 and m.linear_acceleration.z == 9.81


def test_malformed_lidar_latches_before_forwarding(monkeypatch):
    from easyfly_fast_livo2_integration.guard import FastLivoGuard
    from livox_ros_driver2.msg import CustomMsg

    receiver, messages = fixture_receiver(monkeypatch)
    m = CustomMsg()
    m.header.frame_id = "livox_frame"
    m.point_num = 100
    FastLivoGuard.sensor(receiver, "lidar", m)
    assert receiver.engine.failure == "invalid_lidar" and not messages
