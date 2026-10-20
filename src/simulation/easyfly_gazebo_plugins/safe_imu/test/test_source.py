# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Source and ownership evidence for the independently named IMU derivative."""
import hashlib
from pathlib import Path


BASE = Path(__file__).resolve().parents[1]


def test_original_files_and_notices_are_pinned():
    for line in (BASE / 'upstream/SHA256SUMS').read_text().splitlines():
        sha, name = line.split()
        assert hashlib.sha256((BASE / 'upstream' / name).read_bytes()).hexdigest() == sha
    assert 'Apache-2.0' in (BASE / 'NOTICE').read_text()
    assert '3.9.0' in (BASE / 'NOTICE').read_text()


def test_producer_uses_one_measurement_snapshot_without_retimestamps():
    source = (BASE / 'src/safe_imu.cpp').read_text()
    assert 'sensor_->LastUpdateTime()' not in source
    assert 'const auto snapshot = sensor_->ImuMessage();' in source
    assert 'msg_->header.stamp.sec = snapshot.stamp().sec();' in source
    assert 'msg_->header.stamp.nanosec = snapshot.stamp().nsec();' in source
    assert 'sensor_->AngularVelocity()' not in source
    assert 'GZ_REGISTER_SENSOR_PLUGIN(EasyFlyGazeboRosImuSensor)' in source
    assert 'impl_->sensor_update_event_.reset();' in source
