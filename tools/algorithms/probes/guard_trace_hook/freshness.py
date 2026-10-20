# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Opt-in observation of the real Gate.status call, without an additional call."""

from dataclasses import asdict
import math
import time


def snapshot(gate, now, raw, forwarded):
    def age(received):
        return now-received if math.isfinite(received) else None

    checks = dict(
        camera_stamp_positive=gate.camera_stamp > 0,
        imu_stamp_positive=gate.imu_stamp > 0,
        camera_receipt_deadline=now-gate.camera_receipt <= gate.limits.receipt_timeout,
        imu_receipt_deadline=now-gate.imu_receipt <= gate.limits.receipt_timeout,
        camera_source_age=gate.clock-gate.camera_stamp <= gate.limits.max_source_age*1e9,
        imu_source_age=gate.clock-gate.imu_stamp <= gate.limits.max_source_age*1e9)
    return dict(
        now_s=now, clock_ns=gate.clock, imu_ns=gate.imu_stamp, camera_ns=gate.camera_stamp,
        info_ns=gate.infos[-1] if gate.infos else 0,
        camera_receipt_age_s=age(gate.camera_receipt), imu_receipt_age_s=age(gate.imu_receipt),
        camera_source_age_s=(gate.clock-gate.camera_stamp)/1e9,
        imu_source_age_s=(gate.clock-gate.imu_stamp)/1e9,
        camera_forwarded=gate.camera_forwarded, imu_forwarded=gate.imu_forwarded,
        checks=checks, failed_checks=[key for key, value in checks.items() if not value],
        limits=asdict(gate.limits),
        raw_callbacks={key: dict(**value, age_s=now-value['begin_ns']/1e9)
                       for key, value in raw.items()},
        last_forward_completion={key: dict(**value, age_s=now-value['end_ns']/1e9)
                                 for key, value in forwarded.items()},
        pending_before=[dict(stamp_ns=p[0], has_payload=p[1] is not None,
                             has_info=p[0] in gate.infos, imu_reaches=gate.imu_stamp >= p[0],
                             sealed=p[0] < gate.camera_seen, receipt_age_s=now-p[2])
                        for p in gate.pending])


def wrap_status(original, append, raw, forwarded):
    def status(self, now):
        begin = time.monotonic_ns()
        before = snapshot(self, now, raw, forwarded)
        result = original(self, now)
        append(dict(kind='freshness_status', stamp_ns=self.camera_stamp,
                    begin_ns=begin, end_ns=time.monotonic_ns(),
                    state=result[0], reason=result[1], failure=self.failure,
                    imu_ns=self.imu_stamp, clock_ns=self.clock, pending=len(self.pending),
                    freshness=before))
        return result
    return status


def install(guard, append):
    raw, forwarded = {}, {}
    guard.Gate.status = wrap_status(guard.Gate.status, append, raw, forwarded)

    def receive(name, key):
        original = getattr(guard.OpenVinsGuard, name)

        def callback(self, message):
            begin = time.monotonic_ns()
            raw[key] = dict(stamp_ns=guard.stamp_ns(message), begin_ns=begin)
            count = self.gate.imu_forwarded
            result = original(self, message)
            if key == 'imu' and self.gate.imu_forwarded > count and self.gate.failure is None:
                # The successful callback already published the original IMU.
                forwarded['imu_callback'] = dict(
                    stamp_ns=self.gate.imu_stamp, end_ns=time.monotonic_ns())
            return result
        setattr(guard.OpenVinsGuard, name, callback)

    for name, key in [('on_imu', 'imu'), ('on_info', 'info'), ('on_camera', 'camera')]:
        receive(name, key)
    original_forward = guard.OpenVinsGuard.forward

    def forward(self, images):
        enabled = self.gate.failure is None
        result = original_forward(self, images)
        if enabled and images:
            forwarded['camera_publish_batch'] = dict(
                stamp_ns=guard.stamp_ns(images[-1]), end_ns=time.monotonic_ns())
        return result
    guard.OpenVinsGuard.forward = forward
