#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Actual Gazebo measurements through a deliberately blocked native ROS publisher."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('server', type=Path)
p.add_argument('plugin', type=Path)
p.add_argument('interposer', type=Path)
p.add_argument('--overflow', action='store_true')
p.add_argument('--expect-old-block', action='store_true')
p.add_argument('--output', type=Path)
a = p.parse_args()
assert not (a.overflow and a.expect_old_block)
temporary = tempfile.TemporaryDirectory(prefix='easyfly-imu-publication-') if not a.output else None
out = Path(temporary.name) if temporary else a.output
if not temporary:
    out.mkdir(parents=True, exist_ok=False)
for path in [a.server, a.plugin, a.interposer]:
    path.resolve(strict=True)
with socket.socket() as reservation:
    reservation.bind(('127.0.0.1', 0))
    port = reservation.getsockname()[1]
world = out/'fixture.world'
world.write_text(f'''<sdf version="1.6"><world name="imu_publish_test">
<physics name="default" type="ode"><max_step_size>0.001</max_step_size>
<real_time_update_rate>1000</real_time_update_rate></physics><gravity>0 0 -9.81</gravity>
<model name="fixture"><static>true</static><link name="base_link">
<sensor name="imu" type="imu"><always_on>true</always_on><update_rate>200</update_rate>
<plugin name="imu_plugin" filename="{a.plugin.resolve()}"><ros><namespace>/livox</namespace>
<remapping>~/out:=imu</remapping></ros><frame_name>livox_imu_frame</frame_name>
<initial_orientation_as_reference>false</initial_orientation_as_reference></plugin>
</sensor></link></model></world></sdf>''')
os.environ.update(ROS_DOMAIN_ID='211', ROS_LOCALHOST_ONLY='1')
env = dict(os.environ, GAZEBO_MASTER_URI=f'http://127.0.0.1:{port}',
           GAZEBO_MODEL_DATABASE_URI='', ROS_LOG_DIR=str(out/'ros'),
           EASYFLY_IMU_TIMING_TRACE=str(out/'producer'),
           LD_PRELOAD=str(a.interposer.resolve()),
           EASYFLY_TEST_IMU_STALL_PREFIX=str(out/'stall'),
           EASYFLY_TEST_IMU_STALL_MS='7000' if a.overflow else '300')
report = dict(passed=False, scope=__doc__, overflow=a.overflow,
              expected_old_block=a.expect_old_block, error=None, forced=False,
              inputs={str(x.resolve()):hashlib.sha256(x.read_bytes()).hexdigest()
                      for x in [a.server, a.plugin, a.interposer]})
samples = []
rclpy.init()
node = rclpy.create_node('imu_publish_queue_observer')
qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE)
node.create_subscription(Imu, '/livox/imu', lambda msg:samples.append(dict(
    stamp_ns=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec,
    receipt=time.monotonic(), ax=msg.linear_acceleration.x, az=msg.linear_acceleration.z)), qos)
process = None
log = (out/'server.log').open('x')
try:
    process = subprocess.Popen([str(a.server.resolve()), '-s', 'libgazebo_ros_init.so', str(world)],
                               env=env, stdout=log, stderr=subprocess.STDOUT,
                               start_new_session=True)
    report['pid'] = process.pid
    report['start_ticks'] = Path(f'/proc/{process.pid}/stat').read_text().rsplit(')', 1)[1].split()[19]
    end = time.monotonic()+25
    settled = None
    while time.monotonic() < end:
        assert process.poll() is None, 'owned server exited early'
        rclpy.spin_once(node, timeout_sec=.05)
        if (out/'stall.end').exists():
            if settled is None:
                settled = time.monotonic()+1.0
            if time.monotonic() >= settled:
                break
    else:
        raise TimeoutError('real publisher stall was not completed')
    process.send_signal(signal.SIGINT)
    end = time.monotonic()+20
    while process.poll() is None and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.05)
    assert process.poll() == 0, 'server did not close normally'
    trace, = out.glob('producer.*.csv')
    with trace.open() as stream:
        metadata = dict(x.split('=', 1) for x in stream.readline().strip().split() if '=' in x)
        raw = list(csv.DictReader(stream))
    published = [x for x in raw if x['result'] == '2']
    gaps = [(int(b['source_ns'])-int(x['source_ns']))/1e9 for x,b in zip(published,published[1:])]
    before = list(map(int, (out/'stall.begin').read_text().split()))
    after = list(map(int, (out/'stall.end').read_text().split()))
    assert (after[1]-before[1])/1e9 >= (6.9 if a.overflow else .29)
    assert int(metadata['discarded']) == 0 and len(published) >= 200 and len(samples) > 100
    failures = int(metadata.get('publish_failures', 0))
    if a.overflow:
        assert failures == 1 and sum(x['result'] == '3' for x in raw) == 1
        assert max(int(x['source_ns']) for x in published) == before[0], 'failed queue emitted newer stale measurements'
        assert 'output latched stopped' in (out/'server.log').read_text()
    else:
        assert failures == 0 and all(x > 0 for x in gaps)
        if a.expect_old_block:
            assert max(gaps) > .025, 'old sensor callback blocking did not reproduce'
        else:
            assert max(gaps) <= .025, f'captured source gap {max(gaps)} exceeded original limit'
            retained = [x for x in published if before[1] <= int(x['entry_ns']) <= after[1]]
            assert len(retained) >= 20, 'source sampling did not continue during blocked publish'
        assert all(b['stamp_ns'] > x['stamp_ns'] for x,b in zip(samples,samples[1:]))
    report.update(passed=True, metadata=metadata, published=len(published), observed=len(samples),
                  maximum_source_gap_s=max(gaps), actual_stall_s=(after[1]-before[1])/1e9,
                  returncode=process.returncode)
except BaseException as error:
    report['error'] = repr(error)
finally:
    if process is not None and process.poll() is None:
        report['forced'] = True
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5)
    log.close()
    node.destroy_node()
    rclpy.shutdown()
    (out/'observations.json').write_text(json.dumps(samples)+'\n')
    (out/'result.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report), flush=True)
raise SystemExit(0 if report['passed'] and not report['forced'] else 1)
