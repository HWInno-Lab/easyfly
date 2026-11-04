# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_perception_srvs/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project
#

"""Real-process signal regression; a forced kill or nonzero exit is a failure."""
import os
import signal
import subprocess
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_prefix
from builtin_interfaces.msg import Time
import pytest
import rclpy
from rosgraph_msgs.msg import Clock
from easyfly_perception_srvs import node as service_module


class ShutdownTest(unittest.TestCase):
    def test_signal_exit(self):
        os.environ['ROS_DOMAIN_ID'] = '194'
        rclpy.init()
        node = rclpy.create_node('shutdown_regression')
        clocks = node.create_publisher(Clock, '/clock', 10)
        executable = os.path.join(
            get_package_prefix('easyfly_perception_srvs'),
            'lib',
            'easyfly_perception_srvs',
            'perception_services',
        )
        try:
            for mode in ('wall', 'paused', 'active'):
                for sig in (signal.SIGINT, signal.SIGTERM):
                    with self.subTest(mode=mode, signal=sig.name):
                        name = f'shutdown_{mode}_{sig.name.lower()}'
                        with tempfile.TemporaryFile(mode='w+t') as output:
                            process = subprocess.Popen(
                                [
                                    executable,
                                    '--ros-args',
                                    '-r',
                                    f'__node:={name}',
                                    '-p',
                                    f'use_sim_time:={str(mode != "wall").lower()}',
                                ],
                                stdout=output,
                                stderr=output,
                            )
                            try:
                                ready = False
                                deadline = time.monotonic() + 8.0
                                tick = 0
                                while time.monotonic() < deadline:
                                    if mode != 'wall':
                                        clocks.publish(
                                            Clock(
                                                clock=Time(
                                                    sec=10,
                                                    nanosec=tick * 1000000,
                                                )
                                            )
                                        )
                                        if mode == 'active':
                                            tick = (tick + 1) % 1000
                                    rclpy.spin_once(node, timeout_sec=0.02)
                                    output.seek(0)
                                    logs = output.read()
                                    self.assertIsNone(process.poll(), logs)
                                    ready = name in node.get_node_names() and any(
                                        service
                                        == '/platform/perception/capabilities'
                                        for service, _ in node.get_service_names_and_types_by_node(
                                            name, '/'
                                        )
                                    )
                                    if ready:
                                        break
                                self.assertTrue(
                                    ready, 'node did not become ready: ' + logs
                                )
                                # Allow time-source discovery, including a positive
                                # frozen clock rather than only uninitialized zero.
                                if mode != 'wall':
                                    deadline = time.monotonic() + 2.0
                                    while clocks.get_subscription_count() == 0:
                                        self.assertLess(
                                            time.monotonic(), deadline
                                        )
                                        rclpy.spin_once(node, timeout_sec=0.02)
                                    for _ in range(5):
                                        clocks.publish(
                                            Clock(
                                                clock=Time(
                                                    sec=10,
                                                    nanosec=(
                                                        tick * 1000000
                                                        if mode == 'active'
                                                        else 0
                                                    ),
                                                )
                                            )
                                        )
                                        if mode == 'active':
                                            tick += 1
                                        rclpy.spin_once(node, timeout_sec=0.02)
                                process.send_signal(sig)
                                try:
                                    code = process.wait(timeout=3.0)
                                except subprocess.TimeoutExpired:
                                    self.fail(
                                        'signal shutdown required forced termination'
                                    )
                                output.seek(0)
                                self.assertEqual(code, 0, output.read())
                            finally:
                                if process.poll() is None:
                                    process.kill()
                                    process.wait(timeout=3.0)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


@pytest.mark.parametrize('context_ok', [True, False])
def test_shutdown_race_does_not_hide_live_context_errors(
    monkeypatch, context_ok
):
    destroyed, shutdown = [], []

    class FakeNode:
        def destroy_node(self):
            destroyed.append(True)

    def fail_spin(_node):
        raise service_module.rclpy_implementation.RCLError(
            'test wait-set error'
        )

    monkeypatch.setattr(service_module, 'PerceptionServices', FakeNode)
    monkeypatch.setattr(service_module.rclpy, 'init', lambda: None)
    monkeypatch.setattr(service_module.rclpy, 'spin', fail_spin)
    monkeypatch.setattr(service_module.rclpy, 'ok', lambda: context_ok)
    monkeypatch.setattr(
        service_module.rclpy, 'shutdown', lambda: shutdown.append(True)
    )
    if context_ok:
        with pytest.raises(service_module.rclpy_implementation.RCLError):
            service_module.main()
    else:
        service_module.main()
    assert destroyed == [True]
    assert shutdown == ([True] if context_ok else [])


if __name__ == '__main__':
    unittest.main()
