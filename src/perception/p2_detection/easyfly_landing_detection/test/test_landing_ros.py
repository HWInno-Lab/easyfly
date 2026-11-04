# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_landing_detection/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project
#

import csv
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from easyfly_msgs.msg import TagObservationArray, TagObservation, LandingTarget


class LandingRos(unittest.TestCase):
    publication_reliability = ReliabilityPolicy.RELIABLE

    def test_explicit_selection_loss_bad_pose_and_expiration(self):
        rclpy.init()
        node = rclpy.create_node('landing_test')
        outputs = []
        self.addCleanup(rclpy.shutdown)
        self.addCleanup(node.destroy_node)
        pub = node.create_publisher(
            TagObservationArray, '/tag_observations',
            QoSProfile(depth=10, reliability=self.publication_reliability)
        )
        node.create_subscription(
            LandingTarget, '/landing_target', outputs.append, 10
        )
        log = tempfile.TemporaryFile(mode='w+')
        self.addCleanup(log.close)
        evidence = tempfile.TemporaryDirectory(prefix='easyfly-landing-ingress-')
        self.addCleanup(evidence.cleanup)
        trace = Path(evidence.name) / 'ingress.tsv'
        process = subprocess.Popen(
            [
                'ros2',
                'run',
                'easyfly_landing_detection',
                'landing_detection_node',
                '--ros-args',
                '-p',
                'selected_tag_id:=7',
                '-p',
                f'timing_trace_path:={trace}',
            ],
            stdout=log,
            stderr=log,
            start_new_session=True,
        )

        def stop():
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)

        self.addCleanup(stop)

        def spin(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.01)

        previous_stamp = [None]

        def send(kind):
            outputs.clear()
            m = TagObservationArray(valid=True)
            m.header.frame_id = 'camera_color_optical_frame'
            m.header.stamp = node.get_clock().now().to_msg()
            o = TagObservation(tag_id=7, tag_size_m=0.2, valid=True)
            o.header = m.header
            o.pose.position.z = 2.0
            o.pose.orientation.w = 1.0
            m.observations = [o]
            if kind == 'empty':
                m.observations = []
            elif kind == 'other':
                o.tag_id = 8
            elif kind == 'duplicate':
                m.observations = [o, o]
            elif kind == 'quat':
                o.pose.orientation.w = 2.0
            elif kind == 'depth':
                o.pose.position.z = -1.0
            elif kind == 'invalid':
                m.valid = False
            elif kind == 'frame':
                m.header.frame_id = 'map'
            elif kind == 'stale':
                m.header.stamp.sec -= 1
            elif kind == 'replay':
                m.header.stamp = previous_stamp[0]
                o.header = m.header
            previous_stamp[0] = m.header.stamp
            pub.publish(m)
            spin(0.12)
            self.assertTrue(outputs)
            return outputs[-1]

        try:
            discovery_end = time.monotonic() + 3.0
            matched = []
            while not matched and time.monotonic() < discovery_end:
                spin(0.05)
                matched = [info for info in node.get_subscriptions_info_by_topic(
                    '/tag_observations') if info.node_name == 'landing_detection']
            self.assertEqual(len(matched), 1)
            # RTPS discovery does not expose history/depth in this RMW.
            self.assertEqual(matched[0].qos_profile.reliability,
                             ReliabilityPolicy.BEST_EFFORT)
            output_info = node.get_publishers_info_by_topic('/landing_target')
            self.assertEqual(len(output_info), 1)
            self.assertEqual(output_info[0].qos_profile.reliability,
                             ReliabilityPolicy.RELIABLE)
            good = send('good')
            self.assertTrue(good.valid)
            self.assertEqual(good.optical_depth_m, 2.0)
            self.assertEqual(good.yaw_rad, 0.0)
            self.assertFalse(send('replay').valid)
            for kind in (
                'empty',
                'other',
                'duplicate',
                'quat',
                'depth',
                'invalid',
                'frame',
                'stale',
            ):
                self.assertFalse(send(kind).valid, kind)
            self.assertTrue(send('good').valid)
            spin(0.65)
            self.assertFalse(outputs[-1].valid)
            self.assertEqual(trace.stat().st_size, 0, 'no callback-time trace I/O')
        finally:
            stop()
            log.close()
        with trace.open() as stream:
            metadata = stream.readline()
            self.assertIn('dropped=0', metadata)
            events = list(csv.DictReader(stream, delimiter='\t'))
        self.assertTrue(events)
        self.assertTrue(any(e['input'] == '0' for e in events))
        good = [e for e in events if e['valid'] == '1']
        self.assertTrue(good)
        for event in good:
            self.assertEqual(event['input'], '1')
            self.assertGreater(int(event['writer_wall_ns']), 0)
            self.assertGreater(int(event['receive_wall_ns']), 0)
            self.assertGreaterEqual(int(event['publish_end_wall_ns']),
                                    int(event['publish_begin_wall_ns']))
            self.assertGreaterEqual(int(event['publish_begin_wall_ns']),
                                    int(event['entry_wall_ns']))


class BestEffortLandingRos(LandingRos):
    publication_reliability = ReliabilityPolicy.BEST_EFFORT


if __name__ == '__main__':
    unittest.main()
