# Copyright 2026 EasyFly Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Real tracking + pixel_to_3d feedback, synthetic camera/depth/TF, no flight."""

import copy
from pathlib import Path
import signal
import struct
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from easyfly_msgs.msg import DetectionInfo, DetectionInfoArray, Target3DArray, TrackInfoArray
from geometry_msgs.msg import TransformStamped
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster
import yaml


def test_raw_world_stable_feedback_reset_and_rejected_frames(tmp_path):
    """Use real numeric localization, delayed ID inheritance and process-unique reset IDs."""
    tracking = Path(get_package_prefix('easyfly_multi_object_tracking'))
    pixel = Path(get_package_prefix('easyfly_pixel_to_3d'))
    executable = tracking / 'lib/easyfly_multi_object_tracking/tracking_node'
    rejected = subprocess.run(
        ['bash', '-c', 'ulimit -c 0; exec "$@"', 'bash', str(executable),
         '--ros-args', '-p', 'tracker_backend:=not_implemented'],
        capture_output=True, text=True, timeout=8,
    )
    assert rejected.returncode != 0
    assert 'unknown tracking backend' in rejected.stdout + rejected.stderr

    config = tmp_path / 'tracking.yaml'
    config.write_text(yaml.safe_dump({'tracking': {
        'tracker_backend': 'sort', 'publish_debug_image': False,
        'image_width': 64, 'image_height': 48,
        'iou_threshold': .15, 'max_lost_frames': 2, 'min_hits': 1, 'max_tracks': 100,
        'target_classes': ['person'],
        'id_consolidation': {'enabled': True, 'reacquire_radius': 3.0, 'ttl': 10.0},
    }}))
    rclpy.init()
    node = rclpy.create_node('tracking_feedback_fixture')
    raw, stable, positions = {}, {}, {}

    def key(header):
        return header.stamp.sec * 1000000000 + header.stamp.nanosec

    node.create_subscription(TrackInfoArray, '/tracks_raw',
                             lambda msg: raw.__setitem__(key(msg.header), msg), 10)
    node.create_subscription(TrackInfoArray, '/tracks',
                             lambda msg: stable.__setitem__(key(msg.header), msg), 10)
    node.create_subscription(Target3DArray, '/target_positions',
                             lambda msg: positions.__setitem__(key(msg.header), msg), 10)
    det_pub = node.create_publisher(DetectionInfoArray, '/detections', 10)
    depth_pub = node.create_publisher(Image, '/camera/aligned_depth_to_color/depth/image_raw', 10)
    info_pub = node.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
    late_world = node.create_publisher(Target3DArray, '/target_positions', 10)
    reset = node.create_client(Trigger, '/easyfly_multi_object_tracking/reset')
    broadcaster = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.frame_id = 'odom'
    transform.child_frame_id = 'camera_color_optical_frame'
    transform.transform.translation.x = 8.0
    transform.transform.translation.y = 2.0
    transform.transform.rotation.w = 1.0
    broadcaster.sendTransform(transform)
    logs, processes = [], []

    def spin(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.005)

    def until(predicate, timeout=8):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            assert all(p.poll() is None for p in processes), 'real node exited'
            rclpy.spin_once(node, timeout_sec=.01)
        assert predicate(), 'feedback fixture timed out'

    def send(present=True):
        stamp = node.get_clock().now().to_msg()
        info = CameraInfo(width=64, height=48)
        info.header.frame_id = 'camera_color_optical_frame'
        info.header.stamp = stamp
        info.k = [40., 0., 32., 0., 40., 24., 0., 0., 1.]
        info_pub.publish(info)
        depth = Image(width=64, height=48, encoding='32FC1', step=64 * 4,
                      data=struct.pack('<' + 'f' * (64 * 48), *([2.] * (64 * 48))))
        depth.header = copy.deepcopy(info.header)
        depth_pub.publish(depth)
        # Ensure this depth is available before the matching raw-track callback.
        spin(.04)
        detections = DetectionInfoArray()
        detections.header = copy.deepcopy(info.header)
        if present:
            detection = DetectionInfo(class_name='person', confidence=.9,
                                      bbox_center_x=.5, bbox_center_y=.5,
                                      bbox_width=.25, bbox_height=.5)
            detection.header = copy.deepcopy(info.header)
            detections.detections = [detection]
        det_pub.publish(detections)
        identity = key(detections.header)
        until(lambda: identity in raw and identity in stable and identity in positions)
        return identity, detections

    try:
        for package, command in (
            ('tracking', [str(executable), '--ros-args', '-p', f'config_file:={config}']),
            ('pixel', [str(pixel / 'lib/easyfly_pixel_to_3d/pixel_to_3d_node')]),
        ):
            log = (tmp_path / (package + '.log')).open('w')
            logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=log))
        until(lambda: det_pub.get_subscription_count() > 0 and
              depth_pub.get_subscription_count() > 0 and
              info_pub.get_subscription_count() > 0 and reset.service_is_ready())
        broadcaster.sendTransform(transform)
        spin(.2)
        first, _ = send()
        assert raw[first].tracks[0].track_id == stable[first].tracks[0].track_id == 0
        target = positions[first].targets[0]
        assert target.track_id == 0 and target.position_valid
        assert positions[first].header.frame_id == target.header.frame_id == 'odom'
        assert key(target.header) == first
        assert abs(target.position_x - 8.) < 1e-5
        assert abs(target.position_y - 2.) < 1e-5
        assert abs(target.position_z - 2.) < 1e-5
        send()
        for _ in range(3):
            gone, _ = send(False)
        assert not raw[gone].tracks and not stable[gone].tracks
        new, _ = send()
        new_raw = raw[new].tracks[0].track_id
        assert new_raw == 1 and positions[new].targets[0].track_id == new_raw
        # The first real 3D observation returns after new raw ID publication.
        assert stable[new].tracks[0].track_id == new_raw
        merged, _ = send()
        assert raw[merged].tracks[0].track_id == new_raw
        assert positions[merged].targets[0].track_id == new_raw
        assert stable[merged].tracks[0].track_id == 0

        future = reset.call_async(Trigger.Request())
        until(future.done)
        assert future.result().success
        # A late observation of the old raw ID must never attach to a new track.
        late_world.publish(positions[first])
        spin(.05)
        fresh, detection = send()
        fresh_id = raw[fresh].tracks[0].track_id
        assert fresh_id > new_raw
        assert stable[fresh].tracks[0].track_id == fresh_id

        # A duplicate source frame produces invalidation, not another KF step.
        raw.pop(fresh)
        stable.pop(fresh)
        det_pub.publish(detection)
        until(lambda: fresh in raw and fresh in stable)
        assert not raw[fresh].tracks and not stable[fresh].tracks
        resumed, _ = send()
        assert raw[resumed].tracks[0].track_id == fresh_id
        assert stable[resumed].tracks[0].track_id == fresh_id
    finally:
        forced = []
        for process in processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                forced.append(process.pid)
                process.kill()
                process.wait(timeout=5)
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()
        assert not forced, f'forced node cleanup: {forced}'
        assert all(p.returncode == 0 for p in processes), {
            p.pid: p.returncode for p in processes
        }
