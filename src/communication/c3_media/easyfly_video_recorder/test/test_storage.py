# Copyright 2026 EasyFly Project
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

"""Exercise actual encoded files, raw bags and rejected source/storage inputs."""

import json
from pathlib import Path
import sys

import cv2
import numpy as np
import pytest
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from easyfly_video_recorder.storage import image_bgr, inspect_session, Session  # noqa: E402


def options(**changes):
    """Use small real segments and explicit resource budgets."""
    result = {'fps': 10, 'segment_frames': 3, 'max_width': 1280, 'max_height': 720,
              'byte_limit': 8 * 1024 * 1024, 'reserve_bytes': 0, 'record_bag': False}
    result.update(changes)
    return result


def image(index=1):
    """Include row padding and a moving colored stripe in real sensor input."""
    msg = Image()
    msg.width, msg.height, msg.step, msg.encoding = 96, 64, 96 * 3 + 8, 'rgb8'
    msg.header.stamp.sec = index
    msg.header.frame_id = 'camera_optical'
    pixels = np.zeros((64, msg.step), np.uint8)
    pixels[:, :96 * 3].reshape(64, 96, 3)[:, 8 * index:8 * index + 8, 0] = 220
    msg.data = pixels.tobytes()
    return msg


def pose(index=1):
    """Give each sample a known metric position and source timestamp."""
    msg = Odometry()
    msg.header.stamp.sec = index
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'base_link'
    msg.pose.pose.position.x = index * .25
    msg.pose.pose.orientation.w = 1.
    return msg


def test_encoded_segments_and_native_trajectory(tmp_path):
    """Decode every real AVI and compare recorded trajectory with input."""
    session = Session(tmp_path / 'records', options())
    for i in range(1, 8):
        session.image(image(i), '/image')
        session.pose(pose(i), '/odom')
    report = session.finish('operator_stop')
    assert report['status'] == 'complete'
    assert (report['frames'], report['poses'], len(report['segments'])) == (7, 7, 3)
    assert inspect_session(session.path) == report
    decoded = []
    for name in report['segments']:
        cap = cv2.VideoCapture(str(session.path / name), cv2.CAP_OPENCV_MJPEG)
        while True:
            ok, pixels = cap.read()
            if not ok:
                break
            decoded.append(pixels)
        cap.release()
    assert len(decoded) == 7
    for i, pixels in enumerate(decoded, 1):
        assert pixels[:, 8 * i:8 * i + 8, 2].mean() > 190
        assert pixels[:, 8 * i:8 * i + 8, 0].mean() < 15
    rows = [json.loads(s) for s in (session.path / 'trajectory.jsonl').read_text().splitlines()]
    assert [r['pose'][0] for r in rows] == [i * .25 for i in range(1, 8)]
    assert [r['source_ns'] for r in rows] == [i * 10**9 for i in range(1, 8)]


def test_raw_rosbag_roundtrip(tmp_path):
    """Read original CDR through rosbag2 and compare values rather than file presence."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    session = Session(tmp_path / 'records', options(record_bag=True))
    session.image(image(), '/test/image')
    session.pose(pose(), '/test/odom')
    session.finish('operator_stop')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(
        uri=str(session.path / 'bag'), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''))
    result = {}
    while reader.has_next():
        name, data, timestamp = reader.read_next()
        result[name] = deserialize_message(data, Image if name.endswith('image') else Odometry)
        assert timestamp == 10**9
    assert bytes(result['/test/image'].data) == bytes(image().data)
    assert result['/test/odom'].pose.pose.position.x == .25
    assert inspect_session(session.path)['poses'] == 1


@pytest.mark.parametrize('change', ['backwards', 'frame', 'layout', 'nonfinite'])
def test_invalid_sources_are_not_accepted(tmp_path, change):
    """Keep timeline/coordinate changes and malformed layouts out of history."""
    session = Session(tmp_path / 'records', options())
    session.image(image(2), '/image')
    msg = image(3)
    if change == 'backwards':
        msg.header.stamp.sec = 1
    elif change == 'frame':
        msg.header.frame_id = 'different_camera'
    elif change == 'layout':
        msg.step = 1
    with pytest.raises(ValueError):
        if change == 'nonfinite':
            bad = pose()
            bad.pose.pose.position.x = float('nan')
            session.pose(bad, '/odom')
        else:
            session.image(msg, '/image')
    session.finish('invalid_source', failed=True)
    with pytest.raises(ValueError, match='failed'):
        inspect_session(session.path)


def test_reserve_budget_and_corruption(tmp_path):
    """Use real free-space accounting and reject corrupted completed content."""
    with pytest.raises(OSError, match='reserve'):
        Session(tmp_path / 'reserve', options(reserve_bytes=10**18))
    session = Session(tmp_path / 'records', options())
    session.image(image(), '/image')
    session.finish('operator_stop')
    path = session.path / 'video-00000.avi'
    data = bytearray(path.read_bytes())
    data[-1] ^= 1
    path.write_bytes(data)
    with pytest.raises(ValueError, match='fingerprint'):
        inspect_session(session.path)


def test_existing_symlink_and_incomplete_session(tmp_path):
    """Never overwrite existing data or expose incomplete output as playable."""
    root = tmp_path / 'records'
    root.mkdir(mode=0o700)
    alias = tmp_path / 'alias'
    alias.symlink_to(root, target_is_directory=True)
    with pytest.raises(ValueError):
        Session(alias, options())
    session = Session(root, options())
    session.image(image(), '/image')
    assert not (session.path / 'manifest.json').exists()
    session.finish('operator_stop')
    assert image_bgr(image()).shape == (64, 96, 3)
