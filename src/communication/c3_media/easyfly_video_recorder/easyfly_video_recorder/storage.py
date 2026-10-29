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

"""Bounded MJPEG sessions with source time, trajectory and optional rosbag2."""

import hashlib
import json
import math
import os
from pathlib import Path
import stat
import struct
import time
import uuid

import cv2
import numpy as np


def stamp_ns(stamp):
    """Reject malformed ROS timestamps without converting through float."""
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1000000000:
        raise ValueError('invalid source timestamp')
    return stamp.sec * 1000000000 + stamp.nanosec


def image_bgr(message):
    """Decode supported raw layouts, including row padding, into owned pixels."""
    channels = {'bgr8': 3, 'rgb8': 3, 'mono8': 1}.get(message.encoding)
    if channels is None or not 0 < message.width <= 4096 or not 0 < message.height <= 2160:
        raise ValueError('unsupported image encoding or dimensions')
    size = message.height * message.step
    if (message.step < message.width * channels or size > 32 * 1024 * 1024 or
            len(message.data) != size):
        raise ValueError('invalid image layout')
    pixels = np.frombuffer(message.data, np.uint8).reshape(message.height, message.step)
    pixels = pixels[:, :message.width * channels].reshape(
        message.height, message.width, channels).copy()
    if channels == 1:
        return cv2.cvtColor(pixels, cv2.COLOR_GRAY2BGR)
    if message.encoding == 'rgb8':
        return cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
    return pixels


def digest(path):
    """Hash a regular file without following symbolic links."""
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    try:
        if not stat.S_ISREG(os.fstat(fd).st_mode):
            raise ValueError('not a regular recording file')
        result = hashlib.sha256()
        while True:
            block = os.read(fd, 1024 * 1024)
            if not block:
                return result.hexdigest()
            result.update(block)
    finally:
        os.close(fd)


def exclusive(path, binary=False):
    """Create a private file without overwriting an existing pathname."""
    fd = os.open(path, os.O_RDWR | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
    return os.fdopen(fd, 'w+b' if binary else 'w+', encoding=None if binary else 'utf-8')


class AviWriter:
    """Write classic single-stream MJPEG AVI to an exclusively created file."""

    def __init__(self, path, width, height, fps):
        self.file = exclusive(path, binary=True)
        self.path = path
        self.index = []
        self.width, self.height = width, height
        self.tag(b'RIFF')
        self.riff = self.word(0)
        self.tag(b'AVI LIST')
        hdrl = self.word(0)
        self.tag(b'hdrlavih')
        for value in [56, round(1000000 / fps), 0, 0, 16]:
            self.word(value)
        self.total = self.word(0)
        for value in [0, 1, width * height * 3, width, height, 0, 0, 0, 0]:
            self.word(value)
        self.tag(b'LIST')
        strl = self.word(0)
        self.tag(b'strlstrh')
        self.word(56)
        self.tag(b'vidsMJPG')
        for value in [0, 0, 0, 1, fps, 0]:
            self.word(value)
        self.length = self.word(0)
        for value in [width * height * 3, 0xffffffff, 0, 0, (height << 16) | width]:
            self.word(value)
        self.tag(b'strf')
        for value in [40, 40, width, height, (24 << 16) | 1]:
            self.word(value)
        self.tag(b'MJPG')
        for value in [width * height * 3, 0, 0, 0, 0]:
            self.word(value)
        self.patch(strl, self.file.tell() - strl - 4)
        self.patch(hdrl, self.file.tell() - hdrl - 4)
        self.tag(b'LIST')
        self.movi = self.word(0)
        self.start = self.file.tell()
        self.tag(b'movi')

    def tag(self, value):
        """Append fixed container bytes."""
        self.file.write(value)

    def word(self, value):
        """Append a little-endian unsigned container field."""
        position = self.file.tell()
        self.file.write(struct.pack('<I', value))
        return position

    def patch(self, position, value):
        """Finalize a header without disturbing the append position."""
        end = self.file.tell()
        self.file.seek(position)
        self.word(value)
        self.file.seek(end)

    def append(self, jpeg):
        """Store an encoded frame and its seek entry."""
        self.index.append((self.file.tell() - self.start, len(jpeg)))
        self.tag(b'00dc')
        self.word(len(jpeg))
        self.tag(jpeg)
        if len(jpeg) % 2:
            self.tag(b'\0')
        self.file.flush()

    def close(self):
        """Finish the actual container before independently decoding every frame."""
        if self.file.closed:
            return
        try:
            self.patch(self.movi, self.file.tell() - self.movi - 4)
            self.tag(b'idx1')
            self.word(len(self.index) * 16)
            for offset, size in self.index:
                self.tag(b'00dc')
                for value in [16, offset, size]:
                    self.word(value)
            self.patch(self.riff, self.file.tell() - 8)
            self.patch(self.total, len(self.index))
            self.patch(self.length, len(self.index))
            self.file.flush()
            os.fsync(self.file.fileno())
        finally:
            self.file.close()
        if not self.index:
            raise ValueError('empty video segment')
        reader = cv2.VideoCapture(str(self.path), cv2.CAP_OPENCV_MJPEG)
        count = 0
        try:
            while True:
                ok, image = reader.read()
                if not ok:
                    break
                if image.shape != (self.height, self.width, 3):
                    raise ValueError('video readback dimensions differ')
                count += 1
        finally:
            reader.release()
        if count != len(self.index):
            raise ValueError('video readback frame count differs')


class Session:
    """Own one UUID directory; a final manifest appears only after real flush."""

    def __init__(self, directory, config):
        root = Path(directory)
        if not root.is_absolute() or root != root.resolve() or root == Path.home():
            raise ValueError('choose an absolute independent directory without symlinks')
        root.mkdir(mode=0o700, parents=False, exist_ok=True)
        info = root.lstat()
        if (not stat.S_ISDIR(info.st_mode) or info.st_uid != os.geteuid() or
                stat.S_IMODE(info.st_mode) != 0o700):
            raise ValueError('recording root must be an owned private directory (0700)')
        self.config = dict(config)
        self.id = uuid.uuid4().hex
        self.path = root / self.id
        self.path.mkdir(mode=0o700)
        self.frames = 0
        self.poses = 0
        self.last = {}
        self.frame_ids = {}
        self.avi = None
        self.segments = []
        self.bag = None
        self.bag_topics = set()
        self.finished = False
        self.budget(65536)
        self.frame_log = exclusive(self.path / 'frames.jsonl')
        self.pose_log = exclusive(self.path / 'trajectory.jsonl')
        with exclusive(self.path / 'inprogress.json') as stream:
            json.dump({'schema': 2, 'id': self.id, 'config': config}, stream)
        if config.get('record_bag'):
            import rosbag2_py
            self.bag = rosbag2_py.SequentialWriter()
            self.bag.open(rosbag2_py.StorageOptions(
                uri=str(self.path / 'bag'), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))

    def budget(self, extra):
        """Reserve space for indexes and refuse growth before appending input."""
        used = sum(p.stat().st_size for p in self.path.rglob('*') if p.is_file())
        space = os.statvfs(self.path)
        if used + extra + 65536 > self.config['byte_limit']:
            raise OSError('recording byte limit reached')
        if space.f_bavail * space.f_frsize < self.config['reserve_bytes'] + extra + 65536:
            raise OSError('recording free-space reserve reached')

    def source(self, kind, stamp, frame):
        """Reject timeline rollback and frame changes within a session."""
        if not frame or len(frame) > 128 or stamp <= 0:
            raise ValueError('missing source frame or timestamp')
        if kind in self.frame_ids and self.frame_ids[kind] != frame:
            raise ValueError('source coordinate frame changed')
        if stamp <= self.last.get(kind, -1):
            raise ValueError('source timestamp repeated or moved backwards')
        self.frame_ids[kind] = frame
        self.last[kind] = stamp

    def raw(self, topic, type_name, message, timestamp):
        """Preserve selected original CDR messages with source-clock timestamps."""
        if self.bag is None:
            return
        import rosbag2_py
        from rclpy.serialization import serialize_message
        payload = serialize_message(message)
        self.budget(len(payload) * 2 + 32768)
        if topic not in self.bag_topics:
            self.bag.create_topic(rosbag2_py.TopicMetadata(
                name=topic, type=type_name, serialization_format='cdr'))
            self.bag_topics.add(topic)
        self.bag.write(topic, payload, timestamp)

    def image(self, message, topic):
        """Encode one sampled source image and retain its exact original timing."""
        timestamp = stamp_ns(message.header.stamp)
        pixels = image_bgr(message)
        self.source('image', timestamp, message.header.frame_id)
        maximum = self.config['max_width'], self.config['max_height']
        scale = min(1., maximum[0] / pixels.shape[1], maximum[1] / pixels.shape[0])
        if scale < 1:
            dimensions = (max(1, int(pixels.shape[1] * scale)),
                          max(1, int(pixels.shape[0] * scale)))
            pixels = cv2.resize(pixels, dimensions)
        ok, jpeg = cv2.imencode('.jpg', pixels, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok:
            raise ValueError('JPEG encoding failed')
        self.budget(len(jpeg) + 4096 + self.config['segment_frames'] * 16)
        dimensions = (pixels.shape[1], pixels.shape[0])
        if self.avi and ((self.avi.width, self.avi.height) != dimensions or
                         len(self.avi.index) >= self.config['segment_frames']):
            self.avi.close()
            self.avi = None
        if self.avi is None:
            name = 'video-%05d.avi' % len(self.segments)
            self.avi = AviWriter(self.path / name, *dimensions, self.config['fps'])
            self.segments.append(name)
        self.avi.append(jpeg.tobytes())
        self.raw(topic, 'sensor_msgs/msg/Image', message, timestamp)
        self.frame_log.write(json.dumps({
            'index': self.frames, 'source_ns': timestamp, 'frame_id': message.header.frame_id,
            'received_steady_ns': time.monotonic_ns(), 'segment': self.segments[-1],
            'segment_frame': len(self.avi.index) - 1, 'width': dimensions[0],
            'height': dimensions[1]}) + '\n')
        self.frame_log.flush()
        self.frames += 1

    def pose(self, message, topic):
        """Record native Odometry position/orientation/covariance without fitting."""
        timestamp = stamp_ns(message.header.stamp)
        p, q = message.pose.pose.position, message.pose.pose.orientation
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if not all(math.isfinite(x) for x in values + list(message.pose.covariance)):
            raise ValueError('non-finite trajectory sample')
        if abs(sum(x * x for x in values[3:]) - 1.) > .05:
            raise ValueError('invalid trajectory orientation')
        self.source('odom', timestamp, message.header.frame_id)
        self.budget(4096)
        self.pose_log.write(json.dumps({
            'source_ns': timestamp, 'frame_id': message.header.frame_id,
            'child_frame_id': message.child_frame_id, 'pose': values,
            'covariance': list(message.pose.covariance)}) + '\n')
        self.pose_log.flush()
        self.raw(topic, 'nav_msgs/msg/Odometry', message, timestamp)
        self.poses += 1

    def finish(self, reason, failed=False, counters=None):
        """Retain failures as failures, close SQLite, hash files and publish manifest."""
        if self.finished:
            return
        self.finished = True
        error = ''
        try:
            if self.avi:
                self.avi.close()
        except Exception as exc:
            error = str(exc)
            failed = True
        self.bag = None
        for stream in (self.frame_log, self.pose_log):
            try:
                stream.flush()
                os.fsync(stream.fileno())
            except OSError as exc:
                error = str(exc)
                failed = True
            finally:
                stream.close()
        files = {}
        for path in sorted(self.path.rglob('*')):
            if path.is_file() and path.name != 'inprogress.json':
                files[str(path.relative_to(self.path))] = {
                    'sha256': digest(path), 'bytes': path.stat().st_size}
        record = {'schema': 2, 'id': self.id, 'config': self.config,
                  'status': 'failed' if failed or not self.frames else 'complete',
                  'reason': reason, 'error': error, 'frames': self.frames,
                  'poses': self.poses, 'source_frames': self.frame_ids,
                  'counters': counters or {}, 'segments': self.segments, 'files': files}
        with exclusive(self.path / 'manifest.json') as stream:
            json.dump(record, stream, indent=2, allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
        (self.path / 'inprogress.json').unlink()
        return record


def inspect_session(directory):
    """Verify a completed session before any viewer or replay touches payloads."""
    root = Path(directory)
    if not root.is_absolute() or root != root.resolve() or root.is_symlink():
        raise ValueError('invalid recording directory')
    path = root / 'manifest.json'
    if path.is_symlink() or path.stat().st_size > 4 * 1024 * 1024:
        raise ValueError('invalid manifest')
    record = json.loads(path.read_text())
    if record.get('schema') != 2 or record.get('id') != root.name:
        raise ValueError('unknown schema or session identity')
    if record.get('status') != 'complete' or (root / 'inprogress.json').exists():
        raise ValueError('session is incomplete or failed')
    for relative, expected in record['files'].items():
        name = Path(relative)
        if name.is_absolute() or '..' in name.parts or name.as_posix() != relative:
            raise ValueError('unsafe recording filename')
        path = root / name
        if path.resolve() != path or path.stat().st_size != expected['bytes']:
            raise ValueError('recording path or size mismatch')
        if digest(path) != expected['sha256']:
            raise ValueError('recording fingerprint mismatch')
    return record
