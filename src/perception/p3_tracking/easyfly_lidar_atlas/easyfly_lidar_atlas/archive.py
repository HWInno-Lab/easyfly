# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Immutable, bounded native-keyframe storage. No pickle or executable payloads."""
from dataclasses import dataclass
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re

import numpy as np

from .geometry import Edge, GeometryRejected, points, transform

SCHEMA = 'easyfly-lidar-atlas-2'
MAX_BYTES = 128 * 1024 * 1024
FILES = ('scans.npy', 'offsets.npy', 'native_poses.npy', 'atlas_poses.npy', 'stamps_ns.npy')


@dataclass
class Keyframe:
    scan: np.ndarray
    native_pose: np.ndarray
    atlas_pose: np.ndarray
    stamp_ns: int
    session: str
    source_sha256: str


@dataclass
class Archive:
    backend: str
    calibration_sha256: str
    keyframes: list
    edges: list
    identity: str = ''


def _digest(data):
    return hashlib.sha256(data).hexdigest()


def _hash(value):
    return isinstance(value, str) and re.fullmatch('[a-f0-9]{64}', value) is not None


def validate(archive):
    if archive.backend not in ('fast_lio', 'fast_livo2') or not _hash(archive.calibration_sha256) or \
            not 2 <= len(archive.keyframes) <= 256 or \
            not len(archive.keyframes)-1 <= len(archive.edges) <= 768:
        raise GeometryRejected('invalid atlas identity or resource bounds')
    previous = {}
    for keyframe in archive.keyframes:
        if not isinstance(keyframe.session, str) or \
                re.fullmatch('[a-f0-9]{32}', keyframe.session) is None or \
                not _hash(keyframe.source_sha256) or not isinstance(keyframe.stamp_ns, int) or \
                not 0 < keyframe.stamp_ns < 2**63 or \
                keyframe.stamp_ns <= previous.get(keyframe.session, 0):
            raise GeometryRejected('invalid or regressing native keyframe provenance')
        previous[keyframe.session] = keyframe.stamp_ns
        points(keyframe.scan, maximum=4096)
        transform(keyframe.native_pose)
        transform(keyframe.atlas_pose)
    reached = {0}
    adjacency = [set() for _ in archive.keyframes]
    pairs = set()
    for edge in archive.edges:
        if not 0 <= edge.start < edge.end < len(archive.keyframes) or \
                not isinstance(edge.loop, bool) or (edge.start, edge.end) in pairs:
            raise GeometryRejected('invalid, duplicate or out-of-range atlas edge')
        pairs.add((edge.start, edge.end))
        transform(edge.measurement)
        adjacency[edge.start].add(edge.end)
        adjacency[edge.end].add(edge.start)
    pending = [0]
    while pending:
        for vertex in adjacency[pending.pop()] - reached:
            reached.add(vertex)
            pending.append(vertex)
    if len(reached) != len(archive.keyframes):
        raise GeometryRejected('disconnected stored atlas')


def _sync_directory(path):
    fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def save(path, archive):
    """Commit metadata last. Incomplete writes can never load as an atlas."""
    path = Path(path)
    if not path.is_absolute() or not path.parent.is_dir():
        raise GeometryRejected('new absolute atlas directory required')
    validate(archive)
    path.mkdir(mode=0o700, exist_ok=False)
    offsets = np.r_[0, np.cumsum([len(k.scan) for k in archive.keyframes])].astype('<i8')
    arrays = [np.concatenate([k.scan for k in archive.keyframes]).astype('<f4'), offsets,
              np.array([k.native_pose for k in archive.keyframes], dtype='<f8'),
              np.array([k.atlas_pose for k in archive.keyframes], dtype='<f8'),
              np.array([k.stamp_ns for k in archive.keyframes], dtype='<i8')]
    entries = {}
    total = 0
    for name, array in zip(FILES, arrays):
        buffer = io.BytesIO()
        np.save(buffer, array, allow_pickle=False)
        data = buffer.getvalue()
        total += len(data)
        if total > MAX_BYTES:
            raise GeometryRejected('atlas exceeds storage budget')
        fd = os.open(path/name, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC, 0o600)
        with os.fdopen(fd, 'wb') as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        entries[name] = dict(bytes=len(data), sha256=_digest(data))
    document = dict(schema=SCHEMA, backend=archive.backend,
                    source_fingerprint='easyfly-pointcloud2-fields-1',
                    calibration_sha256=archive.calibration_sha256,
                    units='metres_radians', tracked_frame='livox_imu_frame',
                    coordinate_contract='atlas_map isolated from local odom and flight TF',
                    keyframes=[dict(session=k.session, source_sha256=k.source_sha256)
                               for k in archive.keyframes],
                    edges=[dict(start=e.start, end=e.end, loop=e.loop,
                                measurement=e.measurement.tolist()) for e in archive.edges],
                    files=entries)
    data = json.dumps(document, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
    if len(data) > 1024*1024:
        raise GeometryRejected('atlas metadata exceeds budget')
    fd = os.open(path/'manifest.json', os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC, 0o600)
    with os.fdopen(fd, 'wb') as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())
    _sync_directory(path)
    _sync_directory(path.parent)
    return _digest(data)


def _read(path, limit):
    fd = os.open(path, os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW | os.O_NONBLOCK)
    with os.fdopen(fd, 'rb') as stream:
        import stat
        info = os.fstat(stream.fileno())
        if not stat.S_ISREG(info.st_mode) or not 0 < info.st_size <= limit:
            raise GeometryRejected('atlas entry is not a bounded regular file')
        data = stream.read(limit+1)
    if len(data) > limit or len(data) != info.st_size:
        raise GeometryRejected('atlas changed while reading')
    return data


def _array(data):
    buffer = io.BytesIO(data)
    version = np.lib.format.read_magic(buffer)
    if version != (1, 0):
        raise GeometryRejected('unsupported numeric array format')
    shape, fortran, dtype = np.lib.format.read_array_header_1_0(buffer)
    if fortran or dtype.str not in ('<f4', '<f8', '<i8') or not 1 <= len(shape) <= 3 or \
            any(not isinstance(n, int) or n < 1 or n > 256*4096 for n in shape) or \
            math.prod(shape) * dtype.itemsize != len(data)-buffer.tell():
        raise GeometryRejected('malformed or oversized numeric array')
    buffer.seek(0)
    return np.load(buffer, allow_pickle=False)


def load(path, backend, calibration_sha256):
    path = Path(path)
    if not path.is_absolute() or path.is_symlink() or not path.is_dir():
        raise GeometryRejected('absolute atlas directory required')
    metadata = _read(path/'manifest.json', 1024*1024)
    try:
        doc = json.loads(metadata)
        if doc['schema'] != SCHEMA or doc['backend'] != backend or \
                doc['source_fingerprint'] != 'easyfly-pointcloud2-fields-1' or \
                doc['calibration_sha256'] != calibration_sha256 or \
                doc['units'] != 'metres_radians' or doc['tracked_frame'] != 'livox_imu_frame' or \
                set(doc['files']) != set(FILES) or \
                set(p.name for p in path.iterdir()) != set(FILES) | {'manifest.json'}:
            raise GeometryRejected('atlas calibration, source or layout mismatch')
        total = 0
        arrays = []
        for name in FILES:
            expected = doc['files'][name]
            if not isinstance(expected['bytes'], int) or not 0 < expected['bytes'] <= MAX_BYTES:
                raise GeometryRejected('atlas entry size rejected')
            total += expected['bytes']
            if total > MAX_BYTES:
                raise GeometryRejected('atlas aggregate size rejected')
            data = _read(path/name, expected['bytes'])
            if len(data) != expected['bytes'] or _digest(data) != expected['sha256']:
                raise GeometryRejected('atlas entry digest mismatch')
            arrays.append(_array(data))
        scans, offsets, native, corrected, stamps = arrays
        count = len(doc['keyframes'])
        if not 2 <= count <= 256 or offsets.shape != (count+1,) or offsets.dtype.kind != 'i' or \
                native.shape != (count, 4, 4) or corrected.shape != (count, 4, 4) or \
                stamps.shape != (count,) or stamps.dtype.kind != 'i' or \
                scans.ndim != 2 or scans.shape[1] != 3 or offsets[0] != 0 or \
                offsets[-1] != len(scans) or (np.diff(offsets) < 100).any() or \
                (np.diff(offsets) > 4096).any():
            raise GeometryRejected('atlas array dimensions rejected')
        keys = [Keyframe(scans[offsets[i]:offsets[i+1]].copy(), native[i], corrected[i],
                         int(stamps[i]), item['session'], item['source_sha256'])
                for i, item in enumerate(doc['keyframes'])]
        edges = [Edge(item['start'], item['end'], transform(item['measurement']), item['loop'])
                 for item in doc['edges']]
        result = Archive(backend, calibration_sha256, keys, edges, _digest(metadata))
        validate(result)
        return result
    except (KeyError, TypeError, ValueError, OverflowError) as error:
        if isinstance(error, GeometryRejected):
            raise
        raise GeometryRejected('malformed atlas metadata') from error
