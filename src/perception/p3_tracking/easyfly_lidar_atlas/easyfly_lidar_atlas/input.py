# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Bounded native scan/odometry admission; no control-frame conversions."""
import hashlib
import json
import struct
import numpy as np
from scipy.spatial.transform import Rotation
from .geometry import GeometryRejected, pose_distance, transform


def cloud_fingerprint(message):
    """Hash all semantic fields and the exact opaque point-data byte array.

    Re-serialized CDR contains unspecified alignment padding; Humble can
    produce different padding in different readers of the very same message.
    The native PointCloud2.data bytes, including point strides, stay exact.
    """
    if len(message.data) > 16*1024*1024 or len(message.fields) > 32 or \
            len(message.header.frame_id) > 128 or any(len(f.name)>128 for f in message.fields):
        raise GeometryRejected('point cloud fingerprint resource bound')
    metadata = dict(schema='easyfly-pointcloud2-fields-1', stamp_ns=stamp_ns(message),
        frame=message.header.frame_id, width=message.width, height=message.height,
        point_step=message.point_step, row_step=message.row_step,
        is_bigendian=message.is_bigendian, is_dense=message.is_dense,
        fields=[dict(name=f.name, offset=f.offset, datatype=f.datatype, count=f.count)
                for f in message.fields])
    encoded = json.dumps(metadata, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
    digest = hashlib.sha256(len(encoded).to_bytes(4, 'little') + encoded)
    digest.update(bytes(message.data))
    return digest.hexdigest()


def parameter_identity(backend, names, values):
    """Canonical typed parameters, without undefined serialized padding."""
    fields = {1:'bool_value', 2:'integer_value', 3:'double_value', 4:'string_value',
              5:'byte_array_value', 6:'bool_array_value', 7:'integer_array_value',
              8:'double_array_value', 9:'string_array_value'}
    if len(names) != len(values) or len(set(names)) != len(names) or len(names)>256:
        raise GeometryRejected('invalid parameter identity inventory')
    parameters = []
    for name, value in zip(names, values):
        if value.type not in fields or len(name)>256:
            raise GeometryRejected('unsupported native parameter identity type')
        item = getattr(value, fields[value.type])
        if value.type >= 5:
            item = list(item)
            if value.type == 5:
                item = [v if isinstance(v, int) else v[0] for v in item]
        parameters.append((name, value.type, item))
    encoded = json.dumps(dict(schema='easyfly-native-lidar-parameters-2', backend=backend,
        parameters=sorted(parameters)), sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
    if len(encoded)>1024*1024:
        raise GeometryRejected('native calibration identity exceeds bound')
    return hashlib.sha256(encoded).hexdigest()


def odometry_fingerprint(message):
    metadata = json.dumps(['easyfly-odometry-fields-1', stamp_ns(message),
        message.header.frame_id, message.child_frame_id], separators=(',', ':')).encode()
    pose, twist = message.pose.pose, message.twist.twist
    values = [getattr(pose.position, key) for key in ('x','y','z')]
    values += [getattr(pose.orientation, key) for key in ('x','y','z','w')]
    values += list(message.pose.covariance)
    values += [getattr(vector, key) for vector in (twist.linear, twist.angular) for key in ('x','y','z')]
    values += list(message.twist.covariance)
    return hashlib.sha256(metadata + struct.pack('<'+'d'*len(values), *values)).hexdigest()


def stamp_ns(message):
    stamp = message.header.stamp
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise GeometryRejected('invalid native timestamp')
    return stamp.sec*1_000_000_000+stamp.nanosec


def cloud_points(message, *, expected_frame='camera_init'):
    if expected_frame not in ('camera_init', 'body') or \
            message.header.frame_id != expected_frame or message.is_bigendian or \
            not 100 <= message.width*message.height <= 200000 or \
            not 12 <= message.point_step <= 128 or \
            message.row_step < message.width*message.point_step or \
            len(message.data) != message.row_step*message.height or len(message.data) > 16*1024*1024:
        raise GeometryRejected('native scan layout/frame/resource bounds')
    fields = {field.name: field for field in message.fields}
    if len(fields) != len(message.fields):
        raise GeometryRejected('duplicate native scan field')
    result = []
    offsets = []
    for name in ('x', 'y', 'z'):
        field = fields.get(name)
        if field is None or field.datatype != 7 or field.count != 1 or \
                not 0 <= field.offset <= message.point_step-4:
            raise GeometryRejected('native scan XYZ must be bounded float32')
        offsets.append(field.offset)
        result.append(np.ndarray((message.height, message.width), dtype='<f4',
            buffer=message.data, offset=field.offset,
            strides=(message.row_step, message.point_step)).ravel())
    if any(abs(a-b)<4 for i,a in enumerate(offsets) for b in offsets[i+1:]):
        raise GeometryRejected('overlapping native scan fields')
    cloud = np.column_stack(result).astype(np.float64)
    if not np.isfinite(cloud).all() or np.max(abs(cloud)) > 1000:
        raise GeometryRejected('nonfinite or unbounded native scan')
    return cloud


def odometry_pose(message, backend):
    child = 'body' if backend == 'fast_lio' else 'aft_mapped'
    if message.header.frame_id != 'camera_init' or message.child_frame_id != child:
        raise GeometryRejected('unexpected native odometry frame')
    p, q = message.pose.pose.position, message.pose.pose.orientation
    quaternion = np.array([q.x, q.y, q.z, q.w])
    if not np.isfinite(quaternion).all() or abs(np.linalg.norm(quaternion)-1) > 1e-4:
        raise GeometryRejected('invalid native quaternion')
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    pose[:3, 3] = [p.x, p.y, p.z]
    return transform(pose)


class SourceGate:
    def __init__(self):
        self.identities = {}
        self.stamps = {}
        self.receipts = {}
        self.last_pose = None
        self.failure = ''

    def reject(self, reason):
        self.failure = self.failure or reason

    def admit(self, kind, stamp, now_ns, receipt, gid, *, pose=None):
        if self.failure:
            raise GeometryRejected(self.failure)
        gid = bytes(gid)
        if not gid or not any(gid) or stamp <= 0 or stamp > now_ns+20_000_000 or \
                now_ns-stamp > 750_000_000 or stamp <= self.stamps.get(kind, 0):
            self.reject('invalid, stale or nonmonotonic native source')
        elif kind in self.identities and self.identities[kind] != gid:
            self.reject('native source publisher changed')
        elif pose is not None and self.last_pose is not None:
            dt = (stamp-self.stamps['odometry'])/1e9
            distance, angle = pose_distance(self.last_pose, pose)
            if distance > .15+5*dt or angle > .1+2*dt:
                self.reject('native odometry discontinuity')
        if self.failure:
            raise GeometryRejected(self.failure)
        self.identities[kind], self.stamps[kind], self.receipts[kind] = gid, stamp, receipt
        if pose is not None:
            self.last_pose = pose.copy()

    def fresh(self, now_ns, wall):
        return not self.failure and all(kind in self.stamps and
            0 <= now_ns-self.stamps[kind] <= 750_000_000 and
            0 <= wall-self.receipts[kind] <= 1.5 for kind in ('cloud', 'odometry'))
