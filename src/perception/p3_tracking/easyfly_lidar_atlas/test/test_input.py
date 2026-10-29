# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
import numpy as np
import pytest
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from easyfly_lidar_atlas.input import SourceGate, cloud_points, odometry_pose
from easyfly_lidar_atlas.geometry import GeometryRejected


def cloud():
    message = PointCloud2()
    message.header.frame_id = 'camera_init'
    message.height, message.width = 2, 100
    message.point_step, message.row_step = 16, 1608
    values = np.arange(800, dtype='<f4').reshape(200, 4)/100
    message.data = values[:100].tobytes()+b'padding!'+values[100:].tobytes()+b'padding!'
    message.fields = [PointField(name=name, offset=i*4, datatype=7, count=1)
                      for i, name in enumerate(('x', 'y', 'z'))]
    return message, values[:, :3]


def test_real_pointcloud_layout_with_row_padding_and_invalid_fields():
    message, expected = cloud()
    np.testing.assert_array_equal(cloud_points(message), expected)
    message.fields[1].offset = 0
    with pytest.raises(GeometryRejected, match='overlapping'):
        cloud_points(message)
    message, _ = cloud()
    message.is_bigendian = True
    with pytest.raises(GeometryRejected, match='layout'):
        cloud_points(message)
    message, _ = cloud()
    message.data = message.data[:-1]
    with pytest.raises(GeometryRejected, match='layout'):
        cloud_points(message)


@pytest.mark.parametrize('backend,child', [('fast_lio', 'body'), ('fast_livo2', 'aft_mapped')])
def test_native_pose_has_exact_frame_and_metric_quaternion(backend, child):
    message = Odometry()
    message.header.frame_id, message.child_frame_id = 'camera_init', child
    message.pose.pose.orientation.w = 1.
    np.testing.assert_array_equal(odometry_pose(message, backend), np.eye(4))
    message.pose.pose.orientation.w = .9
    with pytest.raises(GeometryRejected, match='quaternion'):
        odometry_pose(message, backend)


@pytest.mark.parametrize('fault', ['duplicate', 'publisher', 'jump', 'future', 'stale'])
def test_source_identity_freshness_discontinuity_latches(fault):
    gate = SourceGate()
    gate.admit('odometry', 1_000_000_000, 1_000_000_000, 1., b'a'*24, pose=np.eye(4))
    stamp, now, gid, pose = 1_100_000_000, 1_100_000_000, b'a'*24, np.eye(4)
    if fault == 'duplicate': stamp = 1_000_000_000
    if fault == 'publisher': gid = b'b'*24
    if fault == 'jump': pose[0, 3] = 1.
    if fault == 'future': stamp += 30_000_000
    if fault == 'stale': now += 800_000_000
    with pytest.raises(GeometryRejected):
        gate.admit('odometry', stamp, now, 1.1, gid, pose=pose)
    assert gate.failure
    with pytest.raises(GeometryRejected):
        gate.admit('odometry', 1_200_000_000, 1_200_000_000, 1.2, b'a'*24, pose=np.eye(4))


def test_original_field_fingerprints_ignore_only_cdr_alignment_padding():
    from rclpy.serialization import deserialize_message, serialize_message
    from easyfly_lidar_atlas.input import cloud_fingerprint, odometry_fingerprint
    message, _ = cloud()
    message.header.frame_id = 'body'
    message.header.stamp.sec = 1
    raw = bytearray(serialize_message(message))
    # CDR: 4-byte encapsulation, 8-byte stamp, 4-byte string length,
    # 'body\0', then 3 unspecified alignment bytes before uint32 height.
    raw[21:24] = b'\xa5\x5a\xa5'
    identical = deserialize_message(bytes(raw), PointCloud2)
    assert message == identical
    assert cloud_fingerprint(message) == cloud_fingerprint(identical)
    identical.data[-1] ^= 1
    assert cloud_fingerprint(message) != cloud_fingerprint(identical)
    odometry = Odometry()
    same = deserialize_message(serialize_message(odometry), Odometry)
    assert odometry_fingerprint(odometry) == odometry_fingerprint(same)
    same.pose.covariance[0] = -1.
    assert odometry_fingerprint(odometry) != odometry_fingerprint(same)


def test_parameter_identity_is_typed_order_independent_and_padding_free():
    from rcl_interfaces.msg import ParameterValue
    from rclpy.serialization import deserialize_message, serialize_message
    from easyfly_lidar_atlas.input import parameter_identity
    names = ['translation', 'enabled', 'profile']
    values = [ParameterValue(type=8, double_array_value=[.011,.02329,.00588]),
              ParameterValue(type=1, bool_value=True), ParameterValue(type=4, string_value='native')]
    copied = [deserialize_message(serialize_message(v), ParameterValue) for v in values]
    original = parameter_identity('fast_lio', names, values)
    assert original == parameter_identity('fast_lio', names[::-1], copied[::-1])
    copied[0].double_array_value[0] += .001
    assert original != parameter_identity('fast_lio', names, copied)
    assert original != parameter_identity('fast_livo2', names, values)
