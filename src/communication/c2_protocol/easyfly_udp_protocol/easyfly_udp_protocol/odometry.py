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

"""Encode native metric odometry without JSON ambiguity or covariance truncation."""

import math
import re
import struct

from nav_msgs.msg import Odometry


VALUES = struct.Struct('!Q85d')


def frame(value):
    """Require a bounded ROS coordinate identifier rather than an executable or path."""
    if not re.fullmatch('[A-Za-z_][A-Za-z0-9_/]{0,63}', value):
        raise ValueError('unsupported coordinate frame')
    return value.encode('ascii')


def validate(values):
    """Reject nonfinite, non-normalized or physically unbounded transport values."""
    if len(values) != 85 or not all(math.isfinite(value) for value in values):
        raise ValueError('invalid odometry numeric values')
    if (any(abs(value) > 1e7 for value in values[:3]) or
            not .99 <= sum(value * value for value in values[3:7]) <= 1.01 or
            any(abs(value) > 1e4 for value in values[7:13]) or
            any(abs(value) > 1e12 for value in values[13:]) or
            any(values[13 + start + i * 7] < 0 for start in (0, 36) for i in range(6))):
        raise ValueError('invalid odometry pose, velocity or covariance')


def encode_odometry(msg):
    """Preserve the original timestamp, two frames, full pose/twist and both 6×6 covariances."""
    source = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
    if not 0 < source < 2**63 or not 0 <= msg.header.stamp.nanosec < 10**9:
        raise ValueError('invalid odometry source timestamp')
    parent, child = frame(msg.header.frame_id), frame(msg.child_frame_id)
    p, q, v, w = (msg.pose.pose.position, msg.pose.pose.orientation,
                  msg.twist.twist.linear, msg.twist.twist.angular)
    values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z, w.x, w.y, w.z,
              *msg.pose.covariance, *msg.twist.covariance]
    validate(values)
    return (VALUES.pack(source, *values) + bytes([len(parent)]) + parent +
            bytes([len(child)]) + child)


def decode_odometry(data):
    """Validate every length and numeric field before creating a ROS output message."""
    if not VALUES.size + 4 <= len(data) <= VALUES.size + 130:
        raise ValueError('invalid odometry payload size')
    values = VALUES.unpack_from(data)
    source, numbers = values[0], values[1:]
    if not 0 < source < 2**63 or source // 10**9 >= 2**31:
        raise ValueError('odometry stamp is outside ROS time representation')
    validate(numbers)
    offset = VALUES.size
    parent_size = data[offset]
    offset += 1
    if not 1 <= parent_size <= 64 or offset + parent_size + 1 >= len(data):
        raise ValueError('invalid parent frame length')
    parent = data[offset:offset + parent_size].decode('ascii')
    offset += parent_size
    child_size = data[offset]
    offset += 1
    if not 1 <= child_size <= 64 or offset + child_size != len(data):
        raise ValueError('invalid child frame length')
    child = data[offset:].decode('ascii')
    frame(parent)
    frame(child)
    msg = Odometry()
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(source, 10**9)
    msg.header.frame_id, msg.child_frame_id = parent, child
    p, q, v, w = (msg.pose.pose.position, msg.pose.pose.orientation,
                  msg.twist.twist.linear, msg.twist.twist.angular)
    p.x, p.y, p.z = numbers[:3]
    q.x, q.y, q.z, q.w = numbers[3:7]
    v.x, v.y, v.z = numbers[7:10]
    w.x, w.y, w.z = numbers[10:13]
    msg.pose.covariance, msg.twist.covariance = numbers[13:49], numbers[49:85]
    return msg
