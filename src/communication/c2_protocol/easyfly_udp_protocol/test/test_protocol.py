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

"""Verify real signatures, frame limits, retired sessions and native covariance fidelity."""

from pathlib import Path
import struct
import sys

from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
from nav_msgs.msg import Odometry
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from easyfly_udp_protocol.odometry import decode_odometry, encode_odometry  # noqa: E402
from easyfly_udp_protocol.protocol import (  # noqa: E402
    decode, Encoder, generate_keys, HEADER, identity, MAX_DATAGRAM, ODOMETRY, PING,
    read_key, ReplayWindow)


def test_actual_signature_destination_and_freshness():
    """Only the pinned sender can authenticate a frame for its intended destination."""
    key = Ed25519PrivateKey.generate()
    encoder = Encoder(key)
    target = identity(Ed25519PrivateKey.generate().public_key())
    packet = encoder.encode(PING, target, b'12345678', now_ns=10**12)
    trusted = {encoder.source: key.public_key()}
    result = decode(packet, trusted, now_ns=10**12 + 100, destination=target)
    assert result.payload == b'12345678' and result.sequence == 1
    for corrupted in [packet[:-1], packet[:-1] + bytes([packet[-1] ^ 1]),
                      packet[:5] + b'\xff' + packet[6:], b'x' * (MAX_DATAGRAM + 1)]:
        with pytest.raises(ValueError):
            decode(corrupted, trusted, now_ns=10**12, destination=target)
    with pytest.raises(ValueError):
        decode(packet, {}, now_ns=10**12, destination=target)
    with pytest.raises(ValueError):
        decode(packet, trusted, now_ns=10**12, destination=encoder.source)
    with pytest.raises(ValueError):
        decode(packet, trusted, now_ns=10**12 + 2 * 10**9, destination=target)
    with pytest.raises(ValueError):
        decode(packet, trusted, now_ns=10**12 - 10**9, destination=target)


def test_replay_reordering_and_live_retired_sender():
    """A former session remains retired even if its old process keeps signing newer timestamps."""
    key = Ed25519PrivateKey.generate()
    old, new = Encoder(key), Encoder(key)
    target = '1' * 32
    trusted = {old.source: key.public_key()}
    guard = ReplayWindow()

    def packet(encoder, now):
        return decode(encoder.encode(PING, target, b'12345678', now_ns=now), trusted, now_ns=now)

    first = packet(old, 10**12)
    guard.accept(first)
    with pytest.raises(ValueError):
        guard.accept(first)
    second, third = packet(old, 10**12 + 1), packet(old, 10**12 + 2)
    guard.accept(third)
    with pytest.raises(ValueError):
        guard.accept(second)
    guard.accept(packet(new, 10**12 + 3))
    with pytest.raises(ValueError):
        guard.accept(packet(old, 10**12 + 4))


def test_native_odometry_preserves_every_covariance_and_frame():
    """Round-trip full native double-precision state, not just a copied position summary."""
    message = Odometry()
    message.header.stamp.sec = 123
    message.header.stamp.nanosec = 456789
    message.header.frame_id = 'vehicle_a/odom'
    message.child_frame_id = 'vehicle_a/base_link'
    message.pose.pose.position.x = 12345.678901234
    message.pose.pose.position.y = -2.5
    message.pose.pose.position.z = 1.25
    message.pose.pose.orientation.w = 1.
    message.twist.twist.linear.x = .7
    message.twist.twist.angular.z = -.8
    message.pose.covariance = [i * .003 for i in range(36)]
    message.twist.covariance = [i * .005 for i in range(36)]
    data = encode_odometry(message)
    assert len(data) + HEADER.size + 64 < MAX_DATAGRAM
    assert decode_odometry(data) == message
    corrupted_packets = [data[:-1], data + b'x',
                         data[:8] + struct.pack('!d', float('nan')) + data[16:]]
    for corrupted in corrupted_packets:
        with pytest.raises(ValueError):
            decode_odometry(corrupted)
    message.pose.pose.orientation.w = 0.
    with pytest.raises(ValueError):
        encode_odometry(message)


def test_identity_files_are_private_and_never_overwritten(tmp_path):
    """Generated private keys remain non-symlink owned files and cannot replace an identity."""
    directory = tmp_path / 'identity'
    public_id = generate_keys(directory)
    key = read_key(directory / 'private.key', private=True)
    assert identity(key.public_key()) == public_id
    assert identity(read_key(directory / 'public.key')) == public_id
    with pytest.raises(FileExistsError):
        generate_keys(directory)
    (directory / 'private.key').chmod(0o644)
    with pytest.raises(ValueError):
        read_key(directory / 'private.key', private=True)


def test_unknown_schema_and_oversized_send_rejected():
    """Reject unsupported outbound kind and data before allocating or signing a network frame."""
    encoder = Encoder(Ed25519PrivateKey.generate())
    with pytest.raises(ValueError):
        encoder.encode(255, '1' * 32, b'x')
    with pytest.raises(ValueError):
        encoder.encode(ODOMETRY, '1' * 32, b'x' * MAX_DATAGRAM)
