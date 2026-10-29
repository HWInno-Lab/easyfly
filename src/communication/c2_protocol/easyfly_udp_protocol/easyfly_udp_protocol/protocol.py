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

"""Versioned bounded datagrams with pinned Ed25519 identities and source/session freshness."""

from dataclasses import dataclass
import hashlib
import os
from pathlib import Path
import re
import stat
import struct
import threading
import time
import uuid

from cryptography.exceptions import InvalidSignature
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey, Ed25519PublicKey


MAX_DATAGRAM = 1200
HEADER = struct.Struct('!4sBBH16s16s16sQQHH')
SIGN_CONTEXT = b'EasyFly/UDP/v1\x00'
BROADCAST = '0' * 32
ODOMETRY, PING, PONG, MESH_HELLO, MESH_LINKS = range(1, 6)
KINDS = {ODOMETRY, PING, PONG, MESH_HELLO, MESH_LINKS}


def raw_public(key):
    """Serialize the exact pinned public key used to derive a network node identity."""
    return key.public_bytes(serialization.Encoding.Raw, serialization.PublicFormat.Raw)


def identity(public_key):
    """Derive a stable 128-bit source identifier from the complete public-key fingerprint."""
    return hashlib.sha256(raw_public(public_key)).hexdigest()[:32]


def read_key(path, private=False):
    """Read a bounded non-symlink raw hexadecimal key; private files must be owned 0600."""
    with os.fdopen(os.open(path, os.O_RDONLY | os.O_NOFOLLOW), 'r') as stream:
        info = os.fstat(stream.fileno())
        if not stat.S_ISREG(info.st_mode) or info.st_size > 128:
            raise ValueError('key must be a bounded regular file')
        if private and (info.st_uid != os.geteuid() or stat.S_IMODE(info.st_mode) != 0o600):
            raise ValueError('private key must be owned with mode 0600')
        value = stream.read(129).strip()
    if not re.fullmatch('[a-fA-F0-9]{64}', value):
        raise ValueError('key must contain exactly 32 raw bytes in hexadecimal')
    cls = Ed25519PrivateKey if private else Ed25519PublicKey
    return cls.from_private_bytes(bytes.fromhex(value)) if private else (
        cls.from_public_bytes(bytes.fromhex(value)))


def generate_keys(directory):
    """Create a new private directory and never overwrite an existing identity."""
    root = Path(directory)
    if not root.is_absolute() or root.resolve() != root:
        raise ValueError('new key directory must be absolute without symlinks')
    root.mkdir(mode=0o700)
    key = Ed25519PrivateKey.generate()
    secret = key.private_bytes(serialization.Encoding.Raw, serialization.PrivateFormat.Raw,
                               serialization.NoEncryption())
    keys = [('private.key', secret, 0o600),
            ('public.key', raw_public(key.public_key()), 0o644)]
    for name, data, mode in keys:
        fd = os.open(root / name, os.O_WRONLY | os.O_CREAT | os.O_EXCL, mode)
        with os.fdopen(fd, 'w') as out:
            out.write(data.hex() + '\n')
    return identity(key.public_key())


@dataclass(frozen=True)
class Packet:
    """Authenticated immutable end-to-end content; forwarding never rewrites its signature."""

    kind: int
    source: str
    destination: str
    session: str
    sequence: int
    issued_ns: int
    ttl_ms: int
    payload: bytes


def decode(data, trusted, now_ns=None, destination=None):
    """Verify framing, signed identity and UTC lifetime before exposing a payload."""
    now_ns = time.time_ns() if now_ns is None else now_ns
    if not HEADER.size + 64 <= len(data) <= MAX_DATAGRAM:
        raise ValueError('invalid datagram size')
    magic, version, kind, flags, source, target, session, sequence, issued, ttl, size = (
        HEADER.unpack_from(data))
    if (magic != b'EFLY' or version != 1 or flags or kind not in KINDS or
            size != len(data) - HEADER.size - 64 or not sequence or
            not 0 < issued < 2**63 or not 1 <= ttl <= 5000 or
            not -250000000 <= now_ns - issued <= ttl * 1000000 or session == bytes(16)):
        raise ValueError('invalid version, fields, source clock or expired packet')
    source, target, session = source.hex(), target.hex(), session.hex()
    if destination is not None and target not in (destination, BROADCAST):
        raise ValueError('packet belongs to another destination')
    if target == BROADCAST and kind not in (MESH_HELLO, MESH_LINKS):
        raise ValueError('this message kind cannot be broadcast')
    key = trusted.get(source)
    if key is None or identity(key) != source:
        raise ValueError('unknown pinned source identity')
    try:
        key.verify(data[-64:], SIGN_CONTEXT + data[:-64])
    except InvalidSignature as exc:
        raise ValueError('invalid datagram signature') from exc
    return Packet(kind, source, target, session, sequence, issued, ttl,
                  data[HEADER.size:-64])


class Encoder:
    """Own a fresh transport session and one sequence shared by all message kinds."""

    def __init__(self, key):
        self.key = key
        self.source = identity(key.public_key())
        self.session = uuid.uuid4().hex
        self.sequence = 0
        self.lock = threading.Lock()

    def encode(self, kind, destination, payload, ttl_ms=1500, now_ns=None):
        """Sign the complete versioned header and bounded binary payload."""
        if (kind not in KINDS or not re.fullmatch('[a-f0-9]{32}', destination) or
                not isinstance(payload, bytes) or len(payload) > MAX_DATAGRAM - HEADER.size - 64 or
                not 1 <= ttl_ms <= 5000 or
                (destination == BROADCAST and kind not in (MESH_HELLO, MESH_LINKS))):
            raise ValueError('unsupported outbound datagram')
        issued = time.time_ns() if now_ns is None else now_ns
        if not 0 < issued < 2**63:
            raise ValueError('invalid outbound source time')
        with self.lock:
            self.sequence += 1
            header = HEADER.pack(b'EFLY', 1, kind, 0, bytes.fromhex(self.source),
                                 bytes.fromhex(destination), bytes.fromhex(self.session),
                                 self.sequence, issued, ttl_ms, len(payload))
        signed = header + payload
        return signed + self.key.sign(SIGN_CONTEXT + signed)


class ReplayWindow:
    """Reject reordered or repeated messages and retired sessions during one receiver lifetime."""

    def __init__(self, maximum_peers=32):
        self.peers = {}
        self.maximum_peers = maximum_peers

    def accept(self, packet):
        """Advance only after signature verification; old source clocks cannot reset a session."""
        before = self.peers.get(packet.source)
        retired = set()
        if before:
            session, sequence, issued, retired = before
            if packet.session in retired:
                raise ValueError('retired transport session')
            if packet.session == session:
                if packet.sequence <= sequence or packet.issued_ns < issued:
                    raise ValueError('replayed or reordered packet')
            elif packet.issued_ns <= issued:
                raise ValueError('retired session or backwards source clock')
            else:
                if len(retired) >= 64:
                    raise ValueError('session retirement budget exhausted')
                retired = retired | {session}
        elif len(self.peers) >= self.maximum_peers:
            raise ValueError('peer replay budget exhausted')
        self.peers[packet.source] = (packet.session, packet.sequence, packet.issued_ns, retired)
