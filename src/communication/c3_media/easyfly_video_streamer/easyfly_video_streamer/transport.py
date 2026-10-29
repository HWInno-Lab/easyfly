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

"""Bounded multipart JPEG transport with explicit freshness and authentication."""

from dataclasses import dataclass
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ipaddress
import json
import os
from pathlib import Path
import re
import ssl
import stat
import struct
import threading
import time
from urllib.parse import quote, unquote
import uuid


def read_token(path):
    """Read an owned private token without putting its value in logs or URLs."""
    if not path:
        return ''
    path = Path(path)
    with os.fdopen(os.open(path, os.O_RDONLY | os.O_NOFOLLOW), 'r') as stream:
        info = os.fstat(stream.fileno())
        if (not stat.S_ISREG(info.st_mode) or info.st_uid != os.geteuid() or
                stat.S_IMODE(info.st_mode) != 0o600 or info.st_size > 256):
            raise ValueError('token must be an owned regular 0600 file')
        token = stream.read(257).strip()
    if not re.fullmatch(r'[A-Za-z0-9_-]{32,128}', token):
        raise ValueError('token must contain 32–128 URL-safe characters')
    return token


def jpeg_dimensions(data):
    """Check JPEG dimensions before handing compressed bytes to a decoder."""
    if not data.startswith(b'\xff\xd8'):
        raise ValueError('not a JPEG frame')
    offset = 2
    while offset + 4 <= len(data):
        if data[offset] != 255:
            raise ValueError('invalid JPEG marker')
        while offset < len(data) and data[offset] == 255:
            offset += 1
        if offset >= len(data):
            break
        marker = data[offset]
        offset += 1
        if marker in (0xd9, 0xda):
            break
        if marker in (0x01, *range(0xd0, 0xd9)):
            continue
        if offset + 2 > len(data):
            raise ValueError('truncated JPEG marker')
        size = struct.unpack('>H', data[offset:offset + 2])[0]
        if size < 2 or offset + size > len(data):
            raise ValueError('truncated JPEG header')
        if marker in (0xc0, 0xc1, 0xc2):
            if size < 8:
                raise ValueError('invalid JPEG size header')
            height, width = struct.unpack('>HH', data[offset + 3:offset + 7])
            if not 0 < width <= 1920 or not 0 < height <= 1080:
                raise ValueError('JPEG exceeds decoded pixel limit')
            return width, height
        offset += size
    raise ValueError('JPEG size header missing')


@dataclass(frozen=True)
class Frame:
    """Immutable JPEG with the unchanged source header and local receipt witness."""

    sequence: int
    source_ns: int
    frame_id: str
    received_ns: int
    stream_id: str
    jpeg: bytes


class LatestFrame:
    """Keep only the latest encoded frame and invalidate stale content."""

    def __init__(self, timeout):
        self.condition = threading.Condition()
        self.timeout = timeout
        self.frame = None
        self.closed = False
        self.sequence = 0
        self.identity = uuid.uuid4().hex

    def offer(self, data, source_ns, frame_id, received_ns):
        """Publish one complete validated encoded source observation."""
        jpeg_dimensions(data)
        if not 0 < source_ns < 2**63 or not re.fullmatch(r'[A-Za-z0-9_/.:\-]{1,128}', frame_id):
            raise ValueError('invalid source metadata')
        with self.condition:
            if self.closed:
                return
            if self.frame and (source_ns <= self.frame.source_ns or
                               frame_id != self.frame.frame_id):
                self.identity = uuid.uuid4().hex
            self.sequence += 1
            self.frame = Frame(self.sequence, source_ns, frame_id,
                               received_ns, self.identity, data)
            self.condition.notify_all()

    def next(self, after, timeout=.2):
        """Return a newer frame only while its local receipt is fresh."""
        with self.condition:
            self.condition.wait_for(lambda: self.closed or (
                self.frame and self.frame.sequence > after and
                (time.monotonic_ns() - self.frame.received_ns) / 1e9 <= self.timeout), timeout)
            if (self.closed or not self.frame or self.frame.sequence <= after or
                    (time.monotonic_ns() - self.frame.received_ns) / 1e9 > self.timeout):
                return None
            return self.frame

    def close(self):
        """Wake all subscribers without allowing old frames to survive restart."""
        with self.condition:
            self.closed = True
            self.frame = None
            self.condition.notify_all()


class StreamServer(ThreadingHTTPServer):
    """Cap active request threads and slow clients before accepting video requests."""

    daemon_threads = False
    block_on_close = True
    allow_reuse_address = True

    def __init__(self, address, frames, token='', certificate='', key='', clients=2,
                 bandwidth=2 * 1024 * 1024):
        if (not ipaddress.ip_address(address[0]).is_loopback and
                not (token and certificate and key)):
            raise ValueError('remote video requires TLS and an explicit bearer token')
        if bool(certificate) != bool(key):
            raise ValueError('certificate and key must be configured together')
        self.frames = frames
        self.token = token
        self.bandwidth = bandwidth
        self.slots = threading.BoundedSemaphore(clients)
        self.context = None
        self.metrics_lock = threading.Lock()
        self.metrics = {'connections': 0, 'frames_sent': 0, 'bytes_sent': 0,
                        'rejected_connections': 0, 'write_failures': 0,
                        'maximum_sender_queue_age_ms': 0.}
        if certificate:
            self.context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            self.context.minimum_version = ssl.TLSVersion.TLSv1_2
            self.context.load_cert_chain(certificate, key)
        super().__init__(address, StreamHandler)

    def get_request(self):
        """Bound TLS handshake and header read time on every accepted socket."""
        connection, address = self.socket.accept()
        connection.settimeout(1.)
        if self.context:
            try:
                connection = self.context.wrap_socket(connection, server_side=True)
            except Exception:
                connection.close()
                raise
        return connection, address

    def process_request(self, request, client_address):
        """Reject excess clients without spawning an unbounded new thread."""
        if not self.slots.acquire(blocking=False):
            with self.metrics_lock:
                self.metrics['rejected_connections'] += 1
            self.shutdown_request(request)
            return
        try:
            super().process_request(request, client_address)
        except Exception:
            self.slots.release()
            raise

    def process_request_thread(self, request, client_address):
        """Return the client slot after actual socket cleanup."""
        try:
            super().process_request_thread(request, client_address)
        finally:
            self.slots.release()


class StreamHandler(BaseHTTPRequestHandler):
    """Serve only status and bounded MJPEG; no remote control endpoint exists."""

    protocol_version = 'HTTP/1.0'

    def log_message(self, format, *args):
        """Avoid emitting credentials, input URLs or untrusted request text."""
        del format, args

    def do_GET(self):  # noqa: N802
        """Authenticate before returning metadata or image bytes."""
        server = self.server
        expected = 'Bearer ' + server.token
        if server.token and not hmac.compare_digest(
                self.headers.get('Authorization', ''), expected):
            self.send_error(401, 'authentication required')
            return
        if self.path == '/status':
            with server.metrics_lock:
                body = json.dumps(server.metrics).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path != '/stream.mjpg':
            self.send_error(404)
            return
        with server.metrics_lock:
            server.metrics['connections'] += 1
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=easyfly-frame')
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        last = 0
        credit = server.bandwidth
        accounting = time.monotonic()
        waiting_since = accounting
        try:
            while not server.frames.closed:
                frame = server.frames.next(last)
                if frame is None:
                    if time.monotonic() - waiting_since > server.frames.timeout:
                        break
                    continue
                now = time.monotonic()
                credit = min(server.bandwidth, credit + (now - accounting) * server.bandwidth)
                accounting = now
                metadata = (
                    '--easyfly-frame\r\nContent-Type: image/jpeg\r\n'
                    'Content-Length: %d\r\nX-Sequence: %d\r\nX-Source-Ns: %d\r\n'
                    'X-Frame-Id: %s\r\nX-Received-Steady-Ns: %d\r\nX-Stream-Id: %s\r\n\r\n'
                    % (len(frame.jpeg), frame.sequence, frame.source_ns,
                       quote(frame.frame_id, safe=''), frame.received_ns, frame.stream_id)
                ).encode('ascii')
                size = len(metadata) + len(frame.jpeg) + 2
                if credit < size:
                    time.sleep(min(.05, (size - credit) / server.bandwidth))
                    continue
                # Credit waiting must never turn a stale frame into a fresh transmission.
                age_ms = (time.monotonic_ns() - frame.received_ns) / 1e6
                if age_ms > server.frames.timeout * 1000:
                    break
                self.wfile.write(metadata + frame.jpeg + b'\r\n')
                self.wfile.flush()
                credit -= size
                last = frame.sequence
                waiting_since = time.monotonic()
                with server.metrics_lock:
                    server.metrics['frames_sent'] += 1
                    server.metrics['bytes_sent'] += size
                    server.metrics['maximum_sender_queue_age_ms'] = max(
                        server.metrics['maximum_sender_queue_age_ms'], age_ms)
        except (OSError, TimeoutError):
            with server.metrics_lock:
                server.metrics['write_failures'] += 1


def read_frame(response, maximum_bytes=1024 * 1024):
    """Parse a bounded multipart message before allocating a decode buffer."""
    boundary = response.readline(128)
    if boundary != b'--easyfly-frame\r\n':
        raise ValueError('stream disconnected or invalid frame boundary')
    fields = {}
    total = 0
    while True:
        line = response.readline(512)
        total += len(line)
        if not line or total > 4096 or not line.endswith(b'\r\n'):
            raise ValueError('invalid multipart header')
        if line == b'\r\n':
            break
        name, separator, value = line.decode('ascii').partition(':')
        name = name.lower()
        if not separator or name in fields:
            raise ValueError('invalid or duplicate frame field')
        fields[name] = value.strip()
    size = int(fields['content-length'])
    if fields.get('content-type') != 'image/jpeg' or not 0 < size <= maximum_bytes:
        raise ValueError('unsupported or oversized stream frame')
    data = response.read(size)
    if len(data) != size or response.read(2) != b'\r\n':
        raise ValueError('truncated stream frame')
    jpeg_dimensions(data)
    frame = Frame(int(fields['x-sequence']), int(fields['x-source-ns']),
                  unquote(fields['x-frame-id']), int(fields['x-received-steady-ns']),
                  fields['x-stream-id'], data)
    if (frame.sequence <= 0 or not 0 < frame.source_ns < 2**63 or frame.received_ns <= 0 or
            not re.fullmatch(r'[A-Za-z0-9_/.:\-]{1,128}', frame.frame_id) or
            not re.fullmatch('[a-f0-9]{32}', frame.stream_id)):
        raise ValueError('invalid remote source metadata')
    return frame
