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

"""Exercise real HTTP sockets, authentication, resource limits and JPEG parsing."""

import http.client
import io
from pathlib import Path
import sys
import threading
import time

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from easyfly_video_streamer.transport import (  # noqa: E402
    jpeg_dimensions, LatestFrame, read_frame, read_token, StreamServer)


def jpeg():
    """Encode a real known-color test pattern."""
    pixels = np.zeros((90, 160, 3), np.uint8)
    pixels[:, 40:60, 2] = 220
    return cv2.imencode('.jpg', pixels)[1].tobytes()


def test_real_authenticated_transfer_and_stale_source():
    """Receive actual JPEG bytes and headers; stale data cannot repeat forever."""
    latest = LatestFrame(.5)
    server = StreamServer(('127.0.0.1', 0), latest, token='a' * 32)
    worker = threading.Thread(target=server.serve_forever, kwargs={'poll_interval': .02})
    worker.start()
    connections = []
    try:
        bad = http.client.HTTPConnection(*server.server_address, timeout=2)
        connections.append(bad)
        bad.request('GET', '/stream.mjpg')
        assert bad.getresponse().status == 401
        good = http.client.HTTPConnection(*server.server_address, timeout=2)
        connections.append(good)
        good.request('GET', '/stream.mjpg', headers={'Authorization': 'Bearer ' + 'a' * 32})
        response = good.getresponse()
        assert response.status == 200
        data = jpeg()
        latest.offer(data, 1000, 'camera/optical', time.monotonic_ns())
        frame = read_frame(response)
        assert (frame.jpeg, frame.source_ns, frame.frame_id) == (data, 1000, 'camera/optical')
        pixels = cv2.imdecode(np.frombuffer(frame.jpeg, np.uint8), cv2.IMREAD_COLOR)
        assert pixels[:, 42:58, 2].mean() > 210
        assert server.metrics['frames_sent'] == 1
        time.sleep(.6)
        with pytest.raises(ValueError):
            read_frame(response)
        assert server.metrics['frames_sent'] == 1
    finally:
        latest.close()
        for connection in connections:
            connection.close()
        server.shutdown()
        server.server_close()
        worker.join()


def test_epoch_and_close():
    """Backward time and coordinate changes have new explicit source identities."""
    frames = LatestFrame(.5)
    frames.offer(jpeg(), 200, 'camera', time.monotonic_ns())
    first = frames.next(0)
    frames.offer(jpeg(), 100, 'camera', time.monotonic_ns())
    second = frames.next(first.sequence)
    assert second.stream_id != first.stream_id
    frames.offer(jpeg(), 300, 'new_camera', time.monotonic_ns())
    assert frames.next(second.sequence).stream_id != second.stream_id
    frames.close()
    assert frames.next(0) is None


def test_private_tokens_and_remote_bind(tmp_path):
    """Refuse exposed credentials and unauthenticated non-loopback listeners."""
    token = tmp_path / 'token'
    token.write_text('a' * 32)
    with pytest.raises(ValueError):
        read_token(token)
    token.chmod(0o600)
    assert read_token(token) == 'a' * 32
    link = tmp_path / 'link'
    link.symlink_to(token)
    with pytest.raises(OSError):
        read_token(link)
    with pytest.raises(ValueError):
        StreamServer(('0.0.0.0', 0), LatestFrame(.5))


@pytest.mark.parametrize('data', [b'', b'bad', b'\xff\xd8\xff\xff\xff\xff',
                                  b'\xff\xd8\xff\xc0\x00\x08\x08\xff\xff\xff\xff\x03'])
def test_malformed_or_huge_jpeg(data):
    """Reject malformed or oversized decoded images before OpenCV allocation."""
    with pytest.raises(ValueError):
        jpeg_dimensions(data)


def test_bounded_multipart():
    """Never allocate an attacker-specified oversized frame buffer."""
    wire = (b'--easyfly-frame\r\nContent-Type: image/jpeg\r\n'
            b'Content-Length: 4294967296\r\n\r\n')
    with pytest.raises(ValueError):
        read_frame(io.BytesIO(wire))


def test_measured_bandwidth_and_connection_cap():
    """Measure real transmitted bytes and reject excess open socket clients."""
    import socket
    frames = LatestFrame(.5)
    budget = 65536
    server = StreamServer(('127.0.0.1', 0), frames, clients=2, bandwidth=budget)
    worker = threading.Thread(target=server.serve_forever, kwargs={'poll_interval': .02})
    worker.start()
    end = threading.Event()
    data = jpeg()

    def publish():
        while not end.is_set():
            frames.offer(data, time.time_ns(), 'camera', time.monotonic_ns())
            end.wait(.005)

    producer = threading.Thread(target=publish)
    producer.start()
    connection = http.client.HTTPConnection(*server.server_address, timeout=2)
    sockets = []
    try:
        connection.request('GET', '/stream.mjpg')
        response = connection.getresponse()
        began = time.monotonic()
        count = 0
        while time.monotonic() - began < 2:
            read_frame(response)
            count += 1
        elapsed = time.monotonic() - began
        with server.metrics_lock:
            sent = server.metrics['bytes_sent']
        assert count > 20
        assert sent <= budget * (1 + elapsed + .02)
        for _ in range(6):
            sockets.append(socket.create_connection(server.server_address, timeout=1))
        time.sleep(.1)
        assert server.metrics['rejected_connections'] >= 4
    finally:
        end.set()
        producer.join()
        frames.close()
        connection.close()
        for client in sockets:
            client.close()
        server.shutdown()
        server.server_close()
        worker.join()
