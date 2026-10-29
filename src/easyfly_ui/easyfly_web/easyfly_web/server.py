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

"""Bounded HTTPS frontend with explicit bearer roles and same-origin mutation checks."""

import argparse
import hmac
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import os
from pathlib import Path
import queue
import signal
import ssl
import threading

from ament_index_python.packages import get_package_share_directory

from .bridge import Bridge
from .configuration import load
from .operations import Operations


class Server(HTTPServer):
    """Handle one bounded request at a time; no unlimited client or ROS worker creation."""

    allow_reuse_address = True

    def __init__(self, config, operations):
        self.config, self.operations = config, operations
        self.assets = Path(get_package_share_directory('easyfly_web')) / 'web'
        self.tls = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        self.tls.minimum_version = ssl.TLSVersion.TLSv1_2
        self.tls.load_cert_chain(config['certificate'], config['private_key'])
        super().__init__((config['bind'], config['port']), Handler)
        self.timeout = .2

    def get_request(self):
        """Limit incomplete TLS handshakes and HTTP bodies to one second."""
        connection, address = self.socket.accept()
        connection.settimeout(1.)
        try:
            return self.tls.wrap_socket(connection, server_side=True), address
        except Exception:
            connection.close()
            raise


class Handler(BaseHTTPRequestHandler):
    """Expose a fixed API and static file table with no remote paths or shell commands."""

    protocol_version = 'HTTP/1.0'

    def log_message(self, format, *args):
        """Avoid logging credentials, mission bodies or attacker-controlled URL strings."""
        del format, args

    def reply(self, status, value, content_type='application/json; charset=utf-8'):
        """Disable caching and executable inline content for every response."""
        body = value if isinstance(value, bytes) else json.dumps(value, allow_nan=False).encode()
        self.send_response(status)
        for key, val in {'Content-Type': content_type, 'Content-Length': str(len(body)),
                         'Cache-Control': 'no-store', 'X-Content-Type-Options': 'nosniff',
                         'Referrer-Policy': 'no-referrer', 'X-Frame-Options': 'DENY',
                         'Content-Security-Policy': "default-src 'self'; object-src 'none'; "
                         "base-uri 'none'; frame-ancestors 'none'; form-action 'self'"}.items():
            self.send_header(key, val)
        self.end_headers()
        try:
            self.wfile.write(body)
        except OSError:
            pass

    def actor(self):
        """Authenticate personal roles before any telemetry, catalog or ownership response."""
        authorization = self.headers.get('Authorization', '')
        actor = None
        for account in self.server.config['accounts']:
            expected = ('Bearer ' + account['token']).encode()
            if hmac.compare_digest(authorization.encode(), expected):
                actor = account
        if actor is None:
            self.reply(401, {'error': 'authentication required'})
        return actor

    def origin(self):
        """Reject alternate Host and Origin headers instead of accepting cross-site requests."""
        from urllib.parse import urlsplit
        expected = self.server.config['origin']
        return (self.headers.get('Host') == urlsplit(expected).netloc and
                self.headers.get('Origin', expected) == expected)

    def do_GET(self):  # noqa: N802
        """Serve the offline frontend or authenticated current native status."""
        if not self.origin():
            self.reply(403, {'error': 'origin or host mismatch'})
            return
        assets = {'/': ('index.html', 'text/html'), '/app.js': ('app.js', 'text/javascript'),
                  '/style.css': ('style.css', 'text/css')}
        if self.path in assets:
            name, kind = assets[self.path]
            self.reply(200, (self.server.assets / name).read_bytes(), kind + '; charset=utf-8')
            return
        actor = self.actor()
        if actor is None:
            return
        if self.path != '/api/status':
            self.reply(404, {'error': 'unknown endpoint'})
            return
        config = self.server.config
        self.reply(200, dict(platform_id=config['platform_id'],
                             boot_id=self.server.operations.boot,
                             account=actor['name'], role=actor['role'],
                             execution_enabled=config['enable_execution'],
                             missions=config['missions'],
                             simulation_time=config['simulation_time'],
                             vehicle=self.server.operations.bridge.snapshot()))

    def do_POST(self):  # noqa: N802
        """Require explicit same-origin JSON requests and exact operation-specific schemas."""
        if not self.origin() or self.headers.get('Origin') != self.server.config['origin']:
            self.reply(403, {'error': 'explicit same-origin request required'})
            return
        actor = self.actor()
        if actor is None:
            return
        try:
            operations = {'/api/review': 'review', '/api/start': 'start',
                          '/api/lookup': 'lookup', '/api/cancel': 'cancel'}
            if self.path not in operations:
                raise ValueError('unknown operation endpoint')
            size = int(self.headers.get('Content-Length', '0'))
            if (self.headers.get('Content-Type') != 'application/json' or
                    self.headers.get('Transfer-Encoding') or not 0 < size <= 4096):
                raise ValueError('invalid body content type or length')
            request = json.loads(self.rfile.read(size))
            value = getattr(self.server.operations, operations[self.path])(actor, request)
            self.reply(200, value)
        except PermissionError as exc:
            self.reply(403, {'error': str(exc)})
        except (ValueError, TypeError, KeyError) as exc:
            self.reply(400, {'error': str(exc)})
        except (RuntimeError, OSError, queue.Full) as exc:
            self.reply(503, {'error': str(exc)})


def main():
    """Start an authenticated gateway; exiting it never cancels or retries native missions."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    config = load(parser.parse_args().config)
    os.environ['ROS_DOMAIN_ID'] = str(config['ros_domain_id'])
    bridge = Bridge(config['simulation_time'])
    operations = None
    server = None
    stopping = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopping.set())
    signal.signal(signal.SIGTERM, lambda *_: stopping.set())
    try:
        operations = Operations(config, bridge)
        server = Server(config, operations)
        while not stopping.is_set():
            server.handle_request()
    finally:
        if server:
            server.server_close()
        if operations:
            operations.close()
        bridge.close()
