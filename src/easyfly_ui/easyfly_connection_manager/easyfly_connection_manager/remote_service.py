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
"""Authenticate remote lifecycle ownership and preserve unresolved requests after restart."""

import argparse
import fcntl
import hmac
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import os
from pathlib import Path
import re
import signal
import sqlite3
import ssl
import stat
import subprocess
import threading
import time
import uuid

from .remote_observer import Observer
from .remote_profiles import command, fingerprint, private_json, validate


def token_file(path):
    """Read a private authentication token without recording it in state or logs."""
    with os.fdopen(os.open(path, os.O_RDONLY | os.O_NOFOLLOW), 'r') as stream:
        info = os.fstat(stream.fileno())
        if (not stat.S_ISREG(info.st_mode) or info.st_uid != os.geteuid() or
                info.st_mode & 0o077 or info.st_size > 256):
            raise ValueError('token file must be owned, private and at most 256 bytes')
        value = stream.read(257).strip()
    if not re.fullmatch('[A-Za-z0-9_-]{32,128}', value):
        raise ValueError('invalid authentication token file')
    return value


def atomic(path, value):
    """Persist state before external effects; never use a partial state as success."""
    temporary = path.with_name(path.name + '.' + uuid.uuid4().hex)
    with os.fdopen(os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600), 'w') as out:
        json.dump(value, out, sort_keys=True, allow_nan=False)
        out.flush()
        os.fsync(out.fileno())
    os.replace(temporary, path)
    descriptor = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def private_directory(path):
    """Create or validate an exclusive administrative state directory."""
    path.mkdir(mode=0o700, parents=True, exist_ok=True)
    info = path.lstat()
    if (not stat.S_ISDIR(info.st_mode) or path.resolve() != path or
            info.st_uid != os.geteuid() or info.st_mode & 0o077):
        raise ValueError('state directory must be private, owned and without symlinks')


def group_members(group):
    """Read process group identities; do not infer ownership from executable names."""
    members = {}
    for path in Path('/proc').glob('[0-9]*/stat'):
        try:
            fields = path.read_text().rsplit(')', 1)[1].split()
            if int(fields[2]) == group and fields[0] != 'Z':
                members[path.parent.name] = fields[19]
        except (OSError, ValueError, IndexError):
            continue
    return members


class Owner:
    """Own installed children and preserve unresolved operations across service restart."""

    def __init__(self, config):
        self.config = config
        self.boot = uuid.uuid4().hex
        self.root = Path(config['state_directory'])
        private_directory(self.root)
        self.lock_fd = os.open(self.root / 'owner.lock',
                               os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
        fcntl.flock(self.lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        # Lock the local DDS domain too: a second state directory is not a second vehicle.
        lock_root = Path('/tmp') / ('easyfly-remote-owner-' + str(os.geteuid()))
        private_directory(lock_root)
        self.domain_fd = os.open(lock_root / ('domain-%d.lock' % config['ros_domain_id']),
                                 os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
        fcntl.flock(self.domain_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.state_path = self.root / 'state.json'
        self.sessions = private_json(self.state_path) if self.state_path.exists() else {}
        for session in self.sessions.values():
            if session['phase'] not in ('stopped', 'failed'):
                session['phase'] = 'unresolved_after_service_restart'
        atomic(self.state_path, self.sessions)
        self.database = sqlite3.connect(self.root / 'requests.sqlite3')
        (self.root / 'requests.sqlite3').chmod(0o600)
        self.database.execute('PRAGMA synchronous=FULL')
        self.database.execute('CREATE TABLE IF NOT EXISTS receipts '
                              '(id TEXT PRIMARY KEY, digest TEXT NOT NULL, result TEXT NOT NULL)')
        self.database.commit()
        self.children = {}
        self.log_threads = {}
        self.log_errors = {}
        self.observer = Observer(config)
        self.started = time.monotonic()
        self.last_authenticated = 0.

    def save(self):
        """Commit actual ownership and lifecycle transitions."""
        atomic(self.state_path, self.sessions)

    def drain_log(self, name, process, directory):
        """Drain native output with a fixed 2 MiB two-file retention budget."""
        current = directory / 'process.log'
        size = 0
        stream = None
        try:
            while True:
                data = os.read(process.stdout.fileno(), 4096)
                if not data:
                    break
                if name in self.log_errors:
                    continue
                try:
                    if stream and size + len(data) > 1024 * 1024:
                        stream.close()
                        stream = None
                        os.replace(current, directory / 'process.previous.log')
                        size = 0
                    if stream is None:
                        stream = current.open('wb')
                        current.chmod(0o600)
                    stream.write(data)
                    stream.flush()
                    size += len(data)
                except OSError as exc:
                    self.log_errors[name] = str(exc)
        finally:
            if stream:
                try:
                    stream.close()
                except OSError as exc:
                    self.log_errors[name] = str(exc)
            process.stdout.close()

    def refresh(self):
        """Require process-group completion before marking a wrapper and its children stopped."""
        changed = False
        for name, process in list(self.children.items()):
            state = self.sessions[name]
            code = process.poll()
            members = group_members(process.pid)
            if code is None:
                state['members'] = members
                if state['phase'] == 'stopping' and time.monotonic() - state['stop_at'] > 15:
                    state['phase'] = 'stop_unknown'
                    changed = True
            elif not members:
                self.log_threads.pop(name).join()
                state.update(phase='stopped' if code == 0 else 'failed', returncode=code,
                             members={}, finished_unix_ns=time.time_ns())
                self.children.pop(name)
                changed = True
            elif state['phase'] != 'owned_descendants_remain':
                state.update(phase='owned_descendants_remain', returncode=code, members=members)
                changed = True
        if changed:
            self.save()

    def status(self):
        """Separate child existence, fresh outputs, interlocks and client connectivity."""
        self.refresh()
        return {'v': 1, 'server_id': self.config['server_id'], 'boot_id': self.boot,
                'uptime_s': time.monotonic() - self.started,
                'client_seen_age_s': time.monotonic() - self.last_authenticated,
                'log_errors': dict(self.log_errors),
                'profiles': {key: {'algorithm': value['algorithm'], 'policy': value['policy'],
                                   'hash': fingerprint(value), **self.observer.snapshot(key)}
                             for key, value in self.config['profiles'].items()},
                'sessions': self.sessions}

    def mutate(self, request):
        """Deduplicate durable request IDs; stale server/session identities cannot act."""
        self.refresh()
        if (not isinstance(request, dict) or set(request) != {
                'v', 'server_id', 'boot_id', 'id', 'client_id', 'op', 'profile', 'session'} or
                type(request['v']) is not int or request['v'] != 1 or
                request['server_id'] != self.config['server_id'] or
                request['boot_id'] != self.boot or request['op'] not in ('start', 'stop') or
                request['profile'] not in self.config['profiles'] or
                any(not isinstance(request[key], str) or not re.fullmatch('[a-f0-9]{32}',
                    request[key]) for key in ('id', 'client_id')) or
                not isinstance(request['session'], str) or
                not re.fullmatch('[a-f0-9]{32}|', request['session'])):
            raise ValueError('invalid or stale remote request identity')
        digest = fingerprint(request)
        row = self.database.execute('SELECT digest,result FROM receipts WHERE id=?',
                                    (request['id'],)).fetchone()
        if row:
            if row[0] != digest:
                raise ValueError('request ID was already used with different content')
            return json.loads(row[1])
        if self.database.execute('SELECT COUNT(*) FROM receipts').fetchone()[0] >= 10000:
            raise ValueError('receipt budget exhausted; archive during offline maintenance')
        pending = {'accepted': False, 'id': request['id'], 'phase': 'request_unresolved'}
        self.database.execute('INSERT INTO receipts VALUES (?,?,?)',
                              (request['id'], digest, json.dumps(pending)))
        self.database.commit()
        try:
            result = self.apply(request)
        except (ValueError, OSError, RuntimeError) as exc:
            result = {'accepted': False, 'id': request['id'], 'reason': str(exc)}
        self.database.execute('UPDATE receipts SET result=? WHERE id=?',
                              (json.dumps(result), request['id']))
        self.database.commit()
        return result

    def apply(self, request):
        """Apply one explicit operation against the exact owned preset and session."""
        name = request['profile']
        profile = self.config['profiles'][name]
        observed = self.observer.snapshot(name)
        permit = observed['grounded'] if profile['policy'] == 'grounded' else observed['isolated']
        if not permit:
            raise ValueError('fresh ground/idle or isolated observation interlock is unavailable')
        state = self.sessions.get(name)
        if request['op'] == 'start':
            if request['session'] or (state and state['phase'] not in ('stopped', 'failed')):
                raise ValueError('profile already owned or unresolved')
            groups = [('fast_lio', 'openvins', 'fast_livo2', 'orbslam3'), ('gcopter', 'super')]
            resource = next((set(group) for group in groups if profile['algorithm'] in group),
                            {profile['algorithm']})
            for other, owned in self.sessions.items():
                if owned['phase'] not in ('stopped', 'failed') and (
                        other not in self.config['profiles'] or
                        self.config['profiles'][other]['algorithm'] in resource):
                    raise ValueError('algorithm resource is already owned or unresolved')
            if observed['output_publishers']:
                raise ValueError('preset output already has an external publisher')
            session = uuid.uuid4().hex
            directory = self.root / session
            directory.mkdir(mode=0o700)
            argv = command(profile, directory, self.config['simulation_time'])
            state = {'session': session, 'client_id': request['client_id'], 'phase': 'starting',
                     'request': request['id'], 'profile_hash': fingerprint(profile),
                     'directory': str(directory), 'members': {}}
            self.sessions[name] = state
            atomic(directory / 'launch.json', {'preset': profile, 'argv': argv,
                                               'server_id': self.config['server_id'], **state})
            self.save()
            process = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                       stdin=subprocess.DEVNULL, start_new_session=True,
                                       cwd=directory)
            self.children[name] = process
            self.log_errors.pop(name, None)
            thread = threading.Thread(target=self.drain_log, args=(name, process, directory),
                                      name='remote-log-' + name)
            self.log_threads[name] = thread
            thread.start()
            state.update(phase='running', pid=process.pid, started_unix_ns=time.time_ns(),
                         members=group_members(process.pid))
            self.save()
        else:
            if (not state or request['session'] != state['session'] or
                    request['client_id'] != state['client_id']):
                raise ValueError('stop requires the exact session and owning client')
            process = self.children.get(name)
            if not process or process.poll() is not None:
                raise ValueError('no directly owned live parent; manual audit is required')
            if state['phase'] in ('stopping', 'stop_unknown'):
                return {'accepted': True, 'id': request['id'], 'session': state['session'],
                        'phase': state['phase']}
            state.update(phase='stopping', stop_at=time.monotonic())
            self.save()
            process.send_signal(signal.SIGINT)
        return {'accepted': True, 'id': request['id'], 'session': state['session'],
                'phase': state['phase']}

    def idle(self):
        """Allow service exit only after every owned object actually ended."""
        self.refresh()
        return not self.children

    def close(self):
        """Release observers and locks after normal child completion only."""
        self.observer.close()
        self.database.close()
        os.close(self.domain_fd)
        os.close(self.lock_fd)


class Server(HTTPServer):
    """Serial bounded TLS request handling avoids an unbounded request-thread pool."""

    allow_reuse_address = True

    def __init__(self, config, owner):
        self.owner = owner
        self.token = token_file(config['token_file'])
        self.tls = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        self.tls.minimum_version = ssl.TLSVersion.TLSv1_2
        self.tls.load_cert_chain(config['certificate'], config['private_key'])
        super().__init__((config['bind'], config['port']), Handler)
        self.timeout = .2

    def get_request(self):
        """Bound TLS handshake and incomplete client request time."""
        connection, address = self.socket.accept()
        connection.settimeout(1.)
        try:
            return self.tls.wrap_socket(connection, server_side=True), address
        except Exception:
            connection.close()
            raise


class Handler(BaseHTTPRequestHandler):
    """Offer status and bounded JSON mutations; never execute URL or shell input."""

    protocol_version = 'HTTP/1.0'

    def log_message(self, format, *args):
        """Keep credentials and attacker-controlled request strings out of logs."""
        del format, args

    def authenticated(self):
        """Authenticate before reporting platform identities or inspecting process state."""
        if not hmac.compare_digest(self.headers.get('Authorization', ''),
                                   'Bearer ' + self.server.token):
            self.reply(401, {'error': 'authentication required'})
            return False
        self.server.owner.last_authenticated = time.monotonic()
        return True

    def reply(self, status, value):
        """Return bounded structured state with no cache or credential contents."""
        body = json.dumps(value, allow_nan=False).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        try:
            self.wfile.write(body)
        except OSError:
            pass

    def do_GET(self):  # noqa: N802
        """Query current state without retrying a mutation."""
        if self.authenticated():
            self.reply(200, self.server.owner.status()) if self.path == '/v1/status' else (
                self.reply(404, {'error': 'unknown endpoint'}))

    def do_POST(self):  # noqa: N802
        """Require exact schema, size, content type and current server identity."""
        if not self.authenticated():
            return
        try:
            if (self.path != '/v1/operation' or
                    self.headers.get('Content-Type') != 'application/json'):
                raise ValueError('unknown operation endpoint or content type')
            size = int(self.headers.get('Content-Length', '0'))
            if not 0 < size <= 8192 or self.headers.get('Transfer-Encoding'):
                raise ValueError('invalid request length')
            request = json.loads(self.rfile.read(size))
            self.reply(200, self.server.owner.mutate(request))
        except (ValueError, TypeError, KeyError, OSError) as exc:
            self.reply(400, {'error': str(exc)})


def main():
    """Run an explicitly configured remote service; losing the GUI never kills algorithms."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    args = parser.parse_args()
    config = validate(private_json(args.config))
    os.environ['ROS_DOMAIN_ID'] = str(config['ros_domain_id'])
    owner = Owner(config)
    server = None
    requested_exit = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: requested_exit.set())
    signal.signal(signal.SIGTERM, lambda *_: requested_exit.set())
    try:
        server = Server(config, owner)
        while True:
            server.handle_request()
            owner.refresh()
            if requested_exit.is_set() and owner.idle():
                break
    finally:
        if server:
            server.server_close()
        if owner.idle():
            owner.close()
