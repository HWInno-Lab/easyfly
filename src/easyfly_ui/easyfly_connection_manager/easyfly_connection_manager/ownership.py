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
"""Private, bounded owner channel; a reconnect never sends a flight command."""
import hmac
import json
import os
from pathlib import Path
import socket
import stat
import struct
import time


def identity(pid):
    root = Path('/proc') / str(pid)
    fields = (root / 'stat').read_text().rsplit(')', 1)[1].split()
    return dict(pid=pid, start=fields[19],
                boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip())


def save_terminal(directory, token, session, status):
    """Save a private terminal receipt even if dependency checks failed."""
    value = dict(token=token, session=session, owner=identity(os.getpid()), status=status)
    target = Path(directory) / 'terminal.json'
    temporary = target.with_suffix('.tmp')
    descriptor = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW,
                         0o600)
    with os.fdopen(descriptor, 'w') as stream:
        json.dump(value, stream, ensure_ascii=False)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, target)


class OwnerChannel:
    """Only the frozen credential and the current user's peer can query/stop."""

    def __init__(self, directory, token, session, worker):
        if len(token) != 64 or not all(c in '0123456789abcdef' for c in token):
            raise ValueError('缺少后端恢复凭据')
        self.token, self.session, self.worker = token, session, worker
        self.path = Path(directory) / 'backend.sock'
        if self.path.exists():
            info = self.path.lstat()
            if not stat.S_ISSOCK(info.st_mode) or info.st_uid != os.getuid():
                raise ValueError('后端恢复 socket 路径不安全')
            # The caller already holds the exclusive backend reservation.
            self.path.unlink()
        self.server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.server.bind(str(self.path))
        self.path.chmod(0o600)
        self.server.listen(4)
        self.server.setblocking(False)
        self.clients = {}

    def poll(self):
        try:
            peer, _ = self.server.accept()
        except BlockingIOError:
            peer = None
        if peer is not None:
            _, uid, _ = struct.unpack('3i', peer.getsockopt(
                socket.SOL_SOCKET, socket.SO_PEERCRED, struct.calcsize('3i')))
            if uid != os.getuid() or len(self.clients) >= 4:
                peer.close()
            else:
                peer.setblocking(False)
                self.clients[peer] = [bytearray(), time.monotonic()]
        for peer, (data, started) in list(self.clients.items()):
            try:
                if time.monotonic() - started > 2:
                    raise ValueError('请求超时')
                try:
                    chunk = peer.recv(16385)
                except BlockingIOError:
                    continue
                if not chunk or len(data) + len(chunk) > 16384:
                    raise ValueError('请求无效或超出预算')
                data.extend(chunk)
                if b'\n' not in data:
                    continue
                request = json.loads(data)
                if (not isinstance(request, dict) or
                        not hmac.compare_digest(str(request.get('token', '')), self.token) or
                        request.get('session') != self.session or
                        request.get('op') not in ('status', 'stop')):
                    raise ValueError('后端身份或操作无效')
                error = ''
                if request['op'] == 'stop':
                    try:
                        self.worker.stop()
                        self.worker.error = None
                    except RuntimeError as exc:
                        error = str(exc)
                        self.worker.error = error
                reply = dict(session=self.session, owner=identity(os.getpid()),
                             status=self.worker.status(), error=error)
                encoded = json.dumps(reply, ensure_ascii=False).encode() + b'\n'
                if len(encoded) > 32768:
                    raise ValueError('状态超出预算')
                # Bounded response; a slow peer cannot retain the launch owner.
                peer.settimeout(0.01)
                peer.sendall(encoded)
            except (OSError, ValueError, TypeError):
                pass
            else:
                if b'\n' not in data:
                    continue
            peer.close()
            self.clients.pop(peer, None)

    def close(self):
        for peer in self.clients:
            peer.close()
        self.clients.clear()
        self.server.close()
        self.path.unlink(missing_ok=True)

    def finish(self, status):
        """Save an authenticated terminal receipt before the owner exits."""
        save_terminal(self.worker.directory, self.token, self.session, status)
