# Copyright 2026 EasyFly Project
#
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

"""Reserve this single-vehicle launch before any simulator process starts."""

import errno
import fcntl
import ipaddress
import os
from pathlib import Path
import socket
import stat
import tempfile
from urllib.parse import urlsplit
import weakref

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit


def master_port(uri):
    """Require a local master: these launches start, not join, a simulator."""
    try:
        parsed = urlsplit(uri)
        host = parsed.hostname
        port = parsed.port
        local = host == "localhost" or ipaddress.ip_address(host).is_loopback
        valid = (
            parsed.scheme == "http"
            and local
            and port is not None
            and 1 <= port <= 65535
            and not parsed.username
            and not parsed.password
            and parsed.path in ("", "/")
            and not parsed.query
            and not parsed.fragment
        )
    except (TypeError, ValueError):
        valid = False
    if not valid:
        raise RuntimeError(
            "SITL requires a local http://localhost:PORT Gazebo master; "
            "refusing to connect/spawn into a remote or invalid master URI"
        )
    return port


def require_free_port(port, kind, description):
    """Probe without connecting, sending data, or modifying an existing server."""
    families = [socket.AF_INET]
    if socket.has_ipv6:
        families.append(socket.AF_INET6)
    for family in families:
        try:
            with socket.socket(family, kind) as probe:
                if family == socket.AF_INET6:
                    probe.setsockopt(
                        socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 1
                    )
                    address = ("::", port)
                else:
                    address = ("0.0.0.0", port)
                # Do not use SO_REUSEADDR/PORT: sharing is precisely forbidden.
                probe.bind(address)
        except OSError as exc:
            if family == socket.AF_INET6 and exc.errno in (
                errno.EAFNOSUPPORT,
                errno.EADDRNOTAVAIL,
            ):
                continue
            raise RuntimeError(
                f"SITL startup refused: {description} port {port} unavailable; "
                "stop the owning simulation explicitly, do not reuse its master"
            ) from exc


class SessionReservation:
    """Hold a kernel lock; a leftover lock file alone never blocks startup."""

    def __init__(self, directory=None):
        self.directory = Path(
            directory
            or (Path(tempfile.gettempdir()) / f"easyfly-sitl-{os.getuid()}")
        )
        self.fd = None

    def acquire(self, uri):
        port = master_port(uri)
        if self.fd is not None:
            raise RuntimeError("SITL reservation was already acquired")
        self.directory.mkdir(mode=0o700, exist_ok=True)
        info = self.directory.lstat()
        if (
            not stat.S_ISDIR(info.st_mode)
            or info.st_uid != os.getuid()
            or info.st_mode & 0o077
        ):
            raise RuntimeError(
                "SITL lock directory must be private and owned by this user"
            )
        fd = os.open(
            self.directory / "single-vehicle.lock",
            os.O_CREAT | os.O_RDWR | os.O_CLOEXEC | os.O_NOFOLLOW,
            0o600,
        )
        try:
            info = os.fstat(fd)
            if (
                not stat.S_ISREG(info.st_mode)
                or info.st_uid != os.getuid()
                or info.st_mode & 0o077
            ):
                raise RuntimeError("Unsafe SITL lock file")
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise RuntimeError(
                    "Another EasyFly SITL session is active; refusing a second "
                    "single-vehicle launch (Gazebo and PX4 ports are shared)"
                ) from exc
            self.fd = fd
            # Atomic against cooperating EasyFly launches, plus fail-closed
            # probes for foreign/orphaned processes. A non-cooperating process
            # may still race after a probe; critical child failure aborts launch.
            require_free_port(port, socket.SOCK_STREAM, "Gazebo master")
            require_free_port(4560, socket.SOCK_STREAM, "PX4 simulator TCP")
            require_free_port(14540, socket.SOCK_DGRAM, "MAVROS FCU UDP")
        except BaseException:
            self.fd = None
            os.close(fd)
            raise

    def close(self):
        """Release only our descriptor; never unlink a live or stale lock file."""
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


def guarded_session(actions, critical_actions, *, lock_directory=None):
    """Guard both SITL entry points and stop timers on critical child failure."""
    reservation = SessionReservation(lock_directory)

    def exited(event, context):
        role = critical_actions.get(event.action)
        if role is not None and event.returncode != 0:
            # Raising also makes LaunchService return nonzero. Merely emitting
            # Shutdown would leave a failed startup disguised as launch exit0.
            raise RuntimeError(
                f"SITL {role} failed with exit {event.returncode}"
            )
        if role in ("Gazebo", "PX4") and not context.is_shutdown:
            raise RuntimeError(f"SITL {role} exited before shutdown")

    def start(context):
        reservation.acquire(
            context.environment.get(
                "GAZEBO_MASTER_URI", "http://localhost:11345"
            )
        )
        # Hold until the entire launch context is torn down, not OnShutdown:
        # that event precedes actual child termination and even pending spawns.
        # Keeping it for context lifetime also covers staggered startup gaps.
        weakref.finalize(context, reservation.close)
        return [
            LogInfo(msg="EASYFLY_SITL_RESERVED: single-vehicle inputs exclusive"),
            RegisterEventHandler(OnProcessExit(on_exit=exited)),
            *actions,
        ]

    return OpaqueFunction(function=start)


def guarded_launch(actions, critical_actions, *, preflight=()):
    """Keep declarations visible to ros2 launch --show-args before guarding."""
    declarations = [a for a in actions if isinstance(a, DeclareLaunchArgument)]
    guarded = [a for a in actions if not isinstance(a, DeclareLaunchArgument)]
    return LaunchDescription(
        [
            *declarations,
            *preflight,
            guarded_session(guarded, critical_actions),
        ]
    )
