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

"""Validate the complete fleet resource allocation before creating simulator children."""

import json
import math
import os
from pathlib import Path
import re
import socket
import stat

from easyfly_px4_sitl.session_guard import require_free_port, SessionReservation


class Reservation(SessionReservation):
    """Keep the original single-session lock while reserving every additional fleet resource."""

    def acquire_fleet(self, config):
        """Fail before launch if any vehicle port is occupied; never delete an existing lock."""
        self.acquire('http://127.0.0.1:' + str(config['gazebo_port']))
        try:
            for vehicle in config['vehicles']:
                slot = vehicle['instance']
                require_free_port(4560 + slot, socket.SOCK_STREAM, 'fleet simulator')
                for base in (14540, 14580, 14280, 13030, 18570):
                    require_free_port(base + slot, socket.SOCK_DGRAM, 'fleet PX4/MAVROS')
        except BaseException:
            self.close()
            raise


def load(path):
    """Require unique domains, identities, ports, CPU sets and separated initial poses."""
    with os.fdopen(os.open(path, os.O_RDONLY | os.O_NOFOLLOW), 'r') as stream:
        info = os.fstat(stream.fileno())
        if not stat.S_ISREG(info.st_mode) or info.st_size > 32768:
            raise ValueError('fleet configuration must be a bounded regular file')
        value = json.load(stream)
    fields = {'version', 'coordinator_domain', 'gazebo_port', 'gazebo_cpus', 'vehicles',
              'camera_rate_hz'}
    if not isinstance(value, dict) or set(value) != fields or value['version'] != 1:
        raise ValueError('invalid fleet fields or version')
    if (type(value['coordinator_domain']) is not int or
            not 0 <= value['coordinator_domain'] <= 232 or
            type(value['gazebo_port']) is not int or not 1024 <= value['gazebo_port'] <= 65535 or
            type(value['camera_rate_hz']) is not int or not 5 <= value['camera_rate_hz'] <= 30 or
            not isinstance(value['vehicles'], list) or not 2 <= len(value['vehicles']) <= 4):
        raise ValueError('invalid fleet domain, master port, sensor rate or vehicle count')
    allowed = os.sched_getaffinity(0)
    used_cpus = set()

    def cpus(values):
        if (not isinstance(values, list) or len(values) < 2 or
                any(type(cpu) is not int for cpu in values) or
                len(values) != len(set(values)) or not set(values) <= allowed or
                set(values) & used_cpus):
            raise ValueError('CPU sets must be disjoint, allowed and contain at least two cores')
        used_cpus.update(values)

    cpus(value['gazebo_cpus'])
    domains, names, slots, poses = {value['coordinator_domain']}, set(), set(), []
    for vehicle in value['vehicles']:
        vehicle_fields = {'id', 'instance', 'domain', 'spawn', 'cpus'}
        if not isinstance(vehicle, dict) or set(vehicle) != vehicle_fields:
            raise ValueError('invalid vehicle fields')
        name, slot, domain = vehicle['id'], vehicle['instance'], vehicle['domain']
        pose = vehicle['spawn']
        if (not isinstance(name, str) or not re.fullmatch('[a-z][a-z0-9_]{0,15}', name) or
                name in names or type(slot) is not int or not 0 <= slot <= 3 or slot in slots or
                type(domain) is not int or not 0 <= domain <= 232 or domain in domains or
                not isinstance(pose, list) or len(pose) != 3 or
                any(type(v) not in (int, float) or not math.isfinite(v) for v in pose) or
                any(abs(v) > 50 for v in pose[:2]) or not .1 <= pose[2] <= .5):
            raise ValueError('invalid vehicle identity, domain, instance or ground spawn')
        if any(math.hypot(pose[0] - other[0], pose[1] - other[1]) < 3 for other in poses):
            raise ValueError('vehicle spawn positions must be separated by at least 3 metres')
        cpus(vehicle['cpus'])
        names.add(name)
        slots.add(slot)
        domains.add(domain)
        poses.append(pose)
    return value


def new_directory(path):
    """Create a private session only at a fresh absolute non-symlink location."""
    root = Path(path)
    if not root.is_absolute() or root.resolve() != root or not root.parent.is_dir():
        raise ValueError('fleet output must be a new absolute path without symlinks')
    root.mkdir(mode=0o700)
    return root
