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

"""Reject resource aliasing before starting native simulators and verify model derivation."""

import copy
import json
import os
from pathlib import Path
import socket
import sys
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from easyfly_fleet_sim.configuration import load, Reservation
from easyfly_fleet_sim.models import derive
from easyfly_px4_sitl.session_guard import SessionReservation
import pytest


def configuration():
    """Select real allowed CPU IDs while keeping the fixture independent of host numbering."""
    cpus = sorted(os.sched_getaffinity(0))
    if len(cpus) < 6:
        pytest.skip('resource allocation fixture needs six allowed CPU IDs')
    return dict(version=1, coordinator_domain=170, gazebo_port=12980,
                gazebo_cpus=cpus[:2], camera_rate_hz=10, vehicles=[
                    dict(id='alpha', instance=0, domain=171, spawn=[0, 0, .3], cpus=cpus[2:4]),
                    dict(id='bravo', instance=1, domain=172, spawn=[5, 0, .3], cpus=cpus[4:6])])


def test_resource_aliases_are_rejected(tmp_path):
    """Domains, native instances, CPU sets, names and initial positions cannot overlap."""
    original = configuration()
    path = tmp_path / 'fleet.json'
    path.write_text(json.dumps(original))
    assert load(path) == original
    for field in ['id', 'instance', 'domain', 'spawn', 'cpus']:
        value = copy.deepcopy(original)
        value['vehicles'][1][field] = value['vehicles'][0][field]
        path.write_text(json.dumps(value))
        with pytest.raises(ValueError):
            load(path)
    value = copy.deepcopy(original)
    value['vehicles'][0]['domain'] = value['coordinator_domain']
    path.write_text(json.dumps(value))
    with pytest.raises(ValueError):
        load(path)


def test_fleet_keeps_existing_single_vehicle_lock(tmp_path):
    """A fleet and legacy single-vehicle launcher cannot simultaneously own simulator inputs."""
    config = configuration()
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        config['gazebo_port'] = probe.getsockname()[1]
    fleet = Reservation(tmp_path / 'locks')
    legacy = SessionReservation(tmp_path / 'locks')
    try:
        fleet.acquire_fleet(config)
        with pytest.raises(RuntimeError, match='Another EasyFly'):
            legacy.acquire('http://127.0.0.1:12981')
    finally:
        fleet.close()
        legacy.close()
    assert (tmp_path / 'locks/single-vehicle.lock').exists()


def test_occupied_secondary_vehicle_port_releases_reservation(tmp_path):
    """A conflicting second FCU port fails before any process and releases the whole lock."""
    config = configuration()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as occupied:
        occupied.bind(('127.0.0.1', 14541))
        fleet = Reservation(tmp_path / 'locks')
        with pytest.raises(RuntimeError, match='unavailable'):
            fleet.acquire_fleet(config)
        assert fleet.fd is None


def test_derived_models_keep_physics_and_isolate_sensor_identity(tmp_path):
    """Use installed real Iris/Livox inputs; generated ports and every ROS frame are distinct."""
    source = Path(get_package_share_directory('easyfly_px4_sitl')) / (
        'models/easyfly_iris/easyfly_iris.sdf')
    lidar = Path(get_package_share_directory('easyfly_gazebo_models')) / (
        'easyfly/runtime_models/mid360/mid360.sdf')
    original = ET.parse(source).getroot().find('model')
    artifacts = []
    for vehicle in configuration()['vehicles']:
        record = derive(source, lidar, tmp_path, vehicle, 10)
        model = ET.parse(record['path']).getroot().find('model')
        assert model.get('name') == vehicle['id']
        assert ET.tostring(model.find("link[@name='base_link']/inertial")) == ET.tostring(
            original.find("link[@name='base_link']/inertial"))
        interface = model.find("plugin[@name='mavlink_interface']")
        assert int(interface.findtext('mavlink_tcp_port')) == 4560 + vehicle['instance']
        assert model.find("model[@name='mid360']/link/sensor/plugin/ros_topic").text.startswith(
            '/fleet/' + vehicle['id'] + '/')
        for frame in [*model.iter('frame_name'), *model.iter('frameName')]:
            assert frame.text.startswith(vehicle['id'] + '/')
        for namespace in model.findall('.//ros/namespace'):
            assert namespace.text.startswith('/fleet/' + vehicle['id'] + '/')
        artifacts.append(record['sha256'])
    assert artifacts[0] != artifacts[1]


def test_launcher_delivers_one_interrupt_to_descendant(tmp_path):
    """Native parent forwarding must not receive a second group-wide interrupt."""
    from easyfly_fleet_sim.runtime import Child
    child_code = """
import pathlib, signal, time, sys
path = pathlib.Path(sys.argv[1])
count = 0
def interrupt(*_):
    global count
    count += 1
signal.signal(signal.SIGINT, interrupt)
path.write_text('ready')
while count == 0:
    time.sleep(.01)
time.sleep(.1)
path.write_text(str(count))
"""
    parent_code = """
import subprocess, signal, sys
child = subprocess.Popen([sys.executable, '-c', sys.argv[1], sys.argv[2]])
signal.signal(signal.SIGINT, lambda *_: child.send_signal(signal.SIGINT))
raise SystemExit(child.wait())
"""
    observed = tmp_path / 'signal-count'
    owner = Child('forwarder', [sys.executable, '-c', parent_code, child_code, str(observed)],
                  dict(os.environ), tmp_path, sorted(os.sched_getaffinity(0))[:2])
    try:
        deadline = time.monotonic() + 5
        while not observed.exists() and time.monotonic() < deadline:
            time.sleep(.01)
        assert observed.read_text() == 'ready'
        owner.interrupt()
        while not owner.stopped() and time.monotonic() < deadline:
            time.sleep(.01)
        assert owner.stopped() and owner.process.returncode == 0
        assert observed.read_text() == '1'
    finally:
        owner.interrupt()


def test_owned_orphan_group_is_retired(tmp_path):
    """A failed launcher cannot leave its still-owned native child or log reader alive."""
    from easyfly_fleet_sim.runtime import Child
    child_code = """
import pathlib, signal, time, sys
signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
pathlib.Path(sys.argv[1]).write_text('ready')
while True:
    time.sleep(.01)
"""
    parent_code = """
import subprocess, pathlib, sys, time
subprocess.Popen([sys.executable, '-c', sys.argv[1], sys.argv[2]])
while not pathlib.Path(sys.argv[2]).exists():
    time.sleep(.01)
"""
    observed = tmp_path / 'orphan-ready'
    owner = Child('orphan', [sys.executable, '-c', parent_code, child_code, str(observed)],
                  dict(os.environ), tmp_path, sorted(os.sched_getaffinity(0))[:2])
    try:
        owner.process.wait(timeout=5)
        assert not owner.stopped()
        owner.interrupt()
        deadline = time.monotonic() + 5
        while not owner.stopped() and time.monotonic() < deadline:
            time.sleep(.01)
        assert owner.stopped() and not owner.members()
    finally:
        owner.interrupt()
