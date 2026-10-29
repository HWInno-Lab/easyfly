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

"""Review or execute an explicitly configured task on an already owned simulation fleet."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import threading
import time

from ament_index_python.packages import get_package_share_directory

from .configuration import load, output_directory
from .control import Controller, durable
from .observations import FleetObservations
from .planner import assign, Planner


def main():
    """Keep review read-only and require the execute flag before native flight action admission."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--manifest', required=True)
    parser.add_argument('--task', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--execute', action='store_true')
    arguments = parser.parse_args()
    output = output_directory(arguments.output)
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(
        Path(get_package_share_directory('easyfly_fleet_sim')) / 'config/raw_image_transport.xml')
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    stopping = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopping.set())
    signal.signal(signal.SIGTERM, lambda *_: stopping.set())
    fleet = FleetObservations(arguments.manifest)
    controller = None
    try:
        config = load(arguments.task, fleet.peers)
        deadline = time.monotonic() + 90
        while not fleet.valid() or not all(peer.grounded() for peer in fleet.peers.values()):
            fleet.pump(.02)
            if stopping.is_set():
                raise InterruptedError('swarm preparation canceled')
            if time.monotonic() > deadline:
                raise TimeoutError('native fleet ground-state preparation timed out: ' +
                                   str({name: peer.fault for name, peer in fleet.peers.items()}))
        starts = {name: (*position[:2], config['takeoff_world_z'])
                  for name, position in fleet.positions().items()}
        goals = (assign(starts, config['targets']) if config['assignment'] == 'minimum_distance'
                 else {name: tuple(position) for name, position in config['targets'].items()})
        review = dict(fleet_session=fleet.manifest['session'], configuration=config,
                      configuration_sha256=hashlib.sha256(json.dumps(
                          config, sort_keys=True).encode()).hexdigest(),
                      vehicle_sessions={name: peer.vehicle['session']
                                        for name, peer in fleet.peers.items()},
                      assignment=goals, plan=Planner(config).plan(starts, goals),
                      execution_requested=arguments.execute)
        durable(output / 'review.json', review)
        if arguments.execute:
            controller = Controller(fleet, config, output, stopping)
            controller.execute(goals)
        else:
            durable(output / 'status.json', dict(phase='review_complete', native_actions_sent=0))
    except BaseException as exc:
        if controller:
            controller.phase, controller.failure = 'failed', str(exc)
        else:
            durable(output / 'status.json', dict(phase='failed', error=str(exc)))
        raise
    finally:
        try:
            if controller:
                controller.close()
        finally:
            fleet.close()
