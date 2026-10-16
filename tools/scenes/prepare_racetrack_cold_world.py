#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Derive an explicitly versioned cold-start world from the locked AWS day world.

Remove saved clock/counter elements, preserving all geometry and saved light poses.
This is world preparation, not a runtime clock or measurement publisher.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET


def sha(data):
    return hashlib.sha256(data).hexdigest()


def derive(workspace):
    base = workspace / 'src/simulation/easyfly_gazebo_worlds'
    source = 'worlds/outdoor/urban/racetrack_aws_day_v1.world'
    before = (base / source).read_bytes()
    if sha(before) != '38dfd8a4ec546a224a3ee934234896f5ebcf5f12711ec03adc4c32286901602c':
        raise ValueError('reviewed original daytime world required')
    document = ET.fromstring(before)
    state = document.find('world/state')
    if state is None or len(document.findall('world/state')) != 1:
        raise ValueError('exactly one saved state required')
    clocks = {name: state.findtext(name) for name in ['sim_time', 'real_time', 'wall_time', 'iterations']}
    if clocks != dict(sim_time='7212 321000000', real_time='744 496821786',
                      wall_time='1539228409 40882448', iterations='742421'):
        raise ValueError('saved clock snapshot changed')
    match = re.search(rb'<state\b[^>]*>.*?</state>', before, flags=re.S)
    block = match.group()
    for name in clocks:
        pattern = rb'[ \t]*<' + name.encode() + rb'>[^<]*</' + name.encode() + rb'>[ \t]*\r?\n'
        block, count = re.subn(pattern, b'', block)
        if count != 1:
            raise ValueError('exactly one saved clock field required: ' + name)
        state.remove(state.find(name))
    after = before[:match.start()] + block + before[match.end():]
    if ET.tostring(document) != ET.tostring(ET.fromstring(after)):
        raise ValueError('changes extend beyond saved clocks')
    destination = 'worlds/outdoor/urban/racetrack_aws_day_v2.world'
    record = dict(scene='racetrack_aws_day_v2', source=source, source_sha256=sha(before),
                  destination=destination, sha256=sha(after), removed_saved_fields=clocks,
                  preserved='Every other byte, including model geometry, poses, physics, '
                            'sensor dependencies, world light definitions and saved light poses.',
                  rationale='Fresh PX4/SITL sessions must cold-start the simulator. The raw v1 '
                            'started at the saved 7212.321-second clock and failed startup; '
                            'v1 and its failed evidence remain separate.',
                  scope='World derivation only. Independent native and SITL evidence required.')
    return {base / destination: after,
            base / 'evidence/racetrack_aws_day_v2/cold-world-s6.json':
                (json.dumps(record, indent=2) + '\n').encode()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    outputs = derive(args.workspace)
    for path, data in outputs.items():
        if args.check:
            if path.read_bytes() != data:
                raise ValueError('derived world record differs: ' + str(path))
        elif path.exists():
            raise ValueError('refusing to overwrite: ' + str(path))
    if not args.check:
        for path, data in outputs.items():
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open('xb') as stream:
                stream.write(data)
    print(json.dumps(dict(check=args.check, sha256=sha(next(iter(outputs.values()))))))


if __name__ == '__main__':
    main()
