#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Reproduce the explicit grounded hospital world from recorded native support.

Only the z token of three include poses changes. Original scene remains intact.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = Path('src/simulation/easyfly_gazebo_worlds')
SOURCE_SHA = '70830baa0c3e6fd61fcf80e4c42c063553562aba931a725f9aea2591058e1a7b'


def reviewed_outputs():
    source = (ROOT / PACKAGE / 'worlds/indoor/hospital/hospital_aws_v1.world').read_bytes()
    if hashlib.sha256(source).hexdigest() != SOURCE_SHA:
        raise ValueError('fixed original hospital world required')
    proof_path = ROOT / PACKAGE / 'evidence/hospital_aws_v2/placement-source-s6.json'
    proof = json.loads(proof_path.read_text())
    execution = proof['native_execution']
    if (proof['source_world_sha256'] != SOURCE_SHA or execution['returncode'] != 0
            or execution['forced_stop'] or execution['flight_goals'] != 0):
        raise ValueError('successful disarmed native support audit required')
    if {r['instance'] for r in proof['corrections']} != {'StorageRack_1', 'WhiteChipChair_1', 'TrolleyBed_1'} or len(proof['corrections']) != 3:
        raise ValueError('exact three reviewed placements required')
    original = ET.fromstring(source)
    output = source
    for record in proof['corrections']:
        before = original.find(f"world/model[@name='{record['instance']}']/include/pose").text
        if before != record['original_pose'] or float(before.split()[2]) != -757769000:
            raise ValueError('original erroneous include pose differs')
        floor = record['floor_z_m']
        bottom = float(record['native_original_collision_bounds']['minz']) - float(before.split()[2])
        if not math.isfinite(floor) or not math.isfinite(bottom) or abs(floor) > 1e-5:
            raise ValueError('reviewed native support differs')
        corrected_z = math.ceil((floor - bottom + .001) * 1000) / 1000
        if corrected_z != record['new_include_z_m'] or not 0 <= corrected_z <= .1:
            raise ValueError('derived grounding calculation differs')
        tokens = before.split()
        tokens[2] = f'{corrected_z:.3f}'
        old = ('<pose>' + before + '</pose>').encode()
        new = ('<pose>' + ' '.join(tokens) + '</pose>').encode()
        if output.count(old) != 1:
            raise ValueError('exactly one literal pose replacement required')
        output = output.replace(old, new)
    current = ET.fromstring(output)
    if [e.text for e in original.iter('uri')] != [e.text for e in current.iter('uri')]:
        raise ValueError('world dependencies changed')
    for record in proof['corrections']:
        node = current.find(f"world/model[@name='{record['instance']}']/include/pose")
        node.text = record['original_pose']
    if ET.tostring(original) != ET.tostring(current):
        raise ValueError('world changed outside the three z tokens')
    record = dict(scene='hospital_aws_v2', source_scene='hospital_aws_v1', source_url=
                  'https://github.com/aws-robotics/aws-robomaker-hospital-world',
                  source_revision='39969a9d250135835cf13b480865647d1a87b6f4',
                  declared_license='MIT-0', source_world_sha256=SOURCE_SHA,
                  world_sha256=hashlib.sha256(output).hexdigest(),
                  placement_evidence_sha256=hashlib.sha256(proof_path.read_bytes()).hexdigest(),
                  model_instances=181, removed_instances=0,
                  changes=[dict(instance=r['instance'], original_pose=r['original_pose'],
                                new_z_m=r['new_include_z_m']) for r in proof['corrections']],
                  all_other_world_bytes_unchanged=True, geometry_or_texture_pixels_changed=False,
                  resources='Three separately versioned repaired asset sets; remaining 41 Fuel sets unchanged.',
                  world_loaded=False, sensors_checked=False, navigation_tested=False,
                  note='Derived grounding and material repairs; not original author intent or inherited runtime acceptance.')
    return {str(PACKAGE / 'worlds/indoor/hospital/hospital_aws_v2.world'): output,
            str(PACKAGE / 'evidence/hospital_aws_v2/import-source.json'):
            (json.dumps(record, indent=2) + '\n').encode()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    outputs = reviewed_outputs()
    for name, data in outputs.items():
        target = ROOT / name
        if args.check:
            if target.read_bytes() != data:
                raise ValueError('world derivation differs: ' + name)
        elif target.exists():
            raise ValueError('refusing overwrite: ' + name)
    if not args.check:
        for name, data in outputs.items():
            target = ROOT / name
            target.parent.mkdir(parents=True, exist_ok=True)
            with target.open('xb') as stream:
                stream.write(data)
    print(json.dumps(dict(scene='hospital_aws_v2', changed_z_tokens=3, check=args.check)))


if __name__ == '__main__':
    main()
