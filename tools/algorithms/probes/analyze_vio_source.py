#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Summarize real static source timing, separately from localization acceptance."""

import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import subprocess

from analyze_imu_delivery import distribution


HERE = Path(__file__).resolve().parent


def analyze(directory):
    lifetime = json.loads((directory / 'lifetime.json').read_text())
    producer, = directory.glob('producer.*.csv')
    guards = list(directory.glob('guard-take.*.json'))
    commands = [
        ('analyze_native_imu.py', 'analysis.json', ['--native', str(directory / 'native.csv'),
         '--python', str(directory / 'python/samples.jsonl'), '--producer', str(producer)]),
        ('analyze_gazebo_camera.py', 'camera-analysis.json', ['--native',
         str(directory / 'gazebo-camera.csv'), '--python',
         str(directory / 'python/samples.jsonl')])]
    if guards:
        assert len(guards) == 1
        commands[0][2].extend(['--guard', str(guards[0])])
    for tool, name, arguments in commands:
        if not (directory / name).exists():
            with (directory / (name + '.log')).open('w') as log:
                subprocess.run(['python3', str(HERE / tool), *arguments, '--output',
                                str(directory / name)], stdout=log, stderr=subprocess.STDOUT,
                               check=True)
    native = json.loads((directory / 'analysis.json').read_text())
    camera = json.loads((directory / 'camera-analysis.json').read_text())
    python = json.loads((directory / 'python/summary.json').read_text())
    rows = [json.loads(line) for line in (directory / 'python/samples.jsonl').open()]
    streams = {}
    for name in ['imu', 'image', 'info', 'clock', 'forwarded_imu', 'forwarded_image']:
        samples = [r for r in rows if r['stream'] == name]
        differences = [(b['stamp_ns']-a['stamp_ns'])/1e9
                       for a, b in zip(samples, samples[1:])]
        nonincreasing = sum(x <= 0 for x in differences)
        streams[name] = dict(count=len(samples), source_nonincreasing=nonincreasing,
                             source_gap_s=distribution(differences), receipt_gap_s=distribution([
                                 b['receipt']-a['receipt'] for a, b in zip(samples, samples[1:])]))
    imus = [r for r in rows if r['stream'] == 'imu']
    noise = [statistics.pstdev(r['values'][i] for r in imus) for i in range(6)] if imus else []
    # Exact producer timestamps distinguish source gaps from subscriber gaps.
    with producer.open() as stream:
        producer_metadata = stream.readline().strip()
        published = [r for r in csv.DictReader(stream) if r['result'] == '2']
    source_gaps = [(int(b['source_ns'])-int(a['source_ns']))/1e9
                   for a, b in zip(published, published[1:])]
    states = python['states']
    limits = dict(imu_source_gap_s=.025, camera_source_gap_s=.25, receipt_timeout_s=.5)
    localization = lifetime.get('localization', bool(python['states']))
    localization_log = directory / 'localization.log'
    if not localization_log.exists():
        localization_log = directory / 'stack.log'
    nested_errors = [line for line in localization_log.read_text().splitlines()
                     if 'process has died' in line] if localization_log.exists() else []
    source_quality = dict(
        imu_source_gaps_over_25ms=sum(x > .025 for x in source_gaps),
        camera_multi_payload_stamp_groups=camera['raw_ros']['different_pixel_groups'],
        image_source_nonincreasing=streams['image']['source_nonincreasing'],
        imu_source_nonincreasing=streams['imu']['source_nonincreasing'],
        finite_observed_imu=all(math.isfinite(v) for r in imus for v in r['values']))
    result = dict(lifetime=lifetime, streams=streams, source_quality=source_quality,
                  nested_process_errors=nested_errors,
                  producer_metadata=producer_metadata, limits_unchanged=limits,
                  imu_six_axis_stddev=noise, states=states, camera=camera,
                  native=native, comparisons=python['comparison'],
                  localized_input_healthy=(localization and
                                           bool(streams['forwarded_image']['count']) and
                                           not any(state >= 3 for state, _ in states)),
                  evidence_complete=(lifetime['clean_execution'] and not nested_errors and
                                     not python.get('error') and
                                     not python['sample_limit_reached'] and
                                     len(imus) >= 100 and camera['raw_ros']['count'] >= 100 and
                                     int(native['metadata']['discarded']) == 0 and
                                     int(native['event_metadata']['discarded']) == 0 and
                                     'discarded=0' in producer_metadata and
                                     'discarded=0' in camera['metadata'] and
                                     (not localization or
                                      (len(guards) == 1 and bool(states) and
                                       native['guard_audit']['discarded'] == 0))))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = analyze(args.case)
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2)
        stream.write('\n')
    print(json.dumps(dict(source_quality=result['source_quality'], states=result['states'],
                          evidence_complete=result['evidence_complete']), indent=2))
    return 0 if result['evidence_complete'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
