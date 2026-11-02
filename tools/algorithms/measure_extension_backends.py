#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Describe fixed-input real factory cost; no cross-hardware or flight thresholds."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

from verify_api_install import clean_environment

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    prefix = args.prefix.resolve()
    env = clean_environment()
    env.update(PATH='/usr/bin:/bin', PYTHONNOUSERSITE='1', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='2')
    commands = []
    def run(name, command):
        with (out/(name+'.log')).open('w') as log:
            code = subprocess.call(list(map(str, command)), env=env, stdout=log, stderr=subprocess.STDOUT)
        commands.append({'name': name, 'command': list(map(str, command)), 'returncode': code})
        (out/'commands.json').write_text(json.dumps(commands, indent=2)+'\n')
        if code:
            raise RuntimeError('benchmark step failed: '+name)
    shutil.copytree(ROOT/'tools/algorithms/backend_performance', out/'source')
    packages = ['easyfly_backend_registry', 'easyfly_tracking_api', 'easyfly_grid_planner_api',
                'easyfly_detection_api', 'easyfly_single_tracking_api', 'easyfly_multi_object_tracking',
                'easyfly_global_planner', 'easyfly_object_detection', 'easyfly_single_object_tracking']
    run('configure', ['cmake', '-S', out/'source', '-B', out/'build', '-DCMAKE_BUILD_TYPE=Release',
                     '-DCMAKE_PREFIX_PATH='+';'.join(str(prefix/p) for p in packages)])
    run('compile', ['cmake', '--build', out/'build', '--parallel', '2'])
    model = prefix/'easyfly_object_detection/share/easyfly_object_detection/models/yolo11n.onnx'
    frame = ROOT/'src/perception/p2_detection/easyfly_object_detection/test/m2_frame.bin'
    measurements = []
    for backend in ['sort', 'astar', 'csrt', 'onnx']:
        run(backend, [out/'build'/('bench_'+backend), model, frame])
        measurements.append(json.loads((out/(backend+'.log')).read_text().strip().splitlines()[-1]))
        run(backend+'-ldd', ['ldd', out/'build'/('bench_'+backend)])
        loaded = (out/(backend+'-ldd.log')).read_text()
        assert '/opt/ros/' not in loaded and 'libpython' not in loaded and 'not found' not in loaded
        if backend in ['sort', 'astar']:
            assert 'libonnxruntime' not in loaded and 'libopencv' not in loaded
    snapshot = {'passed': True, 'scope': __doc__, 'measurements': measurements,
        'thresholds': None, 'hardware_cpuinfo': Path('/proc/cpuinfo').read_text(),
        'host_load_at_end': Path('/proc/loadavg').read_text().strip(),
        'source_files': {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                         for p in [Path(__file__), *sorted((ROOT/'tools/algorithms/backend_performance').glob('*'))] if p.is_file()},
        'inputs': {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in [model, frame]}, 'commands': commands}
    (out/'summary.json').write_text(json.dumps(snapshot, indent=2)+'\n')
    print(json.dumps(measurements, indent=2))


if __name__ == '__main__':
    main()
