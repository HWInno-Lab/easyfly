#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Copy/relocate the four factory closures; build a plain CMake consumer without ROS."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--prefix', type=Path, action='append', required=True, help='ordered oldest to newest isolated install roots')
parser.add_argument('--model', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
root = Path(__file__).resolve().parents[2]
out = args.output.resolve(); out.mkdir(parents=True, exist_ok=False)
packages = ['easyfly_backend_registry', 'easyfly_tracking_api', 'easyfly_sort', 'easyfly_track_id_fusion',
    'easyfly_grid_planner_api', 'easyfly_astar', 'easyfly_detection_api', 'easyfly_yolo_onnx',
    'easyfly_single_tracking_api', 'easyfly_csrt', 'easyfly_kcf', 'easyfly_multi_object_tracking',
    'easyfly_global_planner', 'easyfly_object_detection', 'easyfly_single_object_tracking']
relocated = out / 'relocated'
selected = {}
for package in packages:
    candidates = [prefix.resolve() / package for prefix in args.prefix if (prefix / package).is_dir()]
    if not candidates: parser.error('missing package: ' + package)
    source = candidates[-1]; destination = relocated / package
    shutil.copytree(source, destination, symlinks=False)
    selected[package] = str(source)
env = {k: v for k, v in os.environ.items() if not k.startswith(('AMENT', 'COLCON', 'CMAKE_PREFIX', 'PYTHONPATH', 'LD_LIBRARY'))}
env['PYTHONNOUSERSITE'] = '1'
commands = []
def run(name, command):
    with (out / (name + '.log')).open('w') as log:
        rc = subprocess.call(list(map(str, command)), env=env, stdout=log, stderr=subprocess.STDOUT)
    commands.append(dict(name=name, command=list(map(str, command)), returncode=rc))
    (out / 'summary.json').write_text(json.dumps(dict(selected=selected, commands=commands), indent=2))
    if rc: raise RuntimeError(name + ' failed; original evidence retained')
run('configure', ['cmake', '-S', root/'tools/algorithms/backend_consumer', '-B', out/'build',
    '-DCMAKE_PREFIX_PATH='+';'.join(str(relocated/p) for p in packages), '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON'])
run('compile', ['cmake', '--build', out/'build'])
# The executable's CMake RPATH and installed libraries resolve the relocated closure.
run('run', [out/'build/backend_consumer', args.model.resolve()])
run('ldd', ['ldd', out/'build/backend_consumer'])
ldd = (out/'ldd.log').read_text()
assert not any(name in ldd for name in ('librclcpp', 'librosidl', 'libpython')), ldd
summary = dict(passed=True, selected=selected, commands=commands,
    model_sha256=hashlib.sha256(args.model.read_bytes()).hexdigest(), no_ros_or_python_runtime=True)
(out/'summary.json').write_text(json.dumps(summary, indent=2)); print('relocated four-factory consumer passed')
