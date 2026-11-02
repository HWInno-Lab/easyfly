#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Relocate the public single-tracker factory; compare real CPU CSRT/KCF updates.

Uses deterministic synthetic images, no ROS nodes or flight operations. No
implementation package discovery, model weights or ORT required by the consumer.
"""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess

from verify_api_install import clean_environment

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', type=Path, action='append', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out = args.output.resolve(); out.mkdir(parents=True, exist_ok=False)
    packages = ['easyfly_backend_registry', 'easyfly_single_tracking_api', 'easyfly_single_object_tracking']
    selected = {}
    for package in packages:
        candidates = [p.resolve()/package for p in args.prefix if (p/package).is_dir()]
        if not candidates: parser.error('missing package: '+package)
        source = candidates[-1]
        shutil.copytree(source, out/'staging'/package, symlinks=False)
        selected[package] = str(source)
    (out/'staging').rename(out/'relocated')
    env = clean_environment()
    env.update(PATH='/usr/bin:/bin', PYTHONNOUSERSITE='1')
    commands = []

    def run(name, command):
        with (out/(name+'.log')).open('w') as log:
            result = subprocess.run(list(map(str, command)), env=env, stdout=log,
                                    stderr=subprocess.STDOUT, timeout=120)
        commands.append(dict(name=name, command=list(map(str, command)), returncode=result.returncode))
        (out/'commands.json').write_text(json.dumps(commands, indent=2)+'\n')
        if result.returncode: raise RuntimeError(name+' failed; original evidence retained')

    run('configure', ['cmake', '-S', ROOT/'tools/algorithms/single_tracking_consumer', '-B', out/'build',
        '-DCMAKE_PREFIX_PATH='+';'.join(str(out/'relocated'/p) for p in packages),
        '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON', '-DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF',
        '-DCMAKE_DISABLE_FIND_PACKAGE_easyfly_csrt=TRUE', '-DCMAKE_DISABLE_FIND_PACKAGE_easyfly_kcf=TRUE'])
    run('build', ['cmake', '--build', out/'build', '--parallel', '2'])
    for backend in ['csrt', 'kcf']:
        run(backend, [out/'build/single_tracking_consumer', backend])
    run('ldd', ['ldd', out/'build/single_tracking_consumer'])
    loaded = (out/'ldd.log').read_text()
    assert not any(x in loaded for x in ['librclcpp', 'librosidl', 'libpython', 'not found', 'libonnxruntime'])
    assert str(out/'relocated/easyfly_single_object_tracking/lib') in loaded
    compile_text = (out/'build/compile_commands.json').read_text()
    assert str(ROOT/'src') not in compile_text
    for package in ['easyfly_csrt', 'easyfly_kcf']:
        assert str(out/'relocated'/package/'include') not in compile_text
    summary = dict(passed=True, selected=selected, commands=commands,
        results={b: json.loads((out/(b+'.log')).read_text()) for b in ['csrt', 'kcf']},
        no_ros_python_ort_runtime=True, implementation_packages_not_discovered=True,
        scope='40 deterministic synthetic frames per backend; OpenCL disabled; no warmup exclusion; descriptive CPU comparison only')
    (out/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary['results']))


if __name__ == '__main__':
    main()
