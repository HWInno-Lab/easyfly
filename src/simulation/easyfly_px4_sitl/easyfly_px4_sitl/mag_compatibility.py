# Copyright 2026 EasyFly Project
# SPDX-License-Identifier: Apache-2.0
"""Pinned, simulation-only correction of the magnetometer LPF sample interval."""

import hashlib
import json
from pathlib import Path
import re
import shlex
import shutil
import subprocess

MAG_PATH = 'src/modules/ekf2/EKF/aid_sources/magnetometer/mag_control.cpp'
MAG_SHA256 = 'c4c4d174657dd9e58ad94062eb527574a5ced34ca285358a3ddd47a11ed548e5'
PATCHED_MAG_SHA256 = 'a87be0746865df4525c798f35dda3d50ffa06260472f4c1e06f13b29b1ac69ec'
MAG_PATCH_SHA256 = '7cd0b9fa7df803b44bb93721ad059285ed7afa65a2e4451851fdf08a197f324f'
ALPHA_PATH = 'src/lib/mathlib/math/filter/AlphaFilter.hpp'
ALPHA_SHA256 = 'f5397a18862e2e7fa21aa8dcb589c0737005d6b2d34f53c96d3059692c0bf2d8'
EKF_ARCHIVE = 'src/modules/ekf2/libmodules__ekf2.a'
EKF_SHA256 = '96fbdacfc017348de850ae4970a2d24dfd064e4ba2ee96617854cab5a166cc5d'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def check_inputs(source, resources):
    checks = [(source / MAG_PATH, MAG_SHA256), (source / ALPHA_PATH, ALPHA_SHA256),
              (source / 'build/px4_sitl_default' / EKF_ARCHIVE, EKF_SHA256),
              (resources / 'px4-1.16.2-mag-lpf-interval.patch', MAG_PATCH_SHA256)]
    if any(digest(path) != expected for path, expected in checks):
        raise ValueError('unsupported PX4 magnetometer compatibility input')


def update_expression(source):
    """Compile the actual reviewed production statement, not a reimplemented filter."""
    text = Path(source).read_text()
    matches = re.findall(r'(_mag_lpf\.update\(mag_sample\.mag[\s\S]*?\);)', text)
    if len(matches) != 1:
        raise ValueError('unsupported magnetometer update expression')
    return matches[0] + '\n'


def native_probe(source, build, patched, resources, output, entry):
    includes = [v for v in shlex.split(entry['command']) if v.startswith('-I')]
    results = {}
    for name, code in [('original', source / MAG_PATH), ('patched', patched)]:
        header = output / (name + '-mag-update.inc')
        header.write_text(update_expression(code))
        binary = output / (name + '-mag-lpf-probe')
        args = ['c++', '-std=gnu++14', '-O2', *includes,
                '-DEASYFLY_MAG_UPDATE_HEADER="' + str(header) + '"',
                str(resources / 'px4_mag_lpf_probe.cpp'), '-o', str(binary)]
        subprocess.run(args, cwd=build, check=True)
        run = subprocess.run([str(binary), '--step'], capture_output=True,
                             text=True, check=True, timeout=5)
        result = json.loads(run.stdout)
        if not 0.75 < result['10000'] < 0.80 or not (
                0.18 < result['70000'] < 0.20 if name == 'original'
                else 0.68 < result['70000'] < 0.69):
            raise RuntimeError('native magnetometer interval regression failed')
        results[name] = {'step_140ms': result, 'binary_sha256': digest(binary),
                         'production_expression': header.read_text()}
    return results


def prepare_mag(source, output, resources, compile_command):
    """Replace only mag_control.cpp.o in a private copy of the production archive."""
    check_inputs(source, resources)
    build = source / 'build/px4_sitl_default'
    patched = output / 'mag_control.cpp'
    subprocess.run(['patch', '--batch', '--fuzz=0', '--output=' + str(patched),
                    str(source / MAG_PATH),
                    str(resources / 'px4-1.16.2-mag-lpf-interval.patch')], check=True)
    if digest(patched) != PATCHED_MAG_SHA256:
        raise ValueError('unexpected patched magnetometer source')
    entries = json.loads((build / 'compile_commands.json').read_text())
    entry = next(v for v in entries if v['file'] == str(source / MAG_PATH)
                 and '-DPX4_MAIN=ekf2_app_main' in v['command'])
    comparisons = native_probe(source, build, patched, resources, output, entry)
    obj = output / 'mag_control.cpp.o'
    args = compile_command(entry, obj, patched)
    args = ['-DMODULE_NAME="ekf2"' if v.startswith('-DMODULE_NAME=') else v for v in args]
    subprocess.run(args, cwd=build, check=True)
    archive = output / 'libmodules__ekf2.a'
    shutil.copy2(build / EKF_ARCHIVE, archive)
    subprocess.run(['ar', 'r', str(archive), str(obj)], check=True)
    check_inputs(source, resources)
    return archive, {
        'original_source_sha256': MAG_SHA256, 'patched_source_sha256': PATCHED_MAG_SHA256,
        'patch_sha256': MAG_PATCH_SHA256, 'alpha_filter_sha256': ALPHA_SHA256,
        'original_archive_sha256': EKF_SHA256, 'patched_archive_sha256': digest(archive),
        'compile_args': args, 'native_comparison': comparisons,
        'time_constant_seconds': 0.09, 'sample_clock': 'native magnetometer timestamp_sample',
        'fixture_sha256': digest(resources / 'px4_mag_lpf_probe.cpp')}


def validate_mag(value, source, root):
    """Old HRT-only builds remain valid; an interval build needs exact extra provenance."""
    if 'mag_lpf_interval' not in value:
        return
    mag = value['mag_lpf_interval']
    constants = {'original_source_sha256': MAG_SHA256,
                 'patched_source_sha256': PATCHED_MAG_SHA256,
                 'patch_sha256': MAG_PATCH_SHA256, 'alpha_filter_sha256': ALPHA_SHA256,
                 'original_archive_sha256': EKF_SHA256, 'time_constant_seconds': 0.09,
                 'sample_clock': 'native magnetometer timestamp_sample'}
    if not isinstance(mag, dict) or any(mag.get(k) != v for k, v in constants.items()):
        raise ValueError('unreviewed magnetometer compatibility patch')
    checks = [(source / MAG_PATH, MAG_SHA256), (source / ALPHA_PATH, ALPHA_SHA256),
              (root / 'mag_control.cpp', PATCHED_MAG_SHA256),
              (source / 'build/px4_sitl_default' / EKF_ARCHIVE, EKF_SHA256),
              (root / 'libmodules__ekf2.a', mag['patched_archive_sha256'])]
    if any(digest(path) != expected for path, expected in checks):
        raise ValueError('magnetometer compatibility input fingerprint mismatch')
    for name, slow_bounds in [('original', (0.18, 0.20)), ('patched', (0.68, 0.69))]:
        step = mag['native_comparison'][name]['step_140ms']
        if not 0.75 < step['10000'] < 0.80 or not slow_bounds[0] < step['70000'] < slow_bounds[1]:
            raise ValueError('magnetometer native comparison missing or inconsistent')
