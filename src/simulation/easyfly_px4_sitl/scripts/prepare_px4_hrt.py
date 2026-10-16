#!/usr/bin/env python3
# Copyright 2026 EasyFly Project
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Build an explicit project-local PX4 HRT compatibility input without changing PX4 sources."""

import argparse
import hashlib
import json
from pathlib import Path
import shlex
import shutil
import subprocess

HRT_PATH = 'platforms/posix/src/px4/common/drv_hrt.cpp'
HRT_SHA256 = 'eb826fde8a1eaae3eef98c8f37c686bc21f62ad061d9cc86c8d2fa2a1ea1ede2'
BINARY_SHA256 = '9484a69221d73465f621b80662b51d72cd4739b216c9a4526dbaef35ba67c9a4'
LAYER_PATH = 'platforms/posix/src/px4/common/libpx4_layer.a'


def digest(path):
    """Hash a real input or generated artifact for the immutable manifest."""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def compile_command(entry, output, patched):
    """Retain the original production flags and replace only the object/source targets."""
    args = shlex.split(entry['command'])
    if args.count('-o') != 1 or args.count('-c') != 1 or args[-1] != entry['file']:
        raise ValueError('unsupported PX4 compile command')
    args[args.index('-o') + 1] = str(output)
    args[-1] = str(patched)
    return ['-DMODULE_NAME="px4"' if v.startswith('-DMODULE_NAME=') else v for v in args]


def link_command(line, output, layer, mag_archive=None):
    """Discard CMake's alias postcommands, rejecting anything except one known link step."""
    args = shlex.split(line)
    if args[:2] != [':', '&&']:
        raise ValueError('unsupported PX4 link prefix')
    end = args.index('&&', 2) if '&&' in args[2:] else len(args)
    args = args[2:end]
    if args.count('-o') != 1 or args.count(LAYER_PATH) != 2 or \
            args[args.index('-o') + 1] != 'bin/px4':
        raise ValueError('unsupported PX4 link targets')
    args[args.index('-o') + 1] = str(output)
    args = [str(layer) if v == LAYER_PATH else v for v in args]
    if mag_archive is not None:
        from easyfly_px4_sitl.mag_compatibility import EKF_ARCHIVE
        if args.count(EKF_ARCHIVE) != 1:
            raise ValueError('unsupported PX4 magnetometer link targets')
        args = [str(mag_archive) if v == EKF_ARCHIVE else v for v in args]
    return args


def native_probe(source, build, patched, resources, output):
    """Compile actual PX4 HRT/queue/semaphore code with only outer timer/log sinks stubbed."""
    entry = next(v for v in json.loads((build / 'compile_commands.json').read_text())
                 if v['file'] == str(source / HRT_PATH))
    includes = [v for v in shlex.split(entry['command']) if v.startswith('-I')]
    flags = ['-DCONFIG_ARCH_BOARD_PX4_SITL', '-D__PX4_POSIX', '-D__PX4_LINUX',
             '-DMODULE_NAME="px4"', '-D__STDC_FORMAT_MACROS', '-pthread',
             '-std=gnu++17', '-O2', '-g', '-include', 'visibility.h', *includes]
    sources = [resources / 'px4_hrt_race.cpp',
               source / 'platforms/posix/src/px4/common/px4_sem.cpp']
    sources += [source / 'platforms/common/work_queue' / v
                for v in ('queue.c', 'sq_addafter.c', 'sq_addlast.c')]
    results = {}
    for variant, code in [('original', source / HRT_PATH), ('patched', patched)]:
        binary = output / (variant + '-hrt-race')
        subprocess.run(['c++', *flags, '-DEASYFLY_PX4_HRT_SOURCE="' + str(code) + '"',
                        *map(str, sources), '-o', str(binary)], check=True, cwd=build)
        runs = []
        for _ in range(10):
            result = subprocess.run([str(binary)], check=True, capture_output=True,
                                    text=True, timeout=5)
            value = json.loads(result.stdout)
            if value['queue_corrupt'] != (variant == 'original'):
                raise RuntimeError('native original/fixed comparison did not match')
            runs.append(value)
        reentry = subprocess.run([str(binary), '--reentry'], check=True, capture_output=True,
                                 text=True, timeout=5)
        reentry_result = json.loads(reentry.stdout)
        if not all(reentry_result.values()):
            raise RuntimeError('native callback reentry failed')
        results[variant] = {'source_sha256': digest(code), 'runs': runs,
                            'same_thread_reentry': reentry_result}
    return results


def prepare(source, output, resources, mag_lpf_interval=False):
    """Patch one local copy, verify the race, then relink a separate simulation-only binary."""
    source = source.resolve(strict=True)
    output = output.resolve()
    build = source / 'build/px4_sitl_default'
    original_binary = build / 'bin/px4'
    if digest(source / HRT_PATH) != HRT_SHA256 or digest(original_binary) != BINARY_SHA256:
        raise ValueError(
            'unsupported PX4 HRT source or binary; explicit compatibility review needed')
    if mag_lpf_interval:
        from easyfly_px4_sitl.mag_compatibility import check_inputs, prepare_mag
        check_inputs(source, resources)
    if output == source or output in source.parents or source in output.parents or \
            not output.parent.is_dir() or output.exists():
        raise ValueError('new independent output directory with an existing parent required')
    output.mkdir()
    patched = output / 'drv_hrt.cpp'
    patch = resources / 'px4-1.16.2-hrt.patch'
    subprocess.run(['patch', '--batch', '--fuzz=0', '--output=' + str(patched),
                    str(source / HRT_PATH), str(patch)], check=True)
    comparisons = native_probe(source, build, patched, resources, output)
    entries = json.loads((build / 'compile_commands.json').read_text())
    entry = next(v for v in entries if v['file'] == str(source / HRT_PATH))
    obj = output / 'drv_hrt.cpp.o'
    compile_args = compile_command(entry, obj, patched)
    subprocess.run(compile_args, check=True, cwd=build)
    layer = output / 'libpx4_layer.a'
    shutil.copy2(build / LAYER_PATH, layer)
    subprocess.run(['ar', 'r', str(layer), str(obj)], check=True)
    mag_archive, mag_record = (prepare_mag(source, output, resources, compile_command)
                               if mag_lpf_interval else (None, None))
    input_tree = output / 'px4-input'
    local_build = input_tree / 'build/px4_sitl_default'
    local_bin = local_build / 'bin'
    local_bin.mkdir(parents=True)
    link_lines = subprocess.run(['ninja', '-t', 'commands', 'bin/px4'], cwd=build,
                                check=True, text=True, capture_output=True).stdout.splitlines()
    link_args = link_command(link_lines[-1], local_bin / 'px4', layer, mag_archive)
    subprocess.run(link_args, check=True, cwd=build)
    for alias in (build / 'bin').glob('px4-*'):
        if alias.name == 'px4-alias.sh' and alias.is_file() and not alias.is_symlink():
            shutil.copy2(alias, local_bin / alias.name)
            continue
        if not alias.is_symlink() or alias.resolve() != original_binary.resolve():
            raise ValueError('unexpected PX4 module alias')
        (local_bin / alias.name).symlink_to('px4')
    for link, target in [(input_tree / 'Tools', source / 'Tools'),
                         (local_build / 'etc', build / 'etc'),
                         (local_build / 'build_gazebo-classic', build / 'build_gazebo-classic')]:
        link.symlink_to(target, target_is_directory=True)
    if (source / 'test_data').is_dir():
        (input_tree / 'test_data').symlink_to(source / 'test_data', target_is_directory=True)
    if digest(source / HRT_PATH) != HRT_SHA256 or digest(original_binary) != BINARY_SHA256:
        raise RuntimeError('original PX4 inputs changed during compatibility build')
    manifest = {'schema_version': 1, 'simulation_only': True, 'original_source': str(source),
                'builder_sha256': digest(__file__),
                'native_fixture_sha256': digest(resources / 'px4_hrt_race.cpp'),
                'original_hrt_sha256': HRT_SHA256, 'original_binary_sha256': BINARY_SHA256,
                'patch_sha256': digest(patch), 'patched_hrt_sha256': digest(patched),
                'binary_sha256': digest(local_bin / 'px4'), 'input_tree': str(input_tree),
                'original_layer_sha256': digest(build / LAYER_PATH),
                'patched_layer_sha256': digest(layer), 'native_comparison': comparisons,
                'alias_script_sha256': digest(local_bin / 'px4-alias.sh'),
                'compile_args': compile_args, 'link_args': link_args,
                'safety_thresholds_changed': False}
    if mag_record is not None:
        manifest['mag_lpf_interval'] = mag_record
    (output / 'compatibility.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return manifest


def main():
    """Require an explicit supported PX4 installation and new project-local build destination."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--px4-dir', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--mag-lpf-interval', action='store_true',
                        help='Also apply the pinned simulation-only mag sample interval fix')
    args = parser.parse_args()
    resources = Path(__file__).resolve().parents[1] / 'compat'
    if not resources.is_dir():
        from ament_index_python.packages import get_package_share_directory
        resources = Path(get_package_share_directory('easyfly_px4_sitl')) / 'compat'
    result = prepare(args.px4_dir, args.output_dir, resources, args.mag_lpf_interval)
    print(json.dumps({k: result[k] for k in ('input_tree', 'binary_sha256', 'simulation_only')}))


if __name__ == '__main__':
    main()
