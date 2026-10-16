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

"""Fail closed on missing or changed simulation-only PX4 HRT compatibility inputs."""

import hashlib
import json
import os
from pathlib import Path
import shlex

HRT_PATH = 'platforms/posix/src/px4/common/drv_hrt.cpp'
HRT_SHA256 = 'eb826fde8a1eaae3eef98c8f37c686bc21f62ad061d9cc86c8d2fa2a1ea1ede2'
ORIGINAL_BINARY_SHA256 = '9484a69221d73465f621b80662b51d72cd4739b216c9a4526dbaef35ba67c9a4'
PATCHED_HRT_SHA256 = '2e3a5a6faf2fec42d29cbc4a8be4eb3922ac1140b2ad72d8c5cb4bbb2c74614c'
PATCH_SHA256 = 'be22a4dfcf1fcb242906d893687052ccb19a0cdd48cf205e2f7e6bfa39bef8ab'


def digest(path):
    """Check the actual current bytes rather than trusting a manifest's cached claim."""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_manifest(path):
    """Require a bounded, plain, existing manifest before creating any simulator process."""
    if path.is_symlink() or not path.is_file() or path.stat().st_size > 65536:
        raise ValueError('bounded non-symlink compatibility manifest required')
    value = json.loads(path.read_text())
    if not isinstance(value, dict):
        raise ValueError('compatibility manifest must be an object')
    return value


def validate_tree(tree):
    """Verify local patched inputs, original provenance and all module aliases."""
    tree = tree.resolve(strict=True)
    value = read_manifest(tree.parent / 'compatibility.json')
    if value.get('schema_version') != 1 or value.get('simulation_only') is not True or \
            value.get('safety_thresholds_changed') is not False or \
            value.get('input_tree') != str(tree):
        raise ValueError('compatibility identity or simulation-only policy mismatch')
    source = Path(value['original_source']).resolve(strict=True)
    original_build = source / 'build/px4_sitl_default'
    binary = tree / 'build/px4_sitl_default/bin/px4'
    checks = [(source / HRT_PATH, HRT_SHA256),
              (original_build / 'bin/px4', ORIGINAL_BINARY_SHA256),
              (tree.parent / 'drv_hrt.cpp', PATCHED_HRT_SHA256),
              (binary, value['binary_sha256']),
              (binary.parent / 'px4-alias.sh', value['alias_script_sha256'])]
    if value.get('original_hrt_sha256') != HRT_SHA256 or \
            value.get('original_binary_sha256') != ORIGINAL_BINARY_SHA256 or \
            value.get('patched_hrt_sha256') != PATCHED_HRT_SHA256 or \
            value.get('patch_sha256') != PATCH_SHA256:
        raise ValueError('unreviewed PX4 source, binary or compatibility patch')
    if binary.is_symlink() or not os.access(binary, os.X_OK):
        raise ValueError('real executable compatibility binary required')
    if any(digest(path) != expected for path, expected in checks):
        raise ValueError('PX4 compatibility input fingerprint mismatch')
    if 'mag_lpf_interval' in value:
        from easyfly_px4_sitl.mag_compatibility import validate_mag
        validate_mag(value, source, tree.parent)
    for variant, expected_corrupt in [('original', True), ('patched', False)]:
        runs = value['native_comparison'][variant]['runs']
        if len(runs) != 10 or any(v.get('queue_corrupt') is not expected_corrupt for v in runs):
            raise ValueError('native HRT comparison missing or inconsistent')
    for link, target in [(tree / 'Tools', source / 'Tools'),
                         (tree / 'build/px4_sitl_default/etc', original_build / 'etc'),
                         (tree / 'build/px4_sitl_default/build_gazebo-classic',
                          original_build / 'build_gazebo-classic')]:
        if not link.is_symlink() or link.resolve(strict=True) != target.resolve(strict=True):
            raise ValueError('compatibility shared input binding changed')
    original_aliases = sorted(v.name for v in (original_build / 'bin').glob('px4-*')
                              if v.name != 'px4-alias.sh')
    local_aliases = sorted(v.name for v in binary.parent.glob('px4-*')
                           if v.name != 'px4-alias.sh')
    if not original_aliases or local_aliases != original_aliases or any(
            not (binary.parent / name).is_symlink() or
            (binary.parent / name).resolve(strict=True) != binary for name in local_aliases):
        raise ValueError('module alias could launch an unpatched PX4 binary')
    return value


def validate_staged_tree(directory):
    """Preserve the private rootfs and exact wrapper without silently swapping sources."""
    value = read_manifest(directory / 'easyfly_staging.json')
    source = Path(value['source']).resolve(strict=True)
    validate_tree(source)
    build = directory / 'build/px4_sitl_default'
    expected = {'schema_version': 1, 'independent_rootfs': True, 'source': str(source),
                'staging': str(directory), 'rootfs': str(build / 'rootfs'),
                'wrapper': str(build / 'bin/px4'),
                'binary': str(source / 'build/px4_sitl_default/bin/px4'),
                'rcS': str(source / 'build/px4_sitl_default/etc/init.d-posix/rcS'),
                'plugins': str(source / 'build/px4_sitl_default/build_gazebo-classic'),
                'manifest': str(directory / 'easyfly_staging.json')}
    if value != expected or directory == source or source in directory.parents or \
            directory in source.parents:
        raise ValueError('independent staging source bindings changed')
    for path in [directory / 'build', build, build / 'bin', build / 'rootfs']:
        if path.is_symlink() or not path.is_dir():
            raise ValueError('private real staging directories required')
    wrapper = build / 'bin/px4'
    arguments = [expected['binary'], '-w', expected['rootfs'],
                 '-s', str(build / 'etc/init.d-posix/rcS')]
    command = ' '.join(shlex.quote(v) for v in arguments)
    if wrapper.is_symlink() or not os.access(wrapper, os.X_OK) or \
            wrapper.read_text() != '#!/bin/sh\nexec ' + command + ' "$@"\n':
        raise ValueError('private staging wrapper changed')
    for link, target in [(directory / 'Tools', source / 'Tools'),
                         (build / 'etc', source / 'build/px4_sitl_default/etc'),
                         (build / 'rootfs/etc', source / 'build/px4_sitl_default/etc'),
                         (build / 'build_gazebo-classic',
                          source / 'build/px4_sitl_default/build_gazebo-classic')]:
        if not link.is_symlink() or link.resolve(strict=True) != target.resolve(strict=True):
            raise ValueError('private staging shared input changed')
    return directory


def select_px4_directory(requested=None, compatibility_root=None):
    """Select the reviewed build and reject unpatched staging or changed inputs."""
    try:
        if requested:
            directory = Path(requested).expanduser().resolve(strict=True)
            if (directory / 'easyfly_staging.json').exists():
                return validate_staged_tree(directory)
            if (directory.parent / 'compatibility.json').exists():
                validate_tree(directory)
                return directory
        if compatibility_root is None:
            configured = os.environ.get('EASYFLY_PX4_HRT_COMPAT_DIR', '').strip()
            if configured:
                compatibility_root = Path(configured).expanduser()
            else:
                from ament_index_python.packages import get_package_prefix
                prefix = Path(get_package_prefix('easyfly_px4_sitl'))
                compatibility_root = prefix.parents[1] / 'build/easyfly_px4_hrt_compat'
        tree = Path(compatibility_root).resolve(strict=True) / 'px4-input'
        manifest = validate_tree(tree)
        if requested and directory != Path(manifest['original_source']).resolve(strict=True):
            raise ValueError('requested PX4 installation differs from reviewed original source')
        return tree
    except (OSError, KeyError, TypeError, ValueError) as error:
        raise RuntimeError(
            'SITL refused: reviewed project-local PX4 HRT compatibility build required; '
            'run easyfly_px4_sitl prepare_px4_hrt or check EASYFLY_PX4_HRT_COMPAT_DIR; '
            'no unpatched fallback. Detail: ' + str(error)) from error


def verified_features(directory):
    """Derive qualification features from actual checked inputs, never a path or flag."""
    tree = select_px4_directory(directory)
    if (tree / 'easyfly_staging.json').exists():
        tree = Path(read_manifest(tree / 'easyfly_staging.json')['source'])
    value = validate_tree(tree)
    return frozenset({'mag_lpf_interval_v1'} if 'mag_lpf_interval' in value else ())
