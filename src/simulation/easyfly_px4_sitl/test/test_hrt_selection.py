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

"""Exercise fail-closed simulation selection using inert temporary files, never flight inputs."""

import importlib.util
import json
from pathlib import Path
import shlex

import pytest

SOURCE = Path(__file__).parents[1] / 'easyfly_px4_sitl/hrt_compatibility.py'
SPEC = importlib.util.spec_from_file_location('hrt_selection', SOURCE)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


@pytest.fixture
def fixture(tmp_path, monkeypatch):
    """Use explicitly mocked fingerprints solely to test structural predicates."""
    source, root = tmp_path / 'original', tmp_path / 'compatibility'
    tree = root / 'px4-input'
    original_build, local_build = [p / 'build/px4_sitl_default' for p in (source, tree)]
    for directory in (source / 'Tools', original_build / 'bin', original_build / 'etc',
                      original_build / 'build_gazebo-classic', local_build / 'bin'):
        directory.mkdir(parents=True)
    original_hrt = source / MODULE.HRT_PATH
    original_hrt.parent.mkdir(parents=True)
    original_hrt.write_text('inert original timer')
    (root / 'drv_hrt.cpp').write_text('inert patched timer')
    for build, content in [(original_build, b'\x7fELFinert original'),
                           (local_build, b'\x7fELFinert patched')]:
        (build / 'bin/px4').write_bytes(content)
        (build / 'bin/px4').chmod(0o755)
        (build / 'bin/px4-battery_simulator').symlink_to('px4')
        (build / 'bin/px4-alias.sh').write_text('inert alias script')
    for link, target in [(tree / 'Tools', source / 'Tools'),
                         (local_build / 'etc', original_build / 'etc'),
                         (local_build / 'build_gazebo-classic',
                          original_build / 'build_gazebo-classic')]:
        link.symlink_to(target, target_is_directory=True)
    for name, path in [('HRT_SHA256', original_hrt),
                       ('ORIGINAL_BINARY_SHA256', original_build / 'bin/px4'),
                       ('PATCHED_HRT_SHA256', root / 'drv_hrt.cpp')]:
        monkeypatch.setattr(MODULE, name, MODULE.digest(path))
    value = {'schema_version': 1, 'simulation_only': True, 'safety_thresholds_changed': False,
             'input_tree': str(tree), 'original_source': str(source),
             'original_hrt_sha256': MODULE.HRT_SHA256,
             'original_binary_sha256': MODULE.ORIGINAL_BINARY_SHA256,
             'patched_hrt_sha256': MODULE.PATCHED_HRT_SHA256, 'patch_sha256': MODULE.PATCH_SHA256,
             'binary_sha256': MODULE.digest(local_build / 'bin/px4'),
             'alias_script_sha256': MODULE.digest(local_build / 'bin/px4-alias.sh'),
             'native_comparison': {k: {'runs': [{'queue_corrupt': v}] * 10}
                                   for k, v in [('original', True), ('patched', False)]}}
    (root / 'compatibility.json').write_text(json.dumps(value))
    return source, root, tree, value


def test_default_and_explicit_original_use_reviewed_build(fixture):
    """Selecting the original installation does not accidentally execute its old timer."""
    source, root, tree, _ = fixture
    assert MODULE.select_px4_directory(compatibility_root=root) == tree
    assert MODULE.select_px4_directory(source, root) == tree
    assert MODULE.select_px4_directory(tree, root) == tree


def test_missing_default_has_no_legacy_fallback(tmp_path):
    """Fail before any process starts instead of selecting an old home-directory install."""
    with pytest.raises(RuntimeError, match='no unpatched fallback'):
        MODULE.select_px4_directory(compatibility_root=tmp_path / 'missing')


def test_mag_feature_requires_current_checked_bytes(fixture, monkeypatch):
    """Use inert mocked fingerprints to exercise optional feature/tamper predicates."""
    from easyfly_px4_sitl import mag_compatibility as mag
    source, root, tree, value = fixture
    files = [(source / mag.MAG_PATH, 'MAG_SHA256', 'inert original mag'),
             (source / mag.ALPHA_PATH, 'ALPHA_SHA256', 'inert filter header'),
             (source / 'build/px4_sitl_default' / mag.EKF_ARCHIVE,
              'EKF_SHA256', 'inert original archive'),
             (root / 'mag_control.cpp', 'PATCHED_MAG_SHA256', 'inert patched mag')]
    for path, key, content in files:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
        monkeypatch.setattr(mag, key, mag.digest(path))
    archive = root / 'libmodules__ekf2.a'
    archive.write_text('inert patched archive')
    assert MODULE.verified_features(tree) == frozenset()
    value['mag_lpf_interval'] = {
        'original_source_sha256': mag.MAG_SHA256,
        'patched_source_sha256': mag.PATCHED_MAG_SHA256,
        'patch_sha256': mag.MAG_PATCH_SHA256, 'alpha_filter_sha256': mag.ALPHA_SHA256,
        'original_archive_sha256': mag.EKF_SHA256,
        'patched_archive_sha256': mag.digest(archive), 'time_constant_seconds': 0.09,
        'sample_clock': 'native magnetometer timestamp_sample',
        'native_comparison': {name: {'step_140ms': {'10000': 0.7712, '70000': v}}
                              for name, v in [('original', 0.19), ('patched', 0.6836)]}}
    (root / 'compatibility.json').write_text(json.dumps(value))
    assert MODULE.verified_features(tree) == {'mag_lpf_interval_v1'}
    (root / 'mag_control.cpp').write_text('changed since preparation')
    with pytest.raises(RuntimeError, match='fingerprint mismatch'):
        MODULE.verified_features(tree)


@pytest.mark.parametrize('changed', [
    'binary', 'original', 'timer', 'alias', 'policy',
    'comparison', 'source_identity', 'other_installation'])
def test_changed_input_is_rejected(fixture, changed, tmp_path):
    """Changed code/provenance or a module alias bypass cannot be accepted as a valid build."""
    source, root, tree, value = fixture
    requested = tree
    if changed == 'binary':
        (tree / 'build/px4_sitl_default/bin/px4').write_bytes(b'wrong binary')
    elif changed == 'original':
        (source / 'build/px4_sitl_default/bin/px4').write_bytes(b'changed original')
    elif changed == 'timer':
        (root / 'drv_hrt.cpp').write_text('changed timer')
    elif changed == 'alias':
        alias = tree / 'build/px4_sitl_default/bin/px4-battery_simulator'
        alias.unlink()
        alias.symlink_to(source / 'build/px4_sitl_default/bin/px4')
    elif changed == 'policy':
        value['simulation_only'] = False
    elif changed == 'comparison':
        value['native_comparison']['patched']['runs'][0] = {'queue_corrupt': True}
    elif changed == 'source_identity':
        value['input_tree'] = str(source)
    else:
        requested = tmp_path / 'other-installation'
        requested.mkdir()
    (root / 'compatibility.json').write_text(json.dumps(value))
    with pytest.raises(RuntimeError, match='SITL refused'):
        MODULE.select_px4_directory(requested, root)


def test_private_staging_wrapper_is_preserved_and_checked(fixture, tmp_path):
    """Keep the independently staged rootfs; refuse changed wrapper commands."""
    _, root, source, _ = fixture
    directory = tmp_path / 'private-staging'
    build = directory / 'build/px4_sitl_default'
    for path in [build / 'bin', build / 'rootfs']:
        path.mkdir(parents=True)
    for link, target in [(directory / 'Tools', source / 'Tools'),
                         (build / 'etc', source / 'build/px4_sitl_default/etc'),
                         (build / 'rootfs/etc', source / 'build/px4_sitl_default/etc'),
                         (build / 'build_gazebo-classic',
                          source / 'build/px4_sitl_default/build_gazebo-classic')]:
        link.symlink_to(target, target_is_directory=True)
    (source / 'build/px4_sitl_default/etc/init.d-posix').mkdir()
    (source / 'build/px4_sitl_default/etc/init.d-posix/rcS').write_text('inert boot script')
    value = {'schema_version': 1, 'independent_rootfs': True, 'source': str(source),
             'staging': str(directory), 'rootfs': str(build / 'rootfs'),
             'wrapper': str(build / 'bin/px4'),
             'binary': str(source / 'build/px4_sitl_default/bin/px4'),
             'rcS': str(source / 'build/px4_sitl_default/etc/init.d-posix/rcS'),
             'plugins': str(source / 'build/px4_sitl_default/build_gazebo-classic'),
             'manifest': str(directory / 'easyfly_staging.json')}
    (directory / 'easyfly_staging.json').write_text(json.dumps(value))
    arguments = [value['binary'], '-w', value['rootfs'],
                 '-s', str(build / 'etc/init.d-posix/rcS')]
    command = ' '.join(shlex.quote(v) for v in arguments)
    wrapper = build / 'bin/px4'
    wrapper.write_text('#!/bin/sh\nexec ' + command + ' "$@"\n')
    wrapper.chmod(0o755)
    assert MODULE.select_px4_directory(directory, root) == directory
    wrapper.write_text('#!/bin/sh\nexec /unpatched/px4 "$@"\n')
    with pytest.raises(RuntimeError, match='wrapper changed'):
        MODULE.select_px4_directory(directory, root)
