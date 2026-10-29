# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Exercise installation failure atomicity, hostile archives, locks and data retention."""
import fcntl
import hashlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile

import pytest

TOOL = Path(__file__).with_name('manage_bundle.py')


def bundle(tmp_path, release, extra=()):
    """Create a packaging fixture, not a simulated application acceptance result."""
    archive = tmp_path / (release + '.tar.gz')
    with tarfile.open(archive, 'w:gz') as stream:
        entry = tarfile.TarInfo('rootfs/opt/easyfly/bin/environment')
        data = b'#!/bin/sh\nexec "$@"\n'
        entry.size = len(data)
        entry.mode = 0o755
        stream.addfile(entry, io.BytesIO(data))
        entry = tarfile.TarInfo('rootfs/opt/easyfly/bin/runtime_session.py')
        data = TOOL.with_name('runtime_session.py').read_bytes()
        entry.size = len(data)
        stream.addfile(entry, io.BytesIO(data))
        for entry, data in extra:
            stream.addfile(entry, io.BytesIO(data) if data is not None else None)
    manifest = tmp_path / (release + '.json')
    manifest.write_text(json.dumps({'format': 1, 'release_id': release,
                                   'archive_sha256': hashlib.sha256(archive.read_bytes()).hexdigest()}))
    return archive, manifest


def invoke(action, prefix, *args):
    """Use the public CLI so parsing, ownership and activation are all exercised."""
    return subprocess.run([sys.executable, str(TOOL), action, '--prefix', str(prefix), *args],
                          capture_output=True, text=True, timeout=10)


def install(prefix, inputs):
    """Submit one explicit local archive and its manifest."""
    archive, manifest = inputs
    return invoke('install', prefix, '--archive', str(archive), '--manifest', str(manifest))


def test_upgrade_and_uninstall_preserve_user_data(tmp_path):
    """Two independent release payloads activate without changing existing state."""
    prefix = tmp_path / 'installation'
    assert install(prefix, bundle(tmp_path, 'first')).returncode == 0
    state = prefix / 'state/home/.config/easyfly'
    state.mkdir(parents=True)
    config = state / 'connections.json'
    original = b'{"schema":1,"user_note":"preserve me"}\n'
    config.write_bytes(original)
    assert install(prefix, bundle(tmp_path, 'second')).returncode == 0
    assert os.readlink(prefix / 'current') == 'releases/second'
    assert config.read_bytes() == original
    assert invoke('uninstall', prefix, '--release', 'first').returncode == 0
    assert (prefix / 'current').resolve().name == 'second'
    assert invoke('uninstall', prefix, '--release', 'second').returncode == 0
    assert not (prefix / 'current').is_symlink()
    assert config.read_bytes() == original


def test_corrupt_archive_never_replaces_active_release(tmp_path):
    """Checksums fail before extraction and activation."""
    prefix = tmp_path / 'installation'
    assert install(prefix, bundle(tmp_path, 'first')).returncode == 0
    inputs = bundle(tmp_path, 'second')
    with inputs[0].open('ab') as stream:
        stream.write(b'corruption')
    assert install(prefix, inputs).returncode != 0
    assert os.readlink(prefix / 'current') == 'releases/first'
    assert not (prefix / 'releases/second').exists()


@pytest.mark.parametrize('kind', ['parent', 'absolute', 'symlink_parent'])
def test_archive_cannot_write_outside_staging(tmp_path, kind):
    """Even an attacker-recomputed archive hash does not allow traversal writes."""
    outside = tmp_path / 'outside'
    entries = []
    if kind == 'symlink_parent':
        link = tarfile.TarInfo('rootfs/escape')
        link.type = tarfile.SYMTYPE
        link.linkname = str(tmp_path)
        entries.append((link, None))
        name = 'rootfs/escape/outside'
    else:
        name = '../../outside' if kind == 'parent' else str(outside)
    entry = tarfile.TarInfo(name)
    entry.size = 3
    entries.append((entry, b'bad'))
    prefix = tmp_path / 'installation'
    result = install(prefix, bundle(tmp_path, kind, entries))
    assert result.returncode != 0
    assert not outside.exists()
    assert not (prefix / 'current').is_symlink()
    assert not list((prefix / 'releases').iterdir())


def test_active_process_lock_blocks_upgrade_and_uninstall(tmp_path):
    """The runtime's shared lock is respected by both mutating operations."""
    prefix = tmp_path / 'installation'
    assert install(prefix, bundle(tmp_path, 'first')).returncode == 0
    inputs = bundle(tmp_path, 'second')
    with (prefix / '.lock').open('r+') as lock:
        fcntl.flock(lock, fcntl.LOCK_SH)
        assert install(prefix, inputs).returncode != 0
        assert invoke('uninstall', prefix, '--release', 'first').returncode != 0
    assert os.readlink(prefix / 'current') == 'releases/first'
    assert install(prefix, inputs).returncode == 0


def test_bundle_root_cannot_alias_an_existing_host_tree(tmp_path):
    """A root symlink must not pass entry point validation through a host directory."""
    host = tmp_path / 'host'
    entry = host / 'opt/easyfly/bin/environment'
    entry.parent.mkdir(parents=True)
    entry.write_text('#!/bin/sh\nexit 0\n')
    entry.chmod(0o755)
    archive = tmp_path / 'alias.tar.gz'
    with tarfile.open(archive, 'w:gz') as stream:
        link = tarfile.TarInfo('rootfs')
        link.type = tarfile.SYMTYPE
        link.linkname = str(host)
        stream.addfile(link)
    manifest = tmp_path / 'alias.json'
    manifest.write_text(json.dumps({'format': 1, 'release_id': 'alias',
                                   'archive_sha256': hashlib.sha256(archive.read_bytes()).hexdigest()}))
    prefix = tmp_path / 'installation'
    assert install(prefix, (archive, manifest)).returncode != 0
    assert not (prefix / 'current').is_symlink()
    assert entry.read_text() == '#!/bin/sh\nexit 0\n'


def test_packer_preserves_runtime_sources_and_excludes_mutable_state(tmp_path):
    """Round-trip real packer output, including hard links and mount point directories."""
    root = tmp_path / 'prepared'
    entry = root / 'opt/easyfly/bin/environment'
    entry.parent.mkdir(parents=True)
    entry.write_text('#!/bin/sh\nexec "$@"\n')
    entry.chmod(0o755)
    entry.with_name('runtime_session.py').write_bytes(
        TOOL.with_name('runtime_session.py').read_bytes())
    source = root / 'opt/easyfly/source/example.cpp'
    source.parent.mkdir()
    source.write_text('source bytes\n')
    os.link(source, source.with_name('hardlinked.cpp'))
    for relative in ('opt/easyfly/build/a.o', 'var/cache/apt/cache', 'home/user/config'):
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('excluded')
    (root / 'opt/easyfly/build-record.json').write_text('{"source":"recorded"}')
    build_extra = root / 'opt/easyfly/build-acceptance/private-object'
    build_extra.parent.mkdir()
    build_extra.write_text('not a runtime file')
    (root / 'lib').symlink_to('usr/lib')
    archive, manifest = tmp_path / 'packed.tar.gz', tmp_path / 'packed.json'
    result = subprocess.run([sys.executable, str(TOOL.with_name('pack_bundle.py')),
                             '--rootfs', str(root), '--release-id', 'packed',
                             '--archive', str(archive), '--manifest', str(manifest)],
                            capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr
    prefix = tmp_path / 'installation'
    assert install(prefix, (archive, manifest)).returncode == 0
    installed = prefix / 'current/rootfs'
    assert (installed / 'opt/easyfly/source/example.cpp').read_bytes() == source.read_bytes()
    assert (installed / 'opt/easyfly/source/hardlinked.cpp').read_bytes() == source.read_bytes()
    assert os.readlink(installed / 'lib') == 'usr/lib'
    assert (installed / 'opt/easyfly/build-record.json').read_text() == '{"source":"recorded"}'
    assert not (installed / 'opt/easyfly/build-acceptance').exists()
    for relative in ('dev', 'proc', 'tmp', 'home/easyfly', 'var/lib/easyfly'):
        assert (installed / relative).is_dir()
    for relative in ('opt/easyfly/build', 'var/cache', 'home/user'):
        assert not (installed / relative).exists()
