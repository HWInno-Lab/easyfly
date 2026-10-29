#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Install and run versioned, offline EasyFly root filesystem bundles as one user."""
import argparse
import contextlib
import fcntl
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import signal
import subprocess
import tarfile
import tempfile
import time


def digest(path):
    """Verify the entire archive before creating any release files."""
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def identity(value):
    """Keep release IDs single, ordinary directory names."""
    if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9._-]{0,63}', value):
        raise ValueError('invalid release ID')
    return value


@contextlib.contextmanager
def locked(prefix, shared=False):
    """Active applications prevent upgrades or uninstall beneath their processes."""
    if prefix.is_symlink():
        raise ValueError('installation prefix must not be a symlink')
    prefix.mkdir(parents=True, exist_ok=True, mode=0o700)
    if prefix.stat().st_uid != os.getuid():
        raise ValueError('installation prefix must belong to the current user')
    path = prefix / '.lock'
    fd = os.open(path, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
    try:
        operation = fcntl.LOCK_SH if shared else fcntl.LOCK_EX
        fcntl.flock(fd, operation | fcntl.LOCK_NB)
        yield
    finally:
        os.close(fd)


def real_directory(path):
    """Reject substituted release or state directories instead of following links."""
    if path.is_symlink():
        raise ValueError('real directory required: ' + str(path))
    path.mkdir(exist_ok=True, mode=0o700)
    if not path.is_dir():
        raise ValueError('directory required: ' + str(path))


def extract(archive_path, destination):
    """Retain container symlinks but never write through them on the host."""
    seen = set()
    with tarfile.open(archive_path) as archive:
        for entry in archive:
            relative = PurePosixPath(entry.name)
            if (relative.is_absolute() or '..' in relative.parts or
                    not relative.parts or relative.parts[0] != 'rootfs' or
                    entry.name in seen or not (entry.isdir() or entry.isfile() or entry.issym())):
                raise ValueError('unsupported archive entry: ' + entry.name)
            seen.add(entry.name)
            target = destination / str(relative)
            for parent in reversed(target.parents):
                if parent == destination or destination in parent.parents:
                    if parent.is_symlink():
                        raise ValueError('archive writes through a symlink')
            target.parent.mkdir(parents=True, exist_ok=True)
            if entry.isdir():
                if target.is_symlink() or target.exists() and not target.is_dir():
                    raise ValueError('archive directory collision')
                target.mkdir(exist_ok=True)
            elif entry.issym():
                # Absolute links refer to the container root when run under bwrap.
                # No later archive member may traverse this host-side link.
                target.symlink_to(entry.linkname)
            else:
                with target.open('xb') as stream:
                    shutil.copyfileobj(archive.extractfile(entry), stream)
                target.chmod(entry.mode & 0o777)


def install(args):
    """Extract into a private staging directory and atomically activate on success."""
    meta = json.loads(args.manifest.read_text())
    release = identity(meta.get('release_id'))
    if meta.get('format') != 1 or digest(args.archive) != meta.get('archive_sha256'):
        raise ValueError('unsupported manifest or archive checksum mismatch')
    prefix = args.prefix.absolute()
    with locked(prefix):
        releases = prefix / 'releases'
        real_directory(releases)
        target = releases / release
        if target.exists() or target.is_symlink():
            raise ValueError('release already exists; existing content is preserved')
        staging = Path(tempfile.mkdtemp(prefix='.install-', dir=releases))
        try:
            extract(args.archive, staging)
            for name in ('rootfs', 'rootfs/opt', 'rootfs/opt/easyfly', 'rootfs/opt/easyfly/bin'):
                directory = staging / name
                if directory.is_symlink() or not directory.is_dir():
                    raise ValueError('entry point parents must be real bundle directories')
            entry = staging / 'rootfs/opt/easyfly/bin/environment'
            if entry.is_symlink() or not entry.is_file() or not os.access(entry, os.X_OK):
                raise ValueError('bundle entry point missing')
            supervisor = entry.with_name('runtime_session.py')
            if supervisor.is_symlink() or not supervisor.is_file():
                raise ValueError('bundle runtime supervisor missing')
            (staging / 'bundle.json').write_text(json.dumps(meta, indent=2) + '\n')
            staging.rename(target)
        finally:
            if staging.exists():
                shutil.rmtree(staging)
        current = prefix / 'current'
        if current.exists() and not current.is_symlink():
            raise ValueError('current is not an activation symlink; release installed but not activated')
        temporary = prefix / '.current-new'
        if temporary.exists() or temporary.is_symlink():
            raise ValueError('activation staging path already exists')
        temporary.symlink_to('releases/' + release)
        temporary.replace(current)
    print(json.dumps({'installed': release, 'state_preserved': True}))


def selected(prefix, release=None):
    """Accept only an owned, direct release beneath this installation."""
    releases = prefix / 'releases'
    if releases.is_symlink():
        raise ValueError('releases must not be a symlink')
    if release is None:
        current = prefix / 'current'
        if not current.is_symlink():
            raise ValueError('no active release')
        link = PurePosixPath(os.readlink(current))
        if len(link.parts) != 2 or link.parts[0] != 'releases':
            raise ValueError('invalid active release binding')
        release = link.parts[1]
    release = identity(release)
    target = releases / release
    if target.is_symlink() or not target.is_dir():
        raise ValueError('release directory missing or substituted')
    if json.loads((target / 'bundle.json').read_text()).get('release_id') != release:
        raise ValueError('release marker mismatch')
    root = target / 'rootfs'
    if root.is_symlink() or not root.is_dir():
        raise ValueError('release root filesystem missing or substituted')
    return target


def process_identity(pid):
    """Bind session metadata to the actual process incarnation, not just its PID."""
    return Path('/proc', str(pid), 'stat').read_text().rsplit(')', 1)[1].split()[19]


def join_session(prefix, release, program, gui):
    """Reopen the same live namespace so background sessions retain their identity."""
    marker = prefix / '.runtime.json'
    deadline = time.monotonic() + 5
    while True:
        try:
            meta = json.loads(marker.read_text())
            if (meta['release'] != release.name or
                    process_identity(meta['pid']) != meta['start_ticks']):
                raise ValueError('live runtime identity mismatch')
            break
        except FileNotFoundError:
            if time.monotonic() >= deadline:
                raise RuntimeError('runtime is starting or closing; try again after it settles')
            time.sleep(0.05)
    if gui and not meta['gui']:
        raise ValueError('active runtime has no X11 binding; close it before starting GUI mode')
    root = Path('/proc', str(meta['pid']), 'root')
    expected = (release / 'rootfs').stat()
    actual = root.stat()
    if (actual.st_dev, actual.st_ino) != (expected.st_dev, expected.st_ino):
        raise ValueError('live runtime root mismatch')
    descriptors = []
    try:
        arguments = ['nsenter', '--preserve-credentials']
        for name, option in [('user', '--user'), ('mnt', '--mount'),
                             ('pid', '--pid'), ('net', '--net')]:
            fd = os.open(f'/proc/{meta["pid"]}/ns/{name}', os.O_RDONLY)
            descriptors.append(fd)
            arguments.append(option + '=/proc/self/fd/' + str(fd))
        fd = os.open(root, os.O_RDONLY | os.O_DIRECTORY)
        descriptors.append(fd)
        arguments.append('--root=/proc/self/fd/' + str(fd))
        cwd_fd = os.open(root / 'home/easyfly', os.O_RDONLY | os.O_DIRECTORY)
        descriptors.append(cwd_fd)
        arguments += ['--wd=/proc/self/fd/' + str(cwd_fd), '--',
                      '/usr/bin/env', '-i', 'HOME=/home/easyfly',
                      'PATH=/usr/local/bin:/usr/bin:/bin', 'LANG=C.UTF-8',
                      'LIBGL_ALWAYS_SOFTWARE=1',
                      'QT_QPA_PLATFORM=' + ('xcb' if meta['gui'] else 'offscreen')]
        if meta['gui']:
            arguments += ['DISPLAY=' + meta['display'], 'XAUTHORITY=/tmp/easyfly.Xauthority']
        arguments += ['/opt/easyfly/bin/environment', *program]
        return subprocess.run(arguments, pass_fds=descriptors).returncode
    finally:
        for fd in descriptors:
            os.close(fd)


def run(args):
    """Use only bundled userspace with separately preserved per-user writable state."""
    prefix = args.prefix.absolute()
    with locked(prefix, shared=True):
        release = selected(prefix)
        state = prefix / 'state'
        real_directory(state)
        for name in ('home', 'runtime'):
            real_directory(state / name)
        # The profile reader intentionally creates only its private leaf directory.
        # A brand-new bundled home needs the conventional parent before first use.
        real_directory(state / 'home' / '.config')
        program = args.command
        if program[:1] == ['--']:
            program = program[1:]
        program = program or ['ros2', 'run', 'easyfly_desktop', 'easyfly_desktop',
                              '--connection-lobby']
        runtime_lock = os.open(prefix / '.runtime.lock', os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
        try:
            fcntl.flock(runtime_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            os.close(runtime_lock)
            raise SystemExit(join_session(prefix, release, program, args.gui))
        bwrap = shutil.which('bwrap')
        if not bwrap:
            raise RuntimeError('bubblewrap is required on the Ubuntu 22.04 amd64 host')
        # A non-root namespace UID lets the existing RViz/QGC sandbox nest normally.
        command = [bwrap, '--unshare-user', '--uid', '1000', '--gid', '1000', '--unshare-pid', '--as-pid-1',
                   '--unshare-net', '--die-with-parent', '--ro-bind', str(release / 'rootfs'), '/',
                   '--dev-bind', '/dev', '/dev', '--proc', '/proc', '--tmpfs', '/tmp',
                   '--bind', str(state / 'home'), '/home/easyfly',
                   '--bind', str(state / 'runtime'), '/var/lib/easyfly',
                   '--clearenv', '--setenv', 'HOME', '/home/easyfly',
                   '--setenv', 'PATH', '/usr/local/bin:/usr/bin:/bin',
                   '--setenv', 'LANG', 'C.UTF-8', '--setenv', 'LIBGL_ALWAYS_SOFTWARE', '1',
                   '--chdir', '/home/easyfly']
        if args.gui:
            display = os.environ.get('DISPLAY', '')
            authority = Path(os.environ.get('XAUTHORITY', str(Path.home() / '.Xauthority')))
            if not display or not authority.is_file() or not Path('/tmp/.X11-unix').is_dir():
                raise RuntimeError('X11 display and a readable XAUTHORITY file are required')
            command += ['--ro-bind', '/tmp/.X11-unix', '/tmp/.X11-unix', '--ro-bind',
                        str(authority), '/tmp/easyfly.Xauthority', '--setenv', 'DISPLAY', display,
                        '--setenv', 'XAUTHORITY', '/tmp/easyfly.Xauthority',
                        '--setenv', 'QT_QPA_PLATFORM', 'xcb']
        else:
            command += ['--setenv', 'QT_QPA_PLATFORM', 'offscreen']
        marker = prefix / '.runtime.json'
        marker.unlink(missing_ok=True)
        with tempfile.TemporaryFile(mode='w+') as info:
            command += ['--info-fd', str(info.fileno()), '--', '/opt/easyfly/bin/environment',
                        '/usr/bin/python3', '/opt/easyfly/bin/runtime_session.py', *program]
            process = subprocess.Popen(command, pass_fds=(info.fileno(),), start_new_session=True)
            meta = None
            previous = {}
            try:
                deadline = time.monotonic() + 5
                while process.poll() is None and time.monotonic() < deadline:
                    info.seek(0)
                    try:
                        identity_record = json.load(info)
                        pid = identity_record['child-pid']
                        meta = {'pid': pid, 'start_ticks': process_identity(pid),
                                'release': release.name, 'gui': args.gui,
                                'display': os.environ.get('DISPLAY', '') if args.gui else ''}
                        break
                    except (ValueError, KeyError, FileNotFoundError):
                        time.sleep(0.02)
                if meta:
                    temporary = prefix / '.runtime-new.json'
                    temporary.unlink(missing_ok=True)
                    with temporary.open('x') as stream:
                        json.dump(meta, stream)
                    temporary.replace(marker)
                    def forward(number, _frame):
                        try:
                            if process_identity(meta['pid']) == meta['start_ticks']:
                                os.kill(meta['pid'], number)
                        except FileNotFoundError:
                            pass
                    for number in (signal.SIGINT, signal.SIGTERM):
                        previous[number] = signal.signal(number, forward)
                elif process.poll() is None:
                    raise RuntimeError('runtime identity was not reported; initial command may still be running')
                result = process.wait()
            finally:
                for number, handler in previous.items():
                    signal.signal(number, handler)
                if process.poll() is not None:
                    marker.unlink(missing_ok=True)
                os.close(runtime_lock)
        raise SystemExit(result)


def uninstall(args):
    """Remove only one marked release; all user data and other releases survive."""
    prefix = args.prefix.absolute()
    with locked(prefix):
        release = selected(prefix, args.release)
        current = prefix / 'current'
        if current.is_symlink() and os.readlink(current) == 'releases/' + release.name:
            current.unlink()
        shutil.rmtree(release)
    print(json.dumps({'uninstalled': args.release, 'state_preserved': True}))


def main():
    """Require explicit installation and removal targets; no system files are changed."""
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    for action in ('install', 'run', 'uninstall'):
        child = sub.add_parser(action)
        child.add_argument('--prefix', type=Path, required=True)
        if action == 'install':
            child.add_argument('--archive', type=Path, required=True)
            child.add_argument('--manifest', type=Path, required=True)
        elif action == 'run':
            child.add_argument('--gui', action='store_true')
            child.add_argument('command', nargs=argparse.REMAINDER)
        else:
            child.add_argument('--release', required=True)
    args = parser.parse_args()
    try:
        {'install': install, 'run': run, 'uninstall': uninstall}[args.action](args)
    except (OSError, ValueError, RuntimeError, KeyError, tarfile.TarError) as error:
        parser.exit(1, str(error) + '\n')


if __name__ == '__main__':
    main()
