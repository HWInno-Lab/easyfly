# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Explicit, locked asset preparation. Validation and launch never download."""

from contextlib import contextmanager
import ctypes
import fcntl
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import stat
import tarfile
import tempfile
import time
import urllib.request
from urllib.parse import urlsplit


class CacheError(ValueError):
    pass


def relative(value):
    if not isinstance(value, str) or not value or any(c in value for c in ('\\', '%', ':', '\x00')):
        raise CacheError(f'invalid archive/cache path: {value!r}')
    path = PurePosixPath(value)
    if path.is_absolute() or any(p in ('', '.', '..') for p in value.split('/')):
        raise CacheError(f'noncanonical archive/cache path: {value!r}')
    return path


def digest(path):
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    with os.fdopen(descriptor, 'rb') as stream:
        if not stat.S_ISREG(os.fstat(stream.fileno()).st_mode):
            raise CacheError(f'regular file required: {path}')
        result = hashlib.sha256()
        for data in iter(lambda: stream.read(1024 * 1024), b''):
            result.update(data)
        return result.hexdigest()


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), ensure_ascii=False).encode()


def default_root():
    xdg = os.environ.get('XDG_CACHE_HOME', '').strip()
    base = Path(xdg).expanduser() if xdg else Path.home() / '.cache'
    if not base.is_absolute():
        raise CacheError('XDG_CACHE_HOME must be absolute')
    return base / 'easyfly/assets'


def cache_root(value=None, protected_roots=()):
    path = Path(value).expanduser() if value is not None else default_root()
    if not path.is_absolute():
        raise CacheError('asset cache root must be absolute')
    path = path.resolve()
    protected = [Path(p).resolve() for p in protected_roots]
    # Locate workspace boundaries in both ordinary and symlink installations.
    for origin in [Path(__file__).resolve(), *protected]:
        for ancestor in origin.parents:
            if (ancestor / 'colcon_defaults.yaml').is_file():
                protected.extend(ancestor / name for name in ('src', 'build', 'install'))
            if ancestor.name in ('build', 'install'):
                protected.append(ancestor)
    if any(path == p or path.is_relative_to(p) for p in protected):
        raise CacheError('asset cache must be outside source packages and build/install trees')
    if any((parent / 'package.xml').is_file() for parent in (path, *path.parents)):
        raise CacheError('asset cache must be outside ROS packages')
    return path


def identity(asset):
    if 'archive' not in asset:
        raise CacheError(f'asset has no reviewed archive: {asset["asset_set_id"]}')
    archive = asset['archive']
    revision = asset['provenance']['revision']
    for part in (asset['asset_set_id'], revision):
        if relative(part).name != part:
            raise CacheError('asset ID and revision must be single path components')
    if asset['provenance']['status'] != 'recorded' or not asset['provenance']['declared_license']:
        raise CacheError('recorded asset provenance and license required before fetch')
    targets = [m['target'] for m in archive['members']]
    inventory = [f['path'] for f in asset['files']]
    if len(set(targets)) != len(targets) or len(set(inventory)) != len(inventory) or set(targets) != set(inventory):
        raise CacheError('archive mapping must cover the locked asset inventory exactly once')
    for member in archive['members']:
        relative(member['source']); relative(member['target'])
    relative(archive['prefix']); relative(archive['license']['source'])
    for url in archive['urls']:
        parsed = urlsplit(url)
        if parsed.scheme != 'https' or not parsed.hostname or parsed.username or parsed.password or parsed.fragment:
            raise CacheError('reviewed HTTPS archive URL required')
    lock = hashlib.sha256(canonical(asset)).hexdigest()
    return Path(asset['asset_set_id']) / revision / lock, {
        'schema_version': 1, 'asset_set_id': asset['asset_set_id'], 'revision': revision,
        'lock_sha256': lock, 'archive_sha256': archive['sha256'],
        'archive_size_bytes': archive['size_bytes']}


def _directory(path):
    if path.is_symlink() or not path.is_dir():
        raise CacheError(f'real cache directory required: {path}')


def _mkdir(path, root):
    if path == root:
        path.mkdir(parents=True, exist_ok=True)
        _directory(path)
        return
    _mkdir(path.parent, root)
    path.mkdir(exist_ok=True)
    _directory(path)


def _files(root, expected):
    _directory(root)
    found = set()
    expected_dirs = {p.as_posix() for name in expected for p in PurePosixPath(name).parents if str(p) != '.'}
    for directory, dirs, files in os.walk(root, followlinks=False):
        for name in dirs:
            path = Path(directory) / name
            _directory(path)
            if path.relative_to(root).as_posix() not in expected_dirs:
                raise CacheError(f'unlocked cache directory: {path}')
        for name in files:
            path = Path(directory) / name
            mode = path.lstat()
            if not stat.S_ISREG(mode.st_mode) or mode.st_nlink != 1:
                raise CacheError(f'regular unlinked cache data required: {path}')
            found.add(path.relative_to(root).as_posix())
    return found


def _validate_payload(asset, directory, receipt):
    expected = {'receipt.json', 'LICENSE', 'source.tar.gz'} | {'models/' + f['path'] for f in asset['files']}
    if _files(directory, expected) != expected:
        raise CacheError(f'asset cache file inventory differs: {directory}; existing version is preserved')
    expected_receipt = canonical(receipt) + b'\n'
    if (directory / 'receipt.json').stat().st_size != len(expected_receipt) or \
            (directory / 'receipt.json').read_bytes() != expected_receipt:
        raise CacheError('asset cache receipt differs from the selected lock')
    archive = asset['archive']
    if (directory / 'source.tar.gz').stat().st_size != archive['size_bytes'] or digest(directory / 'source.tar.gz') != archive['sha256']:
        raise CacheError('cached source archive fingerprint mismatch; existing version is preserved')
    if digest(directory / 'LICENSE') != archive['license']['sha256']:
        raise CacheError('cached original license fingerprint mismatch')
    for item in asset['files']:
        if digest(directory / 'models' / item['path']) != item['sha256']:
            raise CacheError(f'cached asset fingerprint mismatch: {item["path"]}; existing version is preserved')
    return directory / 'models'


def validate_cached(asset, root):
    """Rehash published bytes; a receipt alone never makes a cache usable."""
    location, receipt = identity(asset)
    directory = Path(root) / location
    if not directory.exists():
        raise CacheError(f'asset cache missing: {asset["asset_set_id"]}; run scene_catalog fetch <scene>')
    current = Path(root)
    _directory(current)
    for part in location.parts:
        current /= part
        _directory(current)
    return _validate_payload(asset, directory, receipt)


@contextmanager
def _locked(root, asset_id):
    _mkdir(root, root)
    marker = root / 'COLCON_IGNORE'
    if not marker.exists():
        with marker.open('xb'):
            pass
    if marker.is_symlink() or not marker.is_file():
        raise CacheError('regular COLCON_IGNORE marker required at cache root')
    locks = root / '.locks'
    _mkdir(locks, root)
    descriptor = os.open(locks / (asset_id + '.lock'), os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    try:
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise CacheError(f'another fetch holds this asset lock: {asset_id}') from error
        yield
    finally:
        os.close(descriptor)


class _HttpsRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, request, fp, code, msg, headers, newurl):
        if urlsplit(newurl).scheme != 'https':
            raise CacheError('archive redirect must retain HTTPS')
        return super().redirect_request(request, fp, code, msg, headers, newurl)


def _publish(stage, destination):
    """Linux atomic no-replace, including a foreign empty target created mid-fetch."""
    library = ctypes.CDLL(None, use_errno=True)
    try:
        rename = library.renameat2
    except AttributeError as error:
        raise CacheError('atomic no-replace publication requires Linux renameat2') from error
    rename.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
    rename.restype = ctypes.c_int
    if rename(-100, os.fsencode(stage), -100, os.fsencode(destination), 1) != 0:
        number = ctypes.get_errno()
        raise OSError(number, os.strerror(number), str(destination))


def _download(spec, target, opener=None):
    open_url = opener or urllib.request.build_opener(_HttpsRedirect()).open
    failures = []
    for url in spec['urls']:
        try:
            started = time.monotonic()
            with open_url(urllib.request.Request(url, headers={'User-Agent': 'EasyFly-assets/1'}), timeout=20) as response, target.open('xb') as stream:
                received = 0
                while True:
                    data = response.read(min(1024 * 1024, spec['size_bytes'] - received + 1))
                    if not data:
                        break
                    received += len(data)
                    if received > spec['size_bytes'] or time.monotonic() - started > 600:
                        raise CacheError('archive exceeds the locked size or download time budget')
                    stream.write(data)
                stream.flush()
                os.fsync(stream.fileno())
            if received != spec['size_bytes'] or digest(target) != spec['sha256']:
                raise CacheError('downloaded archive size or SHA256 mismatch')
            return url
        except (OSError, ValueError) as error:
            target.unlink(missing_ok=True)
            failures.append(str(error))
    raise CacheError('archive download failed: ' + '; '.join(failures))


def _unpack(asset, stage):
    spec = asset['archive']
    mapping = {}
    for item in spec['members']:
        mapping.setdefault(item['source'], []).append(item)
    license_record = dict(source=spec['license']['source'], target=None,
                          source_sha256=spec['license']['sha256'], replacements=[])
    mapping.setdefault(license_record['source'], []).append(license_record)
    outputs = {f['path']: f['sha256'] for f in asset['files']}
    seen, written = set(), set()
    total = 0
    with tarfile.open(stage / 'source.tar.gz', mode='r|gz') as archive:
        for count, member in enumerate(archive, 1):
            # Tar directory names conventionally have a terminal slash.
            name = str(relative(member.name.rstrip('/')))
            if name in seen or not (member.isfile() or member.isdir()):
                raise CacheError(f'unsupported/duplicate archive entry: {member.name}')
            seen.add(name)
            total += member.size
            if count > spec['max_members'] or total > spec['max_unpacked_bytes']:
                raise CacheError('archive unpacked size/member budget exceeded')
            prefix = spec['prefix']
            if name != prefix and not name.startswith(prefix + '/'):
                raise CacheError('archive entry outside locked prefix')
            source = name[len(prefix) + 1:]
            if source not in mapping:
                continue
            if not member.isfile():
                raise CacheError('mapped archive entry must be a regular file')
            data = archive.extractfile(member).read()
            for item in mapping[source]:
                if hashlib.sha256(data).hexdigest() != item['source_sha256']:
                    raise CacheError(f'archive member fingerprint mismatch: {source}')
                result = data
                for change in item.get('replacements', []):
                    before, after = change['old'].encode(), change['new'].encode()
                    if result.count(before) != change['count']:
                        raise CacheError(f'literal relocation count differs: {source}')
                    result = result.replace(before, after)
                target = item['target']
                expected = outputs[target] if target is not None else spec['license']['sha256']
                if hashlib.sha256(result).hexdigest() != expected:
                    raise CacheError(f'prepared asset fingerprint mismatch: {target}')
                path = stage / 'models' / target if target is not None else stage / 'LICENSE'
                path.parent.mkdir(parents=True, exist_ok=True)
                with path.open('xb') as stream:
                    stream.write(result)
                written.add(target)
    if written != set(outputs) | {None}:
        raise CacheError('archive is missing mapped inventory/license members')


def fetch_asset(asset, root, *, archive_path=None, offline=False, opener=None):
    """Publish one complete version atomically; never repair/overwrite existing versions."""
    root = cache_root(root)
    location, receipt = identity(asset)
    destination = root / location
    if destination.exists() or destination.is_symlink():
        models = validate_cached(asset, root)
        return dict(asset_set_id=asset['asset_set_id'], state='already_ready', model_root=str(models), lock_sha256=receipt['lock_sha256'])
    if offline:
        raise CacheError(f'asset cache missing: {asset["asset_set_id"]}; offline mode cannot prepare it')
    with _locked(root, asset['asset_set_id']):
        if destination.exists() or destination.is_symlink():
            models = validate_cached(asset, root)
            return dict(asset_set_id=asset['asset_set_id'], state='already_ready', model_root=str(models), lock_sha256=receipt['lock_sha256'])
        staging = root / '.staging'
        _mkdir(staging, root)
        with tempfile.TemporaryDirectory(prefix=asset['asset_set_id'] + '-', dir=staging) as temp:
            stage = Path(temp)
            archive = stage / 'source.tar.gz'
            source = None
            if archive_path is not None:
                local = Path(archive_path)
                if local.stat().st_size != asset['archive']['size_bytes'] or digest(local) != asset['archive']['sha256']:
                    raise CacheError('local archive size or SHA256 mismatch')
                shutil.copyfile(local, archive)
                source = str(local.resolve(strict=True))
            else:
                source = _download(asset['archive'], archive, opener)
            # Check copied bytes as well: local archives may change during a copy.
            if archive.stat().st_size != asset['archive']['size_bytes'] or digest(archive) != asset['archive']['sha256']:
                raise CacheError('staged archive size or SHA256 mismatch')
            _unpack(asset, stage)
            (stage / 'receipt.json').write_bytes(canonical(receipt) + b'\n')
            _validate_payload(asset, stage, receipt)
            _mkdir(destination.parent, root)
            # Same-filesystem rename publishes all data and receipt in one step.
            # Existing versions are checked under the asset lock and never replaced.
            _publish(stage, destination)
        models = validate_cached(asset, root)
        return dict(asset_set_id=asset['asset_set_id'], state='prepared', source=source,
                    model_root=str(models), lock_sha256=receipt['lock_sha256'])
