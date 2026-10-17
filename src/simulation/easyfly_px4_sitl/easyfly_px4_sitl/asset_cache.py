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
import zipfile
import zlib


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


def archive_name(spec):
    if spec['format'] not in ('tar.gz', 'zip'):
        raise CacheError('unsupported locked archive format')
    return 'source.' + spec['format']


def archive_sources(spec):
    """Flat, independently locked inputs; the primary source keeps its old layout."""
    result = [(None, spec)]
    seen = set()
    extras = spec.get('supplements', [])
    if not isinstance(extras, list) or len(extras) > 16:
        raise CacheError('at most sixteen supplemental archives required')
    for item in extras:
        if not isinstance(item, dict) or set(item) != {'id', 'archive'}:
            raise CacheError('supplement requires exactly id and archive')
        name, source = item['id'], item['archive']
        if relative(name).name != name or name in seen:
            raise CacheError('unique single-component supplemental archive ID required')
        if not isinstance(source, dict) or 'supplements' in source:
            raise CacheError('supplemental archives cannot nest')
        seen.add(name)
        result.append((name, source))
    return result


def _source_directory(stage, source_id):
    return stage if source_id is None else stage / 'sources' / source_id


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
    sources = archive_sources(archive)
    targets = [m['target'] for _, source in sources for m in source['members']]
    inventory = [f['path'] for f in asset['files']]
    if len(set(targets)) != len(targets) or len(set(inventory)) != len(inventory) or set(targets) != set(inventory):
        raise CacheError('archive mapping must cover the locked asset inventory exactly once')
    for _, source in sources:
        for member in source['members']:
            relative(member['source']); relative(member['target'])
        archive_name(source)
        if source['prefix']:
            relative(source['prefix'])
        elif source['format'] != 'zip':
            raise CacheError('tar archive requires a locked prefix')
        license_spec = source['license']
        if set(license_spec) == {'source', 'sha256'}:
            relative(license_spec['source'])
        elif set(license_spec) == {'notice', 'sha256'}:
            notice = license_spec['notice']
            if (not isinstance(notice, str) or not notice or len(notice.encode()) > 65536
                    or hashlib.sha256(notice.encode()).hexdigest() != license_spec['sha256']):
                raise CacheError('locked external license notice fingerprint/size mismatch')
        else:
            raise CacheError('exactly one archive license source or external notice required')
        for url in source['urls']:
            parsed = urlsplit(url)
            if parsed.scheme != 'https' or not parsed.hostname or parsed.username or parsed.password or parsed.fragment:
                raise CacheError('reviewed HTTPS archive URL required')
    lock = hashlib.sha256(canonical(asset)).hexdigest()
    receipt = {
        'schema_version': 1, 'asset_set_id': asset['asset_set_id'], 'revision': revision,
        'lock_sha256': lock, 'archive_sha256': archive['sha256'],
        'archive_size_bytes': archive['size_bytes']}
    if len(sources) > 1:
        receipt['supplement_archives'] = [dict(id=name, format=spec['format'],
                                             sha256=spec['sha256'], size_bytes=spec['size_bytes'],
                                             license_sha256=spec['license']['sha256'])
                                        for name, spec in sources[1:]]
    return Path(asset['asset_set_id']) / revision / lock, receipt


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
    archive = asset['archive']
    expected = {'receipt.json'} | {'models/' + f['path'] for f in asset['files']}
    sources = archive_sources(archive)
    for name, spec in sources:
        parent = _source_directory(directory, name)
        expected.update(str((parent / n).relative_to(directory)) for n in ('LICENSE', archive_name(spec)))
    if _files(directory, expected) != expected:
        raise CacheError(f'asset cache file inventory differs: {directory}; existing version is preserved')
    expected_receipt = canonical(receipt) + b'\n'
    if (directory / 'receipt.json').stat().st_size != len(expected_receipt) or \
            (directory / 'receipt.json').read_bytes() != expected_receipt:
        raise CacheError('asset cache receipt differs from the selected lock')
    for name, spec in sources:
        parent = _source_directory(directory, name)
        source = parent / archive_name(spec)
        if source.stat().st_size != spec['size_bytes'] or digest(source) != spec['sha256']:
            raise CacheError('cached source archive fingerprint mismatch; existing version is preserved')
        if digest(parent / 'LICENSE') != spec['license']['sha256']:
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


def _entries(spec, stage):
    """Read entries without extractall; the caller checks budgets before reading data."""
    path = stage / archive_name(spec)
    if spec['format'] == 'tar.gz':
        with tarfile.open(path, mode='r|gz') as archive:
            for member in archive:
                if not (member.isfile() or member.isdir()):
                    raise CacheError(f'unsupported archive entry: {member.name}')
                yield member.name, member.size, member.isdir(), lambda: archive.extractfile(member).read()
    else:
        with zipfile.ZipFile(path) as archive:
            for member in archive.infolist():
                kind = stat.S_IFMT(member.external_attr >> 16)
                if (kind not in (0, stat.S_IFREG, stat.S_IFDIR) or member.flag_bits & 1
                        or member.compress_type not in (zipfile.ZIP_STORED, zipfile.ZIP_DEFLATED)
                        or (kind == stat.S_IFDIR and not member.is_dir())
                        or (kind == stat.S_IFREG and member.is_dir())):
                    raise CacheError(f'unsupported archive entry: {member.filename}')
                yield member.filename, member.file_size, member.is_dir(), lambda: archive.read(member)


def _unpack_source(spec, stage, source_directory, outputs):
    mapping = {}
    for item in spec['members']:
        mapping.setdefault(item['source'], []).append(item)
    if 'source' in spec['license']:
        license_record = dict(source=spec['license']['source'], target=None,
                              source_sha256=spec['license']['sha256'], replacements=[])
        mapping.setdefault(license_record['source'], []).append(license_record)
    seen, written = set(), set()
    if 'notice' in spec['license']:
        # This notice is reviewed catalog evidence, explicitly NOT a file in the ZIP.
        (source_directory / 'LICENSE').write_bytes(spec['license']['notice'].encode())
        written.add(None)
    total = 0
    entries = _entries(spec, source_directory)
    try:
        for count, (raw_name, size, is_directory, read) in enumerate(entries, 1):
            # Tar directory names conventionally have a terminal slash.
            name = str(relative(raw_name.rstrip('/') if is_directory else raw_name))
            if name in seen:
                raise CacheError(f'duplicate archive entry: {raw_name}')
            seen.add(name)
            total += size
            if count > spec['max_members'] or total > spec['max_unpacked_bytes']:
                raise CacheError('archive unpacked size/member budget exceeded')
            prefix = spec['prefix']
            if prefix and name != prefix and not name.startswith(prefix + '/'):
                raise CacheError('archive entry outside locked prefix')
            source = name[len(prefix) + 1:] if prefix else name
            if source not in mapping:
                continue
            if is_directory:
                raise CacheError('mapped archive entry must be a regular file')
            data = read()
            if len(data) != size:
                raise CacheError('archive member size differs from header')
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
                path = stage / 'models' / target if target is not None else source_directory / 'LICENSE'
                path.parent.mkdir(parents=True, exist_ok=True)
                with path.open('xb') as stream:
                    stream.write(result)
                written.add(target)
    except (tarfile.TarError, zipfile.BadZipFile, zlib.error, RuntimeError, EOFError) as error:
        raise CacheError(f'invalid locked archive: {error}') from error
    finally:
        entries.close()
    if written != {m['target'] for m in spec['members']} | {None}:
        raise CacheError('archive is missing mapped inventory/license members')


def _unpack(asset, stage):
    outputs = {f['path']: f['sha256'] for f in asset['files']}
    for name, spec in archive_sources(asset['archive']):
        _unpack_source(spec, stage, _source_directory(stage, name), outputs)


def fetch_asset(asset, root, *, archive_path=None, supplement_paths=None, offline=False, opener=None):
    """Publish one complete version atomically; never repair/overwrite existing versions."""
    root = cache_root(root)
    location, receipt = identity(asset)
    destination = root / location
    if destination.exists() or destination.is_symlink():
        models = validate_cached(asset, root)
        return dict(asset_set_id=asset['asset_set_id'], state='already_ready', model_root=str(models), lock_sha256=receipt['lock_sha256'])
    if offline:
        raise CacheError(f'asset cache missing: {asset["asset_set_id"]}; offline mode cannot prepare it')
    sources = archive_sources(asset['archive'])
    paths = {} if supplement_paths is None else dict(supplement_paths)
    expected_paths = {name for name, _ in sources[1:]}
    if (archive_path is not None and set(paths) != expected_paths) or (
            archive_path is None and supplement_paths is not None):
        raise CacheError('local preparation requires primary and every supplemental archive; no implicit download')
    with _locked(root, asset['asset_set_id']):
        if destination.exists() or destination.is_symlink():
            models = validate_cached(asset, root)
            return dict(asset_set_id=asset['asset_set_id'], state='already_ready', model_root=str(models), lock_sha256=receipt['lock_sha256'])
        staging = root / '.staging'
        _mkdir(staging, root)
        with tempfile.TemporaryDirectory(prefix=asset['asset_set_id'] + '-', dir=staging) as temp:
            stage = Path(temp)
            origins = []
            for name, spec in sources:
                parent = _source_directory(stage, name)
                parent.mkdir(parents=True, exist_ok=True)
                archive = parent / archive_name(spec)
                local_path = archive_path if name is None else paths.get(name)
                if local_path is not None:
                    local = Path(local_path)
                    if local.stat().st_size != spec['size_bytes'] or digest(local) != spec['sha256']:
                        raise CacheError('local archive size or SHA256 mismatch')
                    shutil.copyfile(local, archive)
                    origin = str(local.resolve(strict=True))
                else:
                    origin = _download(spec, archive, opener)
                # Check copied bytes too: local archives may change during a copy.
                if archive.stat().st_size != spec['size_bytes'] or digest(archive) != spec['sha256']:
                    raise CacheError('staged archive size or SHA256 mismatch')
                origins.append(dict(id=name, source=origin))
            _unpack(asset, stage)
            (stage / 'receipt.json').write_bytes(canonical(receipt) + b'\n')
            _validate_payload(asset, stage, receipt)
            _mkdir(destination.parent, root)
            # Same-filesystem rename publishes all data and receipt in one step.
            # Existing versions are checked under the asset lock and never replaced.
            _publish(stage, destination)
        models = validate_cached(asset, root)
        result = dict(asset_set_id=asset['asset_set_id'], state='prepared', source=origins[0]['source'],
                      model_root=str(models), lock_sha256=receipt['lock_sha256'])
        if len(origins) > 1:
            result['supplement_sources'] = origins[1:]
        return result
