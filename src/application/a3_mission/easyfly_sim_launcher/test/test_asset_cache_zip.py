"""Fuel-style rootless ZIPs: real bytes, atomic publication and hostile entries."""
import copy
import io
from pathlib import Path
import stat
import struct
import warnings
import zipfile

import pytest

from easyfly_px4_sitl import asset_cache as cache
from test_asset_cache import sha, assert_unpublished, sample, server, http_opener  # noqa: F401
from test_scene_catalog import write_yaml, save, catalog_data  # noqa: F401


def zip_bytes(entries):
    out = io.BytesIO()
    with warnings.catch_warnings(), zipfile.ZipFile(out, 'w', zipfile.ZIP_DEFLATED) as archive:
        warnings.simplefilter('ignore', UserWarning)
        for name, data, mode in entries:
            entry = zipfile.ZipInfo(name)
            entry.create_system = 3
            entry.external_attr = mode << 16
            entry.compress_type = zipfile.ZIP_DEFLATED
            archive.writestr(entry, data)
    return out.getvalue()


@pytest.fixture
def fuel(sample):
    asset, path, root, tar_entries = sample
    entries = [(name.removeprefix('fixture/'), data, stat.S_IFREG | 0o644)
               for name, data, _ in tar_entries[1:]]
    data = zip_bytes(entries)
    path = path.with_suffix('.zip')
    path.write_bytes(data)
    notice = '{"owner":"fixture","license_name":"test license","version":2}\n'
    asset['archive'].update(format='zip', prefix='', size_bytes=len(data), sha256=sha(data),
                            license=dict(notice=notice, sha256=sha(notice.encode())))
    return asset, path, root, entries


def rewrite(fuel, entries):
    asset, path, _, _ = fuel
    data = zip_bytes(entries)
    path.write_bytes(data)
    asset['archive'].update(sha256=sha(data), size_bytes=len(data))


def test_rootless_zip_external_notice_and_offline_reuse(fuel, catalog_data, monkeypatch):
    asset, path, root, _ = fuel
    write_yaml(catalog_data[0]['easyfly_gazebo_models'] / 'manifests/objects.yaml', asset)
    save(catalog_data)  # The actual installed schema accepts this explicit recipe.
    monkeypatch.setattr('socket.socket', lambda *a, **k: pytest.fail('local ZIP opened socket'))
    result = cache.fetch_asset(asset, root, archive_path=path)
    model_root = Path(result['model_root'])
    assert (model_root / 'box/model.sdf').read_text().endswith('<!-- model://box -->')
    assert (model_root.parent / 'source.zip').read_bytes() == path.read_bytes()
    assert not (model_root.parent / 'source.tar.gz').exists()
    assert (model_root.parent / 'LICENSE').read_text() == asset['archive']['license']['notice']
    assert cache.fetch_asset(asset, root, offline=True)['state'] == 'already_ready'
    (model_root.parent / 'LICENSE').write_text('changed')
    with pytest.raises(cache.CacheError, match='license fingerprint'):
        cache.fetch_asset(asset, root, offline=True)


@pytest.mark.parametrize('name,mode', [('../outside', stat.S_IFREG),
    ('/absolute', stat.S_IFREG), ('models/../escape', stat.S_IFREG),
    ('models\\escape', stat.S_IFREG), ('models/%2e', stat.S_IFREG),
    ('models/link', stat.S_IFLNK), ('models/fifo', stat.S_IFIFO),
    ('models/device', stat.S_IFCHR), ('models/box/model.sdf', stat.S_IFREG),
    ('models/fake_dir', stat.S_IFDIR), ('models/fake_file/', stat.S_IFREG)])
def test_hostile_zip_never_publishes(fuel, name, mode):
    asset, path, root, entries = fuel
    rewrite(fuel, entries + [(name, b'/outside', mode | 0o644)])
    with pytest.raises(cache.CacheError):
        cache.fetch_asset(asset, root, archive_path=path)
    assert_unpublished(asset, root)


@pytest.mark.parametrize('change', ['members', 'size', 'missing', 'prefix', 'hash', 'crc', 'encrypted', 'deflate', 'compression'])
def test_zip_limits_missing_members_and_integrity(fuel, change):
    asset, path, root, entries = fuel
    if change == 'members':
        asset['archive']['max_members'] = 1
    elif change == 'size':
        asset['archive']['max_unpacked_bytes'] = 1
    elif change == 'missing':
        rewrite(fuel, entries[:1])
    elif change == 'prefix':
        asset['archive']['prefix'] = 'other'
    elif change == 'hash':
        asset['archive']['members'][0]['source_sha256'] = '0' * 64
    else:
        raw = bytearray(path.read_bytes())
        if change in ('encrypted', 'compression'):
            for signature, offset in [(b'PK\x03\x04', 6), (b'PK\x01\x02', 8)]:
                if change == 'compression':
                    offset += 2
                start = raw.index(signature) + offset
                value = struct.unpack_from('<H', raw, start)[0] | 1 if change == 'encrypted' else 99
                struct.pack_into('<H', raw, start, value)
        elif change == 'crc':
            start = raw.index(b'PK\x01\x02') + 16
            raw[start] ^= 1
        else:
            # Invalid DEFLATE stream, independent of ZIP header validity.
            start = 30 + len(entries[0][0].encode())
            raw[start] = 0xff
        path.write_bytes(raw)
        asset['archive'].update(sha256=sha(raw), size_bytes=len(raw))
    with pytest.raises(cache.CacheError):
        cache.fetch_asset(asset, root, archive_path=path)
    assert_unpublished(asset, root)


@pytest.mark.parametrize('change', ['notice_hash', 'both', 'neither', 'too_big', 'tar_prefix', 'format'])
def test_invalid_license_or_format_rejected_before_cache_creation(fuel, change):
    asset, path, root, _ = fuel
    asset = copy.deepcopy(asset)
    spec = asset['archive']
    if change == 'notice_hash':
        spec['license']['notice'] += 'changed'
    elif change == 'both':
        spec['license']['source'] = 'LICENSE'
    elif change == 'neither':
        del spec['license']['notice']
    elif change == 'too_big':
        spec['license']['notice'] = 'x' * 65537
        spec['license']['sha256'] = sha(spec['license']['notice'].encode())
    elif change == 'tar_prefix':
        spec['format'] = 'tar.gz'
    else:
        spec['format'] = 'rar'
    with pytest.raises(cache.CacheError):
        cache.fetch_asset(asset, root, archive_path=path)
    assert not root.exists()


def test_zip_download_uses_the_same_locked_atomic_pipeline(fuel):
    asset, path, root, _ = fuel
    with server(path.read_bytes()) as (url, _started, _release, requests):
        assert cache.fetch_asset(asset, root, opener=http_opener(url))['state'] == 'prepared'
        assert cache.fetch_asset(asset, root, offline=True)['state'] == 'already_ready'
        assert requests == ['/archive']


def test_prefixed_zip_preserves_an_archive_license(fuel):
    asset, path, root, entries = fuel
    license_bytes = b'original archive license notice'
    rewrite(fuel, [('snapshot/' + n, data, mode) for n, data, mode in entries]
            + [('snapshot/LICENSE', license_bytes, stat.S_IFREG | 0o644)])
    asset['archive'].update(prefix='snapshot', license=dict(source='LICENSE', sha256=sha(license_bytes)))
    model_root = Path(cache.fetch_asset(asset, root, archive_path=path)['model_root'])
    assert (model_root.parent / 'LICENSE').read_bytes() == license_bytes
    assert cache.validate_cached(asset, root) == model_root
