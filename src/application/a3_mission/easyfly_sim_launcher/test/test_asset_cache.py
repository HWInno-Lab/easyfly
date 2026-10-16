"""Real local archives/HTTP and process interruption; never start a simulator."""
import copy
from contextlib import contextmanager
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tarfile
import threading
from concurrent.futures import ThreadPoolExecutor
import urllib.request

import pytest

from easyfly_px4_sitl import asset_cache as cache
from easyfly_px4_sitl.scene_catalog import SceneCatalog
from easyfly_px4_sitl.scene_inputs import resolve_scene
from easyfly_sim_launcher.scene_cli import main
from test_scene_catalog import catalog_data, add_model, write_yaml, save


def sha(data):
    return hashlib.sha256(data).hexdigest()


def tar_bytes(entries):
    buffer = io.BytesIO()
    with tarfile.open(fileobj=buffer, mode='w:gz') as archive:
        for name, data, kind in entries:
            entry = tarfile.TarInfo(name)
            entry.type = kind
            if kind in (tarfile.SYMTYPE, tarfile.LNKTYPE):
                entry.linkname = '/outside'
            if kind == tarfile.REGTYPE:
                entry.size = len(data)
            archive.addfile(entry, io.BytesIO(data) if entry.isfile() else None)
    return buffer.getvalue()


@pytest.fixture
def sample(catalog_data, tmp_path):
    directory, asset = add_model(catalog_data, '<sdf><model name="box"/></sdf><!-- model://box -->')
    asset['provenance']['revision'] = 'fixture_v1'
    raw = (directory / 'model.sdf').read_bytes().replace(b'model://', b'file://models/')
    entries = [('fixture/LICENSE', b'local test license', tarfile.REGTYPE),
               ('fixture/models/box/model.config', (directory / 'model.config').read_bytes(), tarfile.REGTYPE),
               ('fixture/models/box/model.sdf', raw, tarfile.REGTYPE)]
    data = tar_bytes(entries)
    archive = tmp_path / 'fixture.tar.gz'
    archive.write_bytes(data)
    asset['archive'] = dict(format='tar.gz', urls=['https://fixture.invalid/pinned.tar.gz'],
        sha256=sha(data), size_bytes=len(data), prefix='fixture', max_members=20,
        max_unpacked_bytes=100000, license=dict(source='LICENSE', sha256=sha(b'local test license')),
        members=[dict(source=name.removeprefix('fixture/'), target=name.removeprefix('fixture/models/'),
                      source_sha256=sha(content), replacements=(
                          [dict(old='file://models/', new='model://', count=1)] if name.endswith('.sdf') else []))
                 for name, content, _ in entries[1:]])
    write_yaml(catalog_data[0]['easyfly_gazebo_models'] / 'manifests/objects.yaml', asset)
    save(catalog_data)
    return asset, archive, tmp_path / 'cache', entries


@contextmanager
def server(data, mode='normal'):
    started, release = threading.Event(), threading.Event()
    requests = []
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args):
            pass

        def do_GET(self):
            requests.append(self.path)
            self.send_response(200)
            self.send_header('Content-Length', str(len(data) + (100 if mode == 'truncated' else 0)))
            self.end_headers()
            if mode == 'pause':
                self.wfile.write(data[:16]); self.wfile.flush(); started.set()
                release.wait(10)
                rest = data[16:]
            else:
                rest = data
            try:
                self.wfile.write(rest)
            except (BrokenPipeError, ConnectionResetError):
                pass
    http = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    worker = threading.Thread(target=http.serve_forever, daemon=True)
    worker.start()
    url = f'http://127.0.0.1:{http.server_port}/archive'
    try:
        yield url, started, release, requests
    finally:
        release.set(); http.shutdown(); http.server_close(); worker.join(5)


def http_opener(url):
    # Reviewed production URLs stay HTTPS. This explicit test transport sends
    # actual bytes over loopback HTTP instead of contacting any remote host.
    return lambda request, timeout: urllib.request.urlopen(url, timeout=timeout)


def assert_unpublished(asset, root):
    location, _ = cache.identity(asset)
    assert not (root / location).exists()
    assert not list((root / '.staging').glob('*'))


def test_local_prepare_and_offline_launch_resolution(sample, catalog_data, monkeypatch):
    asset, archive, root, _ = sample
    monkeypatch.setattr('socket.socket', lambda *a, **k: pytest.fail('offline preparation opened a socket'))
    result = cache.fetch_asset(asset, root, archive_path=archive)
    assert result['state'] == 'prepared'
    expected = Path(result['model_root'])
    # Remove packaged payload: a cache test must not accidentally resolve it.
    shutil.rmtree(catalog_data[0]['easyfly_gazebo_models'] / 'box')
    catalog = SceneCatalog(catalog_data[0], asset_cache_root=root, asset_mode='cache')
    assert catalog.validate('test').as_dict()['resources_complete']
    assert str(expected) in resolve_scene(catalog, 'test', 'base_sim').model_path.split(':')
    assert not save(catalog_data).validate('test').as_dict()['resources_complete']
    before = {str(p): p.stat().st_mtime_ns for p in root.rglob('*')}
    assert cache.fetch_asset(asset, root, offline=True)['state'] == 'already_ready'
    assert before == {str(p): p.stat().st_mtime_ns for p in root.rglob('*')}


def test_missing_cache_is_read_only_and_never_falls_back(sample, catalog_data, monkeypatch):
    _asset, _archive, root, _ = sample
    monkeypatch.setattr('socket.socket', lambda *a, **k: pytest.fail('validation opened a socket'))
    catalog = SceneCatalog(catalog_data[0], asset_cache_root=root, asset_mode='cache')
    assert not catalog.validate('test').as_dict()['resources_complete']
    assert not root.exists()
    assert main(['fetch','test','--offline','--json'], catalog_factory=lambda **k:(catalog,[])) == 2
    assert not root.exists()


def test_actual_http_and_no_network_on_second_fetch(sample):
    asset, archive, root, _ = sample
    with server(archive.read_bytes()) as (url, _s, _r, requests):
        assert cache.fetch_asset(asset, root, opener=http_opener(url))['state'] == 'prepared'
        assert cache.fetch_asset(asset, root, opener=lambda *a, **k: pytest.fail('ready cache downloaded'))['state'] == 'already_ready'
        assert requests == ['/archive']


@pytest.mark.parametrize('mode', ['truncated', 'oversized', 'corrupt'])
def test_bad_http_bytes_never_publish(sample, mode):
    asset, archive, root, _ = sample
    data = archive.read_bytes()
    if mode == 'truncated':
        data = data[:-30]
    elif mode == 'oversized':
        data += b'extra'
    else:
        data = b'x' + data[1:]
    with server(data, mode) as (url, _s, _r, _requests):
        with pytest.raises(cache.CacheError, match='download failed'):
            cache.fetch_asset(asset, root, opener=http_opener(url))
    assert_unpublished(asset, root)


@pytest.mark.parametrize('name,kind', [('../escape', tarfile.REGTYPE),
    ('/absolute', tarfile.REGTYPE), ('fixture/link', tarfile.SYMTYPE),
    ('fixture/hard', tarfile.LNKTYPE), ('fixture/fifo', tarfile.FIFOTYPE),
    ('fixture/models/box/model.sdf', tarfile.REGTYPE), ('other/file', tarfile.REGTYPE)])
def test_untrusted_tar_entries_never_publish(sample, name, kind):
    asset, archive, root, entries = sample
    raw = tar_bytes(entries + [(name, b'bad', kind)])
    archive.write_bytes(raw); asset['archive'].update(sha256=sha(raw), size_bytes=len(raw))
    with pytest.raises(cache.CacheError):
        cache.fetch_asset(asset, root, archive_path=archive)
    assert_unpublished(asset, root)


@pytest.mark.parametrize('changed', ['source_hash','output_hash','count','member_budget','size_budget','missing'])
def test_mapping_and_unpack_limits_are_checked(sample, changed):
    asset, archive, root, _ = sample
    if changed == 'source_hash':
        asset['archive']['members'][0]['source_sha256'] = '0' * 64
    elif changed == 'output_hash':
        asset['files'][0]['sha256'] = '0' * 64
    elif changed == 'count':
        asset['archive']['members'][1]['replacements'][0]['count'] = 2
    elif changed == 'member_budget':
        asset['archive']['max_members'] = 1
    elif changed == 'size_budget':
        asset['archive']['max_unpacked_bytes'] = 1
    else:
        asset['archive']['members'][0]['source'] = 'absent'
    with pytest.raises(cache.CacheError):
        cache.fetch_asset(asset, root, archive_path=archive)
    assert_unpublished(asset, root)


@pytest.mark.parametrize('changed', ['data','receipt','archive','license','extra','directory','symlink','hardlink'])
def test_existing_corrupt_versions_are_rejected_and_preserved(sample, changed):
    asset, archive, root, _ = sample
    model = Path(cache.fetch_asset(asset, root, archive_path=archive)['model_root'])
    if changed in ('data','receipt','archive','license'):
        target = {'data': model/'box/model.sdf','receipt': model.parent/'receipt.json',
                  'archive': model.parent/'source.tar.gz','license': model.parent/'LICENSE'}[changed]
        target.write_bytes(b'changed')
    elif changed == 'extra':
        (model/'box/extra').write_bytes(b'extra')
    elif changed == 'directory':
        (model/'unlocked').mkdir()
    else:
        target = model/'box/model.sdf'; target.unlink()
        original = archive.parent/'outside'; original.write_bytes(b'outside')
        if changed == 'symlink':
            target.symlink_to(original)
        else:
            os.link(original, target)
    with pytest.raises(cache.CacheError):
        cache.fetch_asset(asset, root, archive_path=archive)
    assert model.exists()
    assert not list((root/'.staging').glob('*'))


def test_revision_and_recipe_versions_are_independent(sample):
    asset, archive, root, _ = sample
    first = cache.fetch_asset(asset, root, archive_path=archive)
    revision = copy.deepcopy(asset); revision['provenance']['revision'] = 'fixture_v2'
    second = cache.fetch_asset(revision, root, archive_path=archive)
    recipe = copy.deepcopy(asset); recipe['inventory_note'] = 'separate reviewed fixture recipe'
    third = cache.fetch_asset(recipe, root, archive_path=archive)
    assert len({r['model_root'] for r in [first,second,third]}) == 3
    for value in [asset, revision, recipe]:
        assert cache.validate_cached(value, root).is_dir()


def test_cache_only_asset_needs_no_packaged_payload(sample, catalog_data):
    asset, archive, root, _ = sample
    asset['storage'] = 'cached'
    asset['model_root'] = dict(external='asset_cache', path='models')
    write_yaml(catalog_data[0]['easyfly_gazebo_models']/'manifests/objects.yaml', asset)
    shutil.rmtree(catalog_data[0]['easyfly_gazebo_models']/'box')
    catalog = SceneCatalog(catalog_data[0], asset_cache_root=root)
    assert not catalog.validate('test').as_dict()['resources_complete']
    cache.fetch_asset(asset, root, archive_path=archive)
    assert catalog.validate('test').as_dict()['resources_complete']


@pytest.mark.parametrize('invalid', ['traversal','mapping','http','provenance'])
def test_unreviewed_mapping_is_rejected_before_writes(sample, invalid):
    asset, archive, root, _ = sample
    if invalid == 'traversal':
        asset['archive']['members'][0]['target'] = '../outside'
        asset['files'][0]['path'] = '../outside'
    elif invalid == 'mapping':
        asset['archive']['members'].pop()
    elif invalid == 'http':
        asset['archive']['urls'] = ['http://fixture.invalid/archive']
    else:
        asset['provenance']['status'] = 'unverified'
    with pytest.raises(cache.CacheError):
        cache.fetch_asset(asset, root, archive_path=archive)
    assert not root.exists()


def test_concurrent_fetch_keeps_a_single_winner(sample):
    asset, archive, root, _ = sample
    with server(archive.read_bytes(), 'pause') as (url, started, release, requests):
        with ThreadPoolExecutor(max_workers=1) as pool:
            winner = pool.submit(cache.fetch_asset, asset, root, opener=http_opener(url))
            try:
                assert started.wait(5)
                with pytest.raises(cache.CacheError, match='another fetch'):
                    cache.fetch_asset(asset, root, opener=http_opener(url))
            finally:
                release.set()
            assert winner.result(timeout=5)['state'] == 'prepared'
        assert requests == ['/archive']


def test_atomic_publication_preserves_foreign_empty_target(sample):
    asset, archive, root, _ = sample
    with server(archive.read_bytes(), 'pause') as (url, started, release, _requests):
        with ThreadPoolExecutor(max_workers=1) as pool:
            pending = pool.submit(cache.fetch_asset, asset, root, opener=http_opener(url))
            assert started.wait(5)
            location, _ = cache.identity(asset)
            destination = root/location
            destination.mkdir(parents=True)
            identity = destination.stat().st_ino
            release.set()
            with pytest.raises(FileExistsError):
                pending.result(timeout=5)
    assert destination.stat().st_ino == identity and not list(destination.iterdir())
    assert not list((root/'.staging').glob('*'))


@pytest.mark.parametrize('number', [signal.SIGINT, signal.SIGTERM])
def test_real_cli_interruption_cleans_owned_staging(sample, catalog_data, number, tmp_path):
    asset, archive, root, _ = sample
    shares = {k:str(v) for k,v in catalog_data[0].items()}
    with server(archive.read_bytes(), 'pause') as (url, started, release, _requests):
        code = f'''
from easyfly_px4_sitl import asset_cache
from easyfly_px4_sitl.scene_catalog import SceneCatalog
from easyfly_sim_launcher.scene_cli import main
import urllib.request
original = asset_cache._download
asset_cache._download = lambda spec, target, opener=None: original(spec, target, lambda req, timeout: urllib.request.urlopen({url!r}, timeout=timeout))
catalog = SceneCatalog({shares!r}, asset_cache_root={str(root)!r})
raise SystemExit(main(['fetch','test','--json'],catalog_factory=lambda **kw:(catalog,[])))
'''
        process = subprocess.Popen([sys.executable,'-c',code],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        try:
            assert started.wait(10)
            process.send_signal(number)
            stdout, stderr = process.communicate(timeout=10)
            assert process.returncode == 2, (stdout,stderr)
            assert 'interrupted' in stdout
        finally:
            release.set()
            if process.poll() is None:
                process.kill(); process.wait()
        assert_unpublished(asset, root)


def test_cache_location_and_cli_scope(sample, catalog_data, tmp_path, monkeypatch, capsys):
    asset, archive, root, _ = sample
    catalog = SceneCatalog(catalog_data[0], asset_cache_root=root)
    monkeypatch.setattr('socket.socket', lambda *a, **k: pytest.fail('local CLI opened a socket'))
    assert main(['fetch','test','--archive',str(archive),'--json'],catalog_factory=lambda **k:(catalog,[])) == 0
    assert json.loads(capsys.readouterr().out)['runtime_verified'] is False
    assert main(['fetch','unknown','--json'],catalog_factory=lambda **k:(catalog,[])) == 2
    with pytest.raises(cache.CacheError, match='outside source'):
        cache.cache_root(catalog_data[0]['easyfly_gazebo_models']/'cache', catalog_data[0].values())
    monkeypatch.setenv('XDG_CACHE_HOME',str(tmp_path/'xdg'))
    assert cache.default_root() == tmp_path/'xdg/easyfly/assets'
