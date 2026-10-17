"""Atomic multi-origin data recipes; no simulator, ROS node or flight goal."""
import copy
import io
import json
from pathlib import Path
import zipfile

import pytest

from easyfly_px4_sitl import asset_cache as cache
from easyfly_sim_launcher.scene_cli import main
from easyfly_px4_sitl.scene_catalog import SceneCatalog
from test_asset_cache import sha, sample, assert_unpublished, server, http_opener  # noqa: F401
from test_scene_catalog import catalog_data, write_yaml, save  # noqa: F401


@pytest.fixture
def composed(sample):
    asset, primary, root, _ = sample
    raw = io.BytesIO()
    with zipfile.ZipFile(raw, 'w', zipfile.ZIP_DEFLATED) as z:
        z.writestr('texture.bin', b'original donor pixels')
    donor = primary.with_name('donor.zip')
    donor.write_bytes(raw.getvalue())
    notice = 'Independent donor license notice; fixture source.'
    target = 'box/materials/textures/donor.bin'
    spec = dict(format='zip', urls=['https://fixture.invalid/donor.zip'],
                sha256=sha(raw.getvalue()), size_bytes=len(raw.getvalue()), prefix='',
                max_members=1, max_unpacked_bytes=1000,
                license=dict(notice=notice, sha256=sha(notice.encode())),
                members=[dict(source='texture.bin', target=target,
                              source_sha256=sha(b'original donor pixels'), replacements=[])])
    asset['archive']['supplements'] = [dict(id='donor', archive=spec)]
    asset['files'].append(dict(path=target, sha256=sha(b'original donor pixels')))
    return asset, primary, root, donor


def prepare(value):
    asset, primary, root, donor = value
    return cache.fetch_asset(asset, root, archive_path=primary, supplement_paths={'donor': donor})


def test_local_composition_preserves_each_source_and_notice(composed, catalog_data, monkeypatch):
    asset, primary, root, donor = composed
    write_yaml(catalog_data[0]['easyfly_gazebo_models'] / 'manifests/objects.yaml', asset)
    save(catalog_data)
    monkeypatch.setattr('socket.socket', lambda *a, **k: pytest.fail('local composition opened socket'))
    result = prepare(composed)
    path = Path(result['model_root']).parent
    assert (path / 'source.tar.gz').read_bytes() == primary.read_bytes()
    assert (path / 'sources/donor/source.zip').read_bytes() == donor.read_bytes()
    assert (path / 'LICENSE').read_bytes() == b'local test license'
    assert (path / 'sources/donor/LICENSE').read_text() == asset['archive']['supplements'][0]['archive']['license']['notice']
    assert (path / 'models/box/materials/textures/donor.bin').read_bytes() == b'original donor pixels'
    receipt = json.loads((path / 'receipt.json').read_text())
    assert receipt['supplement_archives'][0]['sha256'] == sha(donor.read_bytes())
    assert cache.fetch_asset(asset, root, offline=True)['state'] == 'already_ready'


@pytest.mark.parametrize('change', ['archive', 'notice', 'data', 'receipt', 'extra'])
def test_corrupt_composed_cache_is_rejected_and_preserved(composed, change):
    asset, _, root, _ = composed
    path = Path(prepare(composed)['model_root']).parent
    target = {'archive': 'sources/donor/source.zip', 'notice': 'sources/donor/LICENSE',
              'data': 'models/box/materials/textures/donor.bin', 'receipt': 'receipt.json',
              'extra': 'sources/donor/unlocked'}[change]
    (path / target).write_bytes(b'changed')
    with pytest.raises(cache.CacheError):
        cache.fetch_asset(asset, root, offline=True)
    assert path.exists() and (path / target).read_bytes() == b'changed'


@pytest.mark.parametrize('change', ['duplicate_id', 'traversal_id', 'nested', 'duplicate_target',
                                   'missing_mapping', 'license_hash', 'too_many'])
def test_invalid_composition_rejected_before_writes(composed, change):
    asset, _, root, _ = composed
    extra = asset['archive']['supplements'][0]
    if change == 'duplicate_id':
        asset['archive']['supplements'].append(copy.deepcopy(extra))
    elif change == 'traversal_id':
        extra['id'] = '../escape'
    elif change == 'nested':
        extra['archive']['supplements'] = []
    elif change == 'duplicate_target':
        extra['archive']['members'][0]['target'] = asset['files'][0]['path']
    elif change == 'missing_mapping':
        extra['archive']['members'] = []
    elif change == 'license_hash':
        extra['archive']['license']['notice'] += 'changed'
    else:
        asset['archive']['supplements'] *= 17
    with pytest.raises(cache.CacheError):
        prepare(composed)
    assert not root.exists()


@pytest.mark.parametrize('change', ['bytes', 'member_hash', 'member_budget', 'size_budget', 'missing_member'])
def test_bad_second_source_never_publishes_partial_asset(composed, change):
    asset, _, root, donor = composed
    spec = asset['archive']['supplements'][0]['archive']
    if change == 'bytes':
        donor.write_bytes(b'bad')
    elif change == 'member_hash':
        spec['members'][0]['source_sha256'] = '0' * 64
    elif change == 'member_budget':
        spec['max_members'] = 0
    elif change == 'size_budget':
        spec['max_unpacked_bytes'] = 1
    else:
        spec['members'][0]['source'] = 'missing'
    with pytest.raises(cache.CacheError):
        prepare(composed)
    assert_unpublished(asset, root)


@pytest.mark.parametrize('paths', [None, {}, {'other': '/missing'}])
def test_local_input_never_downloads_an_omitted_supplement(composed, paths, monkeypatch):
    asset, primary, root, _ = composed
    monkeypatch.setattr('socket.socket', lambda *a, **k: pytest.fail('omitted local source downloaded'))
    with pytest.raises(cache.CacheError, match='no implicit download'):
        cache.fetch_asset(asset, root, archive_path=primary, supplement_paths=paths)
    assert not root.exists()


def test_two_real_http_transfers_publish_one_version(composed):
    asset, primary, root, donor = composed
    with server(primary.read_bytes()) as (url1, _, _, requests1):
        with server(donor.read_bytes()) as (url2, _, _, requests2):
            def opener(request, timeout):
                url = url2 if request.full_url.endswith('donor.zip') else url1
                return http_opener(url)(request, timeout)
            assert cache.fetch_asset(asset, root, opener=opener)['state'] == 'prepared'
            assert requests1 == ['/archive'] and requests2 == ['/archive']


def test_cli_single_archive_is_ambiguous_for_composed_asset(composed, catalog_data):
    asset, primary, root, _ = composed
    write_yaml(catalog_data[0]['easyfly_gazebo_models'] / 'manifests/objects.yaml', asset)
    save(catalog_data)
    catalog = SceneCatalog(catalog_data[0], asset_cache_root=root)
    assert main(['fetch', 'test', '--archive', str(primary), '--json'],
                catalog_factory=lambda **k: (catalog, [])) == 2
    assert not root.exists()
