# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Preview trust and stale-cache rejection without starting any simulator."""
import hashlib
import json
import struct

import pytest

from easyfly_connection_manager.scene_previews import asset_identity, get_scene_preview
from easyfly_px4_sitl.scene_catalog import SceneCatalog


@pytest.fixture
def sample(tmp_path):
    share, source = tmp_path / 'share', tmp_path / 'source'
    share.mkdir()
    source.mkdir()
    previews = source / 'previews'
    previews.mkdir()
    (share / 'previews').symlink_to(previews, target_is_directory=True)
    world = source / 'world.sdf'
    world.write_bytes(b'<sdf><world name="sample"/></sdf>')
    (share / 'world.sdf').symlink_to(world)
    image = previews / 'sample.png'
    image.write_bytes(b'\x89PNG\r\n\x1a\n' + struct.pack('>I', 13) + b'IHDR' +
                      struct.pack('>II', 1024, 768) + b'\x08\x02\x00\x00\x00' + b'12345678')
    catalog = SceneCatalog.__new__(SceneCatalog)
    catalog.packages = {'easyfly_gazebo_worlds': share}
    catalog.external = {}
    catalog.trusted_sources = (source,)
    catalog.assets = {'asset': {'asset_set_id': 'asset', 'sha256': 'a' * 64}}
    scene = dict(id='sample', asset_sets=['asset'], world=dict(
        package='easyfly_gazebo_worlds', path='world.sdf',
        sha256=hashlib.sha256(world.read_bytes()).hexdigest()))
    row = dict(scene_id='sample', world_sha256=scene['world']['sha256'],
               asset_identity=asset_identity(catalog, scene), file='sample.png',
               kind='native_topview_cutaway', caption='俯视剖切图（2.5米以上结构已裁切）',
               image_sha256=hashlib.sha256(image.read_bytes()).hexdigest())
    manifest = dict(schema_version=1, scenes={'sample': row})
    (previews / 'manifest.json').write_text(json.dumps(manifest))
    return catalog, scene, previews, world, manifest


def test_trusted_source_links_provide_native_capture_semantics(sample):
    catalog, scene, previews, _, _ = sample
    result = get_scene_preview(catalog, scene)
    assert result['path'] == str(previews / 'sample.png')
    assert result['kind'] == 'native_topview_cutaway'
    assert '裁切' in result['caption']


@pytest.mark.parametrize('change', ['world', 'asset', 'image', 'manifest', 'escape', 'dimensions'])
def test_stale_or_unsafe_previews_are_not_displayed(sample, change):
    catalog, scene, previews, world, manifest = sample
    if change == 'world':
        world.write_bytes(b'changed world')
    elif change == 'asset':
        catalog.assets['asset']['sha256'] = 'b' * 64
    elif change == 'image':
        (previews / 'sample.png').write_bytes(b'changed image')
    elif change == 'manifest':
        manifest['schema_version'] = 999
    elif change == 'escape':
        manifest['scenes']['sample']['file'] = '../sample.png'
    else:
        image = previews / 'sample.png'
        data = image.read_bytes()[:16] + struct.pack('>II', 2 ** 31, 2 ** 31) + image.read_bytes()[24:]
        image.write_bytes(data)
        manifest['scenes']['sample']['image_sha256'] = hashlib.sha256(data).hexdigest()
    (previews / 'manifest.json').write_text(json.dumps(manifest))
    assert get_scene_preview(catalog, scene) is None


def test_untrusted_source_link_and_missing_manifest_are_not_read(sample):
    catalog, scene, previews, _, _ = sample
    catalog.trusted_sources = ()
    assert get_scene_preview(catalog, scene) is None
    catalog.trusted_sources = (previews.parent,)
    (previews / 'manifest.json').unlink()
    assert get_scene_preview(catalog, scene) is None
