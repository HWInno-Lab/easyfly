# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Read fingerprinted native scene previews; never starts rendering or downloads."""
import hashlib
import json
import re
import struct


def asset_identity(catalog, scene):
    """Identity is portable metadata from the catalog's pinned asset manifests."""
    return {name: hashlib.sha256(json.dumps(catalog.assets[name], sort_keys=True,
                                          separators=(',', ':')).encode()).hexdigest()
            for name in scene['asset_sets']}


def get_scene_preview(catalog, scene):
    """Return a verified installed PNG and its exact capture semantics, or None.

    SceneCatalog owns path trust, including ament's declared source links. Images
    only describe the pinned static scene; runtime resources are validated again
    at simulation startup and this preview is never used as navigation geometry.
    """
    try:
        location = dict(package='easyfly_gazebo_worlds', path='previews/manifest.json')
        manifest_path = catalog.resolve(location)
        if manifest_path.stat().st_size > 262144:
            return None
        manifest = json.loads(manifest_path.read_text())
        if not isinstance(manifest, dict) or manifest.get('schema_version') != 1:
            return None
        scenes = manifest.get('scenes')
        if not isinstance(scenes, dict):
            return None
        row = scenes.get(scene['id'])
        if not isinstance(row, dict) or row.get('scene_id') != scene['id']:
            return None
        if (row.get('world_sha256') != scene['world']['sha256'] or
                row.get('asset_identity') != asset_identity(catalog, scene)):
            return None
        world = catalog.resolve(scene['world'])
        if (world.stat().st_size > 8 * 1024 * 1024 or
                hashlib.sha256(world.read_bytes()).hexdigest() != row['world_sha256']):
            return None
        filename = row.get('file', '')
        if filename != scene['id'] + '.png' or not re.fullmatch(r'[a-z0-9_]+\.png', filename):
            return None
        if row.get('kind') not in ('native_topview', 'native_topview_cutaway'):
            return None
        image = catalog.resolve(dict(package='easyfly_gazebo_worlds',
                                     path='previews/' + filename))
        if not 32 <= image.stat().st_size <= 8 * 1024 * 1024:
            return None
        data = image.read_bytes()
        if (not data.startswith(b'\x89PNG\r\n\x1a\n') or data[12:16] != b'IHDR' or
                hashlib.sha256(data).hexdigest() != row.get('image_sha256')):
            return None
        width, height = struct.unpack('>II', data[16:24])
        if not (1 <= width <= 2048 and 1 <= height <= 1536):
            return None
        caption = row.get('caption')
        if not isinstance(caption, str) or not 1 <= len(caption) <= 256:
            return None
        return dict(path=str(image), caption=caption, kind=row['kind'],
                    sha256=row['image_sha256'])
    except (OSError, ValueError, KeyError, TypeError):
        return None
