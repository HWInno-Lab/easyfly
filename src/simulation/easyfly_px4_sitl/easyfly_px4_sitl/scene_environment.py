# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Bind installed scene resources without starting a simulator or acquiring a session."""

from pathlib import Path
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory, PackageNotFoundError

from .scene_catalog import CatalogError, SceneCatalog


def source_package(share, name):
    """Trust only ament's package.xml source link, never arbitrary resource links."""
    metadata = Path(share) / 'package.xml'
    if not metadata.is_symlink():
        return None
    source = metadata.resolve(strict=True).parent
    if source.name != name or ET.parse(source / 'package.xml').getroot().findtext('name') != name:
        raise CatalogError(f'ament source package identity mismatch: {name}')
    return source


def installed_catalog(*, px4_dir=None, px4_models=None, gazebo_share=None, package_shares=None):
    """Resolve only installed packages and explicit external roots; offline only.

    Missing optional PX4/Gazebo inputs become validation issues, so list works on
    an installation that has not prepared a compatibility build. Launch passes
    its already verified PX4 directory; no fallback binary is ever selected here.
    """
    names = ('easyfly_gazebo_worlds', 'easyfly_gazebo_models',
             'easyfly_px4_sitl', 'easyfly_gazebo_plugins')
    shares = {name: Path(get_package_share_directory(name)) for name in names}
    shares.update({name: Path(p) for name, p in (package_shares or {}).items()})
    sources = [p for name, share in shares.items() if (p := source_package(share, name))]
    roots, plugin_dirs, notes = {}, [], []
    gazebo = Path(gazebo_share or '/usr/share/gazebo-11')
    if gazebo.is_dir():
        roots['gazebo_share'] = gazebo
    else:
        notes.append(f'Gazebo resource root unavailable: {gazebo}')
    if px4_models is None:
        if px4_dir is None:
            from .hrt_compatibility import select_px4_directory
            try:
                px4_dir = select_px4_directory(os.environ.get('PX4_DIR', '').strip() or None)
            except RuntimeError as error:
                notes.append(str(error))
        if px4_dir is not None:
            px4_models = Path(px4_dir) / 'Tools/simulation/gazebo-classic/sitl_gazebo-classic/models'
    if px4_models is not None and Path(px4_models).is_dir():
        roots['px4_models'] = Path(px4_models)
    elif px4_models is not None:
        notes.append(f'PX4 model root unavailable: {px4_models}')
    candidates = [Path(get_package_prefix('easyfly_gazebo_plugins')) / 'lib',
                  Path(get_package_prefix('gazebo_ros')) / 'lib']
    if px4_dir is not None:
        candidates.insert(0, Path(px4_dir) / 'build/px4_sitl_default/build_gazebo-classic')
    for path in candidates:
        if path.is_dir():
            plugin_dirs.append(path)
    catalog = SceneCatalog(shares, external_roots=roots, plugin_dirs=plugin_dirs,
                           trusted_sources=sources)
    def referenced_packages(value):
        if isinstance(value,dict):
            if 'package' in value and 'path' in value:
                yield value['package']
            for item in value.values():
                yield from referenced_packages(item)
        elif isinstance(value,list):
            for item in value:
                yield from referenced_packages(item)
    for name in set(referenced_packages([catalog.scenes,catalog.assets]))-catalog.packages.keys():
        try:
            share=Path(get_package_share_directory(name))
            source=source_package(share,name)
            catalog.packages[name]=share.resolve(strict=True)
            if source is not None and source not in catalog.trusted_sources:
                catalog.trusted_sources=(*catalog.trusted_sources,source)
        except PackageNotFoundError:
            notes.append(f'referenced resource package unavailable: {name}')
    return catalog, notes
