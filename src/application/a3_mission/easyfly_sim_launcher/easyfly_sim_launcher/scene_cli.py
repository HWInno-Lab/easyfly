# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Offline inventory and explicit asset preparation. No ROS nodes or launch actions."""

import argparse
import json
import sys
from contextlib import contextmanager
from pathlib import Path
import signal
import tarfile

from easyfly_px4_sitl.scene_catalog import CatalogError, ValidationReport
from easyfly_px4_sitl.scene_environment import installed_catalog
from easyfly_px4_sitl.asset_cache import CacheError, fetch_asset


@contextmanager
def fetch_signals():
    """Clean owned staging on CLI interruption; leave signal behavior of launch alone."""
    def interrupted(number, _frame):
        raise KeyboardInterrupt(f'asset fetch interrupted by signal {number}')
    previous = {s: signal.signal(s, interrupted) for s in (signal.SIGINT, signal.SIGTERM)}
    try:
        yield
    finally:
        for number, handler in previous.items():
            signal.signal(number, handler)


def main(argv=None, *, catalog_factory=installed_catalog):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--px4-models', help='Explicit existing PX4 Classic model root')
    parser.add_argument('--px4-dir', help='Explicit existing PX4 tree for offline model/plugin inspection')
    parser.add_argument('--gazebo-share', help='Explicit existing Gazebo resource root')
    commands = parser.add_subparsers(dest='command', required=True)
    for name in ('list', 'validate'):
        command = commands.add_parser(name)
        command.add_argument('--json', action='store_true', help='Machine-readable output')
        command.add_argument('--cache-root', type=Path, help='Explicit external asset cache root')
        command.add_argument('--asset-mode', choices=('packaged', 'cache'), help='Source for assets with reviewed archives')
        if name == 'validate':
            command.add_argument('scene')
    command = commands.add_parser('fetch', help='Explicitly prepare only locked assets required by this scene')
    command.add_argument('scene')
    command.add_argument('--json', action='store_true')
    command.add_argument('--cache-root', type=Path)
    source = command.add_mutually_exclusive_group()
    source.add_argument('--archive', type=Path, help='Prepare the scene\'s single archived asset from a local pinned archive')
    source.add_argument('--offline', action='store_true', help='Verify prepared versions without network or writes')
    args = parser.parse_args(argv)
    try:
        catalog, notes = catalog_factory(px4_dir=args.px4_dir, px4_models=args.px4_models,
                                        gazebo_share=args.gazebo_share, asset_cache_root=args.cache_root,
                                        asset_mode=getattr(args, 'asset_mode', None))
        if args.command == 'fetch':
            if args.scene not in catalog.scenes:
                raise CatalogError('unknown scene: ' + args.scene)
            scene = catalog.scenes[args.scene]
            keys = list(dict.fromkeys(scene['asset_sets'] + scene['runtime']['asset_sets']))
            assets = [catalog.assets[k] for k in keys if 'archive' in catalog.assets[k]]
            if args.archive is not None and len(assets) != 1:
                raise CacheError('--archive requires exactly one archived asset set in this scene')
            licenses = ValidationReport(args.scene)
            for asset in assets:
                catalog._provenance(asset['provenance'], licenses, asset['asset_set_id'])
                for model in asset['models']:
                    catalog._provenance(model['provenance'], licenses, model['name'])
            if licenses.issues:
                raise CacheError('archive provenance/license evidence incomplete: ' + str(licenses.issues))
            with fetch_signals():
                results = [fetch_asset(a, catalog.asset_cache_root, archive_path=args.archive,
                                       offline=args.offline) for a in assets]
            report = dict(scene=args.scene, cache_root=str(catalog.asset_cache_root), assets=results,
                          packaged_asset_sets=[k for k in keys if 'archive' not in catalog.assets[k]],
                          runtime_verified=False, configuration_notes=notes)
            print(json.dumps(report, ensure_ascii=False, indent=2) if args.json else
                  '\n'.join([f'{r["asset_set_id"]}: {r["state"]} {r["model_root"]}' for r in results]) or
                  'No archived assets required; installed resources are unchanged.')
            return 0
        ids = sorted(catalog.scenes) if args.command == 'list' else [args.scene]
        rows = []
        for scene_id in ids:
            report = catalog.validate(scene_id).as_dict()
            scene = catalog.scenes[scene_id]
            rows.append(dict(id=scene_id, name=scene['name'], category=scene['category'],
                             status=scene['status'], supported_profiles=scene['supported_profiles'],
                             source=scene['provenance'], validation=report))
        if args.json:
            print(json.dumps({'scenes':rows, 'configuration_notes':notes}, ensure_ascii=False, indent=2))
        else:
            for row in rows:
                report = row['validation']
                print(f"{row['id']}  {row['category']}  {row['status']}  "
                      f"resources={'complete' if report['resources_complete'] else 'incomplete'}  "
                      f"licenses={'complete' if report['licenses_complete'] else 'incomplete'}  "
                      f"registration={'complete' if report['registration_complete'] else 'incomplete'}  "
                      f"profiles={','.join(row['supported_profiles']) or '-'}")
                if args.command == 'validate':
                    for issue in report['issues']:
                        print(f"  [{issue['scope']}] {issue['message']}")
            for note in notes:
                print(f'configuration: {note}', file=sys.stderr)
        if args.command == 'list':
            return 0
        report = rows[0]['validation']
        return 0 if all(report[key] for key in ('resources_complete', 'licenses_complete', 'registration_complete')) else 1
    except (CatalogError, CacheError, OSError, KeyError, RuntimeError, tarfile.TarError, KeyboardInterrupt) as error:
        if args.json:
            print(json.dumps({'error': str(error)}, ensure_ascii=False))
        else:
            print(f'scene_catalog: {error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
