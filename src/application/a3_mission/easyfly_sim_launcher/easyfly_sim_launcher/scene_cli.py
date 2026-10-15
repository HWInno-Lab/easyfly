# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Read-only scene listing and validation. No ROS nodes or launch actions."""

import argparse
import json
import sys

from easyfly_px4_sitl.scene_catalog import CatalogError
from easyfly_px4_sitl.scene_environment import installed_catalog


def main(argv=None, *, catalog_factory=installed_catalog):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--px4-models', help='Explicit existing PX4 Classic model root')
    parser.add_argument('--px4-dir', help='Explicit existing PX4 tree for offline model/plugin inspection')
    parser.add_argument('--gazebo-share', help='Explicit existing Gazebo resource root')
    commands = parser.add_subparsers(dest='command', required=True)
    for name in ('list', 'validate'):
        command = commands.add_parser(name)
        command.add_argument('--json', action='store_true', help='Machine-readable output')
        if name == 'validate':
            command.add_argument('scene')
    args = parser.parse_args(argv)
    try:
        catalog, notes = catalog_factory(px4_dir=args.px4_dir, px4_models=args.px4_models, gazebo_share=args.gazebo_share)
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
    except (CatalogError, OSError, KeyError, RuntimeError) as error:
        if args.json:
            print(json.dumps({'error': str(error)}, ensure_ascii=False))
        else:
            print(f'scene_catalog: {error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
