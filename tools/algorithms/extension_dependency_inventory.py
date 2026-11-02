#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Discover declared build/runtime/deployment closures without loading algorithms.

The package graph is not an SDK or readiness probe. Plain CMake discovery,
external SDK prefixes and model assets must also be checked by installation tests.
"""
import argparse
import json
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET
from catkin_pkg.condition import evaluate_condition

from verify_api_install import clean_environment

ROOT = Path(__file__).resolve().parents[2]
BUILD = {'depend', 'build_depend', 'build_export_depend', 'buildtool_depend',
         'buildtool_export_depend'}
RUNTIME = {'depend', 'exec_depend'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', action='append', default=[], help='registered backend ID')
    parser.add_argument('--package', action='append', default=[])
    parser.add_argument('--kind', choices=['build', 'runtime', 'deployment'], default='build')
    parser.add_argument('--format', choices=['json', 'names', 'paths'], default='json')
    args = parser.parse_args()
    env = clean_environment()
    env['PYTHONNOUSERSITE'] = '1'
    discovery = subprocess.check_output(['/usr/bin/colcon', 'list', '--base-paths', str(ROOT/'src')],
                                        env=env, text=True)
    packages = {}
    for line in discovery.splitlines():
        name, directory, kind = line.split(maxsplit=2)
        p = Path(directory)/'package.xml'
        packages[name] = {'path': directory, 'type': kind,
                          'xml': ET.parse(p).getroot() if p.exists() else None}
    registry_path = ROOT/'src/easyfly_common/easyfly_backend_registry/easyfly_backend_registry/data/backends.json'
    entries = json.loads(registry_path.read_text())['backends']
    selected = [b for b in entries if not args.backend or b['id'] in args.backend]
    unknown = set(args.backend) - {b['id'] for b in selected}
    if unknown:
        parser.error('unknown backend: ' + ', '.join(sorted(unknown)))
    if args.package:
        unknown = set(args.package) - packages.keys()
        if unknown:
            parser.error('unknown package: ' + ', '.join(sorted(unknown)))
        if not args.backend:
            selected = []
    seeds = args.package + [b['entrypoint']['package'] for b in selected]
    tags = BUILD if args.kind == 'build' else RUNTIME if args.kind == 'runtime' else BUILD | RUNTIME
    internal, external, conditions = set(), set(), []
    context = {'ROS_VERSION': '2', 'ROS_DISTRO': 'humble', 'ROS_PYTHON_VERSION': '3'}
    todo = list(seeds)
    while todo:
        name = todo.pop()
        if name in internal or name in external:
            continue
        if name not in packages:
            external.add(name)
            continue
        internal.add(name)
        xml = packages[name]['xml']
        if xml is not None:
            for e in xml:
                if e.tag in tags:
                    enabled = True
                    if 'condition' in e.attrib:
                        unknown = set(re.findall(r'\$([A-Za-z_][A-Za-z_0-9]*)', e.attrib['condition'])) - context.keys()
                        if unknown:
                            raise ValueError(f'{name}: undefined condition variables: {sorted(unknown)}')
                        enabled = evaluate_condition(e.attrib['condition'], context)
                        conditions.append({'package': name, 'dependency': e.text.strip(),
                                           'condition': e.attrib['condition'], 'included': enabled})
                    if enabled:
                        todo.append(e.text.strip())
    if args.format != 'json':
        print('\n'.join(packages[n]['path'] if args.format == 'paths' else n for n in sorted(internal)))
        return
    print(json.dumps({'schema_version': 1, 'kind': args.kind, 'seeds': seeds,
                      'workspace': {n: {k: v for k, v in packages[n].items() if k != 'xml'}
                                    for n in sorted(internal)},
                      'external': sorted(external), 'condition_context': context,
                      'conditional_edges': conditions,
                      'backends': selected,
                      'limitations': ['Test dependencies excluded; deployment includes build and runtime edges.',
                                      'Conditions evaluated for ROS 2 Humble/Python 3; unknown variables fail.',
                                      'No package.xml means declared dependency metadata is unavailable.',
                                      'SDK discovery, models and runtime readiness require separate validation.']}, indent=2))


if __name__ == '__main__':
    main()
