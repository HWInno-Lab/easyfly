#!/usr/bin/env python3
# Copyright 2026 EasyFly Project
# SPDX-License-Identifier: Apache-2.0
"""Read-only ALG-0 inventory capture. Never builds, launches, or moves sources."""

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[2]


def git(*args):
    return subprocess.check_output(['git', '-C', str(ROOT), *args], text=True)


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def direct_imports(path):
    if path.suffix not in {'.cpp', '.hpp', '.h', '.py'}:
        return []
    value = path.read_text(errors='replace')
    return re.findall(
        r'^\s*(?:#\s*include\s*[<"]([^>"\n]+)[>"]|'
        r'from\s+([\w.]+)\s+import|import\s+([\w.]+))', value, re.M)


def capture(inventory):
    tracked = git('ls-files', '-z').split('\0')
    package_paths = {}
    for name in tracked:
        if name.endswith('/package.xml') and (ROOT / name).is_file():
            xml = ET.parse(ROOT / name).getroot()
            package_paths[xml.findtext('name')] = Path(name).parent

    sources = set()
    components = []
    for entry in inventory['components']:
        paths = []
        for name in entry['source_files']:
            path = ROOT / name
            if not path.is_file():
                raise ValueError(f"{entry['id']}: missing source {name}")
            sources.add(name)
            paths.append(name)
        components.append({'id': entry['id'], 'sources': paths})

    packages = {}
    selected = set(inventory['baseline_packages'])
    selected.update(inventory.get('support_packages', []))
    selected.update(entry['source_package'] for entry in inventory['components']
                    if entry.get('source_package'))
    for name in sorted(selected):
        rel = package_paths[name]
        prefix = rel.as_posix() + '/'
        files = sorted(p for p in tracked if p.startswith(prefix) and
                       (ROOT / p).is_file())
        sources.update(files)
        xml = ET.parse(ROOT / rel / 'package.xml').getroot()
        dependencies = {}
        for node in xml:
            if node.tag.endswith('depend'):
                dependencies.setdefault(node.tag, []).append(node.text.strip())
        cmake = ROOT / rel / 'CMakeLists.txt'
        packages[name] = {
            'path': rel.as_posix(), 'dependencies': dependencies,
            'cmake_find_packages': re.findall(r'find_package\(\s*([\w]+)',
                                             cmake.read_text()) if cmake.exists() else [],
            'source_tests': [p for p in files if '/test/' in p],
            'configs': [p for p in files if '/config/' in p],
            'launch': [p for p in files if '/launch/' in p],
        }

    fingerprints = {}
    for name in sorted(sources):
        path = ROOT / name
        fingerprints[name] = {
            'sha256': sha256(path), 'bytes': path.stat().st_size,
            'direct_imports': [next(item for item in match if item)
                               for match in direct_imports(path)],
        }

    external = []
    for item in inventory['third_party_moves']:
        rel = item['current']
        files = sorted(p for p in tracked if p.startswith(rel.rstrip('/') + '/')
                       and (ROOT / p).is_file())
        digest = hashlib.sha256()
        for name in files:
            digest.update((name + '\0' + sha256(ROOT / name) + '\n').encode())
        external.append({
            'current': rel, 'target': item['target'], 'exists': (ROOT / rel).exists(),
            'tracked_file_count': len(files),
            'tracked_tree_sha256': digest.hexdigest() if files else None,
            'scope': 'tracked paths + NUL + file SHA256 + newline; sorted by path',
        })

    ignored = []
    for name in inventory['local_assets']:
        path = ROOT / name
        check = subprocess.run(['git', '-C', str(ROOT), 'check-ignore', '-v', name],
                               capture_output=True, text=True)
        ignored.append({
            'path': name, 'exists': path.is_file(),
            'sha256': sha256(path) if path.is_file() else None,
            'bytes': path.stat().st_size if path.is_file() else None,
            'ignore_rule': check.stdout.strip() or None,
        })

    internal_refs = {}
    for item in inventory['third_party_moves']:
        directory = ROOT / item['current']
        if directory.is_dir() and item['current'] != 'src/third_party/onnxruntime':
            scan = subprocess.run(
                ['rg', '-n', r'\.\./|/home/|third_party', item['current'],
                 '-g', 'CMakeLists.txt', '-g', '*.cmake', '-g', '*.launch.py',
                 '-g', '*.bash'], cwd=ROOT, text=True, capture_output=True)
            if scan.returncode not in (0, 1):
                raise RuntimeError(scan.stderr)
            internal_refs[item['current']] = scan.stdout.splitlines()
    refs = subprocess.run(
        ['rg', '-n', '--hidden', '--glob', '!.git/**', '--glob', '!src/algorithms/**',
         '--glob', '!tools/algorithms/**', '--glob', '!src/third_party/**',
         '--glob', '!**/evidence/**',
         r'third_party/(slam|planners|onnxruntime|livox_sdk2|librealsense|qgc|QGC)|'
         r'\.\./(slam|planners|onnxruntime|livox_sdk2|librealsense)',
         'src', 'tools', '.gitignore', 'build_all.bash', 'build_layer.bash',
         'easyfly_env.bash'], cwd=ROOT, text=True, capture_output=True)
    if refs.returncode not in (0, 1):
        raise RuntimeError(refs.stderr)
    colcon = subprocess.run(['colcon', 'list', '--base-paths', 'src'], cwd=ROOT,
                            text=True, capture_output=True, check=True)
    return {
        'schema_version': 1, 'source_commit': git('rev-parse', 'HEAD').strip(),
        'working_tree': git('status', '--short').splitlines(),
        'inventory_sha256': sha256(ROOT / 'src/algorithms/migration-inventory.json'),
        'components': components, 'packages': packages, 'files': fingerprints,
        'third_party_trees': external, 'local_assets': ignored,
        'path_references': refs.stdout.splitlines(),
        'upstream_relative_path_references': internal_refs,
        'reference_search_scope': 'tracked source/config/build inputs outside upstream trees; '
                                  'historical evidence and ignored local plans excluded',
        'colcon_packages': colcon.stdout.splitlines(),
        'limitations': [
            'Imports are direct textual inventory, not a transitive dependency proof.',
            'Third-party tree hashes cover tracked files only, excluding ignored SDK/build files.',
            'No build, ROS node, simulator, flight action, download or migration is performed.',
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path,
                        help='JSON output; an existing file is never overwritten')
    args = parser.parse_args()
    inventory = json.loads((ROOT / 'src/algorithms/migration-inventory.json').read_text())
    if args.output.exists():
        parser.error(f'output already exists: {args.output}')
    result = capture(inventory)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
    print(f"Captured {len(result['files'])} files, {len(result['packages'])} packages, "
          f"{len(result['colcon_packages'])} discovered ROS packages → {args.output}")


if __name__ == '__main__':
    main()
