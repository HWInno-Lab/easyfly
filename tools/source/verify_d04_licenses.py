#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Verify D-04 material identity; --require-closed also requires all grants resolved.

An intact dossier is not a legal clearance. This tool never infers a license from
file presence, a package tag, a successful build, or a different upstream branch.
"""
import argparse
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
DEFAULT = ROOT / 'src/third_party/license_reviews/d04-20261102/review.json'
ITEMS = {'D04-IKD', 'D04-LIVO-VIKIT', 'D04-LKH', 'D04-EGO-PORT'}


def read_json(path):
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError('duplicate JSON field: ' + key)
            result[key] = value
        return result
    return json.loads(path.read_text(), object_pairs_hook=unique)


def contained(root, name):
    relative = Path(name)
    if relative.is_absolute() or '..' in relative.parts:
        raise ValueError('uncontained dossier path: ' + name)
    path = (root / relative).resolve(strict=True)
    if not path.is_relative_to(root.resolve()) or not path.is_file():
        raise ValueError('dossier input is not a contained file: ' + name)
    return path


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify(manifest=DEFAULT, root=ROOT):
    root = Path(root).resolve()
    record = read_json(Path(manifest))
    if (type(record.get('schema_version')) is not int or record['schema_version'] != 1
            or not isinstance(record.get('items'), list)):
        raise ValueError('unsupported D-04 dossier')
    items = record['items']
    if len(items) != len(ITEMS) or {row['id'] for row in items} != ITEMS:
        raise ValueError('missing or duplicate D-04 item')
    for name, expected in record['inputs_sha256'].items():
        if sha(contained(root, name)) != expected:
            raise ValueError('material changed; review required: ' + name)
    unresolved = []
    for row in items:
        state = row['status']
        if state not in {'resolved_scoped_owner_grant', 'open_rightsholder_evidence'}:
            raise ValueError('unknown item status: ' + state)
        files = row['files_sha256']
        if not files or not row.get('evidence_paths'):
            raise ValueError('missing material identity or evidence: ' + row['id'])
        for name, expected in files.items():
            if sha(contained(root, name)) != expected:
                raise ValueError('covered source changed: ' + name)
        # Inventory roots have exact coverage: new files require a new review too.
        for directory in row.get('exact_inventory_roots', []):
            base = (root / directory).resolve(strict=True)
            if not base.is_relative_to(root):
                raise ValueError('uncontained inventory root')
            actual = {str(p.relative_to(root)) for p in base.rglob('*') if p.is_file()
                      and not any(x in p.parts for x in ('__pycache__', '.pytest_cache'))}
            expected = {name for name in files if name.startswith(directory + '/')}
            if actual != expected:
                raise ValueError('source inventory changed: ' + directory)
        for name in row['evidence_paths']:
            contained(root, name)
            if name not in record['inputs_sha256'] and name not in files:
                raise ValueError('evidence has no fingerprint: ' + name)
        if state == 'resolved_scoped_owner_grant':
            grant = read_json(contained(root, row['owner_grant']))
            if (row['id'] != 'D04-EGO-PORT' or grant.get('license') != 'Apache-2.0'
                    or grant.get('copyright_notice') != 'Copyright 2026 EasyFly Project'
                    or grant.get('scope') != 'owner_original_ros2_port_contributions_only'
                    or not grant.get('preserve_inherited_licenses')):
                raise ValueError('owner grant has an unsupported scope')
        else:
            if not row.get('required_evidence'):
                raise ValueError('open item lacks a concrete closing condition')
            unresolved.append({'id': row['id'], 'reason': row['required_evidence']})
    return {'materials_verified': True, 'd04_closed': not unresolved,
            'scope': 'Four recorded D-04 issues only; not a platform-wide SBOM or release approval',
            'items': [{'id': row['id'], 'status': row['status'],
                       'covered_files': len(row['files_sha256'])} for row in items],
            'unresolved': unresolved}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, default=DEFAULT)
    parser.add_argument('--root', type=Path, default=ROOT)
    parser.add_argument('--require-closed', action='store_true')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = verify(args.manifest, args.root)
    payload = json.dumps(result, ensure_ascii=False, indent=2) + '\n'
    if args.output:
        with args.output.open('x') as stream:
            stream.write(payload)
    print(payload, end='')
    return 2 if args.require_closed and not result['d04_closed'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
