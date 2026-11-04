#!/usr/bin/env python3
# Copyright 2026 贾国源. SPDX-License-Identifier: Apache-2.0
"""Check the draft transition dossier, not legal clearance or a license grant.

Exit 0: draft materials consistent; 1: inconsistent/missing materials;
2 with --require-ready: consistent draft still has unresolved release decisions.
This schema deliberately cannot approve or activate a license transition.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import sys


ROOT = Path(__file__).resolve().parents[2]
DEFAULT = 'licensing/transition.review.json'
IDENTIFIER = 'LicenseRef-EasyFly-Restricted-1.0'
APACHE_SHA256 = 'cfc7749b96f63bd31c3c42b5c471bf756814053e847c10f3eb003417bc523d30'
CLASSES = {
    'retained_external_license', 'mixed_asset_scope_review',
    'mixed_or_other_license_review',
    'project_maintained_requires_rights_and_combination_review',
}
SKIP = {'.git', '__pycache__', '.pytest_cache', 'build', 'install', 'log'}
EXTERNAL_DRIVERS = (
    'src/communication/c1_drivers/realsense_ros',
    'src/communication/c1_drivers/livox_ros_driver2',
)


def read_json(path):
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError('duplicate JSON field: ' + key)
            result[key] = value
        return result
    return json.loads(path.read_text(encoding='utf-8'), object_pairs_hook=unique)


def relative_name(name):
    if (not isinstance(name, str) or not name or '\\' in name
            or PurePosixPath(name).is_absolute()
            or '..' in PurePosixPath(name).parts
            or str(PurePosixPath(name)) != name):
        raise ValueError('uncontained or noncanonical path: ' + str(name))
    return name


def contained(root, name):
    name = relative_name(name)
    path = root / name
    if not path.resolve(strict=True).is_relative_to(root):
        raise ValueError('uncontained path: ' + name)
    return path


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def external(name):
    return (name == 'src/third_party' or name.startswith('src/third_party/')
            or any(name == x or name.startswith(x + '/') for x in EXTERNAL_DRIVERS))


def inventory(root, name):
    base = contained(root, name)
    if not base.is_dir() or base.is_symlink():
        raise ValueError('package root must be a real directory: ' + name)
    entries = set()
    for directory, dirs, files in os.walk(base, followlinks=False):
        # Same documented generated/cache exclusions as the technical inventory.
        dirs[:] = [x for x in dirs if x not in SKIP]
        for entry in files + [x for x in dirs if (Path(directory) / x).is_symlink()]:
            entries.add((Path(directory) / entry).relative_to(root).as_posix())
    return entries


def verify(manifest=DEFAULT, root=ROOT):
    root = Path(root).resolve()
    record = read_json(contained(root, manifest))
    if (type(record.get('schema_version')) is not int or record['schema_version'] != 1
            or record.get('status') != 'draft_migration_review_not_effective'):
        raise ValueError('unsupported transition review schema/status')
    if record.get('license_identifier') != IDENTIFIER:
        raise ValueError('unknown proposed license identifier')
    if (record.get('effective_release') is not None
            or record.get('approved_scope') != []
            or record.get('root_license_replacement_authorized') is not False):
        raise ValueError('draft review cannot activate a release or approve scope')
    materials = record['materials_sha256']
    required = {
        'LICENSE', 'LICENSES/Apache-2.0.txt', 'licensing/scope.candidate.json',
        'licensing/file-scope.review.json', 'licensing/owner-authority.statement.json',
        'licensing/decisions-20261103.json', 'licensing/legal-review-notes-20261103.json',
        'licensing/use-cases.review.json', 'licensing/EasyFly-Restricted-1.0.draft.txt',
        'licensing/ROOT-LICENSE.draft.txt', 'licensing/AUTHORIZATION.draft.txt',
        'licensing/CONTRIBUTION-POLICY.draft.txt',
    }
    if not required.issubset(materials):
        raise ValueError('missing required license text, notice or review record')
    for name, expected in materials.items():
        path = contained(root, name)
        if not path.is_file() or path.is_symlink() or sha(path) != expected:
            raise ValueError('review material changed: ' + name)
    if (materials['LICENSE'] != APACHE_SHA256
            or materials['LICENSES/Apache-2.0.txt'] != APACHE_SHA256):
        raise ValueError('historical Apache text not preserved')
    candidate = read_json(contained(root, 'licensing/scope.candidate.json'))
    if (candidate.get('status') != 'candidate_inventory_not_a_license_grant'
            or candidate.get('proposed_license_identifier') != IDENTIFIER
            or candidate.get('effective_release') is not None
            or candidate.get('new_license_applied_to_existing_files') is not False
            or candidate.get('default_for_unreviewed_paths') != 'retain_existing_license_and_review'
            or candidate.get('existing_root_license_sha256') != materials['LICENSE']):
        raise ValueError('candidate license scope conflicts with retained draft status')
    if (candidate['file_review_record'] != 'licensing/file-scope.review.json'
            or candidate['file_review_sha256'] != materials[candidate['file_review_record']]):
        raise ValueError('file review reference/hash mismatch')
    owner = read_json(contained(root, 'licensing/owner-authority.statement.json'))
    if (owner.get('scope') != record.get('owner_grant_scope')
            or owner.get('ownership_transfer_claimed') is not False
            or owner.get('existing_grants_revoked') is not False
            or owner.get('company_owns_all_repository_rights_claimed') is not False):
        raise ValueError('owner statement scope expanded or historic rights revoked')

    packages, roots, expected = {}, set(), set()
    for package in candidate['packages']:
        name, path = package['name'], relative_name(package['path'])
        classification = package['classification']
        if name in packages or path in roots or classification not in CLASSES:
            raise ValueError('duplicate or unknown package classification: ' + name)
        roots.add(path)
        packages[name] = package
        if package.get('new_license_approved') is not False:
            raise ValueError('unapproved draft cannot relicense a package: ' + name)
        if external(path) and classification != 'retained_external_license':
            raise ValueError('third-party package reclassified for conversion: ' + path)
        metadata = 'CMakeLists.txt' if package.get('build_type') == '(cmake)' else 'package.xml'
        if sha(contained(root, path + '/' + metadata)) != package['metadata_sha256']:
            raise ValueError('package metadata changed: ' + path)
        if classification != 'retained_external_license':
            entries = inventory(root, path)
            if expected & entries:
                raise ValueError('overlapping candidate package roots: ' + path)
            expected.update(entries)

    review = read_json(contained(root, candidate['file_review_record']))
    seen = set()
    for row in review['files']:
        name = relative_name(row['path'])
        if name in seen:
            raise ValueError('duplicate file review: ' + name)
        seen.add(name)
        package = packages[row['package']]
        if (package['classification'] == 'retained_external_license' or external(name)
                or not name.startswith(package['path'] + '/')):
            raise ValueError('file outside non-external candidate package: ' + name)
        if row.get('new_license_approved') is not False:
            raise ValueError('unapproved draft cannot relicense a file: ' + name)
        path = contained(root, name)
        if row['baseline_kind'] == 'symlink':
            if not path.is_symlink() or os.readlink(path) != row['link_target']:
                raise ValueError('source link changed: ' + name)
        elif row['baseline_kind'] == 'file':
            if path.is_symlink() or not path.is_file() or sha(path) != row['sha256']:
                raise ValueError('source/notice changed: ' + name)
        else:
            raise ValueError('unknown source entry kind: ' + name)
    if seen != expected:
        raise ValueError('source inventory changed; added=' + str(sorted(expected - seen)[:5])
                         + '; missing=' + str(sorted(seen - expected)[:5]))
    return {
        'draft_materials_consistent': True,
        'inventoried_packages': len(packages),
        'checked_non_external_file_entries': len(seen),
        'operative_scope_approved': False,
        'rights_clearance_inferred': False,
        'combination_compatibility_inferred': False,
        'ready_to_switch_root_license': False,
        'pending': record['pending'],
        'limits': 'Draft dossier and enumerated package trees only; not a repository-wide '
                  'SBOM, legal opinion, external publication audit or release approval.',
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=ROOT)
    parser.add_argument('--manifest', default=None, help='draft path relative to --root; default detects current scope')
    parser.add_argument('--require-ready', action='store_true')
    args = parser.parse_args()
    if args.manifest is None and (args.root / 'licensing/scope.json').is_file():
        from verify_license_scope import verify as verify_current
        try:
            result = verify_current(args.root)
        except (ValueError, KeyError, TypeError, OSError, RuntimeError) as error:
            print(json.dumps({'scoped_materials_consistent': False, 'error': str(error)}, ensure_ascii=False))
            return 1
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 2 if args.require_ready and not result['ready_to_publish_entire_workspace'] else 0
    try:
        result = verify(args.manifest or DEFAULT, args.root)
    except (ValueError, KeyError, TypeError, OSError, RuntimeError) as error:
        print(json.dumps({'draft_materials_consistent': False, 'error': str(error)}, ensure_ascii=False))
        return 1
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 2 if args.require_ready and not result['ready_to_switch_root_license'] else 0


if __name__ == '__main__':
    sys.exit(main())
