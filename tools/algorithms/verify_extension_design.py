#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Validate EXT-01 design schemas/examples; no nodes, libraries or flight actions."""
import argparse
import copy
import json
from pathlib import Path

from jsonschema import Draft202012Validator

ROOT = Path(__file__).resolve().parents[2] / 'src/algorithms'
ROLES = {'detection': 'detection', 'multi_tracking': 'multi_tracking',
         'single_tracking': 'single_tracking', 'global_grid': 'grid_planning',
         'combined_grid': 'grid_planning', 'localization': 'localization',
         'trajectory_planning': 'trajectory_planning'}


def read(path):
    def invalid_constant(value):
        raise ValueError('non-finite JSON constant: ' + value)
    def unique_fields(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError('duplicate JSON field: ' + key)
            result[key] = value
        return result
    return json.loads(path.read_text(), parse_constant=invalid_constant,
                      object_pairs_hook=unique_fields)


def validate(registry, selection):
    Draft202012Validator(read(ROOT / 'schemas/backend-registry-v1.schema.json')).validate(registry)
    Draft202012Validator(read(ROOT / 'schemas/backend-selection-v1.schema.json')).validate(selection)
    index = {}
    for row in registry['backends']:
        key = row['capability'], row['id']
        if key in index:
            raise ValueError('duplicate capability/backend: ' + str(key))
        index[key] = row
        schema_path = (ROOT / row['parameter_schema']).resolve()
        if not schema_path.is_relative_to(ROOT.resolve()) or not schema_path.is_file():
            raise ValueError('parameter schema must be a contained existing file')
        Draft202012Validator.check_schema(read(schema_path))
    for role, choice in selection['backends'].items():
        descriptor = index.get((ROLES[role], choice['backend']))
        if descriptor is None:
            raise ValueError('unregistered capability/backend for role: ' + role)
        Draft202012Validator(read(ROOT / descriptor['parameter_schema'])).validate(choice['parameters'])
    nav = selection['navigation']
    needed = ['global_grid'] if nav['strategy'] == 'astar' else (
        ['combined_grid'] if nav['strategy'] == 'combined' else [])
    if any(role not in selection['backends'] for role in needed):
        raise ValueError('navigation grid role has no configured backend')
    # No platform-node schemas are frozen by these two sample backends yet.
    # Never treat the envelope's object shape as validation of node parameters.
    if selection['platform']:
        raise ValueError('design examples have no registered platform-node schemas')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('preserve prior evidence; output must be new')
    results = []
    for path in sorted((ROOT / 'schemas').glob('*.schema.json')):
        Draft202012Validator.check_schema(read(path))
        results.append({'name': 'schema:' + path.name, 'status': 'passed'})
    registry = read(ROOT / 'examples/backend-registry-v1.json')
    selection = read(ROOT / 'examples/backend-selection-v1.json')
    validate(registry, selection)
    results.append({'name': 'existing_sort_astar_examples', 'status': 'passed'})
    mutations = [
        ('unknown_selection_field', lambda r, s: s.update(unknown=True)),
        ('wrong_version', lambda r, s: s.update(schema_version=2)),
        ('boolean_version', lambda r, s: s.update(schema_version=True)),
        ('unknown_backend', lambda r, s: s['backends']['multi_tracking'].update(backend='candidate_mot')),
        ('unknown_backend_parameter', lambda r, s: s['backends']['multi_tracking']['parameters'].update(unknown=1)),
        ('wrong_parameter_type', lambda r, s: s['backends']['multi_tracking']['parameters'].update(min_hits='3')),
        ('invalid_parameter_range', lambda r, s: s['backends']['global_grid']['parameters'].update(max_cells=0)),
        ('unknown_strategy', lambda r, s: s['navigation'].update(strategy='candidate_grid')),
        ('duplicate_backend', lambda r, s: r['backends'].append(copy.deepcopy(r['backends'][0]))),
        ('missing_parameter_schema', lambda r, s: r['backends'][0].update(parameter_schema='schemas/missing.json')),
        ('escaping_parameter_schema', lambda r, s: r['backends'][0].update(parameter_schema='../README.md')),
        ('wrong_role_capability', lambda r, s: s['backends']['global_grid'].update(backend='sort')),
        ('missing_combined_role', lambda r, s: s['backends'].pop('combined_grid')),
        ('unchecked_platform_parameters', lambda r, s: s['platform'].update(tracking={'unknown': 1})),
    ]
    from jsonschema.exceptions import ValidationError
    for name, mutate in mutations:
        candidate_registry, candidate_selection = copy.deepcopy(registry), copy.deepcopy(selection)
        mutate(candidate_registry, candidate_selection)
        try:
            validate(candidate_registry, candidate_selection)
        except (ValueError, ValidationError):
            results.append({'name': name, 'status': 'rejected_as_expected'})
        else:
            raise AssertionError('invalid design input accepted: ' + name)
    import tempfile
    with tempfile.TemporaryDirectory(prefix='easyfly-ext01-json-') as temporary:
        for name, payload in [('duplicate_json_field', '{"schema_version":1,"schema_version":2}'),
                              ('non_finite_json_number', '{"threshold":NaN}')]:
            path = Path(temporary) / 'invalid.json'
            path.write_text(payload)
            try:
                read(path)
            except ValueError:
                results.append({'name': name, 'status': 'rejected_as_expected'})
            else:
                raise AssertionError('invalid JSON accepted: ' + name)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({'scope': 'design validation only; not production runtime',
                                      'passed': True, 'results': results}, indent=2) + '\n')
    print(f'{len(results)} design checks passed: {args.output}')


if __name__ == '__main__':
    main()
