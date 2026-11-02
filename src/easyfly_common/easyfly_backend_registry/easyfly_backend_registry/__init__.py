# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""No ROS imports, runtime loading or readiness claims in this package."""
from dataclasses import dataclass
import copy
import json
import math
from pathlib import Path

from jsonschema import Draft202012Validator

DATA = Path(__file__).resolve().parent / 'data'
ROLES = dict(detection='detection', multi_tracking='multi_tracking', single_tracking='single_tracking',
             global_grid='grid_planning', combined_grid='grid_planning', localization='localization',
             trajectory_planning='trajectory_planning', exploration='exploration')


def loads(text):
    def fields(pairs):
        out = {}
        for key, value in pairs:
            if key in out:
                raise ValueError('duplicate JSON field: ' + key)
            out[key] = value
        return out
    def invalid(value):
        raise ValueError('non-finite JSON constant: ' + value)
    return json.loads(text, object_pairs_hook=fields, parse_constant=invalid)


def read(path):
    return loads(Path(path).read_text())


def contained(path, directory=DATA):
    result = (directory / path).resolve()
    if not result.is_relative_to(directory.resolve()) or not result.is_file():
        raise ValueError('metadata reference must name a contained existing file: ' + path)
    return result


class Registry:
    def __init__(self, directory=DATA):
        self.directory = Path(directory)
        data = read(self.directory / 'backends.json')
        schema = read(self.directory / 'schemas/backend-registry-v1.schema.json')
        Draft202012Validator.check_schema(schema)
        Draft202012Validator(schema).validate(data)
        self.entries = {}
        for row in data['backends']:
            key = row['capability'], row['id']
            if key in self.entries:
                raise ValueError('duplicate backend: ' + str(key))
            Draft202012Validator.check_schema(read(contained(row['parameter_schema'], self.directory)))
            self.entries[key] = row
        self.ui = read(self.directory / 'algorithm_ui.json')
        options = {x['key']: x for x in self.ui['fields']}

    def ids(self, capability):
        return [backend for category, backend in self.entries if category == capability]

    def get(self, capability, backend):
        try:
            return copy.deepcopy(self.entries[capability, backend])
        except KeyError as error:
            raise ValueError('unknown backend: ' + capability + '/' + str(backend)) from error

    def parameters(self, capability, backend, values=None):
        entry = self.get(capability, backend)
        schema = read(contained(entry['parameter_schema'], self.directory))
        result = {k: copy.deepcopy(v['default']) for k, v in schema.get('properties', {}).items() if 'default' in v}
        if values is not None:
            if not isinstance(values, dict):
                raise ValueError('backend parameters must be an object')
            result.update(values)
        def check_finite(value):
            if isinstance(value, float) and not math.isfinite(value):
                raise ValueError('non-finite backend parameter')
            if isinstance(value, dict):
                for item in value.values(): check_finite(item)
            elif isinstance(value, list):
                for item in value: check_finite(item)
        check_finite(result)
        Draft202012Validator(schema).validate(result)
        for key, spec in schema.get('properties', {}).items():
            if spec.get('type') == 'integer' and key in result: result[key] = int(result[key])
        return result

    def validate(self, selection, *, profile=None):
        def finite(value):
            if isinstance(value, float) and not math.isfinite(value):
                raise ValueError('non-finite configuration value')
            if isinstance(value, dict):
                for item in value.values(): finite(item)
            if isinstance(value, list):
                for item in value: finite(item)
        finite(selection)
        value = copy.deepcopy(selection)
        schema = read(self.directory / 'schemas/backend-selection-v1.schema.json')
        Draft202012Validator(schema).validate(value)
        for role, choice in value['backends'].items():
            choice['parameters'] = self.parameters(ROLES[role], choice['backend'], choice['parameters'])
        for role, options in value['platform'].items():
            if role not in ROLES:
                raise ValueError('unknown platform role: ' + role)
            Draft202012Validator(read(self.directory / ('schemas/platform-' + role + '-v1.schema.json'))).validate(options)
        strategy = value['navigation']['strategy']
        needed = ['global_grid'] if strategy == 'astar' else ['combined_grid'] if strategy == 'combined' else []
        if any(role not in value['backends'] for role in needed):
            raise ValueError('navigation requires a configured grid role')
        expected_trajectory = 'ego' if strategy == 'combined' else strategy
        if strategy in ('ego', 'combined', 'gcopter', 'super') and value['backends'].get(
                'trajectory_planning', {}).get('backend') != expected_trajectory:
            raise ValueError('navigation strategy requires matching trajectory backend')
        if profile == 'base_sim' and 'localization' in value['backends']:
            raise ValueError('base_sim cannot start a localization backend')
        if 'exploration' in value['backends']:
            if (value['backends']['exploration']['backend'] != 'fuel' or strategy != 'gcopter' or
                    value['backends'].get('localization', {}).get('backend') != 'fast_lio' or profile != 'navigation_dev'):
                raise ValueError('FUEL requires navigation_dev, FAST-LIO and GCOPTER')
        return value

    def typed_parameters(self, capability, backend, values=None):
        parameters = self.parameters(capability, backend, values)
        if (capability, backend) == ('multi_tracking', 'sort'):
            return SortParameters(**parameters)
        if (capability, backend) == ('grid_planning', 'astar'):
            return AStarParameters(**parameters)
        raise ValueError('no C++ value mapping for this ROS-owned backend')

    def resolve_layers(self, *, profile_defaults=None, file_config=None, cli=None, **options):
        """Defaults < profile < file < CLI, with explicit per-leaf provenance."""
        effective = self.resolve(**options)
        provenance = {}
        def record(value, source, path=''):
            if isinstance(value, dict):
                for key, item in value.items(): record(item, source, path + '/' + key)
            else: provenance[path] = source
        record(effective, 'schema/profile-defaults')
        def merge(target, patch, source, path=''):
            for key, value in patch.items():
                if key in target and isinstance(value, dict) and isinstance(target[key], dict):
                    if 'backend' in value and value['backend'] != target[key].get('backend'):
                        target[key] = copy.deepcopy(value); record(value, source, path + '/' + key)
                    else: merge(target[key], value, source, path + '/' + key)
                else:
                    target[key] = copy.deepcopy(value); record(value, source, path + '/' + key)
        for source, patch in [('profile', profile_defaults), ('file', file_config), ('cli', cli)]:
            if patch is not None:
                if not isinstance(patch, dict): raise ValueError('configuration layer must be an object')
                merge(effective, patch, source)
        return self.validate(effective, profile=options.get('profile')), provenance

    def import_legacy(self, legacy, selection=None):
        """Only reviewed SORT and the two distinct A* role YAML mappings are accepted."""
        if not isinstance(legacy, dict) or set(legacy) - {'tracking', 'global_grid', 'combined_grid'}:
            raise ValueError('unsupported legacy configuration root')
        roles = dict(tracking=('multi_tracking', 'sort'), global_grid=('global_grid', 'astar'),
                     combined_grid=('combined_grid', 'astar'))
        patch = dict(backends={})
        for key, values in legacy.items():
            role, backend = roles[key]
            validated = self.parameters(ROLES[role], backend, values)
            existing = (selection or {}).get('backends', {}).get(role)
            if existing and (existing['backend'] != backend or any(
                    field in existing['parameters'] and existing['parameters'][field] != value
                    for field, value in values.items())):
                raise ValueError('conflicting legacy/new fields for ' + role)
            patch['backends'][role] = dict(backend=backend, parameters=validated)
        return patch

    def import_tracking_yaml(self, document, selection=None):
        """Split the actual legacy tracking.yaml; preserve its missing-field fallbacks."""
        if not isinstance(document, dict) or set(document) != {'tracking'} or not isinstance(document['tracking'], dict):
            raise ValueError('expected legacy tracking mapping')
        values = copy.deepcopy(document['tracking'])
        backend = values.pop('tracker_backend', 'sort')
        self.get('multi_tracking', backend)
        schema = read(self.directory / 'schemas/sort-parameters-v1.schema.json')
        names = set(schema['properties'])
        # These are the existing YAML loader fallbacks, distinct from the full shipped profile.
        parameters = dict(iou_threshold=.3, max_lost_frames=30, min_hits=3, max_tracks=100, dist_gate_scale=1.5)
        parameters.update({key: values.pop(key) for key in list(values) if key in names})
        patch = self.import_legacy({'tracking': parameters}, selection)
        platform_schema = read(self.directory / 'schemas/platform-multi_tracking-v1.schema.json')
        Draft202012Validator(platform_schema).validate(values)
        old_platform = (selection or {}).get('platform', {}).get('multi_tracking', {})
        if any(key in old_platform and old_platform[key] != value for key, value in values.items()):
            raise ValueError('conflicting legacy/new platform fields')
        patch['platform'] = dict(multi_tracking=values)
        return patch

    def check_inputs(self, capability, backend, supplied):
        required = set(self.get(capability, backend)['required_inputs'])
        missing = required - set(supplied)
        if missing:
            raise ValueError('missing required inputs: ' + ', '.join(sorted(missing)))

    def check_source_tree(self, root):
        """Verify referenced packages, launch assets and exported creator declarations.

        Actual creator coverage is additionally executed by each C++ adapter's tests.
        """
        import xml.etree.ElementTree as ET
        root = Path(root)
        packages = {}
        for manifest in root.rglob('package.xml'):
            packages[ET.parse(manifest).getroot().findtext('name')] = manifest.parent
        result = []
        for (capability, backend), row in self.entries.items():
            entry = row['entrypoint']
            if entry['package'] not in packages:
                raise ValueError('missing implementation package: ' + entry['package'])
            package = packages[entry['package']]
            if entry['kind'] == 'ros_launch':
                if not (package / 'launch' / entry['file']).is_file():
                    raise ValueError('missing launch entry: ' + str(entry))
            elif entry['kind'] == 'cpp_creator':
                symbol = entry['symbol'].split('::')[-1]
                if not any(symbol + '(' in p.read_text().replace(' ', '').replace('\n', '')
                           for p in (package / 'include').rglob('*.hpp')):
                    raise ValueError('missing exported creator declaration: ' + str(entry))
            else:
                raise ValueError('source verification not implemented for entry kind: ' + entry['kind'])
            result.append(dict(capability=capability, backend=backend, package=entry['package']))
        return result

    def check_installation(self, capability, backend, package_share):
        """Check deployable launch assets; never equate presence with readiness."""
        entry = self.get(capability, backend)['entrypoint']
        root = Path(package_share(entry['package']))
        if entry['kind'] == 'ros_launch' and not (root / 'launch' / entry['file']).is_file():
            raise ValueError('registered launch is not installed: ' + str(entry))
        return entry

    def legacy_fields(self):
        from .ui_schema import generated_ui
        return generated_ui(self.ui, list(self.entries.values()))['fields']

    def resolve(self, *, strategy='astar', localization='fast_lio', overrides=None, profile=None):
        roles = dict(multi_tracking='sort', single_tracking='csrt', detection='onnx',
                     global_grid='astar', combined_grid='astar')
        if localization != 'disabled':
            self.get('localization', localization)
            roles['localization'] = localization
        if strategy in ('ego', 'combined', 'gcopter', 'super'):
            roles['trajectory_planning'] = 'ego' if strategy == 'combined' else strategy
        choices = {role: dict(backend=backend, parameters=self.parameters(ROLES[role], backend,
                           dict(max_cells=4000000, unstick_cells=5) if role == 'global_grid' else
                           dict(max_cells=1000000, unstick_cells=0) if role == 'combined_grid' else None))
                   for role, backend in roles.items()}
        choices['global_grid']['parameters'] = dict(max_cells=4000000, unstick_cells=5)
        choices['combined_grid']['parameters'] = dict(max_cells=1000000, unstick_cells=0)
        result = dict(schema_version=1, backends=choices, platform={}, navigation=dict(
            strategy=strategy, grid_role='global_grid', combined_grid_role='combined_grid'))
        if overrides is not None:
            if not isinstance(overrides, dict) or set(overrides) - {'schema_version', 'backends', 'platform', 'navigation'}:
                raise ValueError('unknown configuration fields')
            if 'schema_version' in overrides and (type(overrides['schema_version']) is not int or overrides['schema_version'] != 1):
                raise ValueError('unsupported schema_version')
            for group in ['backends', 'platform', 'navigation']:
                if group in overrides:
                    if not isinstance(overrides[group], dict):
                        raise ValueError('configuration group must be an object')
                    result[group].update(copy.deepcopy(overrides[group]))
        return self.validate(result, profile=profile)


@dataclass(frozen=True)
class SortParameters:
    iou_threshold: float
    max_lost_frames: int
    min_hits: int
    max_tracks: int
    dist_gate_scale: float

@dataclass(frozen=True)
class AStarParameters:
    max_cells: int
    unstick_cells: int


def fields():
    return Registry().legacy_fields()
