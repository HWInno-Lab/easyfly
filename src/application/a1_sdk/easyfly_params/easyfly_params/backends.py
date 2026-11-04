# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_params/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project
"""One effective selection for terminal, desktop and launch; no ROS processes started."""
import copy
import os
from pathlib import Path, PurePosixPath
import json
import yaml
from ament_index_python.packages import get_package_share_directory
from easyfly_backend_registry import Registry, loads, ROLES
from .algorithms import validate, fields, localization_backend
from .configuration import load_profile

COMPUTE_ROLES = ('detection', 'multi_tracking', 'single_tracking', 'global_grid', 'combined_grid')

def _merge(target, patch):
    for key, value in patch.items():
        if isinstance(value, dict) and isinstance(target.get(key), dict): _merge(target[key], value)
        else: target[key] = copy.deepcopy(value)
    return target

def _resolve_selection(profile_name, raw_options=None, file_config=None, cli_config=None):
    raw_options = raw_options or {}
    from .simulation import resolve_simulation_modules
    modules = resolve_simulation_modules(profile_name, raw_options)
    options = modules['options']; profile = load_profile(profile_name); registry = Registry()
    strategy = options['strategy'] if options['strategy'] != 'profile' else profile['launch_arguments'].get('navigation_strategy', 'astar')
    localization = localization_backend(options, profile_name)
    defaults = {'backends': {}}
    # Read exactly the installed legacy computation files the nodes would load.
    legacy_files = {}
    if 'multi_tracking' in modules['enabled_roles']:
        tracking_file = Path(get_package_share_directory('easyfly_multi_object_tracking')) / 'config/tracking.yaml'
        tracking = yaml.safe_load(tracking_file.read_text())['tracking']
        native_tracking = tracking.pop('backend_parameters', {})
        selected_tracking = tracking.get('tracker_backend', 'sort')
        registry.get('multi_tracking', selected_tracking)
        old_names = {'iou_threshold','max_lost_frames','min_hits','max_tracks','dist_gate_scale'}
        if selected_tracking == 'sort':
            patch = registry.import_tracking_yaml({'tracking': tracking})
            choice = patch['backends']['multi_tracking']
        else:
            if old_names.intersection(tracking): raise ValueError('legacy SORT parameters conflict with selected backend')
            choice = dict(backend=selected_tracking, parameters={})
        checked = registry.parameters('multi_tracking', selected_tracking, native_tracking)
        for key in native_tracking:
            if key in tracking and checked[key] != tracking[key]: raise ValueError('legacy/new tracking parameter conflict: ' + key)
            choice['parameters'][key] = checked[key]
        defaults['backends']['multi_tracking'] = choice
        legacy_files['multi_tracking'] = str(tracking_file)
    if 'detection' in modules['enabled_roles']:
        detection_file = Path(get_package_share_directory('easyfly_object_detection')) / 'config/detection.yaml'
        detection = yaml.safe_load(detection_file.read_text())['detection']
        schema = registry.get('detection', detection.get('backend', 'onnx'))['parameter_schema']
        properties = json.loads((registry.directory / schema).read_text())['properties']
        detector_values = {k:v for k,v in detection.items() if k in properties}
        native_detection = detection.get('backend_parameters', {})
        checked = registry.parameters('detection', detection.get('backend', 'onnx'), native_detection)
        for key in native_detection:
            if key in detector_values and detector_values[key] != checked[key]: raise ValueError('legacy/new detector parameter conflict: ' + key)
            detector_values[key] = checked[key]
        defaults['backends']['detection'] = dict(backend=detection.get('backend', 'onnx'), parameters=detector_values)
        legacy_files['detection'] = str(detection_file)
    if options['exploration'] == 'fuel':
        defaults['backends']['exploration'] = dict(backend='fuel', parameters=registry.parameters('exploration', 'fuel'))
    ui = {'backends': {}}
    for field in fields():
        key = field['key']; role = field.get('backend_role')
        if key not in raw_options or not role: continue
        value = options[key]
        if field['kind'] == 'choice':
            if value != 'profile':
                ui['backends'].setdefault(role, {})['backend'] = value
                ui['backends'][role].setdefault('parameters', {})
        else:
            ui['backends'].setdefault(role, {}).setdefault('parameters', {})[field['backend_parameter']] = value
    if options['provider'] != 'auto':
        ui['backends'].setdefault('detection', {}).setdefault('parameters', {})['use_gpu'] = options['provider'] == 'cuda'
    # Numeric controls belong to the described backend, never another implementation.
    base = registry.resolve_layers(strategy=strategy, localization=localization, profile=profile_name,
        profile_defaults=defaults, file_config=file_config)[0]
    for field in fields():
        if field.get('backend_parameter') and field['key'] in raw_options:
            selected = ui['backends'].get(field['backend_role'], {}).get('backend', base['backends'][field['backend_role']]['backend'])
            if selected != field['parameter_backend']: raise ValueError('参数不属于所选后端：' + field['label'])
    cli = _merge(ui, cli_config or {})
    effective, provenance = registry.resolve_layers(strategy=strategy, localization=localization, profile=profile_name,
        profile_defaults=defaults, file_config=file_config, cli=cli)
    if any(value for value in effective['platform'].values()): raise ValueError('platform overrides require the dedicated node configuration')
    # Specialized localization/trajectory/exploration routes remain guarded by their own adapters.
    if effective['navigation']['strategy'] != strategy: raise ValueError('navigation strategy conflicts with algorithms configuration')
    if effective['backends'].get('localization', {}).get('backend', 'disabled') != localization:
        raise ValueError('localization selection must use the existing guarded adapter')
    for role in ('localization', 'trajectory_planning', 'exploration'):
        choice = effective['backends'].get(role)
        if choice and choice['parameters'] != registry.parameters(ROLES[role], choice['backend']):
            raise ValueError('use the dedicated adapter parameters: ' + role)
    if 'detection' in modules['enabled_roles']:
        model = effective['backends']['detection']['parameters'].get('model_path', '')
        if model.startswith('package://'):
            package, relative = model[len('package://'):].split('/', 1)
            share = Path(get_package_share_directory(package)).resolve()
            if PurePosixPath(relative).is_absolute() or '..' in PurePosixPath(relative).parts:
                raise ValueError('model asset escapes package share')
            asset = (share / relative).resolve()
            effective['backends']['detection']['parameters']['model_path'] = str(asset)
    enabled = modules['enabled_roles']
    return dict(selection=effective, provenance=provenance, enabled_roles=enabled,
        legacy_files=legacy_files, simulation_modules=modules)

def launch_values(resolved):
    rows = resolved['selection']['backends']; result = {}
    for role in COMPUTE_ROLES:
        item = rows[role]
        result[role + '_backend'] = item['backend']
        result[role + '_parameters'] = json.dumps(item['parameters'], separators=(',', ':'), allow_nan=False)
    return result

def parse_layers(file_name='', json_text=''):
    if file_name:
        path = Path(file_name).expanduser()
        if not path.is_file() or path.stat().st_size > 1024*1024: raise ValueError('backend configuration file missing or too large')
        file_value = loads(path.read_text())
    else: file_value = None
    if len(json_text) > 1024*1024: raise ValueError('backend configuration too large')
    return file_value, loads(json_text) if json_text else None


class SelectionError(ValueError):
    def __init__(self, code, message):
        self.code = code
        super().__init__(code + ': ' + message)


def check_dependencies(resolved):
    """Presence checks only. Runtime health and actual provider remain observed evidence."""
    from ament_index_python.packages import get_package_prefix
    registry = Registry(); missing = []
    nodes = dict(detection=('easyfly_object_detection','detection_node'),
        multi_tracking=('easyfly_multi_object_tracking','tracking_node'),
        single_tracking=('easyfly_single_object_tracking','single_object_tracking_node'),
        global_grid=('easyfly_global_planner','global_planner_node'),
        combined_grid=('easyfly_global_planner','navigation_coordinator_node'))
    for role in resolved['enabled_roles']:
        item = resolved['selection']['backends'][role]
        try:
            registry.check_installation(ROLES[role], item['backend'], get_package_share_directory)
            package, executable = nodes[role]
            path = Path(get_package_prefix(package)) / 'lib' / package / executable
            if not path.is_file() or not os.access(path, os.X_OK): missing.append(str(path))
        except LookupError as error: missing.append(str(error))
    if 'detection' in resolved['enabled_roles']:
        model = resolved['selection']['backends']['detection']['parameters'].get('model_path')
        if model and not Path(model).is_file(): missing.append('model: ' + model)
    if missing: raise SelectionError('dependency_missing', '; '.join(missing))
    return dict(phase='configured', dependencies_present=True, runtime_ready=False)


def resolve_selection(profile_name, raw_options=None, file_config=None, cli_config=None):
    from jsonschema.exceptions import ValidationError
    from ament_index_python.packages import PackageNotFoundError
    try:
        return _resolve_selection(profile_name, raw_options, file_config, cli_config)
    except SelectionError:
        raise
    except (PackageNotFoundError, FileNotFoundError, ModuleNotFoundError) as error:
        raise SelectionError('dependency_missing', str(error)[:4096]) from error
    except (ValueError, TypeError, ValidationError) as error:
        raise SelectionError('unsupported_configuration', str(error)[:4096]) from error
