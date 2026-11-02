# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Resolve the selected simulation algorithms without starting ROS processes."""
import json

from .algorithms import validate, localization_backend
from .configuration import config_directory, load_profile


def module_definitions():
    return json.loads((config_directory() / 'simulation_modules.json').read_text())


def resolve_simulation_modules(profile_name, raw_options=None):
    options = validate(raw_options or {})
    profile = load_profile(profile_name)
    perception = profile['entrypoint'] != 'base'
    backend = localization_backend(options, profile_name)
    strategy = options['strategy']
    if strategy == 'profile':
        strategy = profile['launch_arguments'].get('navigation_strategy', 'astar')
    definitions = {row['id']: row for row in module_definitions()['modules']}
    requested = set()
    for name, row in definitions.items():
        if name == 'slam':
            enabled = perception and backend != 'disabled'
        elif name == 'exploration':
            enabled = options['exploration'] != 'none'
        else:
            enabled = options[row['flag']] and (perception or not row['flag'].startswith('start_'))
        if enabled:
            requested.add(name)
    if 'exploration' in requested and (
            profile_name != 'navigation_dev' or backend != 'fast_lio' or strategy != 'gcopter'):
        raise ValueError('FUEL 需要导航开发运行方案、FAST-LIO 和 GCOPTER')
    effective, automatic, reasons, blocked = set(), set(), {}, {}

    def include(name, parent=None, visiting=()):
        if name in visiting:
            raise ValueError('仿真算法依赖形成循环：' + name)
        if name == 'slam' and backend == 'disabled':
            blocked[name] = '当前配置已关闭 SLAM'
            return False
        if name == 'target_tracking' and backend != 'fast_lio':
            blocked[name] = '当前目标跟随适配器需要 FAST-LIO'
            return False
        row = definitions[name]
        dependencies = row['requires'] + row.get('strategy_requires', {}).get(strategy, [])
        for dependency in dependencies:
            if not include(dependency, name, (*visiting, name)):
                blocked[name] = '缺少可用依赖：' + definitions[dependency]['title']
                return False
        effective.add(name)
        if name not in requested:
            automatic.add(name)
        if parent:
            reasons.setdefault(name, set()).add(parent)
        return True

    for name in sorted(requested):
        include(name)
    normalized = dict(options)
    arguments = {}
    for name, row in definitions.items():
        if 'flag' not in row:
            continue
        normalized[row['flag']] = name in effective
        if row['flag'].startswith('start_'):
            arguments[row['flag']] = 'true' if name in effective else 'false'
    roles = []
    for name, role in [('object_detection', 'detection'), ('tracking', 'multi_tracking'),
                       ('single_tracking', 'single_tracking')]:
        if name in effective:
            roles.append(role)
    if 'navigation' in effective:
        if strategy == 'astar':
            roles.append('global_grid')
        elif strategy == 'combined':
            roles.append('combined_grid')
    return dict(requested=sorted(requested), effective=sorted(effective),
                automatic=sorted(automatic), reasons={k: sorted(v) for k, v in reasons.items()},
                blocked=blocked, launch_arguments=arguments, options=normalized,
                enabled_roles=roles, localization=backend, strategy=strategy)
