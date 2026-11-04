# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_params/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project
#

"""Bounded, shared UI configuration for existing ROS algorithm adapters."""
import json
import math
from pathlib import Path
import re


def fields():
    from easyfly_backend_registry import fields as registered_fields
    return registered_fields()


def validate(options):
    schema = fields()
    defaults = {field['key']: field['default'] for field in schema}
    if not isinstance(options, dict) or set(options) - defaults.keys():
        raise ValueError('算法配置字段不受支持')
    result = dict(defaults, **options)
    for field in schema:
        value = result[field['key']]
        kind = field['kind']
        valid = False
        if kind == 'bool':
            valid = type(value) is bool
        elif kind == 'choice':
            valid = value in [item[0] for item in field['choices']]
        elif kind == 'topic':
            valid = isinstance(value, str) and len(value) <= 256 and re.fullmatch(
                r'/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*', value)
        elif kind == 'string':
            valid = isinstance(value, str) and len(value) <= 128 and (
                not value or re.fullmatch(r'[A-Za-z][A-Za-z0-9_/]*', value))
        elif kind in ('int', 'number'):
            valid = type(value) in (int, float) and math.isfinite(value) and (
                field['min'] <= value <= field['max'])
            if kind == 'int':
                valid = valid and int(value) == value
                if valid:
                    result[field['key']] = int(value)
            elif valid:
                result[field['key']] = float(value)
        if not valid:
            raise ValueError('算法参数无效：' + field['label'])
    if (result['canny_low'] >= result['canny_high'] or
            result['saturation_min'] > result['saturation_max'] or
            result['value_min'] > result['value_max'] or
            result['morphology_kernel'] % 2 != 1):
        raise ValueError('算法阈值顺序或形态学核无效')
    if result['slam_blind'] >= result['slam_det_range']:
        raise ValueError('SLAM 最小有效距离必须小于最大探测距离')
    return result


def localization_backend(options, profile):
    selection = options['slam']
    from easyfly_backend_registry import Registry
    registered = Registry().ids('localization')
    if selection in registered and profile == 'base_sim':
        labels = dict(next(field for field in fields() if field['key'] == 'slam')['choices'])
        raise ValueError(labels.get(selection, selection) +
                         ' 需要选择感知、导航或降落准备运行方案')
    if selection in registered and selection != 'fast_lio':
        defaults = {field['key']: field['default'] for field in fields()}
        if any(options[key] != defaults[key] for key in defaults if key.startswith('slam_')):
            raise ValueError('所选定位后端使用固定 Gazebo 标定配置，不能应用 FAST-LIO 参数')
    return ('disabled' if profile == 'base_sim' else
            'fast_lio' if selection == 'profile' else selection)


def slam_enabled(options, profile):
    return localization_backend(options, profile) != 'disabled'


def slam_parameters(options):
    mapping = dict(slam_lidar_topic='common.lid_topic', slam_imu_topic='common.imu_topic',
                   slam_point_filter_num='point_filter_num', slam_max_iteration='max_iteration',
                   slam_filter_size_surf='filter_size_surf',
                   slam_filter_size_map='filter_size_map',
                   slam_blind='preprocess.blind', slam_det_range='mapping.det_range')
    return {parameter: options[key] for key, parameter in mapping.items()}


def detector_parameters(options, kind):
    common = ('max_age_seconds', 'opencv_threads', 'max_detections')
    specific = (('min_axis_px', 'max_axis_ratio', 'max_radial_error', 'max_area_error',
                 'canny_low', 'canny_high') if kind == 'ellipse' else
                ('hue_min', 'hue_max', 'saturation_min', 'saturation_max', 'value_min',
                 'value_max', 'morphology_kernel', 'min_area_px', 'min_length_px',
                 'min_aspect_ratio', 'min_fill_ratio'))
    return {key: options[key] for key in common + specific}
