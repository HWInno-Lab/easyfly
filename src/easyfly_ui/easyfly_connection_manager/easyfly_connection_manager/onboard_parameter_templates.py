# Copyright 2026 EasyFly Project
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Built-in physical-node parameter templates, separate from persisted recipes."""

import copy
import hashlib
import math
import os
from pathlib import Path
import re
import stat

import yaml

from . import onboard_profiles as profiles


EXTRA_PARAMETERS = {
    'target_tracking': profiles.TRACK_DEFAULTS,
    'lidar_rays': dict(
        input_topic='',
        source_frame='',
        range_min=0.0,
        range_max=0.0,
        rays_topic='/livox/lidar/rays',
    ),
    'gcopter': dict(
        map_min=[],
        map_max=[],
        resolution=0.0,
        inflation_radius=0.0,
        source_frame='',
        z_min=0.0,
        z_max=0.0,
    ),
    'coord': dict(
        uav_state_topic='/uav_state',
        odom_frame='odom',
        base_frame='base_link',
        publish_rate=50.0,
        state_timeout=0.5,
    ),
    'fast_lio': {
        'preprocess.lidar_type': 0,
        'preprocess.scan_line': 0,
        'preprocess.timestamp_unit': -1,
        'preprocess.scan_rate': 0,
        'mapping.extrinsic_T': [],
        'mapping.extrinsic_R': [],
        'mapping.extrinsic_est_en': False,
    },
    'map_odom': dict(
        slam_frame='camera_init',
        odom_topic='/odom',
        slam_topic='/Odometry',
        map_frame='map',
        odom_frame='odom',
        smooth_alpha=0.1,
        body_to_base_translation=[],
    ),
    'tracking': dict(config_file='', tracker_backend='sort', perception_version=1, camera_id='front'),
    'pointcloud_filter': dict(config_file=''),
    'pixel_to_3d': dict(config_file='', perception_version=1, camera_id='front'),
    'semantic_map': dict(
        config_file='',
        grid_footprint_mode='cluster_boxes',
        grid_height_limit_m=0.0,
        ground_model_frame='',
    ),
    'flight_control': dict(
        control_rate=20.0,
        land_timeout=60.0,
        max_altitude=50.0,
        max_horizontal_speed=20.0,
        trajectory_startup_grace=8.0,
    ),
}
REQUIRED = {
    'lidar_rays': dict(
        input_topic='真实 Livox CustomMsg 绝对话题；不可使用仿真点云。',
        source_frame='与 GCOPTER 地图相同的真实输出参考系，依赖有效 TF。',
        range_min='真实雷达最小可信距离，单位米。',
        range_max='真实雷达最大可信距离，单位米。',
    ),
    'gcopter': dict(
        map_min='场地地图 xyz 最小边界，单位米。',
        map_max='场地地图 xyz 最大边界，单位米。',
        resolution='按实际场地设置体素边长，单位米。',
        inflation_radius='按机体尺寸和安全距离设置膨胀半径，单位米。',
        source_frame='与物理 lidar_rays 相同的真实参考系。',
        z_min='审核后的最低飞行高度，必须位于地图范围内。',
        z_max='审核后的最高飞行高度，必须位于地图范围内。',
    ),
    'fast_lio': {
        'preprocess.lidar_type': (
            '1 Livox CustomMsg（AVIA接口，MID360使用CustomMsg也选1）；'
            '2 Velodyne16；3 Ouster64；4 MID360 PointCloud2。'
        ),
        'preprocess.scan_line': '按真实雷达扫描线数填写正整数。',
        'preprocess.timestamp_unit': '按真实雷达时间戳单位填写：0 秒、1 毫秒、2 微秒、3 纳秒。',
        'preprocess.scan_rate': '按真实雷达扫描频率填写正整数，单位 Hz。',
        'mapping.extrinsic_T': '填写真实雷达到 IMU 的平移外参，3 个有限数值，单位米。',
        'mapping.extrinsic_R': '填写真实雷达到 IMU 的旋转矩阵，9 个有限数值，按行排列。',
    },
    'map_odom': {
        'body_to_base_translation': '填写真实 SLAM body 到 base 原点平移，3 个有限数值，单位米。',
    },
}


def catalog():
    """Return editable defaults without adding fields to legacy recipe documents."""
    result = {}
    for name, entry in profiles.template()['components'].items():
        parameters = copy.deepcopy(EXTRA_PARAMETERS.get(name, {}))
        parameters.update(copy.deepcopy(entry['parameters']))
        accepts_file = name not in ('mavlink', 'local_planner', 'precision_landing')
        note = '直接编辑下方参数；保存到平台时生成对应配置文件。'
        if name in ('tracking', 'pointcloud_filter', 'pixel_to_3d', 'semantic_map'):
            note += '模块内部算法配置继续使用其默认配置，或通过 config_file 指定真实文件。'
        if name in ('camera_calib', 'livox'):
            note += '专用标定／设备配置文件仍需按实际硬件提供，不由 ROS 参数模板替代。'
        if name == 'precision_landing':
            note = (
                '请复制 easyfly_precision_landing/config/hardware_template.yaml，填写实测固定下视外参、'
                '标记和控制限值；通过 config_file 上传，并将同一文件用于飞行控制参数文件。'
            )
        elif not accepts_file:
            note = '该适配器使用启动参数表；直接编辑参数，不生成 ROS 参数文件。'
        result[name] = dict(
            parameters=parameters,
            required=copy.deepcopy(REQUIRED.get(name, {})),
            accepts_file=accepts_file,
            notes=note,
            format='gcopter'
            if name == 'gcopter'
            else (
                'tracking'
                if name == 'target_tracking'
                else (
                    'flat'
                    if name in ('realsense', 'down_camera')
                    else ('ros' if accepts_file else 'arguments')
                )
            ),
        )
    return result


def is_template_file(path, name):
    """Recognize generated names for presentation only, never grant file authority."""
    return name in profiles.CATALOG and bool(
        re.fullmatch(
            r'[a-f0-9]{16}-easyfly-template-' + re.escape(name) + r'\.yaml', Path(path).name
        )
    )


def _number(value, key):
    if type(value) not in (int, float):
        raise ValueError('模板数值参数类型无效：' + key)
    try:
        result = float(value)
    except (OverflowError, ValueError) as error:
        raise ValueError('模板数值参数必须为有限数：' + key) from error
    if not math.isfinite(result):
        raise ValueError('模板数值参数必须为有限数：' + key)
    return result


def normalize_parameters(recipe, names):
    """Recover ROS numeric types lost in JSON, only for explicit template components."""
    if (
        not isinstance(names, list)
        or any(not isinstance(name, str) for name in names)
        or len(names) != len(set(names))
    ):
        raise ValueError('内置模板组件清单无效')
    if not isinstance(recipe, dict) or not isinstance(recipe.get('components'), dict):
        raise ValueError('内置模板运行方案无效')
    result = copy.deepcopy(recipe)
    templates = catalog()
    vectors = {
        'fast_lio': {'mapping.extrinsic_T', 'mapping.extrinsic_R'},
        'map_odom': {'body_to_base_translation'},
        'gcopter': {'map_min', 'map_max'},
    }
    for name in names:
        spec = templates.get(name)
        entry = result['components'].get(name)
        if (
            not spec
            or not spec['accepts_file']
            or not isinstance(entry, dict)
            or entry.get('enabled') is not True
            or not isinstance(entry.get('parameters'), dict)
        ):
            raise ValueError('内置模板仅用于已启用且支持参数文件的组件：' + name)
        defaults = spec['parameters']
        parameters = copy.deepcopy(defaults)
        parameters.update(entry['parameters'])
        for key, value in parameters.items():
            if not isinstance(key, str):
                raise ValueError('模板参数名称必须为字符串')
            if key in vectors.get(name, set()):
                if not isinstance(value, list):
                    raise ValueError('模板外参数组类型无效：' + key)
                parameters[key] = [_number(item, key) for item in value]
            elif type(defaults.get(key)) is float:
                parameters[key] = _number(value, key)
            elif type(defaults.get(key)) is int:
                if type(value) is int:
                    number = value
                else:
                    number = _number(value, key)
                    if not number.is_integer():
                        raise ValueError('模板整数参数需要有效整数：' + key)
                if not -(2 ** 63) <= number < 2 ** 63:
                    raise ValueError('模板整数参数需要有效整数：' + key)
                parameters[key] = int(number)
            elif type(defaults.get(key)) is bool and type(value) is not bool:
                raise ValueError('模板布尔参数类型无效：' + key)
            elif type(defaults.get(key)) is str and not isinstance(value, str):
                raise ValueError('模板文本参数类型无效：' + key)
        _physical(parameters)
        entry['parameters'] = parameters
    try:
        profiles.validate(result)
    except (TypeError, KeyError, OverflowError) as error:
        raise ValueError('内置模板运行方案结构无效') from error
    return result


def _physical(parameters):
    for key, value in parameters.items():
        if (key == 'use_sim_time' and value is not False) or key.startswith('gazebo'):
            raise ValueError('机载参数模板不能启用模拟时间或 Gazebo 参数')
        if not profiles.scalar(value):
            raise ValueError('模板参数必须是有限标量或同类型数组：' + key)
        values = value if isinstance(value, list) else [value]
        if any(type(item) is int and not -(2 ** 63) <= item < 2 ** 63 for item in values):
            raise ValueError('模板整数参数必须在 ROS 64 位有符号整数范围内：' + key)


def _vector(value, length, name, key):
    if (
        not isinstance(value, list)
        or len(value) != length
        or any(type(v) not in (int, float) or not math.isfinite(v) for v in value)
    ):
        raise ValueError(name + '：请填写真实硬件参数 ' + key)
    return value


def validate_hardware(name, parameters):
    """Require explicit physical geometry without guessing or forbidding zero offsets."""
    if name == 'lidar_rays':
        from easyfly_trajectory_optimizer.hardware import validate_lidar_parameters

        validate_lidar_parameters(parameters)
    if name == 'map_odom':
        offset = _vector(
            parameters.get('body_to_base_translation'), 3, name, 'body_to_base_translation'
        )
        if any(abs(value) > 100 for value in offset):
            raise ValueError('map_odom：body_to_base_translation 超出节点支持的范围')
    if name != 'fast_lio':
        return
    if type(parameters.get('preprocess.lidar_type')) is not int or parameters[
        'preprocess.lidar_type'
    ] not in (1, 2, 3, 4):
        raise ValueError('FAST-LIO：请按真实雷达填写 preprocess.lidar_type')
    if (
        type(parameters.get('preprocess.scan_line')) is not int
        or parameters['preprocess.scan_line'] <= 0
    ):
        raise ValueError('FAST-LIO：请填写真实 preprocess.scan_line')
    if type(parameters.get('preprocess.timestamp_unit')) is not int or parameters[
        'preprocess.timestamp_unit'
    ] not in (0, 1, 2, 3):
        raise ValueError('FAST-LIO：请填写真实 preprocess.timestamp_unit')
    if (
        type(parameters.get('preprocess.scan_rate')) is not int
        or parameters['preprocess.scan_rate'] <= 0
    ):
        raise ValueError('FAST-LIO：请填写真实 preprocess.scan_rate')
    _vector(parameters.get('mapping.extrinsic_T'), 3, name, 'mapping.extrinsic_T')
    rotation = _vector(parameters.get('mapping.extrinsic_R'), 9, name, 'mapping.extrinsic_R')
    rows = [rotation[i:i + 3] for i in (0, 3, 6)]
    for i in range(3):
        for j in range(3):
            if abs(sum(rows[i][k] * rows[j][k] for k in range(3)) - int(i == j)) > 0.01:
                raise ValueError('FAST-LIO：mapping.extrinsic_R 必须是正交旋转矩阵')
    a, b, c, d, e, f, g, h, i = rotation
    determinant = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    if abs(determinant - 1.0) > 0.01:
        raise ValueError('FAST-LIO：mapping.extrinsic_R 必须为右手旋转矩阵')


def materialize(recipe, names, root):
    """Validate every requested template before writing immutable generated files."""
    result = normalize_parameters(recipe, names)
    templates = catalog()
    documents = []
    for name in names:
        spec = templates.get(name)
        entry = result['components'].get(name)
        if not spec or not spec['accepts_file'] or not entry or not entry['enabled']:
            raise ValueError('内置模板仅用于已启用且支持参数文件的组件：' + name)
        parameters = copy.deepcopy(spec['parameters'])
        parameters.update(entry['parameters'])
        _physical(parameters)
        validate_hardware(name, parameters)
        # All adapters use physical time. Realsense consumes a flat mapping,
        # ordinary ROS nodes consume a matching wildcard parameter scope.
        values = dict(parameters, use_sim_time=False)
        if spec['format'] == 'gcopter':
            mapper_keys = ('map_min', 'map_max', 'resolution', 'inflation_radius', 'source_frame')
            document = {
                'gcopter_voxel_map': {
                    'ros__parameters': {key: parameters[key] for key in mapper_keys}
                },
                'gcopter_navigation': {
                    'ros__parameters': {key: parameters[key] for key in ('z_min', 'z_max')}
                },
            }
            from easyfly_trajectory_optimizer.hardware import validate_hardware_parameters

            validate_hardware_parameters(document)
        elif spec['format'] == 'tracking':
            document = {
                'target_tracking': {key: parameters[key] for key in profiles.TRACK_DEFAULTS}
            }
            profiles.validate_tracking_configuration(document)
        else:
            document = values if spec['format'] == 'flat' else {'/**': {'ros__parameters': values}}
        content = yaml.dump(document, Dumper=_Dumper, allow_unicode=True, sort_keys=True).encode()
        if not 0 < len(content) <= 65536:
            raise ValueError('生成的模板超过 64 KiB：' + name)
        identity = hashlib.sha256(content).hexdigest()
        entry['parameters'] = parameters
        documents.append((name, identity, content))
    profiles.validate(result)
    directory = Path(root)
    if (
        not directory.is_absolute()
        or directory.resolve() != directory
        or not directory.is_dir()
        or directory.stat().st_uid != os.geteuid()
        or directory.stat().st_mode & 0o022
    ):
        raise ValueError('模板目录必须是机载服务配置的规范、受管目录')
    for name, identity, content in documents:
        path = directory / (identity[:16] + '-easyfly-template-' + name + '.yaml')
        if path.exists() or path.is_symlink():
            _existing(path, content)
    for name, identity, content in documents:
        path = directory / (identity[:16] + '-easyfly-template-' + name + '.yaml')
        try:
            descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
        except FileExistsError:
            _existing(path, content)
        else:
            with os.fdopen(descriptor, 'wb') as stream:
                stream.write(content)
                stream.flush()
                os.fsync(stream.fileno())
        if name == 'target_tracking':
            result['components'][name]['parameters'] = {'config_file': str(path)}
            result['components'][name]['parameter_file'] = ''
        else:
            if name == 'gcopter':
                result['components'][name]['parameters'] = {}
            result['components'][name]['parameter_file'] = str(path)
    return result


class _Dumper(yaml.SafeDumper):
    """Serialize independent values without aliases rejected by hardware admission."""

    def ignore_aliases(self, data):
        """Emit values rather than YAML anchors."""
        return True


def _existing(path, content):
    if path.is_symlink():
        raise ValueError('已有不同内容或不安全的模板文件，未覆盖')
    with os.fdopen(os.open(path, os.O_RDONLY | os.O_NOFOLLOW), 'rb') as stream:
        info = os.fstat(stream.fileno())
        if (
            not stat.S_ISREG(info.st_mode)
            or info.st_uid != os.geteuid()
            or info.st_mode & 0o022
            or stream.read(65537) != content
        ):
            raise ValueError('已有不同内容或不安全的模板文件，未覆盖')
