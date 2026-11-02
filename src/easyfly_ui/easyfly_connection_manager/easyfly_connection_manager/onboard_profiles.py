# Copyright 2026 EasyFly Project
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Typed, environment-independent hardware recipes; never accept shell commands."""

import copy
import hashlib
import json
import math
import os
from pathlib import Path
import re
import stat

import yaml

from ament_index_python.packages import get_package_prefix, get_package_share_directory


def component(package, executable, title, output="", message="", plugin="", files=()):
    """Describe an installed adapter and its observable output."""
    return dict(
        package=package,
        executable=executable,
        title=title,
        output=output,
        message=message,
        plugin=plugin,
        files=list(files),
    )


CATALOG = {
    "mavlink": component(
        "easyfly_mavlink_bridge",
        "mavlink_bridge.launch.py",
        "飞控通信 / MAVROS",
        "/uav_state",
        "easyfly_msgs/msg/UAVState",
    ),
    "realsense": component(
        "realsense2_camera",
        "rs_launch.py",
        "RealSense 相机",
        "/camera/color/image_raw",
        "sensor_msgs/msg/Image",
    ),
    "livox": component(
        "livox_ros_driver2",
        "livox_ros_driver2_node",
        "Livox 雷达",
        "/livox/lidar",
        "livox_ros_driver2/msg/CustomMsg",
        files=("user_config_path",),
    ),
    "camera_calib": component(
        "easyfly_camera_calib",
        "",
        "相机外参 / TF",
        plugin="easyfly_camera_calib::CalibTfPublisher",
        files=("calib_file",),
    ),
    "coord": component(
        "easyfly_coord_transform",
        "",
        "飞控里程计 / 坐标变换",
        "/odom",
        "nav_msgs/msg/Odometry",
        plugin="easyfly_coord_transform::OdomTfPublisher",
    ),
    "fast_lio": component(
        "fast_lio",
        "fastlio_mapping",
        "FAST-LIO 定位建图",
        "/Odometry",
        "nav_msgs/msg/Odometry",
        files=("parameter_file",),
    ),
    "map_odom": component(
        "easyfly_fast_lio_integration",
        "",
        "地图与飞控坐标对齐",
        plugin="easyfly_fast_lio_integration::MapOdomTf",
        files=("parameter_file",),
    ),
    "detection": component(
        "easyfly_object_detection",
        "",
        "YOLO 目标检测",
        "/detections",
        "easyfly_msgs/msg/DetectionInfoArray",
        plugin="easyfly_object_detection::DetectionNode",
    ),
    "tracking": component(
        "easyfly_multi_object_tracking",
        "",
        "多目标跟踪",
        "/tracks",
        "easyfly_msgs/msg/TrackInfoArray",
        plugin="easyfly_multi_object_tracking::TrackingNode",
    ),
    "pixel_to_3d": component(
        "easyfly_pixel_to_3d", "", "目标三维定位", plugin="easyfly_pixel_to_3d::PixelTo3dNode"
    ),
    "pointcloud_filter": component(
        "easyfly_pointcloud_filter",
        "pointcloud_filter_node",
        "点云滤波",
        "/pointcloud_filtered",
        "sensor_msgs/msg/PointCloud2",
    ),
    "semantic_map": component(
        "easyfly_semantic_map",
        "semantic_map_node",
        "障碍与占据地图",
        "/occupancy_grid",
        "nav_msgs/msg/OccupancyGrid",
    ),
    "navigation": component("easyfly_global_planner", "navigate.launch.py", "全局导航 / 避障"),
    "local_planner": component("easyfly_local_planner", "local_planner.launch.py", "EGO 局部规划"),
    "flight_control": component(
        "easyfly_flight_control", "flight_control_node", "飞行控制动作服务", files=("parameter_file",)
    ),
    "mission_fsm": component("easyfly_mission_fsm", "mission_fsm.launch.py", "任务状态机"),
    "mission_services": component("easyfly_mission_srvs", "mission_services", "任务目录与管理服务"),
    "color_line": component(
        "easyfly_color_line_detection",
        "color_line_detection_node",
        "彩线检测",
        "/color_line_observations",
        "easyfly_msgs/msg/ColorLineObservationArray",
    ),
    "ellipse": component(
        "easyfly_ellipse_detection",
        "ellipse_detection_node",
        "椭圆检测",
        "/ellipse_observations",
        "easyfly_msgs/msg/EllipseObservationArray",
    ),
}

# Physical adapters are separate from image tracking and simulation profiles.
CATALOG.update(
    {
        'down_camera': component(
            'realsense2_camera',
            'rs_launch.py',
            '下视 RealSense 相机',
            '/camera_down/color/image_raw',
            'sensor_msgs/msg/Image',
        ),
        'target_tracking': component(
            'easyfly_target_tracking', 'target_tracking_node', '目标跟随动作服务', files=('config_file',)
        ),
        'precision_landing': component(
            'easyfly_precision_landing',
            'hardware_precision_landing.launch.py',
            '固定下视精准降落',
            '/tag_observations',
            'easyfly_msgs/msg/TagObservationArray',
            files=('config_file',),
        ),
        'lidar_rays': component(
            'easyfly_trajectory_optimizer',
            'lidar_rays_node',
            '真实雷达射线',
            '/livox/lidar/rays',
            'sensor_msgs/msg/PointCloud2',
            files=('parameter_file',),
        ),
        'gcopter': component(
            'easyfly_trajectory_optimizer',
            'hardware_gcopter.launch.py',
            'GCOPTER 轨迹规划',
            '/planning/fuel_raw_grid',
            'easyfly_msgs/msg/VoxelGrid',
            files=('parameter_file',),
        ),
        'fuel': component(
            'easyfly_fuel_exploration', 'fuel.launch.py', 'FUEL 自主探索', files=('parameter_file',)
        ),
    }
)

# Presentation and responsibility metadata belong to the catalog, not to the
# persisted recipe schema. A coordinate helper is not an alternative SLAM, and a
# local trajectory planner is not an alternative global navigation strategy.
COMPONENT_GROUPS = {
    'devices': (
        '设备与标定',
        ('mavlink', 'realsense', 'down_camera', 'livox', 'camera_calib', 'coord'),
    ),
    'localization': ('定位与建图', ('fast_lio', 'map_odom')),
    'mapping': ('地图处理', ('pointcloud_filter', 'semantic_map')),
    'navigation': (
        '导航与规划',
        (
            'navigation',
            'local_planner',
            'lidar_rays',
            'gcopter',
            'fuel',
            'target_tracking',
            'precision_landing',
        ),
    ),
    'perception': ('图像感知', ('detection', 'tracking', 'pixel_to_3d', 'color_line', 'ellipse')),
    'services': ('平台运行服务', ('flight_control', 'mission_fsm', 'mission_services')),
}
COMPONENT_ROLES = {
    'fast_lio': 'primary_localization',
    'map_odom': 'map_frame_alignment',
    'navigation': 'global_navigation',
    'local_planner': 'local_trajectory',
}
COMPONENT_DETAILS = {
    'mavlink': dict(
        description='通过 MAVROS 连接飞控，将飞控遥测转换为 EasyFly 飞机状态。',
        inputs=['飞控 MAVLink 链路及对应串口或网络配置'],
        notes='通信连接不等于任务执行许可；需按实际接线配置地址、端口或串口。',
    ),
    'realsense': dict(
        description='启动 RealSense 相机驱动，提供配置启用的彩色、深度等图像数据。',
        inputs=['已连接的 RealSense 相机及驱动配置'],
        notes='深度与彩色图像是否对齐取决于设备及配置，不能仅凭图像话题存在确认标定有效。',
    ),
    'livox': dict(
        description='启动 Livox 雷达驱动，为定位与点云处理提供雷达测量数据。',
        inputs=['Livox 设备及其设备、网络配置文件'],
        notes='设备地址、数据格式及 IMU 数据需与下游定位配置匹配。',
    ),
    'camera_calib': dict(
        description='读取选定的传感器标定配置，发布相机及传感器之间的静态坐标变换。',
        inputs=['标定文件中的参考系与外参'],
        notes='这里应用已有标定，不会自动采集图像或重新求解相机标定。',
    ),
    'coord': dict(
        description='将飞控飞机状态转换为里程计，并提供机体相关坐标变换。',
        inputs=['有效飞机位姿与姿态状态'],
        notes='它是飞控坐标适配，不是独立 SLAM 或另一套定位算法。',
    ),
    'fast_lio': dict(
        description='融合激光雷达与 IMU 测量，估计运动位姿并增量建立点云地图。',
        inputs=['匹配的激光点云与 IMU 测量', '雷达、IMU 外参及时间配置'],
        notes='需按实际传感器选择标定与参数；模块启动不代表定位精度已经验收。',
    ),
    'map_odom': dict(
        description='结合 SLAM 位姿和飞控里程计，建立地图与飞控里程计之间的坐标变换。',
        inputs=['FAST-LIO 位姿', '飞控里程计及对应参考系外参'],
        notes='它负责坐标对齐，不替代 FAST-LIO，也不独立执行建图。',
    ),
    'detection': dict(
        description='对输入图像进行 YOLO 目标检测，输出类别、置信度与二维检测框。',
        inputs=['相机图像', '检测模型与推理配置'],
        notes='二维检测不提供稳定目标身份或世界坐标；推理设备以实际可用的后端为准。',
    ),
    'tracking': dict(
        description='关联连续检测帧，以 SORT 维护多目标轨迹并整合目标身份。',
        inputs=['二维目标检测结果', '可选的目标三维位置回流'],
        notes='原始匹配轨迹用于深度定位；预测展示框不能冒充新的三维观测。',
    ),
    'pixel_to_3d': dict(
        description='结合目标框、深度与相机内参反投影目标位置，并转换到配置的参考系。',
        inputs=['当前检测帧匹配的原始轨迹', '对齐深度图与相机内参', '相机到目标参考系的 TF'],
        notes='深度或变换无效时不能得到有效三维位置；不保证遮挡情况下的位置正确性。',
    ),
    'pointcloud_filter': dict(
        description='按距离与体素尺寸过滤雷达点云，并按测量时间转换到配置的输出参考系。',
        inputs=['Livox 点云', '测量时刻有效的坐标变换'],
        notes='缺少或过期的 TF 不会使用最新位姿替代；本模块不生成导航路线。',
    ),
    'semantic_map': dict(
        description='从过滤点云识别可信地面和障碍，发布障碍信息、非地面点云与占据地图。',
        inputs=['过滤后的点云', '新鲜的飞机状态及有效坐标变换'],
        notes='可信地面与输入有效性条件不满足时，地图会失效；不能把旧地图视为当前可用。',
    ),
    'navigation': dict(
        description='按选定策略协调全局导航、路线规划与执行，支持 A*、EGO、组合及 GCOPTER / FUEL 策略。',
        inputs=['有效地图、飞机状态和定位坐标关系', '显式提交的导航目标'],
        notes='EGO 或组合策略还需要局部规划模块；启动导航服务不会自动提交目标或起飞。',
    ),
    'local_planner': dict(
        description='运行 EGO 局部轨迹规划及轨迹服务器，供受管导航链生成局部运动参考。',
        inputs=['可信非地面点云', '匹配参考系的里程计', '导航协调器接纳的目标'],
        notes='它是局部轨迹模块，不是第二套全局导航；受管模式不直接接纳旧 RViz 目标入口。',
    ),
    'flight_control': dict(
        description='提供起飞、航点、轨迹与降落等动作服务，并管理飞行控制权与参考输出。',
        inputs=['新鲜有效的飞控状态与里程计', '通过准入检查的显式飞行动作请求'],
        notes='模块启动不执行飞行；普通位置与轨迹路径仍使用 PX4 的控制回路。',
    ),
    'mission_fsm': dict(
        description='按已提交任务的步骤组织执行，调用飞行动作与导航并发布任务进度。',
        inputs=['通过校验的任务内容', '已就绪的飞行动作和导航服务'],
        notes='启动状态机不会自动执行任务；任务提交、取消与执行状态仍需单独管理。',
    ),
    'mission_services': dict(
        description='提供任务目录、内容检查与审阅，以及受管任务的提交、查询和取消接口。',
        inputs=['配置的任务目录', '任务状态机及受管任务请求身份'],
        notes='读取或保存任务不等于执行；执行仍需内容审阅、身份和当前运行条件检查。',
    ),
    'color_line': dict(
        description='从彩色图像提取配置颜色的线状区域，输出像素坐标下的彩线几何观测。',
        inputs=['BGR8 或 RGB8 彩色图像', '颜色阈值及几何筛选参数'],
        notes='不直接提供世界坐标或飞控命令；有效空列表表示当前图像未检出目标。',
    ),
    'ellipse': dict(
        description='从图像提取并筛选椭圆轮廓，输出像素坐标下的椭圆几何观测。',
        inputs=['彩色或单通道图像', '轮廓及椭圆拟合筛选参数'],
        notes='不直接提供世界坐标或飞控命令；有效空列表表示当前图像未检出目标。',
    ),
}
for _name in (
    'down_camera',
    'target_tracking',
    'precision_landing',
    'lidar_rays',
    'gcopter',
    'fuel',
):
    COMPONENT_DETAILS[_name] = dict(
        description=CATALOG[_name]['title'],
        inputs=['真实传感器、标定及匹配参考系'],
        notes='模块启动不提交目标或起飞；必须先核验真实硬件参数。',
    )
for _group, (_title, _names) in COMPONENT_GROUPS.items():
    for _name in _names:
        CATALOG[_name].update(
            group=_group, group_title=_title, role=COMPONENT_ROLES.get(_name, _name)
        )
        CATALOG[_name].update(COMPONENT_DETAILS[_name])

NODE_NAMES = {
    "target_tracking": "easyfly_target_tracking",
    "precision_landing": "landing_transform",
    "lidar_rays": "lidar_rays",
    "gcopter": "gcopter_navigation",
    "fuel": "fuel_exploration",
    "color_line": "color_line_detection",
    "ellipse": "ellipse_detection",
    "mavlink": "mavlink_bridge",
    "livox": "livox_lidar_publisher",
    "camera_calib": "calib_tf_publisher",
    "coord": "odom_tf_publisher",
    "fast_lio": "laser_mapping",
    "map_odom": "map_odom_tf",
    "detection": "easyfly_object_detection",
    "tracking": "easyfly_multi_object_tracking",
    "pixel_to_3d": "easyfly_pixel_to_3d",
    "pointcloud_filter": "easyfly_pointcloud_filter",
    "semantic_map": "easyfly_semantic_map",
    "flight_control": "flight_controller",
    "mission_services": "easyfly_mission_srvs",
    "mission_fsm": "mission_fsm",
    "local_planner": "ego_planner_node",
    "navigation": "navigation_coordinator",
}
DEPENDENCIES = {
    "target_tracking": ["pixel_to_3d", "flight_control"],
    "precision_landing": ["down_camera", "flight_control"],
    "lidar_rays": ["livox", "camera_calib", "coord"],
    "gcopter": ["lidar_rays", "map_odom", "flight_control"],
    "fuel": ["gcopter"],
    "coord": ["mavlink"],
    "fast_lio": ["livox"],
    "map_odom": ["fast_lio", "coord"],
    "detection": ["realsense"],
    "tracking": ["detection"],
    "pixel_to_3d": ["tracking", "camera_calib", "coord"],
    "pointcloud_filter": ["fast_lio"],
    "semantic_map": ["pointcloud_filter", "coord"],
    "local_planner": ["semantic_map", "map_odom"],
    "navigation": ["semantic_map", "coord", "map_odom"],
    "flight_control": ["mavlink", "coord"],
    "mission_fsm": ["flight_control", "navigation"],
    "mission_services": ["mission_fsm"],
    "color_line": ["realsense"],
    "ellipse": ["realsense"],
}
TOPIC = re.compile(r"/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*")
FILES = {"parameter_file", "config_file", "calib_file", "user_config_path"}


TRACK_DEFAULTS = dict(
    input_topic='/target_positions',
    uav_state_topic='/uav_state',
    reissue_threshold=1.0,
    default_follow_distance=4.0,
    default_follow_height=1.5,
    max_altitude=2.5,
    max_target_speed=3.0,
    max_target_jump=2.0,
    jump_guard_margin=0.5,
    max_observation_gap=1.0,
    max_follow_radius=35.0,
    state_timeout=1.0,
    child_cancel_timeout=5.0,
    child_cancel_max_wait=30.0,
    first_seen_timeout=15.0,
    lost_timeout=5.0,
    reacquire_radius=2.5,
)


def validate_tracking_configuration(document):
    """Match the native follow node's strict custom YAML contract."""
    if not isinstance(document, dict) or set(document) != {'target_tracking'}:
        raise ValueError('expected target_tracking configuration')
    values = document['target_tracking']
    if not isinstance(values, dict) or set(values) != set(TRACK_DEFAULTS):
        raise ValueError('target_tracking requires all known configuration keys')
    for key, default in TRACK_DEFAULTS.items():
        value = values[key]
        if isinstance(default, str):
            if not isinstance(value, str) or not TOPIC.fullmatch(value):
                raise ValueError('target_tracking requires absolute topics: ' + key)
        elif type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError('target_tracking requires positive finite limits: ' + key)
    if (
        values['default_follow_height'] > values['max_altitude']
        or values['child_cancel_timeout'] > values['child_cancel_max_wait']
    ):
        raise ValueError('inconsistent target_tracking safety limits')
    return values


def digest(value):
    """Content identity is independent of JSON formatting."""
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, allow_nan=False, separators=(",", ":")).encode()
    ).hexdigest()


def template():
    """Provide editable hardware choices, without guessing device IDs or calibration."""
    components = {}
    for name, spec in CATALOG.items():
        parameters = {}
        if name == "mavlink":
            parameters = dict(fcu_url="/dev/ttyACM0:921600", target_system=1, gcs_url="")
        elif name == "livox":
            parameters = dict(
                xfer_format=1,
                multi_topic=0,
                data_src=0,
                publish_freq=10.0,
                output_data_type=0,
                frame_id="livox_frame",
                user_config_path="",
            )
        elif name in ("realsense", "down_camera"):
            parameters = dict(
                camera_namespace="",
                camera_name="camera",
                enable_color=True,
                enable_depth=True,
                **{"align_depth.enable": True}
            )
        elif name in ('target_tracking', 'precision_landing'):
            parameters = dict(config_file='')
            if name == 'precision_landing':
                parameters.update(
                    image_topic='/camera_down/color/image_raw',
                    camera_info_topic='/camera_down/color/camera_info',
                )
        elif name == "camera_calib":
            parameters = dict(calib_file="")
        elif name == "fast_lio":
            from easyfly_params.algorithms import slam_parameters, validate as validate_algorithms

            parameters = slam_parameters(validate_algorithms({}))
        elif name == "detection":
            parameters = dict(use_gpu_override="auto", publish_debug_image_override="false")
        elif name == "navigation":
            parameters = dict(strategy="astar", localization_backend="fast_lio")
        elif name == "local_planner":
            parameters = dict(
                max_acceleration="2.0", virtual_ceil_height="3.2", navigation_managed=True
            )
        elif name in ("color_line", "ellipse"):
            from easyfly_params.algorithms import (
                detector_parameters,
                validate as validate_algorithms,
            )

            parameters = dict(
                detector_parameters(validate_algorithms({}), name),
                input_topic="/camera/color/image_raw",
                output_topic=spec["output"],
            )
        if name == 'down_camera':
            parameters.update(camera_name='camera_down', serial_no='', enable_depth=False)
        components[name] = dict(
            enabled=False,
            parameters=parameters,
            parameter_file="",
            remappings={},
            output_topic=spec["output"],
            depends_on=DEPENDENCIES.get(name, []),
            startup_timeout_s=45,
        )
    return dict(schema_version=1, name="机载运行方案", autostart=False, components=components)


def scalar(value):
    """ROS scalar and homogeneous scalar-array parameters only."""
    if type(value) in (str, int, float, bool):
        return (not isinstance(value, str) or len(value) <= 4096) and (
            not isinstance(value, float) or math.isfinite(value)
        )
    return (
        isinstance(value, list)
        and len(value) <= 256
        and all(type(item) in (str, int, float, bool) and scalar(item) for item in value)
        and len({type(item) for item in value}) <= 1
    )


def component_dependencies(recipe, name):
    """Return declared inputs plus the managed navigation strategy requirement."""
    if name not in CATALOG:
        raise ValueError('unknown onboard component: ' + str(name))
    entry = recipe.get('components', {}).get(name, {})
    dependencies = list(entry.get('depends_on', DEPENDENCIES.get(name, [])))
    if (
        name == 'navigation'
        and entry.get('parameters', {}).get('strategy', 'astar') in ('ego', 'combined')
        and 'local_planner' not in dependencies
    ):
        dependencies.append('local_planner')
    if name == 'navigation' and entry.get('parameters', {}).get('strategy') == 'gcopter':
        dependencies = ['gcopter', 'fuel']
    return dependencies


def complete_recipe(recipe):
    """Expose newly installed choices without enabling them or changing saved components."""
    validate(recipe)
    result = copy.deepcopy(recipe)
    for name, entry in template()['components'].items():
        result['components'].setdefault(name, entry)
    return result


def enable_dependencies(recipe, names):
    """
    Prepare an explicit UI selection without changing or saving its input draft.

    Dependency enabling is a preview action. Validation does not silently enable
    omitted sensor inputs: operators may still supply those inputs externally.
    """
    selected = [names] if isinstance(names, str) else list(names)
    result = copy.deepcopy(recipe)
    components = result.setdefault('components', {})
    defaults = template()['components']
    added, visiting, visited = [], set(), set()

    def enable(name):
        if name not in CATALOG:
            raise ValueError('unknown onboard component: ' + str(name))
        if name in visiting:
            raise ValueError('cyclic onboard dependencies')
        if name in visited:
            return
        visiting.add(name)
        if name not in components:
            components[name] = copy.deepcopy(defaults[name])
        if not components[name].get('enabled', False):
            components[name]['enabled'] = True
            added.append(name)
        for parent in component_dependencies(result, name):
            enable(parent)
        visiting.remove(name)
        visited.add(name)

    for name in selected:
        enable(name)
    return dict(recipe=result, added=added)


def validate_link(value, optional=False):
    """Accept known MAVLink transports without shell or unrelated file paths."""
    if optional and value == "":
        return
    if not isinstance(value, str):
        raise ValueError("飞控/转发地址必须是字符串")
    serial = re.fullmatch(r"(/dev/[A-Za-z0-9_./-]+):([0-9]{4,7})", value)
    if serial and os.path.normpath(serial[1]) == serial[1] and 1200 <= int(serial[2]) <= 4000000:
        return
    scheme, separator, endpoints = value.partition("://")
    if separator and scheme in ("udp", "tcp"):
        parts = endpoints.split("@")
        if 1 <= len(parts) <= (2 if scheme == "udp" else 1):
            for endpoint in parts:
                if not endpoint and scheme == "udp" and len(parts) == 2:
                    continue
                match = re.fullmatch(r"[A-Za-z0-9.-]*:([0-9]{1,5})", endpoint)
                if not match or not 1 <= int(match[1]) <= 65535:
                    break
            else:
                return
    raise ValueError("需要 /dev/设备:波特率、udp://本地:端口@飞控:端口或 tcp://主机:端口")


def validate(recipe, require_files=False, standalone=False):
    """Validate the whole dependency graph before any configuration or process changes."""
    if (
        not isinstance(recipe, dict)
        or set(recipe) != {"schema_version", "name", "autostart", "components"}
        or type(recipe["schema_version"]) is not int
        or recipe["schema_version"] != 1
        or not isinstance(recipe["name"], str)
        or not 1 <= len(recipe["name"]) <= 128
        or type(recipe["autostart"]) is not bool
        or not isinstance(recipe["components"], dict)
        or not set(recipe["components"]) <= CATALOG.keys()
    ):
        raise ValueError("invalid onboard recipe")
    active_roles = {}
    for name, entry in recipe["components"].items():
        if (
            not isinstance(entry, dict)
            or set(entry)
            != {
                "enabled",
                "parameters",
                "parameter_file",
                "remappings",
                "output_topic",
                "depends_on",
                "startup_timeout_s",
            }
            or type(entry["enabled"]) is not bool
            or not isinstance(entry["parameters"], dict)
            or len(entry["parameters"]) > 128
            or not isinstance(entry["parameter_file"], str)
            or not isinstance(entry["remappings"], dict)
            or len(entry["remappings"]) > 32
            or not isinstance(entry["depends_on"], list)
            or any(
                not isinstance(key, str) or key not in CATALOG or key == name
                for key in entry["depends_on"]
            )
            or len(set(entry["depends_on"])) != len(entry["depends_on"])
            or type(entry["startup_timeout_s"]) is not int
            or not 2 <= entry["startup_timeout_s"] <= 180
            or not isinstance(entry["output_topic"], str)
            or bool(entry["output_topic"]) != bool(CATALOG[name]["output"])
            or (entry["output_topic"] and not TOPIC.fullmatch(entry["output_topic"]))
        ):
            raise ValueError("invalid onboard component: " + name)
        for key, value in entry["parameters"].items():
            if (
                not re.fullmatch("[A-Za-z_][A-Za-z0-9_.]*", key)
                or not scalar(value)
                or key == "use_sim_time"
                or key.startswith("gazebo")
            ):
                raise ValueError("invalid hardware parameter: " + name + "/" + key)
        for source, target in entry["remappings"].items():
            if (
                not TOPIC.fullmatch(source)
                or not isinstance(target, str)
                or not TOPIC.fullmatch(target)
            ):
                raise ValueError("remappings require absolute ROS topic names")
        if not entry["enabled"]:
            continue
        role = CATALOG[name].get('role', name)
        if role in active_roles:
            raise ValueError('同一职责不能同时启用多个组件：' + active_roles[role] + ' / ' + name)
        active_roles[role] = name
        if name == 'down_camera':
            serial = entry['parameters'].get('serial_no', '')
            front = recipe['components'].get('realsense', {})
            if not isinstance(serial, str) or not serial.strip():
                raise ValueError('下视相机必须填写真实 serial_no')
            if front.get('enabled') and (
                not front.get('parameters', {}).get('serial_no')
                or front['parameters']['serial_no'] == serial
            ):
                raise ValueError('双相机必须指定不同的真实 serial_no')
        if (
            not standalone
            and name == 'navigation'
            and entry['parameters'].get('strategy') == 'gcopter'
        ):
            if (
                entry['parameters'].get('session_service', '/planning/gcopter_session')
                != '/planning/gcopter_session'
            ):
                raise ValueError('GCOPTER navigation requires its isolated session service')
            if not all(
                recipe['components'].get(key, {}).get('enabled') for key in ('gcopter', 'fuel')
            ):
                raise ValueError('GCOPTER navigation requires gcopter and fuel')
        if name == 'gcopter' and recipe['components'].get('local_planner', {}).get('enabled'):
            raise ValueError('GCOPTER 与 EGO 不得同时输出运动参考')
        if name in ('target_tracking', 'precision_landing') and entry['parameter_file']:
            raise ValueError(name + ' 必须使用专用 config_file，不能用 ROS 参数覆盖')
        if name in ('gcopter', 'fuel', 'lidar_rays') and entry['remappings']:
            raise ValueError(
                'exploration hardware topics cannot be remapped outside the reviewed adapter'
            )
        if name == 'fuel' and entry['parameters'].get('extended_profile', False) is not False:
            raise ValueError('hardware FUEL requires extended_profile=false')
        if name == 'precision_landing' and (
            set(entry['parameters']) - {'config_file', 'image_topic', 'camera_info_topic'}
            or entry['remappings']
        ):
            raise ValueError(
                'precision hardware config and camera topics must be explicit; no overrides'
            )
        if name == 'target_tracking' and set(entry['parameters']) - (
            set(TRACK_DEFAULTS) | {'config_file', 'perception_version', 'camera_id'}
        ):
            raise ValueError('unknown target_tracking parameter override')
        if name == "mavlink":
            validate_link(entry["parameters"].get("fcu_url"))
            validate_link(entry["parameters"].get("gcs_url", ""), optional=True)
            target = entry["parameters"].get("target_system", 1)
            if type(target) is not int or not 1 <= target <= 255:
                raise ValueError("飞控 target_system 必须在 1–255")
        if name == "navigation" and (
            entry["parameters"].get("localization_backend", "fast_lio") != "fast_lio"
            or entry["parameters"].get("strategy", "astar")
            not in ("astar", "ego", "combined", "gcopter")
        ):
            raise ValueError("当前机载导航适配支持 FAST-LIO 与 A*/EGO/组合/GCOPTER 策略")
        if (
            not standalone
            and name == 'navigation'
            and entry['parameters'].get('strategy', 'astar') in ('ego', 'combined')
            and not recipe['components'].get('local_planner', {}).get('enabled', False)
        ):
            raise ValueError('EGO／组合导航策略需要启用受管 EGO 局部规划（local_planner）')
        if name in ("mavlink", "local_planner") and entry["parameter_file"]:
            raise ValueError(name + " 使用启动参数表，不接受 ROS 参数文件")
        if (
            name == "local_planner"
            and entry["parameters"].get("navigation_managed", True) is not True
        ):
            raise ValueError("机载局部规划必须由统一导航会话管理")
        # Native adapters with different timing/calibration contracts require a
        # separate adapter, never a simulation launch with one flag changed.
        for key in CATALOG[name]["files"] if require_files else ():
            path = (
                entry["parameter_file"]
                if key == "parameter_file"
                else entry["parameters"].get(key)
            )
            if not isinstance(path, str) or not path:
                raise ValueError("hardware configuration required: " + name + "/" + key)
    perception_contract(recipe)
    order(recipe)
    return recipe


def order(recipe):
    """Order declared/strategy dependencies; omitted sensor inputs may be external."""
    active = {k: v for k, v in recipe["components"].items() if v["enabled"]}
    result, visiting = [], set()

    def visit(name):
        if name in visiting:
            raise ValueError("cyclic onboard dependencies")
        if name in result:
            return
        visiting.add(name)
        for parent in component_dependencies(recipe, name):
            if parent in active:
                visit(parent)
        visiting.remove(name)
        result.append(name)

    for name in active:
        visit(name)
    return result


def read_configuration(path, roots):
    """Freeze owned configuration bytes; no symlinks or unrelated remote file reads."""
    target = Path(path)
    if (
        not target.is_absolute()
        or target.resolve() != target
        or not any(target.is_relative_to(Path(root)) for root in roots)
    ):
        raise ValueError(
            "configuration must be inside a configured hardware directory: " + str(path)
        )
    with os.fdopen(os.open(target, os.O_RDONLY | os.O_NOFOLLOW), "rb") as stream:
        info = os.fstat(stream.fileno())
        if (
            not stat.S_ISREG(info.st_mode)
            or info.st_uid != os.geteuid()
            or info.st_mode & 0o022
            or not 0 < info.st_size <= 1024 * 1024
        ):
            raise ValueError("hardware file must be owned, non-writable by others, at most 1 MiB")
        data = stream.read(1024 * 1024 + 1)
    if len(data) > 1024 * 1024:
        raise ValueError("oversized hardware file")
    if target.suffix.lower() in (".yaml", ".yml", ".json"):
        try:
            if any(isinstance(token, yaml.tokens.AliasToken) for token in yaml.scan(data)):
                raise ValueError("hardware configuration aliases are not supported")
            document = yaml.safe_load(data)
        except yaml.YAMLError as error:
            raise ValueError("invalid hardware YAML/JSON: " + str(target)) from error

        def check(value):
            if isinstance(value, dict):
                if value.get("use_sim_time", False) is not False:
                    raise ValueError("hardware configuration cannot enable simulated time")
                for child in value.values():
                    check(child)
            elif isinstance(value, list):
                for child in value:
                    check(child)
            elif isinstance(value, float) and not math.isfinite(value):
                raise ValueError("hardware configuration contains a non-finite number")

        check(document)
    return data


def preflight(recipe, roots, directory=None):
    """Resolve installed adapters and freeze all explicit hardware parameter files."""
    validate(recipe, require_files=True)
    frozen = copy.deepcopy(recipe)
    hashes = {}
    precision_bytes = None
    flight_bytes = None
    rays_frame = None
    map_frame = None
    for name in order(recipe):
        spec, entry = CATALOG[name], frozen["components"][name]
        get_package_share_directory(spec["package"])
        if spec["executable"] and not spec["executable"].endswith(".launch.py"):
            executable = (
                Path(get_package_prefix(spec["package"]))
                / "lib"
                / spec["package"]
                / spec["executable"]
            )
            if not executable.is_file() or not os.access(executable, os.X_OK):
                raise ValueError("installed onboard executable missing: " + name)
        paths = [("parameter_file", entry)] + [
            (key, entry["parameters"])
            for key in FILES - {"parameter_file"}
            if key in entry["parameters"]
        ]
        for key, mapping in paths:
            if not mapping.get(key):
                continue
            data = read_configuration(mapping[key], roots)
            if name == 'target_tracking' and key == 'config_file':
                values = validate_tracking_configuration(yaml.safe_load(data))
                for field in set(entry['parameters']) & set(TRACK_DEFAULTS):
                    if entry['parameters'][field] != values[field]:
                        raise ValueError('target_tracking recipe/file parameters differ: ' + field)
            if name == 'precision_landing' and key == 'config_file':
                from easyfly_precision_landing.hardware_configuration import validate_document

                validate_document(yaml.safe_load(data))
                precision_bytes = data
            if name == 'flight_control' and key == 'parameter_file':
                flight_bytes = data
            if key == "parameter_file" and name not in ("realsense", "down_camera"):
                document = yaml.safe_load(data)
                names = {NODE_NAMES.get(name, name)}
                if name == "navigation":
                    names.update(("easyfly_global_planner", "obstacle_supervisor"))
                elif name == "mission_fsm":
                    names.add("search_area")
                if name == 'gcopter':
                    names.update(('gcopter_voxel_map', 'gcopter_navigation'))
                scopes = names | {"/" + value for value in names} | {"/**"}
                if not isinstance(document, dict) or not any(
                    scope in scopes
                    and isinstance(body, dict)
                    and isinstance(body.get("ros__parameters"), dict)
                    for scope, body in document.items()
                ):
                    raise ValueError("ROS 参数文件没有匹配的节点或 /** 作用域：" + name)
                if name == 'navigation':
                    selected = entry['parameters'].get('strategy', 'astar')
                    for scope, body in document.items():
                        values = body.get('ros__parameters', {}) if isinstance(body, dict) else {}
                        if (
                            scope in scopes
                            and 'strategy' in values
                            and values['strategy'] != selected
                        ):
                            raise ValueError('navigation recipe/file strategy differs')
                        if (
                            scope in scopes
                            and selected == 'gcopter'
                            and 'session_service' in values
                            and values['session_service'] != '/planning/gcopter_session'
                        ):
                            raise ValueError('GCOPTER navigation file has wrong session service')
                if name == 'gcopter':
                    if entry['parameters']:
                        raise ValueError(
                            'GCOPTER requires reviewed file parameters with no recipe overrides'
                        )
                    from easyfly_trajectory_optimizer.hardware import validate_hardware_parameters

                    mapper, planner = validate_hardware_parameters(document)
                    map_frame = mapper['source_frame']
                    if set(document) != {'gcopter_voxel_map', 'gcopter_navigation'}:
                        raise ValueError(
                            'GCOPTER accepts only its two explicit node parameter scopes'
                        )
                if name == 'lidar_rays':
                    from easyfly_trajectory_optimizer.hardware import validate_lidar_parameters

                    values = {}
                    for scope in ('/**', '/lidar_rays', 'lidar_rays'):
                        values.update(document.get(scope, {}).get('ros__parameters', {}))
                    values.update(entry['parameters'])
                    validate_lidar_parameters(values)
                    rays_frame = values['source_frame']
                if name == 'fuel':
                    for body in document.values():
                        values = body.get('ros__parameters', {}) if isinstance(body, dict) else {}
                        if values.get('extended_profile', False) is not False:
                            raise ValueError('hardware FUEL file must use extended_profile=false')
                        if (
                            'native_max_speed' in values
                            and 'native_max_speed' in entry['parameters']
                            and values['native_max_speed']
                            != entry['parameters']['native_max_speed']
                        ):
                            raise ValueError('FUEL recipe/file native_max_speed differs')
                if name == "map_odom" and not any(
                    isinstance(body, dict)
                    and "body_to_base_translation" in body.get("ros__parameters", {})
                    for scope, body in document.items()
                    if scope in scopes
                ):
                    raise ValueError("请显式配置真实雷达 body_to_base_translation 外参")
            identity = hashlib.sha256(data).hexdigest()
            hashes[name + "/" + key] = dict(path=mapping[key], sha256=identity)
            if directory is not None:
                destination = directory / (name + "-" + key + Path(mapping[key]).suffix)
                with destination.open("xb") as stream:
                    stream.write(data)
                destination.chmod(0o600)
                mapping[key] = str(destination)
    if map_frame is not None and rays_frame != map_frame:
        raise ValueError(
            'GCOPTER and physical lidar_rays must share the same reviewed source_frame'
        )
    if precision_bytes is not None:
        if flight_bytes != precision_bytes:
            raise ValueError(
                'precision_landing and flight_control must share identical reviewed configuration'
            )
        flight_entry = frozen['components'].get('flight_control', {})
        if any(key.startswith('precision_') for key in flight_entry.get('parameters', {})):
            raise ValueError(
                'precision control values must come from the shared configuration; no overrides'
            )
    return frozen, hashes


def perception_contract(recipe):
    """Return version-specific telemetry bindings without changing persisted legacy recipes."""
    versions = {entry['parameters'].get('perception_version', 1) for name, entry in recipe['components'].items()
                if entry['enabled'] and name in ('detection', 'tracking', 'pixel_to_3d', 'target_tracking')}
    if any(type(v) is not int for v in versions) or not versions <= {1, 2} or len(versions) > 1:
        raise ValueError('all enabled perception components must select the same perception_version')
    version = next(iter(versions), 1)
    if version == 2:
        for name, topic in [('detection', '/perception/v2/detections'), ('tracking', '/perception/v2/tracks')]:
            entry = recipe['components'].get(name)
            if entry and entry['enabled'] and entry['output_topic'] != entry['remappings'].get(topic, topic):
                raise ValueError('V2 recipe requires versioned output topic for ' + name)
    return version


def observation_catalog(recipe):
    import copy
    result = copy.deepcopy(CATALOG)
    if perception_contract(recipe) == 2:
        for role, topic, message in [
            ('detection', '/perception/v2/detections', 'DetectionArrayV2'),
            ('tracking', '/perception/v2/tracks', 'ImageTrackArrayV2'),
            # pixel_to_3d has no independent startup telemetry in recipe schema v1.
        ]:
            result[role]['output'] = topic
            result[role]['message'] = 'easyfly_msgs/msg/' + message
    return result
