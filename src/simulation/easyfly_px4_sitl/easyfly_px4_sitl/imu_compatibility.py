"""Bind the corrected native IMU to the exact existing PX4 protobuf library."""
import hashlib
import json
from pathlib import Path


def require_native_imu(px4_directory):
    from ament_index_python.packages import get_package_share_directory, get_package_prefix
    share = Path(get_package_share_directory('easyfly_gazebo_plugins'))
    manifest = json.loads((share / 'native_imu/native_imu.json').read_text())
    library = Path(px4_directory) / 'build/px4_sitl_default/build_gazebo-classic/libsensor_msgs.so'
    plugin = Path(get_package_prefix('easyfly_gazebo_plugins')) / 'lib/libeasyfly_native_imu.so'
    if (manifest.get('schema') != 1 or not plugin.is_file() or
            hashlib.sha256(library.read_bytes()).hexdigest() != manifest['sensor_msgs_sha256']):
        raise ValueError('原生 IMU 与所选 PX4 消息库不匹配；请安装匹配的仿真插件')
