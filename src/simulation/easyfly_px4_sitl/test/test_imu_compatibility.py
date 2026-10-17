# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Reject changed native-message inputs before simulator launch, without loading them."""
import hashlib
import json

import pytest

from easyfly_px4_sitl.imu_compatibility import require_native_imu


def test_exact_message_library_and_independent_plugin_are_required(tmp_path, monkeypatch):
    import ament_index_python.packages as packages
    share = tmp_path / 'share'
    (share / 'native_imu').mkdir(parents=True)
    prefix = tmp_path / 'prefix'
    (prefix / 'lib').mkdir(parents=True)
    plugin = prefix / 'lib/libeasyfly_native_imu.so'
    plugin.write_bytes(b'not loaded by this preflight fixture')
    px4 = tmp_path / 'px4'
    library = px4 / 'build/px4_sitl_default/build_gazebo-classic/libsensor_msgs.so'
    library.parent.mkdir(parents=True)
    library.write_bytes(b'reviewed fixture bytes')
    (share / 'native_imu/native_imu.json').write_text(json.dumps(dict(
        schema=1, sensor_msgs_sha256=hashlib.sha256(library.read_bytes()).hexdigest())))
    monkeypatch.setattr(packages, 'get_package_share_directory', lambda _: str(share))
    monkeypatch.setattr(packages, 'get_package_prefix', lambda _: str(prefix))
    require_native_imu(px4)
    library.write_bytes(b'changed fixture bytes')
    with pytest.raises(ValueError, match='不匹配'):
        require_native_imu(px4)
    plugin.unlink()
    with pytest.raises(ValueError, match='不匹配'):
        require_native_imu(px4)
