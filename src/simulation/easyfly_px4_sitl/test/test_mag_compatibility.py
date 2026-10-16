# Copyright 2026 EasyFly Project
# SPDX-License-Identifier: Apache-2.0
"""Check the optional native sensor-rate correction without starting a simulator."""
import importlib.util
from pathlib import Path
from unittest import mock

import pytest

from easyfly_px4_sitl import mag_compatibility as mag

SPEC = importlib.util.spec_from_file_location(
    'prepare_hrt_mag', Path(__file__).parents[1] / 'scripts/prepare_px4_hrt.py')
builder = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(builder)


def test_mag_link_keeps_the_existing_hrt_fix_and_rejects_missing_archive():
    line = (f': && c++ main.o -o bin/px4 {builder.LAYER_PATH} '
            f'{mag.EKF_ARCHIVE} {builder.LAYER_PATH} && cd /original/bin')
    args = builder.link_command(line, Path('/private/px4'), Path('/private/hrt.a'),
                                Path('/private/ekf.a'))
    assert args == ['c++', 'main.o', '-o', '/private/px4', '/private/hrt.a',
                    '/private/ekf.a', '/private/hrt.a']
    with pytest.raises(ValueError, match='magnetometer link'):
        builder.link_command(line.replace(mag.EKF_ARCHIVE, ''), Path('/private/px4'),
                             Path('/private/hrt.a'), Path('/private/ekf.a'))


def test_changed_mag_input_is_rejected_before_any_write(tmp_path):
    source = tmp_path / 'px4'
    source.mkdir()
    output = tmp_path / 'new'
    with mock.patch.object(builder, 'digest', side_effect=[builder.HRT_SHA256,
                                                         builder.BINARY_SHA256]), \
            mock.patch.object(mag, 'digest', return_value='changed'), \
            mock.patch.object(builder.subprocess, 'run') as run:
        with pytest.raises(ValueError, match='unsupported PX4 magnetometer'):
            builder.prepare(source, output, tmp_path, mag_lpf_interval=True)
        run.assert_not_called()
    assert not output.exists()


@pytest.mark.parametrize('value', [None, {}, {'time_constant_seconds': 0.01}])
def test_incomplete_or_changed_mag_policy_is_rejected(tmp_path, value):
    with pytest.raises(ValueError, match='unreviewed magnetometer'):
        mag.validate_mag({'mag_lpf_interval': value}, tmp_path, tmp_path)


def test_legacy_hrt_only_manifest_remains_supported(tmp_path):
    mag.validate_mag({}, tmp_path, tmp_path)
