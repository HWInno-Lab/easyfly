# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_semantic_map/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project

"""Shutdown after dynamic tracking, DDS reader turnover and valid clock reset recovery."""

import signal

import pytest
from test_shutdown_ros import run_active_mapping_shutdown


@pytest.mark.parametrize('workload', ['moving-readers', 'clock-reset'])
@pytest.mark.parametrize('stop_signal', [signal.SIGINT, signal.SIGTERM])
@pytest.mark.parametrize('capture', [False, True])
def test_dynamic_shutdown(tmp_path, workload, stop_signal, capture):
    """Keep safety bounds unchanged and require useful processing, not only input sends."""
    run_active_mapping_shutdown(tmp_path, 'running', stop_signal, capture, workload)
