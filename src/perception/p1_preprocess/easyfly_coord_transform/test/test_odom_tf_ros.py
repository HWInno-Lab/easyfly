# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_coord_transform/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project

#
#

import subprocess
import sys
from pathlib import Path


def test_ros_interface_contract(tmp_path):
    result = subprocess.run(
        [sys.executable, str(Path(__file__).with_name("odom_tf_probe.py"))],
        capture_output=True,
        text=True,
        timeout=60,
    )
    (tmp_path / "probe.log").write_text(result.stdout + result.stderr)
    assert result.returncode == 0, result.stdout + result.stderr
