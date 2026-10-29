# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
import subprocess
import sys
from pathlib import Path

def test_ros_interface_contract(tmp_path):
    result = subprocess.run([sys.executable, str(Path(__file__).with_name("bridge_probe.py"))],
                            capture_output=True, text=True, timeout=60)
    (tmp_path / "probe.log").write_text(result.stdout + result.stderr)
    assert result.returncode == 0, result.stdout + result.stderr
