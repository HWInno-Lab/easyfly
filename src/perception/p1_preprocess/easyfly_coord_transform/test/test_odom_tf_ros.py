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
