# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_trajectory_optimizer/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project
#

"""Original native-flight failure, re-solved by the installed bounded worker."""
import gzip
from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_prefix


def test_actual_rolling_reverse_request_with_residual_velocity(tmp_path):
    source = Path(__file__).parent / 'data/rolling_reverse.request.gz'
    request = tmp_path / 'reverse.request'
    request.write_bytes(gzip.decompress(source.read_bytes()))
    result = tmp_path / 'reverse.polynomial'
    worker = (Path(get_package_prefix('easyfly_trajectory_optimizer')) /
              'lib/easyfly_trajectory_optimizer/gcopter_worker')
    run = subprocess.run([str(worker), str(request), str(result)],
                         capture_output=True, text=True, timeout=4)
    assert run.returncode == 0, run.stdout + run.stderr
    rows = result.read_text().splitlines()
    assert rows[0] == 'EASYFLY_POLYNOMIAL_1'
    _, speed, acceleration, jerk = map(float, rows[2].split())
    assert speed <= .6 and acceleration <= .15 and jerk <= .3
    # The original measured velocity points away from the goal. It must not be
    # zeroed or changed by uniform retiming in order to force the solve to pass.
    expected = list(map(float, request.read_text().splitlines()[5].split()))
    coefficients = list(map(float, rows[4].split()))
    assert all(abs(coefficients[axis * 6 + 4] - expected[axis]) < 1e-4
               for axis in range(3))
