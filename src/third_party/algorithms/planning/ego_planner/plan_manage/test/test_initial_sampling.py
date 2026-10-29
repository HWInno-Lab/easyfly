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

"""
Compile the actual sampling block against a linear trajectory fixture.

This exercises the early-break path with stale points, without requiring a ROS
node or map. Changes to the production loop are used directly by the fixture.
"""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest


class InitialSamplingTest(unittest.TestCase):
    def test_sampling_rejects_invalid_first_interval_and_accepts_valid_seed(self):
        source = (Path(__file__).resolve().parents[1] / "src/planner_manager.cpp").read_text()
        flag = source.index("bool flag_too_far")
        starts = list(re.finditer(r"\bdouble\s+t\s*=\s*0\.0\s*;", source[:flag]))
        self.assertTrue(starts, "sampling block declaration missing")
        start = starts[-1].start()
        decrement = re.search(r"\bt\s*-=\s*ts\s*;", source[flag:])
        self.assertIsNotNone(decrement, "sampling block end missing")
        end = flag + decrement.start()
        block = source[start:end]
        fixture = (
            r"""
#include <cmath>
#include <vector>
#include <limits>
#define RCLCPP_ERROR(...) ((void)0)
namespace Eigen {
struct Vector3d {
  double x;
  Vector3d operator-(Vector3d rhs) const { return {x-rhs.x}; }
  double norm() const { return std::abs(x); }
};
}
struct LinearTrajectory {
  Eigen::Vector3d evaluate(double t) const { return {t}; }
};
bool sample(double ts, bool stale_points) {
  LinearTrajectory gl_traj;
  struct { double ctrl_pt_dist = 0.2; } pp_;
  std::vector<Eigen::Vector3d> point_set(stale_points ? 7 : 0);
  constexpr double time = 1.0;
  constexpr size_t max_samples = 10000;
  int continous_failures_count_ = 0;
"""
            + block
            + r"""
  return true;
}
int main() {
  const double invalid[] = {std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity(), 0.0, -1.0, 1e-4, 11.0};
  for (double ts : invalid)
    for (bool stale : {false, true})
      if (sample(ts, stale)) return 1;
  if (!sample(0.1, false) || !sample(0.5, false)) return 2;
  return 0;
}
"""
        )
        with tempfile.TemporaryDirectory() as directory:
            cpp = Path(directory) / "sampling.cpp"
            binary = Path(directory) / "sampling"
            cpp.write_text(fixture)
            subprocess.run(
                [
                    "c++",
                    "-std=c++14",
                    "-O2",
                    "-Wall",
                    "-Werror=uninitialized",
                    str(cpp),
                    "-o",
                    str(binary),
                ],
                check=True,
            )
            subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    unittest.main()
