"""Compile the actual sampling block against a linear trajectory fixture.

This exercises the early-break path with stale points, without requiring a ROS
node or map. Changes to the production loop are used directly by the fixture.
"""
from pathlib import Path
import subprocess
import tempfile
import unittest


class InitialSamplingTest(unittest.TestCase):
    def test_sampling_rejects_invalid_first_interval_and_accepts_valid_seed(self):
        source = (Path(__file__).resolve().parents[1] /
                  'src/planner_manager.cpp').read_text()
        flag = source.index('bool flag_too_far')
        start = source.rfind('        double t = 0.0;', 0, flag)
        end = source.index('        t -= ts;', start)
        block = source[start:end]
        fixture = r'''
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
''' + block + r'''
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
'''
        with tempfile.TemporaryDirectory() as directory:
            cpp = Path(directory) / 'sampling.cpp'
            binary = Path(directory) / 'sampling'
            cpp.write_text(fixture)
            subprocess.run(['c++', '-std=c++14', '-O2', '-Wall',
                            '-Werror=uninitialized', str(cpp), '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    unittest.main()
