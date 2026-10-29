"""Pinned native implementation, timestamp overflow and strictly scoped corrections."""
import hashlib
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parent


def test_native_provenance_and_only_reviewed_changes():
    for line in (ROOT / 'upstream/SHA256SUMS').read_text().splitlines():
        digest, name = line.split()
        assert hashlib.sha256((ROOT / 'upstream' / name).read_bytes()).hexdigest() == digest
    original = (ROOT / 'upstream/gazebo_imu_plugin.cpp').read_text()
    expected = original.replace('GazeboImuPlugin', 'EasyFlyNativeImuPlugin').replace(
        'updateConnection_->~Connection();', 'updateConnection_.reset();').replace(
        '_info.simTime.sec * 1000000 + _info.simTime.nsec / 1000',
        'static_cast<int64_t>(_info.simTime.sec) * 1000000 + _info.simTime.nsec / 1000').replace(
        '  last_time_ = current_time;\n  double t',
        '  last_time_ = current_time;\n'
        '  // A paused or reset world has no positive integration interval. Establish\n'
        '  // the actual new source-time baseline and emit no synthetic measurement.\n'
        '  // The unchanged relay rejects the subsequent epoch rollback and stays halted.\n'
        '  if (dt <= 0.0) {\n    return;\n  }\n  double t')
    assert (ROOT / 'native_imu.cpp').read_text() == expected
    assert (ROOT / 'gazebo_imu_plugin.h').read_text() == (
        ROOT / 'upstream/gazebo_imu_plugin.h').read_text().replace(
            'GazeboImuPlugin', 'EasyFlyNativeImuPlugin')


def test_real_timestamp_expression_crosses_signed_32_bit_boundary(tmp_path):
    original = (ROOT / 'upstream/gazebo_imu_plugin.cpp').read_text()
    corrected = (ROOT / 'native_imu.cpp').read_text()
    results = []
    for name, source in [('original', original), ('corrected', corrected)]:
        expression = source.split('imu_message_.set_time_usec(')[1].split(');')[0]
        code = tmp_path / (name + '.cpp')
        code.write_text('#include <cstdint>\n#include <cstdio>\n'
                        'int main(){struct {struct {int32_t sec,nsec;} simTime;} _info;'
                        'for(int s=2147;s<=2149;++s){_info.simTime={s,0};'
                        'int64_t result=' + expression + ';'
                        'if(result!=int64_t(s)*1000000)return 1;}return 0;}')
        binary = tmp_path / name
        subprocess.run(['c++', '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        str(code), '-o', str(binary)], check=True)
        results.append(subprocess.run([str(binary)], capture_output=True))
    assert results[0].returncode != 0 and b'overflow' in results[0].stderr
    assert results[1].returncode == 0 and not results[1].stderr
