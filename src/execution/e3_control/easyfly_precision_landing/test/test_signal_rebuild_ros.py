# Copyright 2026 EasyFly Project
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Stop during real subscription replacement; never invalidate its live context."""
import os
import signal
import subprocess
import sys

import pytest

CHILD = r'''import os,signal,sys,time
import rclpy
from easyfly_precision_landing.node import LandingTransform, run_observer
class Rebuild(LandingTransform):
    def __init__(self):
        super().__init__()
        self.completed = False
        self.timer = self.create_timer(.02, self.stop_during_rebuild)
    def stop_during_rebuild(self):
        self.timer.cancel()
        original = self.destroy_subscription
        def interrupted(subscription):
            # Send a real signal between retiring an endpoint and recreating it.
            result = original(subscription)
            os.kill(os.getpid(), int(sys.argv[1]))
            time.sleep(.05)
            assert self.context.ok(), 'context invalidated inside receiver replacement'
            return result
        self.destroy_subscription = interrupted
        self.reset_receivers()
        assert self.generation == 1
        self.completed = True
        print('REAL_RECEIVERS_REPLACED_WITH_VALID_CONTEXT', flush=True)
    def destroy_node(self):
        assert self.completed
        assert self.context.ok(), 'node must be destroyed before its context'
        return super().destroy_node()
run_observer(Rebuild, args=[])
assert not rclpy.ok()
'''


@pytest.mark.parametrize('stop_signal', [signal.SIGINT, signal.SIGTERM])
def test_signal_during_real_reader_replacement(stop_signal, tmp_path):
    result = subprocess.run([sys.executable, '-c', CHILD, str(int(stop_signal))],
                            env=dict(os.environ), capture_output=True, text=True, timeout=8)
    (tmp_path/'child.log').write_text(result.stdout+result.stderr)
    assert result.returncode == 0, result.stdout+result.stderr
    assert 'REAL_RECEIVERS_REPLACED_WITH_VALID_CONTEXT' in result.stdout
