# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Native responses and clean client retirement, including failed queries."""
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time


def main():
    server, query = sys.argv[1:]
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    env = dict(os.environ, GAZEBO_MASTER_URI=f'http://127.0.0.1:{port}',
               GAZEBO_MODEL_DATABASE_URI='')
    with tempfile.TemporaryDirectory(prefix='native-model-query-') as directory:
        world = Path(directory) / 'world.sdf'
        world.write_text('''<sdf version="1.6"><world name="query_contract">
          <model name="witness"><static>true</static><pose>1 2 3 0 0 0.25</pose>
          <link name="body"><collision name="solid"><geometry><box><size>1 1 1</size>
          </box></geometry></collision></link></model></world></sdf>''')
        with (Path(directory) / 'server.log').open('w+') as log:
            process = subprocess.Popen([server, str(world)], env=env,
                                       stdout=log, stderr=subprocess.STDOUT)
            try:
                deadline = time.monotonic() + 15
                while True:
                    result = subprocess.run([query, 'witness', '--pose'], env=env,
                                            capture_output=True, text=True, timeout=12)
                    if result.returncode == 0:
                        break
                    assert process.poll() is None and time.monotonic() < deadline, result.stderr
                for _ in range(6):
                    result = subprocess.run([query, 'witness', '--pose'], env=env,
                                            capture_output=True, text=True, timeout=10)
                    assert result.returncode == 0, result.stderr
                    values = [float(v) for v in result.stdout.split()]
                    assert len(values) == 6
                    assert all(abs(a-b) < 1e-8 for a, b in zip(values, [1, 2, 3, 0, 0, .25]))
                info = subprocess.run([query, 'witness', '--info'], env=env,
                                      capture_output=True, text=True, timeout=10)
                assert info.returncode == 0, info.stderr
                assert 'collision {' in info.stdout and '::solid"' in info.stdout, info.stdout
                missing = subprocess.run([query, 'not_present', '--pose'], env=env,
                                         capture_output=True, text=True, timeout=10)
                assert missing.returncode == 1 and not missing.stdout
                assert 'identity/pose unavailable' in missing.stderr
            finally:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=3)
                log.seek(0)
                assert process.returncode == 0, log.read()
    print('6 repeated poses, collision identity, missing model rejection, clean shutdown: passed')


if __name__ == '__main__':
    main()
