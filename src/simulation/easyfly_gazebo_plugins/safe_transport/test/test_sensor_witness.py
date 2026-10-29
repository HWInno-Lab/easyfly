# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Real native scan delivery and retirement, including completed deferred work."""
import csv
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time


def stop(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
    try:
        return process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=3)
        raise


def main():
    server, ordinary, retired = sys.argv[1:]
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    env = dict(os.environ, GAZEBO_MASTER_URI=f'http://127.0.0.1:{port}',
               GAZEBO_MODEL_DATABASE_URI='')
    with tempfile.TemporaryDirectory(prefix='native-sensor-witness-') as directory:
        root = Path(directory)
        world = root / 'world.sdf'
        world.write_text('''<sdf version="1.6"><world name="witness">
          <model name="sensor"><static>true</static><pose>1 2 3 0 0 0</pose>
          <link name="body"><sensor name="scan" type="ray"><always_on>true</always_on>
          <update_rate>20</update_rate><ray><scan><horizontal><samples>8</samples>
          <resolution>1</resolution><min_angle>-.2</min_angle><max_angle>.2</max_angle>
          </horizontal></scan><range><min>.1</min><max>5</max><resolution>.01</resolution>
          </range></ray></sensor></link></model></world></sdf>''')
        with (root / 'server.log').open('w+') as server_log:
            gz = subprocess.Popen([server, str(world)], env=env,
                                  stdout=server_log, stderr=subprocess.STDOUT)
            try:
                for index, binary in enumerate([ordinary, retired]):
                    output = root / f'{index}.csv'
                    with (root / f'{index}.log').open('w+') as log:
                        observer = subprocess.Popen([binary, str(output)], env=env,
                                                    stdout=log, stderr=subprocess.STDOUT)
                        try:
                            deadline = time.monotonic() + 12
                            while True:
                                log.seek(0)
                                if '/scan' in log.read():
                                    break
                                assert observer.poll() is None and time.monotonic() < deadline
                                time.sleep(.05)
                            time.sleep(.6)
                            code = stop(observer)
                            log.seek(0)
                            text = log.read()
                            assert code == 0, text
                            assert 'drained live=0 pending=0' in text, text
                            if index:
                                assert 'deferred_total=1 ' in text, text
                            with output.open() as stream:
                                assert 'discarded=0' in next(stream)
                                rows = list(csv.DictReader(stream))
                            assert len(rows) >= 5
                            stamps = [int(row['stamp_ns']) for row in rows]
                            assert all(a < b for a, b in zip(stamps, stamps[1:]))
                            assert all(abs(float(row['x'])-1) < 1e-8 and
                                       abs(float(row['y'])-2) < 1e-8 and
                                       abs(float(row['z'])-3) < 1e-8 for row in rows)
                        finally:
                            if observer.poll() is None:
                                stop(observer)
            finally:
                code = stop(gz)
                server_log.seek(0)
                assert code == 0, server_log.read()
    print('original native scans and both ordinary/deferred-completed retirement passed')


if __name__ == '__main__':
    main()
