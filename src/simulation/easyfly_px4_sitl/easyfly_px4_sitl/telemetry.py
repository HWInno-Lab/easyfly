# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Configure only a launch-owned simulator's native heartbeat stream."""
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess

from launch.actions import LogInfo, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessIO, OnProcessStart

from .hrt_compatibility import read_manifest
from .scene_inputs import scene_value

MARKER = 'EasyFly PX4 telemetry: '


def process_identity(pid):
    """Include parent and start ticks so a changed PID is never trusted."""
    proc = Path('/proc') / str(pid)
    fields = (proc/'stat').read_text().rsplit(')', 1)[1].split()
    return (str((proc/'exe').resolve(strict=True)), int(fields[1]), fields[19])


def configured_heartbeat(status, streams, rate):
    """Require the actual offboard link and its configured native stream rate."""
    for instance, block in re.findall(r'(?ms)^instance #(\d+):\n(.*?)(?=^instance #|\Z)', status):
        if 'UDP (14580, remote port: 14540)' not in block:
            continue
        for stream_instance, stream_block in re.findall(
                r'(?ms)^instance #(\d+):\n(.*?)(?=^instance #|\Z)', streams):
            if stream_instance != instance:
                continue
            match = re.search(r'(?m)^\s*HEARTBEAT\s+([0-9.]+)\s+\(([0-9.]+)\)', stream_block)
            if match and float(match[1]) == rate and float(match[2]) == rate:
                return True
    return False


def configure_owned(pid, binary, rate, *, identity=process_identity, runner=subprocess.run):
    """Use the pinned native client; neither ACK nor a fabricated state is feedback."""
    if rate != 5:
        raise ValueError('only the reviewed 5Hz simulator heartbeat is supported')
    binary = Path(binary).resolve(strict=True)
    client = binary.parent/'px4-mavlink'
    if client.resolve(strict=True) != binary:
        raise RuntimeError('PX4 telemetry client differs from selected native binary')
    owned = identity(pid)
    if owned[:2] != (str(binary), os.getpid()):
        raise RuntimeError('refuse telemetry command to an unowned PX4 process')
    commands = [
        [str(client), '--instance', '0', 'stream', '-u', '14580',
         '-s', 'HEARTBEAT', '-r', str(rate)],
        [str(client), '--instance', '0', 'status'],
        [str(client), '--instance', '0', 'status', 'streams'],
    ]
    outputs = []
    for command in commands:
        if identity(pid) != owned:
            raise RuntimeError('PX4 identity changed before telemetry command')
        result = runner(command, capture_output=True, text=True, timeout=5, check=False)
        if result.returncode != 0 or identity(pid) != owned:
            raise RuntimeError('native PX4 telemetry command failed or process changed')
        outputs.append(result.stdout + result.stderr)
    deferred = not any(outputs[1:])
    if not deferred and not configured_heartbeat(outputs[1], outputs[2], rate):
        raise RuntimeError('native offboard HEARTBEAT rate not confirmed: ' + json.dumps(outputs))
    return dict(pid=pid, start_ticks=owned[2], rate_hz=rate, local_port=14580,
                remote_port=14540, native_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
                status=outputs[1], streams=outputs[2], native_output_deferred=deferred,
                command_returncodes=[0, 0, 0])


def heartbeat_configuration_action(target, tree):
    """Capture scene inputs in their launch scope before delayed process events."""
    def register(context):
        rate = int(scene_value('heartbeat_rate_hz').perform(context))
        if rate == 1:
            return []  # Existing scenes and raw defaults keep upstream telemetry.
        if rate != 5:
            raise ValueError('unsupported managed simulator heartbeat rate')
        selected = Path(tree)
        manifest = selected/'easyfly_staging.json'
        binary = (Path(read_manifest(manifest)['binary']) if manifest.exists() else
                  selected/'build/px4_sitl_default/bin/px4')
        pending = bytearray()
        configured = [False]
        requested = [None]

        def output(event):
            if configured[0]:
                return []
            pending.extend(event.text)
            del pending[:-65536]
            if requested[0] is None:
                if b'Startup script returned successfully' not in pending:
                    return []
                report = configure_owned(event.pid, binary, rate)
                requested[0] = report
                pending.clear()
                if report['native_output_deferred']:
                    # PX4 printf() writes status to the daemon's stdout, not
                    # the command client's pipe. Verify later target IO events.
                    return []
            else:
                report = requested[0]
                text = pending.decode(errors='replace')
                if not configured_heartbeat(text, text, rate):
                    return []
                if process_identity(event.pid) != (
                        str(binary.resolve(strict=True)), os.getpid(), report['start_ticks']):
                    raise RuntimeError('PX4 identity changed before native stream proof')
                report.update(status=text, streams=text, native_output_deferred=False)
            configured[0] = True
            return [LogInfo(msg=MARKER + json.dumps(report, sort_keys=True))]

        def require_ready(_context):
            if not configured[0]:
                raise RuntimeError('managed PX4 telemetry startup deadline expired')
            return []

        return [
            RegisterEventHandler(OnProcessIO(target_action=target, on_stdout=output)),
            RegisterEventHandler(OnProcessStart(target_action=target, on_start=[
                TimerAction(period=30., actions=[OpaqueFunction(function=require_ready)])])),
        ]
    return OpaqueFunction(function=register)
