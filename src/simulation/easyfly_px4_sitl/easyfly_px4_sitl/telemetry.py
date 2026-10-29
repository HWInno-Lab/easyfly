# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Configure only a launch-owned simulator's native feedback streams."""
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


def configured_heartbeat(status, streams, rate, *, instance=0):
    """Require the actual offboard link and its configured native stream rate."""
    return configured_stream(status, streams, rate, 'HEARTBEAT', instance=instance)


def configured_stream(status, streams, rate, name, *, instance=0):
    """Match one named stream on the owned offboard instance, not a GCS link."""
    expected_link = f'UDP ({14580 + instance}, remote port: {14540 + instance})'
    for link_instance, block in re.findall(r'(?ms)^instance #(\d+):\n(.*?)(?=^instance #|\Z)', status):
        if expected_link not in block:
            continue
        for stream_instance, stream_block in re.findall(
                r'(?ms)^instance #(\d+):\n(.*?)(?=^instance #|\Z)', streams):
            if stream_instance != link_instance:
                continue
            match = re.search(r'(?m)^\s*' + re.escape(name) + r'\s+([0-9.]+)\s+\(([0-9.]+)\)', stream_block)
            if match and float(match[1]) == rate and float(match[2]) == rate:
                return True
    return False


def configure_owned(pid, binary, rate, *, battery_rate_hz=0, instance=0,
                    identity=process_identity, runner=subprocess.run):
    """Use the pinned native client; neither ACK nor a fabricated state is feedback."""
    if type(instance) is not int or not 0 <= instance <= 3:
        raise ValueError('reviewed simulator instance must be 0..3')
    if rate != 5:
        raise ValueError('only the reviewed 5Hz simulator heartbeat is supported')
    if battery_rate_hz not in (0, 5):
        raise ValueError('optional native battery rate must be 5Hz')
    binary = Path(binary).resolve(strict=True)
    client = binary.parent/'px4-mavlink'
    if client.resolve(strict=True) != binary:
        raise RuntimeError('PX4 telemetry client differs from selected native binary')
    owned = identity(pid)
    if owned[:2] != (str(binary), os.getpid()):
        raise RuntimeError('refuse telemetry command to an unowned PX4 process')
    commands = [
        [str(client), '--instance', str(instance), 'stream', '-u', str(14580 + instance),
         '-s', 'HEARTBEAT', '-r', str(rate)],
        [str(client), '--instance', str(instance), 'status'],
        [str(client), '--instance', str(instance), 'status', 'streams'],
    ]
    if battery_rate_hz:
        commands.insert(1, [str(client), '--instance', str(instance), 'stream', '-u', str(14580 + instance),
                            '-s', 'BATTERY_STATUS', '-r', str(battery_rate_hz)])
    outputs = []
    for command in commands:
        if identity(pid) != owned:
            raise RuntimeError('PX4 identity changed before telemetry command')
        result = runner(command, capture_output=True, text=True, timeout=5, check=False)
        if result.returncode != 0 or identity(pid) != owned:
            raise RuntimeError('native PX4 telemetry command failed or process changed')
        outputs.append(result.stdout + result.stderr)
    status, streams = outputs[-2:]
    deferred = not any((status, streams))
    if not deferred and not (configured_heartbeat(status, streams, rate, instance=instance) and
            (not battery_rate_hz or configured_stream(status, streams, battery_rate_hz, 'BATTERY_STATUS', instance=instance))):
        raise RuntimeError('native offboard telemetry rate not confirmed: ' + json.dumps(outputs))
    return dict(pid=pid, start_ticks=owned[2], rate_hz=rate, local_port=14580 + instance,
                remote_port=14540 + instance, instance=instance, native_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
                status=status, streams=streams, native_output_deferred=deferred,
                battery_rate_hz=battery_rate_hz, command_returncodes=[0] * len(commands))


def heartbeat_configuration_action(target, tree):
    """Capture scene inputs in their launch scope before delayed process events."""
    def register(context):
        rate = int(scene_value('heartbeat_rate_hz').perform(context))
        battery = int(scene_value('battery_rate_hz').perform(context))
        if rate == 1:
            if battery:
                raise ValueError('native battery override requires reviewed 5Hz heartbeat')
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
                report = configure_owned(event.pid, binary, rate, battery_rate_hz=battery)
                requested[0] = report
                pending.clear()
                if report['native_output_deferred']:
                    # PX4 printf() writes status to the daemon's stdout, not
                    # the command client's pipe. Verify later target IO events.
                    return []
            else:
                report = requested[0]
                text = pending.decode(errors='replace')
                if not (configured_heartbeat(text, text, rate) and
                        (not battery or configured_stream(text, text, battery, 'BATTERY_STATUS'))):
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
