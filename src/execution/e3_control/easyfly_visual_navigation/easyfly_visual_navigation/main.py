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

"""Review or explicitly execute one bounded visual task and its native ownership lifecycle."""

import argparse
import json
from pathlib import Path
import signal
import threading

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from .configuration import configuration
from .control import Controller, durable
from .inputs import Inputs


def main():
    """Review by default; require explicit execution and a hovering vehicle."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    config = configuration(args.config)
    output = Path(args.output)
    if not output.is_absolute() or output.exists():
        raise ValueError('output must be a new absolute session directory')
    output.mkdir(mode=0o700)
    durable(output / 'configuration.json', config)
    if not args.execute:
        durable(output / 'review.json', dict(
            configuration=config, execution_requested=False,
            prerequisites='settled offboard hover, exact visual/depth tuple, source-time TF, '
                          'fresh semantic geometry, controller idle, bounded route'))
        print(json.dumps(dict(review=str(output / 'review.json'), execution_requested=False)))
        return
    stopping = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopping.set())
    signal.signal(signal.SIGTERM, lambda *_: stopping.set())
    context = Context()
    rclpy.init(context=context, signal_handler_options=SignalHandlerOptions.NO)
    node = owner = executor = None
    failure = ''
    try:
        node = Inputs(config, context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        owner = Controller(node, executor, output, stopping)
        try:
            owner.execute()
        except BaseException as error:
            failure = str(error)
            owner.phase, owner.reason = 'failed', failure
            owner.retire()
        durable(output / 'result.json', dict(
            success=not failure and owner.success, error=failure, session=owner.session,
            native_status=owner.native_status, cancel_sent=owner.cancel_sent,
            commands=owner.command_count, measured_visual_samples=node.samples,
            ownership_released=owner.native_status in ('not_sent', 'rejected', 'terminal')))
    finally:
        if owner is not None:
            owner.close()
        if executor is not None:
            if node is not None:
                executor.remove_node(node)
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        context.shutdown()
    if failure:
        raise SystemExit(failure)
