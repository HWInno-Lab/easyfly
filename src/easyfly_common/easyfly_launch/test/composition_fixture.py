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
"""Real ROS load service with bounded delays; no component or flight is fabricated."""
import asyncio
import json
from pathlib import Path
import sys
import threading
import time

from composition_interfaces.srv import LoadNode
from launch import LaunchDescription, LaunchService
from launch.actions import (EmitEvent, GroupAction, OpaqueCoroutine,
                            SetLaunchConfiguration, TimerAction)
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor

from easyfly_launch.composition import LoadComposableNodes


def main():
    phase, output = sys.argv[1:]
    calls = []
    requests = []
    context = Context()
    rclpy.init(args=[], context=context)
    node = rclpy.create_node('component_shutdown_fixture', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    done = threading.Event()

    def reply(request, response):
        calls.append(request.plugin_name)
        requests.append(dict(name=request.node_name, namespace=request.node_namespace,
                             remaps=list(request.remap_rules),
                             gains=[p.value.double_value for p in request.parameters
                                    if p.name == 'gain']))
        # Keep the real client response pending until launch has shut down.
        if phase == 'response':
            done.wait(3.)
        response.success = phase != 'rejected'
        response.full_node_name = '/' + request.plugin_name
        response.error_message = '' if response.success else 'fixture rejection'
        return response

    if phase != 'waiting':
        node.create_service(LoadNode, '/component_fixture/_container/load_node', reply)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    service = LaunchService()
    descriptions = [ComposableNode(package='fixture', plugin=p) for p in ['first', 'second']]
    if phase == 'scoped':
        descriptions = [ComposableNode(
            package='fixture', plugin=p, name=LaunchConfiguration('component_name'),
            namespace=LaunchConfiguration('component_namespace'),
            remappings=[('/input', LaunchConfiguration('output_topic'))],
            parameters=[{'gain': ParameterValue(LaunchConfiguration('gain'), value_type=float)}])
            for p in ['first', 'second']]

    async def stop_after_calls(context):
        target = 1 if phase == 'response' else 2
        deadline = time.monotonic() + 8
        while len(calls) < target:
            if time.monotonic() > deadline:
                raise RuntimeError('fixture service was not called')
            await asyncio.sleep(.01)
        if phase in ('loaded', 'scoped'):
            await asyncio.sleep(.1)  # Receive the final response before normal shutdown.
        await context.emit_event(Shutdown(reason='explicit fixture stop'))

    actions = [LoadComposableNodes(composable_node_descriptions=descriptions,
                                   target_container='/component_fixture')]
    if phase == 'scoped':
        actions = [GroupAction(actions=[
            SetLaunchConfiguration('component_name', 'scoped_component'),
            SetLaunchConfiguration('component_namespace', '/scoped'),
            SetLaunchConfiguration('output_topic', '/scoped/output'),
            SetLaunchConfiguration('gain', '2.5'), *actions]),
            SetLaunchConfiguration('component_name', 'outside_group'),
            SetLaunchConfiguration('gain', '999')]
    if phase == 'waiting':
        actions.append(TimerAction(period=.2, actions=[EmitEvent(event=Shutdown(reason='stop'))]))
    elif phase != 'rejected':
        actions.append(OpaqueCoroutine(coroutine=stop_after_calls))
    service.include_launch_description(LaunchDescription(actions))
    try:
        result = service.run()
    finally:
        done.set()
        executor.shutdown()
        thread.join(timeout=4)
        executor.remove_node(node)
        node.destroy_node()
        context.try_shutdown()
    adapter = service.context.locals.easyfly_component_adapter
    report = dict(calls=calls, requests=requests, returncode=result,
                  thread_alive=thread.is_alive(),
                  adapter_guard_released=adapter.ros_executor._sigint_gc is None,
                  adapter_context_closed=not adapter.ros_context.ok())
    Path(output).write_text(json.dumps(report) + '\n')
    return result


if __name__ == '__main__':
    raise SystemExit(main())
