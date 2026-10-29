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
"""Keep component client operations on the launch event loop through shutdown."""
import asyncio
import os

from composition_interfaces.srv import LoadNode
from launch import Action
from launch.event_handlers import OnShutdown
import launch.logging
from launch_ros.actions import ComposableNodeContainer as RosContainer, Node
from launch_ros.actions.load_composable_nodes import get_composable_node_load_request
from launch_ros.ros_adapters import ROSAdapter
from launch_ros.utilities import add_node_name


class _ComponentROSAdapter(ROSAdapter):
    """Release executor guards on its spin thread before context destruction."""

    def _run(self):
        try:
            super()._run()
        finally:
            # ROSAdapter.shutdown joins this thread before destroying its node
            # and context. Humble omits this executor cleanup in the base class.
            self.ros_executor.shutdown()


def _component_node(context):
    # Keep one owned adapter per launch context. An unrelated stock adapter may
    # already exist; never alter its lifetime or install a global monkeypatch.
    if not hasattr(context.locals, 'easyfly_component_adapter'):
        adapter = _ComponentROSAdapter(argv=[
            '--ros-args', '-r', f'__node:=easyfly_component_loader_{os.getpid()}'])
        context.extend_globals({'easyfly_component_adapter': adapter})
        context.register_event_handler(OnShutdown(
            on_shutdown=lambda *args, **kwargs: adapter.shutdown()))
    return context.locals.easyfly_component_adapter.ros_node


class LoadComposableNodes(Action):
    """
    Load once in sequence; yield while waiting and stop before adapter destruction.

    Humble's stock loader runs blocking client calls in executor threads while
    its OnShutdown handler destroys the same clients on the launch thread. This
    action performs every client operation on that launch thread. No callback
    can resume across shutdown without checking context.is_shutdown first.
    """

    def __init__(self, *, composable_node_descriptions, target_container, **kwargs):
        super().__init__(**kwargs)
        self.descriptions = composable_node_descriptions
        self.target = target_container
        self.logger = launch.logging.get_logger(__name__)

    def execute(self, context):
        """Freeze scoped request parameters before returning to the launch visitor."""
        if context.is_shutdown:
            return
        requests = [get_composable_node_load_request(d, context) for d in self.descriptions]
        if not requests:
            return
        node = _component_node(context)
        client = node.create_client(LoadNode, self.target + '/_container/load_node')
        context.add_completion_future(context.asyncio_loop.create_task(
            self._load(context, node, client, requests)))

    async def _load(self, context, node, client, requests):
        pending = None
        try:
            for request in requests:
                while not context.is_shutdown:
                    if client.service_is_ready():
                        break
                    await asyncio.sleep(.05)
                if context.is_shutdown:
                    return
                pending = client.call_async(request)
                while not context.is_shutdown and not pending.done():
                    await asyncio.sleep(.01)
                if context.is_shutdown:
                    return
                response = pending.result()  # Preserve actual RPC errors.
                if not response.success:
                    raise RuntimeError(
                        f'Failed to load {request.plugin_name} in {self.target}: '
                        + response.error_message)
                if response.full_node_name:
                    add_node_name(context, response.full_node_name)
                self.logger.info(f"Loaded node '{response.full_node_name}' in '{self.target}'")
                pending = None
        finally:
            if pending is not None and not pending.done():
                pending.cancel()
            # OnShutdown already destroys all adapter clients. Never touch a
            # destroyed handle, and never start the next request after shutdown.
            if not context.is_shutdown:
                node.destroy_client(client)


class ComposableNodeContainer(RosContainer):
    """The standard container process with an event-loop-owned load sequence."""

    def __init__(self, *, composable_node_descriptions=None, **kwargs):
        super().__init__(composable_node_descriptions=composable_node_descriptions, **kwargs)
        self.descriptions = composable_node_descriptions or []

    def execute(self, context):
        """Start the unchanged container and load only enabled descriptions."""
        if context.is_shutdown:
            return
        descriptions = [d for d in self.descriptions
                        if d.condition() is None or d.condition().evaluate(context)]
        actions = Node.execute(self, context) or []
        if descriptions:
            actions.append(LoadComposableNodes(
                composable_node_descriptions=descriptions, target_container=self.node_name))
        return actions
