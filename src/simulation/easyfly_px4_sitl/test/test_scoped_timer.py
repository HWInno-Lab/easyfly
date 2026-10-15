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

"""Run real launch timers/scopes without processes, nodes or simulator writes."""

import launch.logging
import pytest
from launch import LaunchDescription, LaunchService
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    OpaqueFunction,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
)
from launch.substitutions import LaunchConfiguration
from launch.events import Shutdown
from easyfly_px4_sitl.scoped_timer import ScopedTimerAction


@pytest.fixture(autouse=True)
def isolated_logs(tmp_path):
    previous = launch.logging.launch_config._log_dir
    launch.logging.launch_config.log_dir = str(tmp_path)
    yield
    launch.logging.launch_config._log_dir = previous


def test_delayed_scope_preserves_inputs_without_leaking_to_parent(monkeypatch):
    name = 'EASYFLY_SCOPED_TIMER_TEST_VALUE'
    monkeypatch.delenv(name, raising=False)
    seen = []

    def inspect_delayed(context):
        seen.append(('delayed', dict(context.launch_configurations),
                     context.environment.get(name)))
        assert LaunchConfiguration('headless').perform(context) == 'true'
        assert 'config_file' not in context.launch_configurations
        assert context.environment[name] == 'private-model-path'
        return []

    def inspect_parent(context):
        seen.append(('parent', dict(context.launch_configurations),
                     context.environment.get(name)))
        assert context.launch_configurations == {'config_file': 'polluted.yaml'}
        assert name not in context.environment
        return []

    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        SetLaunchConfiguration('config_file', 'polluted.yaml'),
        GroupAction(forwarding=False, actions=[
            DeclareLaunchArgument('headless', default_value='true'),
            SetEnvironmentVariable(name, 'private-model-path'),
            ScopedTimerAction(period=.01, actions=[OpaqueFunction(function=inspect_delayed)]),
        ]),
        OpaqueFunction(function=inspect_parent),
        ScopedTimerAction(period=.03, actions=[OpaqueFunction(function=inspect_parent)]),
    ]))
    assert service.run() == 0
    assert [entry[0] for entry in seen] == ['parent', 'delayed', 'parent']


def test_nested_delayed_scope_preserves_clock_and_strategy():
    seen = []

    def inspect_nested(context):
        seen.append(dict(context.launch_configurations))
        assert LaunchConfiguration('use_sim_time').perform(context) == 'true'
        assert LaunchConfiguration('navigation_strategy').perform(context) == 'combined'
        assert 'headless' not in context.launch_configurations
        return []

    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        GroupAction(forwarding=False, launch_configurations={'headless': 'true'}, actions=[
            ScopedTimerAction(period=.01, actions=[
                GroupAction(forwarding=False, launch_configurations={
                    'use_sim_time': 'true', 'navigation_strategy': 'combined'}, actions=[
                    ScopedTimerAction(period=.01, actions=[
                        OpaqueFunction(function=inspect_nested)]),
                ]),
            ]),
        ]),
    ]))
    assert service.run() == 0
    assert len(seen) == 1


def test_shutdown_still_cancels_pending_scoped_actions():
    seen = []

    def mark(context):
        seen.append(True)
        return []

    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        ScopedTimerAction(period=.2, actions=[OpaqueFunction(function=mark)]),
        ScopedTimerAction(period=.01, actions=[EmitEvent(event=Shutdown(reason='test-only'))]),
    ]))
    assert service.run() == 0
    assert not seen
