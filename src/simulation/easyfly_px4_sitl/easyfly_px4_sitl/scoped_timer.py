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

"""Keep delayed simulation actions inside their original launch input scope."""

from launch.actions import (
    GroupAction,
    ResetLaunchConfigurations,
    SetEnvironmentVariable,
    TimerAction,
    UnsetEnvironmentVariable,
)


class ScopedTimerAction(TimerAction):
    """Humble TimerAction retains locals, but not scoped configs/environment."""

    def execute(self, context):
        self._saved_configurations = dict(context.launch_configurations)
        self._saved_environment = dict(context.environment)
        return super().execute(context)

    def handle(self, context):
        actions = super().handle(context)
        restore_environment = [
            UnsetEnvironmentVariable(name)
            for name in context.environment
            if name not in self._saved_environment
        ]
        restore_environment.extend(
            SetEnvironmentVariable(name, value)
            for name, value in self._saved_environment.items()
        )
        # Restore only while visiting these actions. The caller's scope remains
        # unchanged afterward, including on nested timers. Shutdown cancellation
        # and period resolution stay owned by the original TimerAction.
        return [GroupAction(scoped=True, actions=[
            ResetLaunchConfigurations(self._saved_configurations),
            *restore_environment,
            *actions,
        ])]
