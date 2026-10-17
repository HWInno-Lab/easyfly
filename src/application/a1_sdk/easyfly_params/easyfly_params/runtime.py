# Copyright 2026 EasyFly Project
#
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

"""Local typed configuration services; never write flight/backend parameters."""
import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from easyfly_msgs.srv import QueryParams
from std_srvs.srv import Trigger

from .configuration import load_profile


class ParameterManager(Node):
    def __init__(self):
        super().__init__('easyfly_params')
        readonly = ParameterDescriptor(read_only=True)
        self.declare_parameter('profile', 'navigation_dev', readonly)
        self.effective = load_profile(self.get_parameter('profile').value)
        self.declare_parameter('schema_version', 1, readonly)
        for key, value in self.effective['contracts'].items():
            if key != 'schema_version':
                self.declare_parameter(key, value, readonly)
        self.declare_parameter('operator_note', '')
        self.declare_parameter('export_pretty', False)
        initial = self.validate(
            [self.get_parameter(n) for n in ('operator_note', 'export_pretty')]
        )
        if not initial.successful:
            raise ValueError(initial.reason)
        # Frozen contract overrides must match the actual selected profile.
        for key, value in self.effective['contracts'].items():
            if self.get_parameter(key).value != value:
                raise ValueError(
                    f'{key}: startup override differs from profile contract'
                )
        self.add_on_set_parameters_callback(self.validate)
        self.create_service(QueryParams, '/platform/params/query', self.query)
        self.create_service(Trigger, '/platform/params/export', self.export)

    @staticmethod
    def validate(parameters):
        for p in parameters:
            if p.name == 'operator_note':
                if (
                    type(p.value) is not str
                    or len(p.value.encode('utf-8')) > 512
                ):
                    return SetParametersResult(
                        successful=False,
                        reason='operator_note must be a string <=512 bytes',
                    )
            elif p.name == 'export_pretty':
                if type(p.value) is not bool:
                    return SetParametersResult(
                        successful=False, reason='export_pretty must be bool'
                    )
            else:
                return SetParametersResult(
                    successful=False,
                    reason='parameter is read-only or unsupported',
                )
        return SetParametersResult(successful=True)

    def query(self, request, response):
        try:
            if not self.has_parameter(request.param_name):
                raise ValueError(
                    'unknown local parameter; remote writes are not supported'
                )
            if request.param_value != '':
                value = json.loads(request.param_value)
                if type(value) not in (str, bool, int, float):
                    raise ValueError(
                        'expected a JSON scalar, not null/array/object'
                    )
                parameter = rclpy.parameter.Parameter(
                    request.param_name, value=value
                )
                result = self.set_parameters_atomically([parameter])
                if not result.successful:
                    raise ValueError(result.reason)
            response.success = True
            response.current_value = json.dumps(
                self.get_parameter(request.param_name).value,
                ensure_ascii=False,
                allow_nan=False,
            )
            response.message = (
                'local typed value as JSON; empty request means read'
            )
        except (
            ValueError,
            TypeError,
            rclpy.exceptions.ParameterException,
        ) as exc:
            response.success = False
            response.message = str(exc)
            if self.has_parameter(request.param_name):
                response.current_value = json.dumps(
                    self.get_parameter(request.param_name).value,
                    ensure_ascii=False,
                    allow_nan=False,
                )
        return response

    def export(self, _request, response):
        result = dict(self.effective)
        result['operator_note'] = self.get_parameter('operator_note').value
        result[
            'scope'
        ] = 'managed launch inputs and frozen source contracts, not all live ROS parameters'
        response.success = True
        response.message = json.dumps(
            result,
            ensure_ascii=False,
            allow_nan=False,
            indent=2 if self.get_parameter('export_pretty').value else None,
        )
        return response


def main():
    rclpy.init()
    node = None
    try:
        node = ParameterManager()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
