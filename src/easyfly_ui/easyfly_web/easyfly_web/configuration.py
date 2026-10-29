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

"""Explicit TLS, account and mission allowlist configuration."""

import ipaddress
import re
from urllib.parse import urlsplit

from easyfly_connection_manager.remote_profiles import private_json
from easyfly_connection_manager.remote_service import token_file


HEX = re.compile('[a-f0-9]{32}')
NAME = re.compile('[A-Za-z0-9_][A-Za-z0-9_.-]{0,127}')


def load(path):
    """Require separate personal roles and explicit execution enablement before binding."""
    value = private_json(path)
    fields = {'platform_id', 'bind', 'port', 'origin', 'certificate', 'private_key',
              'state_directory', 'ros_domain_id', 'simulation_time', 'enable_execution',
              'missions', 'accounts'}
    if not isinstance(value, dict) or set(value) != fields:
        raise ValueError('invalid Web configuration fields')
    if not isinstance(value['platform_id'], str) or not HEX.fullmatch(value['platform_id']):
        raise ValueError('platform_id must be 32 lowercase hexadecimal characters')
    ipaddress.IPv4Address(value['bind'])
    if type(value['port']) is not int or not 1 <= value['port'] <= 65535:
        raise ValueError('invalid Web port')
    if type(value['ros_domain_id']) is not int or not 0 <= value['ros_domain_id'] <= 232:
        raise ValueError('invalid ROS domain')
    if any(type(value[key]) is not bool for key in ('simulation_time', 'enable_execution')):
        raise ValueError('clock and execution choices must be explicit booleans')
    parsed = urlsplit(value['origin'])
    if (parsed.scheme != 'https' or not parsed.hostname or parsed.username or parsed.password or
            parsed.path or parsed.query or parsed.fragment or
            (parsed.port or 443) != value['port']):
        raise ValueError('origin must be an exact HTTPS origin matching the listener port')
    if (not isinstance(value['missions'], list) or len(value['missions']) > 64 or
            any(not isinstance(name, str) or not NAME.fullmatch(name) or '..' in name
                for name in value['missions']) or
            len(set(value['missions'])) != len(value['missions'])):
        raise ValueError('missions must be unique catalog basenames')
    if not isinstance(value['accounts'], list) or not 1 <= len(value['accounts']) <= 16:
        raise ValueError('configure one to sixteen accounts')
    names, tokens = set(), set()
    for account in value['accounts']:
        if (not isinstance(account, dict) or set(account) != {'name', 'role', 'token_file'} or
                not isinstance(account['name'], str) or not NAME.fullmatch(account['name']) or
                account['role'] not in ('viewer', 'operator') or account['name'] in names):
            raise ValueError('invalid or duplicate Web account')
        token = token_file(account['token_file'])
        if token in tokens:
            raise ValueError('accounts must have distinct authentication tokens')
        account['token'] = token
        tokens.add(token)
        names.add(account['name'])
    return value
