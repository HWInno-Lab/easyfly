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
"""Verified HTTPS client with explicit identities and no automatic mutation retries."""

import argparse
import http.client
import json
import re
import ssl
from urllib.parse import urlsplit
import uuid

from .remote_profiles import private_json
from .remote_service import token_file


class Client:
    """Use one administrator-issued client configuration for status and lifecycle requests."""

    def __init__(self, config):
        if (set(config) != {'url', 'ca_file', 'token_file', 'server_id', 'client_id'} or
                any(not re.fullmatch('[a-f0-9]{32}', config[key])
                    for key in ('server_id', 'client_id'))):
            raise ValueError('invalid remote client configuration')
        self.config = config
        self.url = urlsplit(config['url'])
        if (self.url.scheme != 'https' or not self.url.hostname or self.url.username or
                self.url.password or self.url.query or self.url.fragment or
                self.url.path not in ('', '/')):
            raise ValueError('client requires a verified HTTPS origin without credentials')
        self.token = token_file(config['token_file'])
        self.context = ssl.create_default_context(cafile=config['ca_file'] or None)
        self.context.minimum_version = ssl.TLSVersion.TLSv1_2

    def request(self, method, path, value=None):
        """Bound response and timeout; a lost mutation response remains unknown to the client."""
        connection = http.client.HTTPSConnection(self.url.hostname, self.url.port,
                                                 context=self.context, timeout=2.)
        try:
            headers = {'Authorization': 'Bearer ' + self.token}
            data = None
            if value is not None:
                data = json.dumps(value, allow_nan=False).encode()
                headers['Content-Type'] = 'application/json'
            connection.request(method, path, body=data, headers=headers)
            response = connection.getresponse()
            data = response.read(262145)
            if len(data) > 262144:
                raise ValueError('oversized remote response')
            result = json.loads(data)
            if response.status != 200:
                raise ValueError('remote request rejected: ' + str(result.get('error', 'unknown')))
            return result
        finally:
            connection.close()

    def status(self):
        """Authenticate the expected platform identity in addition to TLS host verification."""
        result = self.request('GET', '/v1/status')
        if result.get('v') != 1 or result.get('server_id') != self.config['server_id']:
            raise ValueError('remote platform identity does not match the saved configuration')
        return result

    def operation(self, snapshot, operation, profile, session='', request_id=None):
        """Send exactly one operation for the previously observed platform boot and session."""
        body = {'v': 1, 'server_id': self.config['server_id'], 'boot_id': snapshot['boot_id'],
                'id': request_id or uuid.uuid4().hex, 'client_id': self.config['client_id'],
                'op': operation, 'profile': profile, 'session': session}
        return self.request('POST', '/v1/operation', body)


def main():
    """Offer an installed CLI as well as the desktop remote-management window."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--settings', required=True)
    parser.add_argument('operation', choices=['status', 'start', 'stop'])
    parser.add_argument('--profile', default='')
    parser.add_argument('--session', default='')
    args = parser.parse_args()
    client = Client(private_json(args.settings))
    snapshot = client.status()
    result = snapshot if args.operation == 'status' else client.operation(
        snapshot, args.operation, args.profile, args.session)
    print(json.dumps(result, ensure_ascii=False, indent=2))
