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

"""Explicit bounded trust and direct-neighbor configuration."""

import ipaddress
import json
import os
from pathlib import Path
import re
import stat

from easyfly_udp_protocol.protocol import identity, read_key


TOPIC = re.compile(r'/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*')


def unique_object(pairs):
    """Reject duplicate JSON fields instead of silently changing identity configuration."""
    value = {}
    for key, item in pairs:
        if key in value:
            raise ValueError('duplicate configuration field')
        value[key] = item
    return value


def endpoint(value):
    """Validate a literal IPv4 endpoint without DNS or multicast ambiguity."""
    if (not isinstance(value, list) or len(value) != 2 or
            not isinstance(value[0], str) or type(value[1]) is not int or
            not 1 <= value[1] <= 65535):
        raise ValueError('endpoint must be [IPv4, port]')
    address = ipaddress.IPv4Address(value[0])
    if address.is_multicast or address.is_unspecified or int(address) == 2**32 - 1:
        raise ValueError('mesh endpoints must be explicit unicast addresses')
    return str(address), value[1]


def load(path):
    """Load explicit private signing key, pinned members, topic bindings and direct edges."""
    with os.fdopen(os.open(path, os.O_RDONLY | os.O_NOFOLLOW), 'r') as stream:
        info = os.fstat(stream.fileno())
        if not stat.S_ISREG(info.st_mode) or info.st_size > 32768:
            raise ValueError('configuration must be a bounded regular file')
        value = json.loads(stream.read(32769), object_pairs_hook=unique_object)
    required = {'version', 'private_key_file', 'bind', 'input_topic', 'peers'}
    if (not isinstance(value, dict) or set(value) != required or value['version'] != 1 or
            not isinstance(value['peers'], list) or not 1 <= len(value['peers']) <= 15 or
            not isinstance(value['input_topic'], str) or
            not TOPIC.fullmatch(value['input_topic'])):
        raise ValueError('invalid mesh configuration')
    if not isinstance(value['private_key_file'], str) or not Path(
            value['private_key_file']).is_absolute():
        raise ValueError('private key path must be absolute')
    key = read_key(value['private_key_file'], private=True)
    own = identity(key.public_key())
    binding = endpoint(value['bind'])
    trusted, direct, outputs = {}, {}, {}
    for peer in value['peers']:
        fields = {'public_key_file', 'endpoint', 'output_topic'}
        if not isinstance(peer, dict) or set(peer) != fields:
            raise ValueError('invalid mesh peer fields')
        if not isinstance(peer['public_key_file'], str) or not Path(
                peer['public_key_file']).is_absolute():
            raise ValueError('public key path must be absolute')
        public = read_key(peer['public_key_file'])
        source = identity(public)
        if source == own or source in trusted:
            raise ValueError('duplicate mesh identity')
        topic = peer['output_topic']
        if (not isinstance(topic, str) or not TOPIC.fullmatch(topic) or
                topic == value['input_topic'] or topic in outputs.values()):
            raise ValueError('mesh topics must be unique absolute names')
        trusted[source], outputs[source] = public, topic
        if peer['endpoint'] is not None:
            target = endpoint(peer['endpoint'])
            if target == binding or target in direct.values():
                raise ValueError('direct mesh endpoints must be distinct')
            direct[source] = target
    if not direct:
        raise ValueError('at least one direct neighbor is required')
    return key, binding, trusted, direct, outputs, value['input_topic']
