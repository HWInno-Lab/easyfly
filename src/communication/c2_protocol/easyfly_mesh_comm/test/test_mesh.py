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

"""Routing and trust boundaries with real signed packets and configuration files."""

import json

from easyfly_mesh_comm.configuration import load
from easyfly_mesh_comm.topology import decode_links, encode_links, Topology
from easyfly_udp_protocol.protocol import (BROADCAST, decode, Encoder, generate_keys,
                                           MESH_LINKS, read_key, ReplayWindow)
import pytest


A, B, C, D = [str(i) * 32 for i in range(1, 5)]


def test_reciprocal_routes_expire_and_recover():
    """A diamond switches next hop only after fresh reciprocal alternate edges exist."""
    graph = Topology(A, {B, C, D})
    graph.update(B, {A, D}, 2.)
    graph.update(C, {A, D}, 8.)
    assert D not in graph.routes({B, C}, 1.)
    graph.update(D, {B, C}, 8.)
    assert graph.routes({B, C}, 1.)[D] == [A, B, D]
    assert graph.routes({B, C}, 3.)[D] == [A, C, D]
    assert graph.routes(set(), 3.) == {}
    assert graph.routes({C}, 9.) == {}
    graph.update(C, {A, D}, 12.)
    graph.update(D, {C}, 12.)
    assert graph.routes({C}, 10.)[D] == [A, C, D]


def test_link_payload_strict_membership():
    """Unknown, duplicate, malformed and self edges do not create topology."""
    good = encode_links({B, C})
    assert decode_links(good, A, {A, B, C}) == {B, C}
    bad = [good[:-1], good + b'x', encode_links({D}), encode_links({A}),
           b'\x02' + bytes.fromhex(B) * 2]
    for value in bad:
        with pytest.raises(ValueError):
            decode_links(value, A, {A, B, C})


def test_forwarded_packet_identity_and_lifetime(tmp_path):
    """Byte-for-byte relay preserves origin and cannot extend TTL or suppress replay checks."""
    source = generate_keys(str(tmp_path / 'source'))
    private = read_key(tmp_path / 'source/private.key', private=True)
    trust = {source: private.public_key()}
    original = Encoder(private).encode(MESH_LINKS, BROADCAST, encode_links({B}),
                                       1000, now_ns=10000000000)
    relay = decode(original, trust, now_ns=10500000000)
    destination = decode(bytes(original), trust, now_ns=10900000000)
    assert relay == destination and destination.source == source
    window = ReplayWindow()
    window.accept(destination)
    with pytest.raises(ValueError):
        window.accept(destination)
    with pytest.raises(ValueError):
        decode(original, trust, now_ns=11000000001)
    damaged = original[:50] + bytes([original[50] ^ 1]) + original[51:]
    with pytest.raises(ValueError):
        decode(damaged, trust, now_ns=10900000000)


def test_configuration_identity_and_topic_isolation(tmp_path):
    """Trust is explicit, duplicate keys/endpoints/topics cannot silently alias vehicles."""
    ids = [generate_keys(str(tmp_path / name)) for name in ['a', 'b', 'c']]
    value = dict(version=1, private_key_file=str(tmp_path / 'a/private.key'),
                 bind=['127.0.0.1', 9100], input_topic='/a/source', peers=[
                     dict(public_key_file=str(tmp_path / 'b/public.key'),
                          endpoint=['127.0.0.1', 9101], output_topic='/a/b'),
                     dict(public_key_file=str(tmp_path / 'c/public.key'),
                          endpoint=None, output_topic='/a/c')])
    path = tmp_path / 'config.json'
    path.write_text(json.dumps(value))
    _, binding, trusted, direct, outputs, topic = load(path)
    assert binding == ('127.0.0.1', 9100) and topic == '/a/source'
    assert set(trusted) == set(ids[1:]) and set(direct) == {ids[1]}
    assert set(outputs.values()) == {'/a/b', '/a/c'}
    for field, replacement in [('public_key_file', value['peers'][0]['public_key_file']),
                               ('endpoint', ['127.0.0.1', 9101]),
                               ('output_topic', '/a/source')]:
        bad = json.loads(json.dumps(value))
        bad['peers'][1][field] = replacement
        path.write_text(json.dumps(bad))
        with pytest.raises(ValueError):
            load(path)
    path.write_text('{"version":1,"version":2}')
    with pytest.raises(ValueError):
        load(path)
