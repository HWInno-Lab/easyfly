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

"""Source continuity contracts independent of flight geometry and detector accuracy."""

from collections import OrderedDict
from types import SimpleNamespace as S

from easyfly_visual_navigation.inputs import Inputs
from easyfly_visual_navigation import inputs
import pytest


def test_discovery_settles_before_pinning_and_then_latches_changes(monkeypatch):
    """Tolerate asynchronous initial discovery only before any source generation is admitted."""
    clock = [10.]
    monkeypatch.setattr(inputs.time, 'monotonic', lambda: clock[0])
    endpoints = {'/image': [b'a'], '/tf': [b'b'], '/tf_static': [b'c'], '/clock': [b'd']}
    peer = S(discovery_at=0., topics={'observation': '/image'},
             config={'use_sim_time': True}, writers=None, discovered_candidates=None,
             candidates_since=0., fault='', get_publishers_info_by_topic=lambda topic: [
                 S(endpoint_gid=list(gid)) for gid in endpoints[topic]])
    Inputs.discover(peer)
    assert peer.writers is None
    clock[0] += .3
    endpoints['/tf'].append(b'e')
    Inputs.discover(peer)
    assert peer.writers is None and not peer.fault
    clock[0] += .7
    Inputs.discover(peer)
    assert peer.writers is None
    clock[0] += .4
    Inputs.discover(peer)
    assert peer.writers is not None and not peer.fault
    clock[0] += .3
    endpoints['/image'] = [b'z']
    Inputs.discover(peer)
    assert 'generation changed' in peer.fault


def test_duplicate_source_cannot_renew_age_or_replace_content(monkeypatch):
    """Preserve the first receipt of a distinct source stamp despite rapid duplicate publishing."""
    clock = [10.]
    monkeypatch.setattr(inputs.time, 'monotonic', lambda: clock[0])
    peer = S(streams={'state': OrderedDict()}, clock_ns=10 * 10**9)
    first = S(header=S(stamp=S(sec=10, nanosec=0)), value='original')
    Inputs.receive(peer, 'state', first)
    clock[0] += .4
    Inputs.receive(peer, 'state', S(header=first.header, value='modified duplicate'))
    assert Inputs.latest(peer, 'state').value == 'original'
    clock[0] += .2
    with pytest.raises(RuntimeError, match='expired'):
        Inputs.latest(peer, 'state')
