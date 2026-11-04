# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_mesh_comm/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project

"""Bounded reciprocal link-state routing; an advertised one-way edge is never enough."""

from collections import deque


def encode_links(neighbors):
    """Encode a bounded canonical list of directly verified peer identities."""
    values = sorted(set(neighbors))
    if len(values) > 15:
        raise ValueError('too many mesh neighbors')
    return bytes([len(values)]) + b''.join(bytes.fromhex(peer) for peer in values)


def decode_links(data, source, members):
    """Reject malformed, self-referential and unpinned link advertisements."""
    if not data or data[0] > 15 or len(data) != 1 + data[0] * 16:
        raise ValueError('invalid mesh link payload')
    values = [data[i:i + 16].hex() for i in range(1, len(data), 16)]
    if values != sorted(set(values)) or source in values or not set(values) <= members:
        raise ValueError('invalid mesh neighbor identity')
    return set(values)


class Topology:
    """Compute deterministic shortest-hop paths from fresh mutually confirmed edges."""

    def __init__(self, own, members):
        self.own = own
        self.members = set(members) | {own}
        self.advertisements = {}

    def update(self, source, neighbors, deadline):
        """Store at most one authenticated advertisement per pinned member."""
        if source not in self.members or source == self.own:
            raise ValueError('invalid topology source')
        self.advertisements[source] = (set(neighbors), deadline)

    def routes(self, direct, now):
        """Expire stale advertisements without extending lifetime when packets are relayed."""
        self.advertisements = {peer: value for peer, value in self.advertisements.items()
                               if value[1] > now}
        graph = {peer: value[0] for peer, value in self.advertisements.items()}
        graph[self.own] = set(direct)
        paths = {self.own: [self.own]}
        queue = deque([self.own])
        while queue:
            current = queue.popleft()
            for peer in sorted(graph.get(current, set())):
                if current not in graph.get(peer, set()) or peer in paths:
                    continue
                paths[peer] = paths[current] + [peer]
                queue.append(peer)
        paths.pop(self.own)
        return paths
