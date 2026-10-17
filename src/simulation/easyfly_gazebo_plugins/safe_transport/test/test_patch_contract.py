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

"""Source provenance and process-local activation contracts."""

import ast
import hashlib
from pathlib import Path

PATCH = Path(__file__).resolve().parents[1]
ROOT = Path(__file__).resolve().parents[5]


def test_unmodified_upstream_sources():
    """Require every original implementation to retain its imported bytes."""
    folder = PATCH / 'upstream'
    entries = [line.split() for line in (folder / 'SHA256SUMS').read_text().splitlines()]
    assert len(entries) == 11
    assert {name for _, name in entries} == {p.name for p in folder.glob('*.cc')}
    for expected, name in entries:
        data = (folder / name).read_bytes()
        assert hashlib.sha256(data).hexdigest() == expected
        assert b'Open Source Robotics Foundation' in data
        assert b'Apache License, Version 2.0' in data


def test_both_sitl_entries_select_only_the_private_server():
    """Activation uses an explicit executable, never a global library override."""
    folder = ROOT / 'src/simulation/easyfly_px4_sitl/launch'
    for name in ('px4_sitl.launch.py', 'perception_sitl.launch.py'):
        tree = ast.parse((folder / name).read_text())
        server_assignment = next(
            item for item in ast.walk(tree) if isinstance(item, ast.Assign)
            and any(isinstance(t, ast.Name) and t.id == 'gazebo_server'
                    for t in item.targets))
        assert ast.literal_eval(server_assignment.value.right) == (
            '/easyfly_gazebo_plugins/easyfly_gzserver')
        process_assignment = next(
            item for item in ast.walk(tree) if isinstance(item, ast.Assign)
            and any(isinstance(t, ast.Name) and t.id == 'gzserver' for t in item.targets))
        command = next(k.value for k in process_assignment.value.keywords if k.arg == 'cmd')
        assert isinstance(command.elts[0], ast.Name)
        assert command.elts[0].id == 'gazebo_server'
        assert 'LD_PRELOAD' not in (folder / name).read_text()
        assert '/transport' not in (folder / name).read_text()


def test_topic_manager_changes_only_registered_node_finalization():
    original = (PATCH / 'upstream/TopicManager.cc').read_text()
    corrected = (PATCH / 'src/topic_manager.cpp').read_text()
    start = corrected.index('  // Node owns its publishers')
    end = corrected.index('  PublicationPtr_M::iterator iter;', start)
    assert 'retiring = this->nodes;' in corrected[start:end]
    assert 'node->Fini();' in corrected[start:end]
    assert corrected[:start] + corrected[end:] == original
