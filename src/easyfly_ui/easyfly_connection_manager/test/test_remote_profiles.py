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
"""Check remote preset admission and required exclusive runtime preparation."""

import copy
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from easyfly_connection_manager.remote_profiles import (  # noqa: E402
    command, private_json, validate)


def config():
    """Load the installed contract example without embedding actual credentials."""
    root = Path(__file__).resolve().parents[1]
    return json.loads((root / 'config/remote-server.example.json').read_text())


@pytest.mark.parametrize('change', [
    'shell', 'clock', 'unsafe_policy', 'wrong_type', 'wrong_output'])
def test_remote_presets_reject_ambiguous_inputs(change):
    """Reject remote executable injection, clock disagreement and misleading output identities."""
    value = config()
    profile = value['profiles']['color_line']
    if change == 'shell':
        profile['algorithm'] = '/bin/sh'
    elif change == 'clock':
        profile['parameters']['use_sim_time'] = True
    elif change == 'unsafe_policy':
        profile.update(algorithm='fast_lio', policy='isolated_observation')
    elif change == 'wrong_type':
        profile['output_type'] = 'std_msgs/msg/String'
    else:
        profile['parameters']['output_topic'] = '/somewhere_else'
    with pytest.raises(ValueError):
        validate(value)


def test_orb_runtime_is_created_exclusively(tmp_path):
    """Meet the real ORB constructor's existing absolute-directory requirement."""
    profile = copy.deepcopy(config()['profiles']['color_line'])
    profile.update(algorithm='orbslam3', parameters={})
    argv = command(profile, tmp_path, True)
    assert 'runtime_directory:=' + str(tmp_path / 'atlas') in argv
    assert (tmp_path / 'atlas').is_dir()
    assert (tmp_path / 'atlas').stat().st_mode & 0o777 == 0o700
    with pytest.raises(FileExistsError):
        command(profile, tmp_path, True)


def test_private_configuration_boundary(tmp_path):
    """Refuse shared and symlink configuration files even when JSON is valid."""
    path = tmp_path / 'configuration.json'
    path.write_text(json.dumps(config()))
    path.chmod(0o600)
    assert validate(private_json(path))['profiles']
    link = tmp_path / 'link'
    link.symlink_to(path)
    with pytest.raises(OSError):
        private_json(link)
    path.chmod(0o644)
    with pytest.raises(ValueError):
        private_json(path)
