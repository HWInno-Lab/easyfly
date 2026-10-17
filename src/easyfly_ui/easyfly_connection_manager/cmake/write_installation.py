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
"""Record optional verified PX4 input as installation data, never shell text."""
import json
import os
from pathlib import Path
import sys

root = os.environ.get('EASYFLY_PX4_HRT_COMPAT_DIR', '')
value = str(Path(root) / 'px4-input') if root else ''
Path(sys.argv[1]).write_text(json.dumps({'px4_directory': value}))
