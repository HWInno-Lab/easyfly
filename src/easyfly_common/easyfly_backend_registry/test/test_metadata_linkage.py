# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Different installed registry snapshots must coexist inside one process."""
import copy
import json
from pathlib import Path
import shutil
import subprocess
import sys


def test_old_and_new_dso_lookups_do_not_interpose_in_either_load_order(tmp_path):
    package = Path(__file__).resolve().parents[1]
    original = package/'easyfly_backend_registry'
    libraries = []
    for name, extra in [('old', False), ('new', True)]:
        tree = tmp_path/name
        shutil.copytree(original, tree)
        if extra:
            path = tree/'data/backends.json'
            data = json.loads(path.read_text())
            row = copy.deepcopy(next(b for b in data['backends'] if b['id'] == 'csrt'))
            row['id'] = 'fixture_extra_single'
            data['backends'].append(row)
            path.write_text(json.dumps(data))
        include = tree/'include/easyfly_backend_registry'
        include.mkdir(parents=True)
        subprocess.run([sys.executable, str(package/'cmake/generate_metadata.py'),
            str(tree/'data'), str(include/'metadata.hpp')], check=True)
        source = tree/'probe.cpp'
        source.write_text('#include <easyfly_backend_registry/metadata.hpp>\n'
            'extern "C" int '+name+'_probe() {try {\n'
            'easyfly::backends::require_backend("single_tracking", "fixture_extra_single");\n'
            'easyfly::backends::resolve_parameters("single_tracking", "fixture_extra_single", {});\n'
            'return 1; } catch (const std::invalid_argument &) {return 0;} }\n')
        library = tree/'libprobe.so'
        subprocess.run(['c++', '-std=c++17', '-shared', '-fPIC', '-I'+str(tree/'include'),
            '-I'+str(package/'include'), str(source), '-o', str(library)], check=True)
        libraries.append(library)
    # Fresh processes prevent an earlier load order from affecting the next one.
    for order in [(0, 1), (1, 0)]:
        script = 'import ctypes\n'
        for index in order:
            name = ['old', 'new'][index]
            script += name+'=ctypes.CDLL('+repr(str(libraries[index]))+', mode=ctypes.RTLD_GLOBAL)\n'
        script += 'assert old.old_probe() == 0\nassert new.new_probe() == 1\n'
        subprocess.run([sys.executable, '-c', script], check=True)
