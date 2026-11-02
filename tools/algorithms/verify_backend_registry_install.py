#!/usr/bin/env python3
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
import os,subprocess,shutil,json,argparse
from pathlib import Path
parser=argparse.ArgumentParser(description='Verify relocated backend-registry C++ and Python consumers without ROS runtime imports.')
parser.add_argument('--prefix', type=Path, required=True, help='installed easyfly_backend_registry package prefix')
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--python', type=Path, default=Path('/usr/bin/python3'))
args=parser.parse_args()
out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
source=args.prefix.resolve();relocated=out/'relocated';shutil.copytree(source,relocated,symlinks=False)
(out/'CMakeLists.txt').write_text('''cmake_minimum_required(VERSION 3.16)
project(registry_consumer LANGUAGES CXX)
find_package(easyfly_backend_registry REQUIRED)
add_executable(consumer main.cpp)
target_link_libraries(consumer easyfly_backend_registry::easyfly_backend_registry)
''')
(out/'main.cpp').write_text('''#include <easyfly_backend_registry/metadata.hpp>
#include <stdexcept>
struct Sort {double iou_threshold{.15}; int max_lost_frames{45}, min_hits{3}, max_tracks{100}; double dist_gate_scale{1.5};};
int main() {
  auto & d=easyfly::backends::require_backend("multi_tracking", "sort");
  easyfly::backends::validate_sort(Sort{});
  if (!d.entry || !d.package) return 1;
  try {easyfly::backends::require_backend("multi_tracking", "missing"); return 2;} catch(const std::invalid_argument &) {}
  return 0;
}
''')
env={k:v for k,v in os.environ.items() if not k.startswith(('AMENT','COLCON','CMAKE_PREFIX','PYTHONPATH','LD_LIBRARY'))};env['PYTHONNOUSERSITE']='1'
commands=[]
def run(name,cmd,env=env):
 with (out/(name+'.log')).open('w') as log:rc=subprocess.run(list(map(str,cmd)),env=env,stdout=log,stderr=subprocess.STDOUT).returncode
 commands.append(dict(name=name,command=list(map(str,cmd)),returncode=rc));(out/'commands.json').write_text(json.dumps(commands,indent=2));assert rc==0,name
run('configure',['cmake','-S',out,'-B',out/'build','-DCMAKE_PREFIX_PATH='+str(relocated)])
run('compile',['cmake','--build',out/'build'])
run('run',[out/'build/consumer'])
run('ldd',['ldd',out/'build/consumer'])
python=args.python.absolute();module=next(relocated.glob('local/lib/python*/dist-packages'))
script='import sys; sys.path.insert(0, '+repr(str(module))+'); from easyfly_backend_registry import Registry; r=Registry(); assert len(r.entries)>=12; assert r.resolve()["backends"]["combined_grid"]["parameters"]["max_cells"]==1000000; print("relocated Python metadata, no ROS imports")'
run('python',[python,'-c',script])
(out/'summary.json').write_text(json.dumps(dict(passed=True,commands=commands),indent=2));print('relocated registry C++ and Python consumers passed')
