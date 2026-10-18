# Copyright 2026 EasyFly Project
#
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

include(CMakeFindDependencyMacro)
find_dependency(easyfly_astar)
find_dependency(easyfly_grid_planner_api)
get_filename_component(_easyfly_planner_include "${CMAKE_CURRENT_LIST_DIR}/../../../include" ABSOLUTE)
set(easyfly_global_planner_INCLUDE_DIRS "${_easyfly_planner_include}")
foreach(_easyfly_planner_dependency easyfly_astar easyfly_grid_planner_api)
  get_target_property(_easyfly_planner_headers
    ${_easyfly_planner_dependency}::${_easyfly_planner_dependency} INTERFACE_INCLUDE_DIRECTORIES)
  list(APPEND easyfly_global_planner_INCLUDE_DIRS ${_easyfly_planner_headers})
endforeach()
unset(_easyfly_planner_include)
unset(_easyfly_planner_headers)
unset(_easyfly_planner_dependency)
