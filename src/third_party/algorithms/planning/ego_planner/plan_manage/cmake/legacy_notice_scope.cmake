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
# Exact inherited-file scope; see ../NOTICE.md and ../legacy-notice-scope.json.
# A content change requires an explicit renewed provenance review.
set(_legacy_notice_records
  include/plan_manage/ego_replan_fsm.h
  75c399a9842cf8c705413fdc0f1e3299484038014802b7cb7d19f0b6c2d151be
  include/plan_manage/planner_manager.h
  7e934d1e5eccd3f41ca5fe1209846621e7e597c800e5d0cfd3b72775e77e8ea3
  launch/advanced_param.launch.py
  6ea5a47e0a733e2dc5c14ed0d0d58b5f9e2b00e991b83dfd7563d333a0fd9c64
  launch/sensors.launch.py
  e36ae98ba390d2e10e40cff045e82836c352a12c1af45059fd24590c5f9b5042
  launch/simulator.launch.py
  57674192a8dd9adc7197b64ec66119ec55aefb8cee04db088a18e09898365e08
  launch/single_run_in_sim.launch.py
  5a3ac6fd63eac74431e508485338e4c7d153d98991d6c1d4286469338c79d6d9
  launch/single_run_real.launch.py
  05a49a0af0ffa8c16db8770e823c1462eda6f53bd8618a80418396e119f8c0ac
  src/ego_planner_node.cpp
  71cf23a5a18ed6eb89d6f183b25c6a8ad07c3180bed31335b1e886fc2af0d1d0
  src/ego_replan_fsm.cpp
  883dd1164422dc0b965765b950e14f5acc0d15928b6c4a6452931711a512d661
  src/planner_manager.cpp
  ab8b6c672505de198ca438d7787f02f9b3dca331627fb7c1ed0bc062a5a659a3
  src/traj_server.cpp
  7de7b78647c0219c6825d4400396f2731ab84de7273e8af61674b0e0fa1425f9
)
set(_legacy_notice_files "")
set(_legacy_cpp_files "")
list(LENGTH _legacy_notice_records _record_count)
math(EXPR _last_record "${_record_count} - 1")
foreach(_index RANGE 0 ${_last_record} 2)
  math(EXPR _hash_index "${_index} + 1")
  list(GET _legacy_notice_records ${_index} _relative)
  list(GET _legacy_notice_records ${_hash_index} _expected)
  set(_path "${CMAKE_CURRENT_SOURCE_DIR}/${_relative}")
  file(SHA256 "${_path}" _actual)
  if(NOT _actual STREQUAL _expected)
    message(FATAL_ERROR "Renew the inherited copyright review for ${_relative}")
  endif()
  list(APPEND _legacy_notice_files "${_path}")
  if(_relative MATCHES "\\.(h|hpp|cpp)$")
    list(APPEND _legacy_cpp_files "${_path}")
  endif()
endforeach()
find_package(ament_cmake_copyright REQUIRED)
find_package(ament_cmake_cpplint REQUIRED)
ament_copyright(EXCLUDE ${_legacy_notice_files} ${AMENT_LINT_AUTO_FILE_EXCLUDE})
file(GLOB_RECURSE _all_cpp CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/*.c" "${CMAKE_CURRENT_SOURCE_DIR}/*.cc"
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/*.cxx"
  "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hh"
  "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp" "${CMAKE_CURRENT_SOURCE_DIR}/*.hxx")
list(REMOVE_ITEM _all_cpp ${_legacy_cpp_files})
# All other cpplint rules still run on these exact inherited C++ files.
if(_all_cpp)
  ament_cpplint(EXCLUDE "${_legacy_cpp_files}")
  ament_cpplint(${_legacy_cpp_files}
    TESTNAME cpplint_legacy_notices FILTERS -legal/copyright)
else()
  ament_cpplint(${_legacy_cpp_files} FILTERS -legal/copyright)
endif()
list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright ament_cmake_cpplint)
