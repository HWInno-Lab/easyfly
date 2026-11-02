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

# Public targets expose plain algorithm libraries only. Use standard CMake
# dependencies so installed adapter consumption does not initialize a ROS build.
include(CMakeFindDependencyMacro)
find_dependency(easyfly_backend_registry)
find_dependency(easyfly_tracking_api)

# Generic factory headers expose the API only. Legacy concrete headers require
# explicit find_package/target_link_libraries for the corresponding implementation.
get_filename_component(easyfly_multi_object_tracking_INCLUDE_DIRS
  "${CMAKE_CURRENT_LIST_DIR}/../../../include" ABSOLUTE)
foreach(_api easyfly_tracking_api)
  get_target_property(_includes ${_api}::${_api} INTERFACE_INCLUDE_DIRECTORIES)
  list(APPEND easyfly_multi_object_tracking_INCLUDE_DIRS ${_includes})
endforeach()
