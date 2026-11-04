# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_multi_object_tracking/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Copyright 2026 EasyFly Project
#

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
