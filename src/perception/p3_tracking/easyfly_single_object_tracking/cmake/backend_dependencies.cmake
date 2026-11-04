# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_single_object_tracking/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
# Plain CMake consumers of the synchronous factory do not initialize ROS.
include(CMakeFindDependencyMacro)
find_dependency(easyfly_backend_registry)
find_dependency(easyfly_single_tracking_api)
get_filename_component(easyfly_single_object_tracking_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../../include" ABSOLUTE)

list(APPEND easyfly_single_object_tracking_INCLUDE_DIRS "${easyfly_single_object_tracking_INCLUDE_DIRS}/easyfly_single_object_tracking")
