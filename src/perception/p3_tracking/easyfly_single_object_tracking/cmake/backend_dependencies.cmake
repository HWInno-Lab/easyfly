# Plain CMake consumers of the synchronous factory do not initialize ROS.
include(CMakeFindDependencyMacro)
find_dependency(easyfly_backend_registry)
find_dependency(easyfly_single_tracking_api)
get_filename_component(easyfly_single_object_tracking_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../../include" ABSOLUTE)

list(APPEND easyfly_single_object_tracking_INCLUDE_DIRS "${easyfly_single_object_tracking_INCLUDE_DIRS}/easyfly_single_object_tracking")
