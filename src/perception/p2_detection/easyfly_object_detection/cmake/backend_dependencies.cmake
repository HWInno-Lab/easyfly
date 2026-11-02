# Plain CMake consumers of the synchronous factory do not initialize ROS.
include(CMakeFindDependencyMacro)
find_dependency(easyfly_backend_registry)
find_dependency(easyfly_detection_api)
get_filename_component(easyfly_object_detection_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../../include" ABSOLUTE)
