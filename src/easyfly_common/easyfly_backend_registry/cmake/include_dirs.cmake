# Plain CMake compatibility variable; consumers can use the exported INTERFACE target.
get_filename_component(easyfly_backend_registry_INCLUDE_DIRS
  "${CMAKE_CURRENT_LIST_DIR}/../../../include" ABSOLUTE)
