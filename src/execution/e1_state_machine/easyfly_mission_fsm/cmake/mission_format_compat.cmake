# Compatibility for existing consumers. This is a forwarding target, not another parser library.
find_package(easyfly_mission_format REQUIRED)
if(NOT TARGET easyfly_mission_fsm::easyfly_mission_format)
  add_library(easyfly_mission_fsm::easyfly_mission_format INTERFACE IMPORTED)
  set_property(TARGET easyfly_mission_fsm::easyfly_mission_format PROPERTY
    INTERFACE_LINK_LIBRARIES easyfly_mission_format::easyfly_mission_format)
  get_filename_component(_easyfly_mission_fsm_include
    "${CMAKE_CURRENT_LIST_DIR}/../../../include" ABSOLUTE)
  set_property(TARGET easyfly_mission_fsm::easyfly_mission_format PROPERTY
    INTERFACE_INCLUDE_DIRECTORIES "${_easyfly_mission_fsm_include}")
  unset(_easyfly_mission_fsm_include)
endif()
