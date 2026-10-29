cmake_minimum_required(VERSION 3.5)
project(path_searching)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(plan_env REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  visualization_msgs
  cv_bridge
  Eigen3
  nav_msgs
  message_filters
  pcl_msgs
  pcl_conversions
  PCL
  plan_env
)

include_directories(
    include
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${message_filters_INCLUDE_DIRS}
    ${pcl_msgs_INCLUDE_DIRS}
)

add_library(path_searching SHARED
    src/dyn_a_star.cpp
)

ament_target_dependencies(path_searching
    ${dependencies}
)

target_include_directories(path_searching PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS path_searching
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION lib/cmake/${PROJECT_NAME}
)

if(BUILD_TESTING)
    set(ament_cmake_uncrustify_ADDITIONAL_ARGS LANGUAGE C++)
  find_package(ament_lint_auto REQUIRED)
    # Preserve native analysis; teach cppcheck the C++ and test-framework syntax.
  find_package(ament_cmake_cppcheck REQUIRED)
  ament_cppcheck(LANGUAGE c++ LIBRARIES googletest qt
    INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cppcheck)
  include(cmake/legacy_notice_scope.cmake)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})
ament_export_include_directories(include)

install(FILES ../NOTICE.md ../legacy-notice-scope.json
  DESTINATION share/${PROJECT_NAME}/source_notices)
install(DIRECTORY ../licenses DESTINATION share/${PROJECT_NAME}/source_notices)

ament_package()
