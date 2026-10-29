set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
cmake_minimum_required(VERSION 3.8)
project(plan_env)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_VISIBILITY_PRESET default)
set(CMAKE_VISIBILITY_INLINES_HIDDEN OFF)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  pcl_conversions
  pcl_ros
  PCL
  Eigen3
  message_filters
  pcl_msgs
  cv_bridge
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${message_filters_INCLUDE_DIRS}
  ${pcl_msgs_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/obj_generator.cpp
  src/obj_predictor.cpp
  src/grid_map.cpp
  src/raycast.cpp
  # 添加其他源文件
)

# Mapping is a real-time input consumer even in a diagnostic Debug build.
# Keep symbols/assertions and the selected Debug ABI, but optimize this library
# so voxel aging/inflation does not starve odom beyond its 1s validity budget.
# The separately instrumented depth regression retains its explicit -O1.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(${PROJECT_NAME} PRIVATE
    $<$<CONFIG:Debug>:-O2> $<$<CONFIG:>:-O2>)
endif()

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_inflation_safety test/test_inflation_safety.cpp)
  target_include_directories(test_inflation_safety PRIVATE include)
  ament_add_gtest(test_inflation_union test/test_inflation_union.cpp)
  target_include_directories(test_inflation_union PRIVATE include)
  target_compile_options(test_inflation_union PRIVATE
    -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_libraries(test_inflation_union -fsanitize=address,undefined)
  set_tests_properties(test_inflation_union PROPERTIES
    ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0:halt_on_error=1;UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1" TIMEOUT 30)
  ament_add_gtest(test_trusted_cloud_persistence test/test_trusted_cloud_persistence.cpp)
  target_include_directories(test_trusted_cloud_persistence PRIVATE include)
  ament_add_gtest(test_odom_freshness test/test_odom_freshness.cpp)
  target_include_directories(test_odom_freshness PRIVATE include)
  ament_add_gtest(test_virtual_ceiling test/test_virtual_ceiling.cpp)
  target_include_directories(test_virtual_ceiling PRIVATE include)
  # Instrument the actual depth path, not only a duplicated arithmetic model.
  ament_add_gtest(test_depth_projection_memory
    test/test_depth_projection_memory.cpp src/grid_map.cpp src/raycast.cpp)
  ament_target_dependencies(test_depth_projection_memory ${dependencies})
  target_link_libraries(test_depth_projection_memory ${PCL_LIBRARIES})
  target_compile_options(test_depth_projection_memory PRIVATE
    -fsanitize=address -fno-omit-frame-pointer -O1)
  target_link_libraries(test_depth_projection_memory -fsanitize=address)
  set_tests_properties(test_depth_projection_memory PROPERTIES
    ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0:halt_on_error=1" TIMEOUT 30)
endif()

install(FILES ../NOTICE.md ../legacy-notice-scope.json
  DESTINATION share/${PROJECT_NAME}/source_notices)
install(DIRECTORY ../licenses DESTINATION share/${PROJECT_NAME}/source_notices)

ament_package()
