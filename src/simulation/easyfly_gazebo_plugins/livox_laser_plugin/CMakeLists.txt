# Livox LiDAR Gazebo 仿真插件 (ROS2 移植版)
# 编译产物: liblivox_laser_simulation.so
# 作为 easyfly_gazebo_plugins 的子目录编译, 不单独成包

set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_CXX_STANDARD_REQUIRED)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rosidl_cmake REQUIRED)  # rosidl_get_typesupport_target

add_library(livox_laser_simulation SHARED
  src/livox_points_plugin.cpp
  src/livox_ode_multiray_shape.cpp
)

target_compile_definitions(livox_laser_simulation PRIVATE
  SCAN_MODE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/scan_mode"
)

target_include_directories(livox_laser_simulation PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${GAZEBO_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
)

# livox_ros_driver2 为 ament_auto 混合包(库+消息), 其导出已包含 rosidl typesupport
ament_target_dependencies(livox_laser_simulation
  rclcpp
  sensor_msgs
  gazebo_dev
  livox_ros_driver2
)

# Ubuntu/Debian 将 gazebo 插件基类(含 RayPlugin)拆分至 gazebo-plugin-base 包:
#   /usr/lib/<arch>/gazebo-11/plugins/libRayPlugin.so
find_library(GAZEBO_RAY_PLUGIN_LIB RayPlugin
  PATHS /usr/lib/x86_64-linux-gnu/gazebo-11/plugins /usr/lib/gazebo-11/plugins
  NO_DEFAULT_PATH)
if(NOT GAZEBO_RAY_PLUGIN_LIB)
  message(FATAL_ERROR "libRayPlugin.so not found (需要安装 gazebo-plugin-base 包)")
endif()

target_link_libraries(livox_laser_simulation
  gazebo
  ${GAZEBO_RAY_PLUGIN_LIB}
  ${GAZEBO_LIBRARIES}
)

install(TARGETS livox_laser_simulation
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装扫描模式 CSV (供部署场景使用)
install(DIRECTORY scan_mode/
  DESTINATION share/${PROJECT_NAME}/livox_laser_plugin/scan_mode
)

install(FILES README.md DESTINATION share/${PROJECT_NAME}/livox_laser_plugin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_livox_ray_geometry test/test_ray_geometry.cpp TIMEOUT 15)
  target_include_directories(test_livox_ray_geometry PRIVATE include)
  ament_target_dependencies(test_livox_ray_geometry gazebo_dev)
  target_link_libraries(test_livox_ray_geometry ${GAZEBO_LIBRARIES})
endif()
