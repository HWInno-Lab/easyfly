cmake_minimum_required(VERSION 3.16)
project(easyfly_object_detection)
include(CTest)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(easyfly_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(easyfly_detection_api REQUIRED)
find_package(easyfly_yolo_onnx REQUIRED)
# Compatibility build target; computation/runtime installation belongs to the algorithm package.
add_library(${PROJECT_NAME}_core INTERFACE)
target_link_libraries(${PROJECT_NAME}_core INTERFACE easyfly_yolo_onnx::easyfly_yolo_onnx)
add_library(${PROJECT_NAME}_nodes SHARED src/detection_node.cpp)
target_compile_options(${PROJECT_NAME}_nodes PRIVATE -Wall -Wextra)
target_include_directories(${PROJECT_NAME}_nodes PRIVATE include)
ament_target_dependencies(${PROJECT_NAME}_nodes rclcpp rclcpp_components ament_index_cpp sensor_msgs easyfly_msgs)
target_link_libraries(${PROJECT_NAME}_nodes ${PROJECT_NAME}_core yaml-cpp)
set_target_properties(${PROJECT_NAME}_nodes PROPERTIES INSTALL_RPATH "$ORIGIN;$ORIGIN/../../easyfly_yolo_onnx/lib")
rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "easyfly_object_detection::DetectionNode" EXECUTABLE detection_node)
install(TARGETS ${PROJECT_NAME}_nodes LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../../models/yolo11n.onnx
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../../models/LICENSE-AGPL-3.0
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../../models/README.md
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../third_party/model-assets.provenance.json
  DESTINATION share/${PROJECT_NAME}/models)
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_inference_ros test/test_inference_ros.py TIMEOUT 120
    ENV ROS_DOMAIN_ID=216 ROS_LOG_DIR=/tmp/easyfly_alg5_detector)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(
    NAME easyfly_object_detection_detection_contract
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/test/test_detection_contract.py)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(rclcpp easyfly_msgs OpenCV easyfly_detection_api easyfly_yolo_onnx)
ament_package()
