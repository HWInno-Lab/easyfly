cmake_minimum_required(VERSION 3.8)
project(easyfly_utils)

find_package(ament_cmake REQUIRED)

# Header-only library — 安装头文件即可
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  add_executable(test_geometry_contract test/test_geometry_contract.cpp)
  target_include_directories(test_geometry_contract PRIVATE include)
  add_test(NAME test_geometry_contract COMMAND test_geometry_contract)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
