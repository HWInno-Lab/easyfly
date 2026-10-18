cmake_minimum_required(VERSION 3.8)
project(easyfly_local_planner)

find_package(ament_cmake REQUIRED)

# 本包为 EGO-Planner 适配/编排层: 无自有 C++ 节点, 只提供 launch (上游算法在
# src/third_party/algorithms/planning/ego_planner/, 见 third_party 目录规范)。
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
