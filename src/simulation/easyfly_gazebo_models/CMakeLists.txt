cmake_minimum_required(VERSION 3.8)

find_package(ament_cmake REQUIRED)
project(easyfly_gazebo_models)

# 纯数据包——只把模型文件安装到 install 目录
install(DIRECTORY cardboard_box fire_truck meshes mid360 person_standing person_walking
  manifests licenses easyfly third_party
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".git" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
  PATTERN ".cache" EXCLUDE
  PATTERN "cache" EXCLUDE
  PATTERN "downloads" EXCLUDE
  PATTERN "logs" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
  PATTERN "*.log" EXCLUDE
  PATTERN "*.ulg" EXCLUDE)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
