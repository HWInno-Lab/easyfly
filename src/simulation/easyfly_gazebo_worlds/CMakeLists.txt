cmake_minimum_required(VERSION 3.8)

find_package(ament_cmake REQUIRED)
project(easyfly_gazebo_worlds)

install(FILES empty.world perception_demo.world precision_landing.world
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scenes schemas licenses worlds evidence
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".git" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
  PATTERN "*.log" EXCLUDE)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
