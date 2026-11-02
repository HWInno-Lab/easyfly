cmake_minimum_required(VERSION 3.8)

find_package(ament_cmake REQUIRED)
project(easyfly_gazebo_worlds)

install(FILES empty.world perception_demo.world precision_landing.world
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scenes schemas licenses worlds evidence generators previews
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".git" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
  PATTERN "*.log" EXCLUDE)
install(FILES README.md SCENE-LIFECYCLE.md DESTINATION share/${PROJECT_NAME})

# Directory installs do not prune removed source files on an in-place upgrade.
install(CODE [[
  foreach(retired IN ITEMS
      scenes/primitives_corridor_seed1042_v1.yaml
      scenes/racetrack_aws_day_v1.yaml
      scenes/hospital_aws_v1.yaml
      worlds/benchmark/navigation/primitives_corridor_seed1042_v1.world
      worlds/outdoor/urban/racetrack_aws_day_v1.world
      worlds/indoor/hospital/hospital_aws_v1.world
      generators/primitives_corridor_seed1042_v1.json)
    file(REMOVE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/easyfly_gazebo_worlds/${retired}")
  endforeach()
]])

ament_package()
