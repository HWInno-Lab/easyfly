cmake_minimum_required(VERSION 3.16)
project(classical_detection_consumer LANGUAGES CXX)
find_package(easyfly_ellipse REQUIRED)
find_package(easyfly_color_line REQUIRED)
add_executable(consumer main.cpp)
target_link_libraries(consumer PRIVATE
  easyfly_ellipse::easyfly_ellipse easyfly_color_line::easyfly_color_line)
enable_testing()
add_test(NAME installed_detectors COMMAND consumer)
