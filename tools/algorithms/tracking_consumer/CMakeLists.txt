cmake_minimum_required(VERSION 3.16)
project(installed_tracking_consumer LANGUAGES CXX)
find_package(easyfly_sort REQUIRED)
find_package(easyfly_track_id_fusion REQUIRED)
find_package(easyfly_multi_object_tracking REQUIRED)
add_executable(tracking_consumer main.cpp)
target_link_libraries(tracking_consumer PRIVATE
  easyfly_sort::easyfly_sort
  easyfly_track_id_fusion::easyfly_track_id_fusion
  easyfly_multi_object_tracking::easyfly_multi_object_tracking_backends)
target_compile_options(tracking_consumer PRIVATE -Wall -Wextra -Wpedantic -Werror)
enable_testing()
add_test(NAME installed_real_sort_and_deferred_identity COMMAND tracking_consumer)
