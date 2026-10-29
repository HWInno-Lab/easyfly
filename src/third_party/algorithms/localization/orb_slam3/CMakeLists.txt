cmake_minimum_required(VERSION 3.16)
project(easyfly_orbslam3_core VERSION 1.0.0 LANGUAGES C CXX)
find_package(ament_cmake REQUIRED)
find_package(OpenCV 4.4 REQUIRED COMPONENTS core imgproc features2d calib3d highgui imgcodecs)
find_package(Eigen3 3.1 REQUIRED NO_MODULE)
find_package(Pangolin 0.6 REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_INSTALL_RPATH "$ORIGIN")
configure_file(Thirdparty/g2o/config.h.in easyfly_orbslam3_g2o_config.h)
set(ORB_G2O_SOURCES
  Thirdparty/g2o/g2o/types/types_sba.cpp
  Thirdparty/g2o/g2o/types/types_six_dof_expmap.cpp
  Thirdparty/g2o/g2o/types/types_seven_dof_expmap.cpp
  Thirdparty/g2o/g2o/core/hyper_graph_action.cpp
  Thirdparty/g2o/g2o/core/hyper_graph.cpp
  Thirdparty/g2o/g2o/core/marginal_covariance_cholesky.cpp
  Thirdparty/g2o/g2o/core/matrix_structure.cpp
  Thirdparty/g2o/g2o/core/batch_stats.cpp
  Thirdparty/g2o/g2o/core/parameter.cpp
  Thirdparty/g2o/g2o/core/cache.cpp
  Thirdparty/g2o/g2o/core/optimizable_graph.cpp
  Thirdparty/g2o/g2o/core/solver.cpp
  Thirdparty/g2o/g2o/core/optimization_algorithm_factory.cpp
  Thirdparty/g2o/g2o/core/estimate_propagator.cpp
  Thirdparty/g2o/g2o/core/factory.cpp
  Thirdparty/g2o/g2o/core/sparse_optimizer.cpp
  Thirdparty/g2o/g2o/core/hyper_dijkstra.cpp
  Thirdparty/g2o/g2o/core/parameter_container.cpp
  Thirdparty/g2o/g2o/core/optimization_algorithm.cpp
  Thirdparty/g2o/g2o/core/optimization_algorithm_with_hessian.cpp
  Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.cpp
  Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.cpp
  Thirdparty/g2o/g2o/core/jacobian_workspace.cpp
  Thirdparty/g2o/g2o/core/robust_kernel.cpp
  Thirdparty/g2o/g2o/core/robust_kernel_factory.cpp
  Thirdparty/g2o/g2o/core/robust_kernel_impl.cpp
  Thirdparty/g2o/g2o/stuff/timeutil.cpp
  Thirdparty/g2o/g2o/stuff/os_specific.c
  Thirdparty/g2o/g2o/stuff/string_tools.cpp
  Thirdparty/g2o/g2o/stuff/property.cpp
)
add_library(easyfly_orbslam3_g2o SHARED ${ORB_G2O_SOURCES})
target_include_directories(easyfly_orbslam3_g2o PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/g2o>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<INSTALL_INTERFACE:include/easyfly_orbslam3/Thirdparty/g2o>
  $<INSTALL_INTERFACE:include/easyfly_orbslam3>)
target_link_libraries(easyfly_orbslam3_g2o PUBLIC Eigen3::Eigen)
file(GLOB DBOW_SOURCES CONFIGURE_DEPENDS Thirdparty/DBoW2/DBoW2/*.cpp Thirdparty/DBoW2/DUtils/*.cpp)
add_library(easyfly_orbslam3_dbow2 SHARED ${DBOW_SOURCES})
target_link_libraries(easyfly_orbslam3_dbow2 PUBLIC ${OpenCV_LIBS})
file(GLOB ORB_SOURCES CONFIGURE_DEPENDS src/*.cc src/*.cpp src/CameraModels/*.cpp)
add_library(easyfly_orbslam3 SHARED ${ORB_SOURCES})
set(EASYFLY_ORB_GRAVITY 9.8 CACHE STRING "Gravity magnitude of the calibrated EasyFly simulation")
if(NOT EASYFLY_ORB_GRAVITY MATCHES "^[0-9]+([.][0-9]+)?$" OR
   EASYFLY_ORB_GRAVITY LESS_EQUAL 0 OR EASYFLY_ORB_GRAVITY GREATER 20)
  message(FATAL_ERROR "Invalid calibrated ORB gravity")
endif()
target_compile_definitions(easyfly_orbslam3 PUBLIC EASYFLY_ORB_GRAVITY=${EASYFLY_ORB_GRAVITY})
target_include_directories(easyfly_orbslam3 BEFORE PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/CameraModels>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/Sophus>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<INSTALL_INTERFACE:include/easyfly_orbslam3>
  $<INSTALL_INTERFACE:include/easyfly_orbslam3/include>
  $<INSTALL_INTERFACE:include/easyfly_orbslam3/include/CameraModels>
  $<INSTALL_INTERFACE:include/easyfly_orbslam3/Thirdparty/Sophus>)
target_link_libraries(easyfly_orbslam3 PUBLIC easyfly_orbslam3_g2o easyfly_orbslam3_dbow2
  Eigen3::Eigen ${OpenCV_LIBS} ${Pangolin_LIBRARIES} Boost::serialization OpenSSL::Crypto Threads::Threads)
target_compile_options(easyfly_orbslam3 PRIVATE -Wno-deprecated-declarations)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/vocabulary)
if(NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/vocabulary/ORBvoc.txt)
  execute_process(COMMAND ${CMAKE_COMMAND} -E tar xzf ${CMAKE_CURRENT_SOURCE_DIR}/Vocabulary/ORBvoc.txt.tar.gz
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/vocabulary COMMAND_ERROR_IS_FATAL ANY)
endif()
install(TARGETS easyfly_orbslam3 easyfly_orbslam3_g2o easyfly_orbslam3_dbow2 EXPORT orbslam3Targets LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include/easyfly_orbslam3/include FILES_MATCHING PATTERN "*.h")
install(DIRECTORY Thirdparty/ DESTINATION include/easyfly_orbslam3/Thirdparty FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/easyfly_orbslam3_g2o_config.h DESTINATION include/easyfly_orbslam3)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/vocabulary/ORBvoc.txt DESTINATION share/${PROJECT_NAME}/vocabulary)
install(FILES LICENSE README.md Dependencies.md DESTINATION share/${PROJECT_NAME}/upstream)
install(FILES Thirdparty/DBoW2/LICENSE.txt DESTINATION share/${PROJECT_NAME}/licenses RENAME DBoW2-LICENSE.txt)
install(FILES Thirdparty/DBoW2/LICENSE-SOURCE.md DESTINATION share/${PROJECT_NAME}/licenses RENAME DBoW2-LICENSE-SOURCE.md)
install(FILES Thirdparty/DBoW2/README.txt DESTINATION share/${PROJECT_NAME}/licenses RENAME DBoW2-UPSTREAM-NOTICE.txt)
install(FILES Thirdparty/g2o/license-bsd.txt DESTINATION share/${PROJECT_NAME}/licenses RENAME g2o-license-bsd.txt)
install(FILES Thirdparty/Sophus/LICENSE.txt DESTINATION share/${PROJECT_NAME}/licenses RENAME Sophus-LICENSE.txt)
ament_export_targets(orbslam3Targets HAS_LIBRARY_TARGET)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gazebo_gravity test/test_gazebo_gravity.cpp)
  target_link_libraries(test_gazebo_gravity easyfly_orbslam3)
endif()
ament_package(CONFIG_EXTRAS cmake/dependencies.cmake)
