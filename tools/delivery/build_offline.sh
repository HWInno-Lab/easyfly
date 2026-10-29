#!/bin/bash
# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
set -eu
export HOME=/root LANG=C.UTF-8
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export CMAKE_BUILD_PARALLEL_LEVEL=4 MAKEFLAGS=-j4 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
export CMAKE_PREFIX_PATH=/opt/easyfly/dependencies/pangolin
export LD_LIBRARY_PATH=/opt/easyfly/dependencies/pangolin/lib
export QT_QPA_PLATFORM=offscreen ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=226
set +u
. /opt/ros/humble/setup.sh
set -u
cd /opt/easyfly/source
colcon list --base-paths src > /opt/easyfly/inputs/packages.tsv
cmake -S /opt/easyfly/source/src/third_party/algorithms/localization/orb_slam3_dependencies/pangolin -B /opt/easyfly/build-pangolin -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/easyfly/dependencies/pangolin -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DBUILD_TESTS=OFF -DBUILD_PANGOLIN_VIDEO=OFF -DBUILD_PANGOLIN_PYTHON=OFF
cmake --build /opt/easyfly/build-pangolin -j4
cmake --install /opt/easyfly/build-pangolin
colcon --log-base /opt/easyfly/log-sdk build --base-paths src --build-base /opt/easyfly/build --install-base /opt/easyfly/install --executor sequential --cmake-clean-first --packages-select livox_sdk2 librealsense2 --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_PROJECT_realsense2_INCLUDE=/opt/easyfly/source/tools/algorithms/sdk_offline.cmake -DALG83_JSON_SOURCE=/opt/easyfly/inputs/nlohmann-json -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DBUILD_GLSL_EXTENSIONS=OFF -DBUILD_UNIT_TESTS=OFF -DIMPORT_DEPTH_CAM_FW=OFF -DCHECK_FOR_UPDATES=OFF -DBUILD_WITH_DDS=OFF -DENABLE_CCACHE=OFF
export LD_LIBRARY_PATH=/opt/easyfly/install/livox_sdk2/lib:/opt/easyfly/install/librealsense2/lib:$LD_LIBRARY_PATH
colcon --log-base /opt/easyfly/log-workspace build --base-paths src --build-base /opt/easyfly/build --install-base /opt/easyfly/install --executor sequential --cmake-clean-first --continue-on-error --packages-skip livox_sdk2 librealsense2 --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DBUILD_SOPHUS_TESTS=OFF -DBUILD_SOPHUS_EXAMPLES=OFF -DEASYFLY_ENABLE_MEDIA=ON -DEASYFLY_ENABLE_RVIZ_EMBED=ON -DEASYFLY_PX4_GAZEBO_BUILD=/opt/easyfly/inputs/px4-original/build/px4_sitl_default/build_gazebo-classic -DEASYFLY_FASTDDS_ARCHIVE=/opt/easyfly/inputs/fastdds-2.6.11.tar -DORT_ROOT=/opt/easyfly/inputs/onnxruntime -DFASTLIO_OUTPUT_ROOT=/var/lib/easyfly/fastlio -DPangolin_DIR=/opt/easyfly/dependencies/pangolin/lib/cmake/Pangolin -DLIVOX_LIDAR_SDK_LIBRARY:FILEPATH=/opt/easyfly/install/livox_sdk2/lib/liblivox_lidar_sdk_shared.so -DLIVOX_LIDAR_SDK_INCLUDE_DIR:PATH=/opt/easyfly/install/livox_sdk2/include -Drealsense2_DIR:PATH=/opt/easyfly/install/librealsense2/lib/cmake/realsense2
