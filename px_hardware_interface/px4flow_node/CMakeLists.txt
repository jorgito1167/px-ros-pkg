cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()
rosbuild_genmsg()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(
  ../../mavlink/include/mavlink/v1.0
  /usr/include/eigen3
)

set(PX4FLOW_NODE_SRCS
  src/SerialComm.cc
  src/px4flow_node.cc
)

set(FILTER_NODE_SRCS
  src/Kalman_filter.cpp
)
rosbuild_add_boost_directories()
rosbuild_add_executable(filter_node ${FILTER_NODE_SRCS})
rosbuild_add_executable(px4flow_node ${PX4FLOW_NODE_SRCS})
rosbuild_link_boost(px4flow_node thread system signals)

