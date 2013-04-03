find_package(catkin REQUIRED)
STRING(REGEX REPLACE "^([0-9]+)\\.[0-9]+\\.[0-9]+" "\\1" catkin_MAJOR_VERSION ${catkin_VERSION})
STRING(REGEX REPLACE "^[0-9]+\\.([0-9])+\\.[0-9]+" "\\1" catkin_MINOR_VERSION ${catkin_VERSION})
if((NOT ${catkin_MAJOR_VERSION} LESS 0) AND (NOT ${catkin_MINOR_VERSION} LESS 5))
  include(catkin.cmake)
  return()
endif()
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rosbuild_add_executable(openni_swipe src/openni_swipe.cpp)

# https://code.ros.org/trac/ros-pkg/ticket/5287
target_link_libraries(openni_swipe XnVNite_1_3_1)

