if(NOT USE_ROSBUILD)
  include(catkin.cmake)
  return()
endif()
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

set(PC_NITE_DEV_INCLUDE_DIRS "/usr/include/nite")
find_package(PkgConfig)
pkg_check_modules(PC_LIBOPENNI REQUIRED libopenni)
include_directories(
  ${PC_LIBOPENNI_INCLUDE_DIRS}
  ${PC_NITE_DEV_INCLUDE_DIRS})
if($ENV{ROS_DISTRO} STREQUAL "groovy")
  add_definitions(-DUSE_PCL_AS_PCL_MSGS)
endif()

set(PC_NITE_DEV_LIBRARIES XnVNite_1_3_1;OpenNI)
rosbuild_find_ros_package(openni_camera)
rosbuild_add_executable(openni_scene src/openni_scene.cpp)
target_link_libraries(openni_scene ${PC_NITE_DEV_LIBRARIES})
