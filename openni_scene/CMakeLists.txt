cmake_minimum_required(VERSION 2.8.3)
project(openni_scene)
find_package(catkin REQUIRED COMPONENTS roscpp roslib sensor_msgs pcl_ros pcl_msgs)

find_package(PkgConfig)
pkg_check_modules(PC_LIBOPENNI REQUIRED libopenni)
set(PC_NITE_DEV_INCLUDE_DIRS "/usr/include/nite")
set(PC_NITE_DEV_LIBRARIES XnVNite_1_3_1;OpenNI)

catkin_package(
    DEPENDS openni nite
    CATKIN-DEPENDS roscpp roslib sensor_msgs pcl_ros pcl_msgs
    #INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

if(NOT EXISTS ${PC_NITE_DEV_INCLUDE_DIRS})
  message(WARNING "-- Nite is not found, so could not compile ${PROJECT_NAME}")
else()

  include_directories(
    ${PC_LIBOPENNI_INCLUDE_DIRS}
    ${PC_NITE_DEV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS})
  add_definitions(${PC_LIBOPENNI_CFLAGS_OTHER})
  if($ENV{ROS_DISTRO} STREQUAL "groovy")
    add_definitions(-DUSE_PCL_AS_PCL_MSGS)
  endif()
  add_executable(openni_scene src/openni_scene.cpp)
  target_link_libraries(openni_scene ${catkin_LIBRARIES} ${PC_NITE_DEV_LIBRARIES})
  add_dependencies(openni_scene pcl_msgs_gencpp)


  install(TARGETS ${PROJECT_NAME} openni_scene
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

endif()

install(FILES openni_scene.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
