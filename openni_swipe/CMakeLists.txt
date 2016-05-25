cmake_minimum_required(VERSION 2.8.3)
project(openni_swipe)
find_package(catkin REQUIRED COMPONENTS roscpp roslib std_msgs)

find_package(PkgConfig)
pkg_check_modules(PC_LIBOPENNI REQUIRED libopenni)
set(PC_NITE_DEV_INCLUDE_DIRS "/usr/include/nite")
set(PC_NITE_DEV_LIBRARIES XnVNite_1_3_1;OpenNI)

catkin_package(
    DEPENDS openni nite
    CATKIN-DEPENDS roscpp roslib std_msgs
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
  add_executable(openni_swipe src/openni_swipe.cpp)
  target_link_libraries(openni_swipe ${catkin_LIBRARIES} ${PC_NITE_DEV_LIBRARIES})

  install(TARGETS ${PROJECT_NAME} openni_swipe
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

endif()

install(FILES openni_swipe.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
