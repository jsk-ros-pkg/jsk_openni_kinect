cmake_minimum_required(VERSION 2.8.3)
project(openni2_tracker)
find_package(catkin REQUIRED COMPONENTS geometry_msgs
					#orocos_kdl
					roscpp
					roslib
					tf)
find_package(orocos_kdl)
# Find OpenNI2
#find_package(PkgConfig)
#pkg_check_modules(OpenNI2 REQUIRED libopenni2)
find_path(OpenNI2_INCLUDEDIR
	  NAMES OpenNI.h
	  HINTS /usr/include/openni2)
find_library(OpenNI2_LIBRARIES
	     NAMES OpenNI2 DummyDevice OniFile PS1090
	     HINTS /usr/lib/ /usr/lib/OpenNI2/Drivers
	     PATH_SUFFIXES lib) 
message(STATUS ${OpenNI2_LIBRARIES})

#extract NiTE
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/NiTE-2.0.0/NiTEDevEnvironment
  DEPENDS NiTE-Linux-x64-2.0.0.tar.bz2
  COMMAND tar -xvjf ${PROJECT_SOURCE_DIR}/NiTE-Linux-x64-2.0.0.tar.bz2
  COMMAND cd NiTE-2.0.0 && ./install.sh
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
add_custom_target(install_nite2 ALL
  DEPENDS  ${PROJECT_SOURCE_DIR}/NiTE-2.0.0/NiTEDevEnvironment)



# Find Nite2
message(status $ENV{NITE2_INCLUDE})
message(status $ENV{NITE2_REDIST64})
find_path(Nite2_INCLUDEDIR
	  NAMES NiTE.h
	  HINTS $ENV{NITE2_INCLUDE})
find_library(Nite2_LIBRARY
	     NAMES NiTE2
	     HINTS $ENV{NITE2_REDIST64}
	     PATH_SUFFIXES lib) 

catkin_package()

include_directories(${catkin_INCLUDEDIR}
		    ${OpenNI2_INCLUDEDIR}
		    ${Nite2_INCLUDEDIR})
add_executable(openni2_tracker src/openni2_tracker.cpp)
target_link_libraries(openni2_tracker ${catkin_LIBRARIES} ${OpenNI2_LIBRARIES} ${Nite2_LIBRARY})

#install NiTE?
install(DIRECTORY Bin
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
   USE_SOURCE_PERMISSIONS
 )

install(TARGETS openni2_tracker RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES openni2_tracker.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
