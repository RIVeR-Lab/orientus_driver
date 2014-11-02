cmake_minimum_required(VERSION 2.8.3)
project(orientus_sdk_c)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES orientus_sdk_c orientus_sdk_c_rs232
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include/orientus_sdk_c include/orientus_sdk_c/rs232)

## Declare a library
add_library(orientus_sdk_c
  src/an_packet_protocol.c src/orientus_packets.c
)
add_library(orientus_sdk_c_rs232
  src/rs232/rs232.c
)

## Declare an executable
add_executable(packet_example src/packet_example.c)

## Specify libraries to link a library or executable target against
target_link_libraries(packet_example
  ${catkin_LIBRARIES}
  orientus_sdk_c
  orientus_sdk_c_rs232
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS orientus_sdk_c orientus_sdk_c_rs232 packet_example
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_orientus_sdk_c.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
