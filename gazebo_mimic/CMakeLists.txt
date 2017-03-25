cmake_minimum_required(VERSION 2.8.3)
project(gazebo_mimic)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
find_package(gazebo REQUIRED)

set(CMAKE_CXX_FLAGS "-O3 -std=c++11")

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES gazebo_mimic_plugin
   CATKIN_DEPENDS gazebo_ros
   DEPENDS gazebo_ros
)

###########
## Build ##
###########
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(gazebo_mimic_plugin src/gazebo_mimic_plugin/mimic_plugin.cpp)

target_link_libraries(gazebo_mimic_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS gazebo_mimic_plugin
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

