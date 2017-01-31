cmake_minimum_required(VERSION 2.8.3)
project(yumi_contact_sensor_plugin)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

cmake_minimum_required(VERSION 2.8.12 FATAL_ERROR)
project(yumi_contact_sensor_plugin)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++11 -Wl,-z,defs")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  gazebo_msgs
  message_generation
  contact_force_msgs
)


find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
)


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp
    gazebo_msgs
    tf
    contact_force_msgs
  DEPENDS 
    gazebo
    SDF)


add_library(yumi_contact_sensor_plugin src/yumi_contact_sensor_plugin.cpp)
add_dependencies(yumi_contact_sensor_plugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(yumi_contact_sensor_plugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ContactPlugin ${catkin_LIBRARIES})

install(TARGETS 
  yumi_contact_sensor_plugin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)