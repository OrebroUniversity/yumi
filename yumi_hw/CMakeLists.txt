cmake_minimum_required(VERSION 2.8.3)
project(yumi_hw)

set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE RelWithDebInfo)
#set(CMAKE_BUILD_TYPE Debug)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  control_toolbox
  controller_interface
  controller_manager
  hardware_interface
  joint_limits_interface
  kdl_parser
  realtime_tools
  roscpp
  std_msgs
  tf
  transmission_interface
  urdf
  simple_message
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS system thread)

# The definition is copied from the CMakeList for the simple_message package.
add_definitions(-DROS=1)           #build using ROS libraries
add_definitions(-DLINUXSOCKETS=1)  #build using LINUX SOCKETS libraries


## Generate services in the 'srv' folder
add_service_files(
  FILES
  YumiGrasp.srv
)
# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

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
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS cmake_modules control_toolbox controller_interface controller_manager hardware_interface joint_limits_interface kdl_parser realtime_tools roscpp std_msgs tf transmission_interface urdf simple_message
#  DEPENDS gazebo
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/yumi_hw.cpp
  src/yumi_hw_rapid.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(yumi_hw ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

## Declare a C++ executable
add_executable(yumi_hw_ifce_node src/yumi_hw_ifce_node.cpp)

add_executable(yumi_gripper_node src/yumi_gripper_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(yumi_hw_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(yumi_gripper_node ${PROJECT_NAME}_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES} )
target_link_libraries( yumi_hw_ifce_node ${catkin_LIBRARIES} ${PROJECT_NAME} simple_message)
target_link_libraries( yumi_gripper_node ${catkin_LIBRARIES} ${PROJECT_NAME} simple_message)


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} yumi_hw_ifce_node yumi_gripper_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY rapid/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

