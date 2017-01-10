cmake_minimum_required(VERSION 2.8.3)
project(yumi_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

catkin_package()
#############
## Install ##
#############

## Mark cpp header files for installation
install(DIRECTORY config/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

