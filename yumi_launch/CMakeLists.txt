cmake_minimum_required(VERSION 2.8.3)
project(yumi_launch)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  yumi_description
  yumi_hw
  yumi_moveit_config
  yumi_support
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES yumi_launch
  CATKIN_DEPENDS yumi_description yumi_hw yumi_moveit_config yumi_support
#  DEPENDS system_lib
)

