cmake_minimum_required(VERSION 2.8.12)
project(contact_force_msgs)

set(CMAKE_CXX_FLAGS "-O3 -std=c++11")

find_package(catkin REQUIRED COMPONENTS 
  std_msgs
  geometry_msgs
  message_generation)

add_message_files(FILES 
  YumiContactState.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs)

catkin_package(CATKIN_DEPENDS 
  std_msgs
  geometry_msgs
  message_runtime)