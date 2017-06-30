cmake_minimum_required(VERSION 2.8.3)
project(yumi_test_controllers)

add_compile_options(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  angles
  control_msgs
  control_toolbox
  controller_interface
  forward_command_controller
  realtime_tools
  sensor_msgs
)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS
    angles
    control_msgs
    control_toolbox
    controller_interface
    forward_command_controller
    realtime_tools
    sensor_msgs
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})


add_executable(test_traj_vel_control src/test_traj_vel_control.cpp)
target_link_libraries(test_traj_vel_control ${catkin_LIBRARIES})


add_executable(test_vel_control src/test_vel_control.cpp)
target_link_libraries(test_vel_control ${catkin_LIBRARIES})


add_executable(test_group_vel_control src/test_vel_control.cpp)
target_link_libraries(test_group_vel_control ${catkin_LIBRARIES})
