#!/bin/bash
# PROGRAMMER: Frederick Wachter
# DATE CREATED: 2016-05-20
# LAST MODIFIED: 2016-05-24
# PURPOSE: Make it easier to run YuMi scripts - Run YuMi in RViz

source yumi_ws/devel/setup.bash # source the catkin workspace
roslaunch yumi_moveit_config moveit_planning_execution.launch # launch YuMi in RViz


