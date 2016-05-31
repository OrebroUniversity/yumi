#!/bin/bash
# PROGRAMMER: Frederick Wachter
# DATE CREATED: 2016-05-20
# LAST MODIFIED: 2016-05-24
# PURPOSE: Make it easier to run YuMi scripts - Run server to send trajectories to YuMi
# NOTES: 10.37.129.22

if [ -z "$1" ]; then
	echo "Please input the robot ip as an argument"
	exit
fi

source yumi_ws/devel/setup.bash # source the catkin workspace
roslaunch yumi_support robot_interface.launch robot_ip:=$1 # run YuMi server to send files to real YuMi robot at the specified IP address


