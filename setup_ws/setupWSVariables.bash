#!/bin/bash
# PROGRAMMER: Frederick Wachter
# DATE CREATED: 2016-05-20
# LAST MODIFIED: 2016-05-20
# PURPOSE: Create workspace variables for YuMi during initial setup

echo "Adding Command Line Workspace Variables..." # notify user the process has started

# Run Files that add Command Line Capabilities for YuMi
bash ~/catkin_ws/src/yumi/misc/yumi_scripts/addYuMiCommands.bash ~/catkin_ws/src/yumi # run file to add command line variables to bashrc

source ~/.bashrc # source bashrc to finalize changes for current terminal window

echo "Finished." # notify user that the process is finished


