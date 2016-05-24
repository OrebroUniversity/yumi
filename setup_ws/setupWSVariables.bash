#!/bin/bash
# PROGRAMMER: Frederick Wachter
# DATE CREATED: 2016-05-20
# LAST MODIFIED: 2016-05-24
# PURPOSE: Create workspace variables for YuMi during initial setup

# Get the directory location of the YuMi folder
if [ -z "$1" ]; then 
	echo "Please input the path of where the yumi directory is located"
	exit
fi

echo "Adding Command Line Workspace Variables..." # notify user the process has started

# Run Files that add Command Line Capabilities for YuMi
bash ${1}/misc/yumi_scripts/addYuMiCommands.bash ${1} # run file to add command line variables to bashrc

source ~/.bashrc # source bashrc to finalize changes for current terminal window

echo "Finished." # notify user that the process is finished


