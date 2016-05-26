#!/bin/bash
# PROGRAMMER: Frederick Wachter
# DATE CREATED: 2016-05-20
# LAST MODIFIED: 2016-05-26
# PURPOSE: Create workspace variables for YuMi during initial setup

# Get the directory location of the YuMi folder
if [ -z "$1" ]; then 
	echo "Please input the path of where the yumi directory is located"
	exit
fi

echo "Adding command line workspace variables... " # notify user the process has started

# Add YuMi alias for running scriptsecho "" >> ~/.bashrc # add in blank line before addition
echo "" >> ~/.bashrc # add in a blank ine before addition
echo "# From: YuMi Github Repo" >> ~/.bashrc # add header for added section
echo "# Purpose: Alias for YuMi commands" >> ~/.bashrc # describe purpose for added section
echo "alias yumi='bash ${1}/misc/yumi_scripts/yumi.bash'" >> ~/.bashrc # allow for YuMi to be run from command line for the real controller
echo "alias yumi_demo='bash ${1}/misc/yumi_scripts/yumi_demo.bash'" >> ~/.bashrc # allow for YuMi to be run from command line for the fake controller
echo "alias yumi_server='bash ${1}/misc/yumi_scripts/yumi_server.bash'" >> ~/.bashrc # run the YuMi server from command line to send path commands to the real controller
echo "" >> ~/.bashrc # add in blank line underneath addition

source ~/.bashrc # source bashrc to finalize changes for current terminal window

echo "Finished." # notify user that the process is finished


