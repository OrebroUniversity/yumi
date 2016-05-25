#!/bin/bash
# PROGRAMMER: Frederick Wachter
# DATE CREATED: 2016-05-20
# LAST MODIFIED: 2016-05-20
# PURPOSE: Add Linux commands to run YuMi scripts

# Check if a path was given as an argument
if [ -z "$1" ]; then 
	echo "Please input the path of where the yumi script files are located"
	exit
fi

# Add YuMi alias for running scripts
echo "" >> ~/.bashrc # add in blank line before addition
echo "# From: YuMi Github Repo" >> ~/.bashrc
echo "# Purpose: Alias for YuMi commands" >> ~/.bashrc
echo "alias yumi='bash ${1}/misc/yumi_scripts/yumi.bash'" >> ~/.bashrc
echo "alias yumi_demo='bash ${1}/misc/yumi_scripts/yumi_demo.bash'" >> ~/.bashrc
echo "alias yumi_server='bash ${1}/misc/yumi_scripts/yumi_server.bash'" >> ~/.bashrc
echo "" >> ~/.bashrc # add in blank line underneath addition


