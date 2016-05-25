#!/bin/bash
# PROGRAMMER: Frederick Wachter
# DATE CREATED: 2016-05-24
# LAST MODIFIED: 2016-05-25
# PURPOSE: Setup workspace to be able to run YuMi files

# How to set up the workspace:
#  - Clone this repo into the home directory: cd && git clone https://www.github.com/ethz-asl/yumi
#      (or move the file to the home directory: mv <path>/yumi ~/yumi)
#  - Run the following command: bash ~/yumi/setup_ws/setupWS.bash

#=========================================
#---------- PRE-SETUP CHECKING -----------
#=========================================
cd ~ # go to the home directory

# Check if yumi_ws already exists in the home directory
# REFERENCE: http://stackoverflow.com/questions/59838/check-if-a-directory-exists-in-a-shell-script
if [ -d "yumi_ws" ]; then
    echo "YuMi workspace directory already exists (~/yumi_ws). Cannot create workspace."
    echo "[ERROR]: Setup failed."
    echo "Possible fixes: Delete existing directory or fix existing directory."
    exit
fi

# Ask the user if they are sure they would like to continue the setup
# REFERENCE: http://stackoverflow.com/questions/1885525/how-do-i-prompt-a-user-for-confirmation-in-bash-script
read -p "Are you sure you would like to continue with the setup (y/n)? " response # get response from user
case "$response" in 
	y|Y ) echo "Continuing setup... ";; # continue with the setup
	n|N ) echo "Exiting setup... " && echo "Exited setup." && exit;; # exit the setup
	* ) echo "invalid response. Please use 'y' for Yes and 'n' for No" && exit;; # invalid input
esac


#======================================
#---------- VERIFY INSTALLS -----------
#======================================
echo "[Part 1/3] Verifying installs... " # notify the user that installs will potentially be made

# Ask user if ROS Ingigo is installed
# REFERENCE: http://stackoverflow.com/questions/1885525/how-do-i-prompt-a-user-for-confirmation-in-bash-script
read -p "Is ROS Indigo installed on this machine (y/n)? " response # get response from user
case "$response" in 
	y|Y ) existROS=1;; # ROS has already been setup
	n|N ) existROS=0;; # Set flag to install ROS
	* ) echo "invalid response. Please use 'y' for Yes and 'n' for No" && exit;; # invalid input
esac

# If ROS Indigo has not been setup yet
if [ $existROS -eq 0 ]; then
	sudo apt-get update # update potential install list

	sudo apt-get install ros-indigo-desktop-full -y # install indigo desktop

	sudo rosdep init # initialize ROS
	rosdep update # update ROS dependencies
fi

# Ensure that MoveIt! is Downloaded
sudo apt-get install ros-indigo-moveit-full -y # install MoveIt!
source /opt/ros/indigo/setup.bash # setup the environment

echo "Verified installs." # notify user that the installs have been verified


#======================================
#---------- SETUP WORKSPACE -----------
#======================================
clear # clear the terminal window
echo "[Part 2/3] Setting up workspace... " # notify user the workspace setup has started

# Create Workspace
mkdir yumi_ws # create workspace folder
mkdir yumi_ws/src # create folder to contain all files in workspace

# Add GitHub repo's neccessary to run YuMi files
mv yumi yumi_ws/src # move already cloned YuMi repo into workspace
git clone https://github.com/ros-industrial/abb.git yumi_ws/src/abb_driver # clone the GitHub repo for the ABB driver
git clone https://github.com/ros-industrial/industrial_core.git yumi_ws/src/industrial_driver # clone the GitHub repo for the ROS-Industrial driver

# Build Workspace
cd ~/yumi_ws # go to the YuMi workspace
catkin_make # build the workspace

echo "Finished setting up the workspace." # notify user that the workspace has been setup


#======================================
#-------- FINALIZE INSTALLS -----------
#======================================
clear # clear the terminal window
echo "[Part 3/3] Finalizing installs... " # notify user installations are being finalized

# Add Workspace Variables to Allow Command Line Capabilities
bash ~/yumi_ws/src/yumi/setup_ws/setupWSVariables.bash ~/yumi_ws/src/yumi # setup command line variables for running YuMi easier
if [ $existROS -eq 0 ]; then # if this is the first time for ROS install
	echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc # source ROS
fi
echo "source ~/yumi_ws/devel/setup.bash" >> ~/.bashrc # source workspace for future terminal windows on startup
echo "# End of Addition" >> ~/.bashrc # indicate that there are no more additions for YuMi
source ~/.bashrc # ensure all changes has been soruced

echo "Finished setting up workspace." # notify user the setup has finished

#----------------------------------------------
#-- Add in checks to ensure correct install? --
#---------- Note Create: 2016-05-24 -----------
#----------------------------------------------


#======================================
#---------- USER DIRECTIONS -----------
#======================================
clear # clear the terminal window
echo "Workspace setup successfully." # notify user the setup was successful

#--------------------------------------
#-- Need to add in Wiki page to repo --
#------ Note Create: 2016-05-24 -------
#--------------------------------------

# Give directions to user on how to run YuMi files
echo "" # add in a blank space before instructions
echo "Please use the following command before continuing:"
echo "cd && source ~/.bashrc" # go to the home directory and ensure bashrc has been source after changes
echo ""
echo "Please refer to the repo Wiki page for futher instructions"
echo "Wiki location: www.github.com/ethz-asl/yumi/wiki"
echo "" # add in a blank space after instructions


