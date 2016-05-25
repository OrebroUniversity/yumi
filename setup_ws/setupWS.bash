#!/bin/bash
# PROGRAMMER: Frederick Wachter
# DATE CREATED: 2016-05-24
# LAST MODIFIED: 2016-05-25
# PURPOSE: Setup workspace to be able to run YuMi files

# Before Running this File:
#   - If not done already, clone YuMi repo into home directory: git clone https://github.com/ethz-asl/yumi.git
#      - Or move the already cloned repo into the home directory
#   - Run the following command: cd && bash yumi/setup_ws/setupWS.bash

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
read -p "Are you sure you would like to continue with the setup (y/n)? " response
case "$response" in 
	y|Y ) echo "Continuing setup... ";;
	n|N ) echo "Exiting setup... " && echo "Exited setup." && exit;;
	* ) echo "invalid response. Please use 'y' for Yes and 'n' for No" && exit;;
esac


#======================================
#---------- VERIFY INSTALLS -----------
#======================================
echo "[Part 1/3] Verifying installs... " # notify the user that installs will potentially be made

# Ask user if ROS Ingigo is installed
# REFERENCE: http://stackoverflow.com/questions/1885525/how-do-i-prompt-a-user-for-confirmation-in-bash-script
read -p "Is ROS Indigo installed on this machine (y/n)? " response
case "$response" in 
	y|Y ) testing=1;;
	n|N ) testing=0;;
	* ) echo "invalid response. Please use 'y' for Yes and 'n' for No" && exit;;
esac

# If ROS Indigo has not been setup yet
if [ $testing -eq 0 ]; then
	sudo apt-get update # update potential install list

	sudo apt-get install ros-indigo-desktop-full -y # install indigo desktop
	sudo apt-get install ros-indigo-moveit-full -y # install MoveIt!

	source /opt/ros/indigo/setup.bash # setup the environment
	sudo rosdep init # initialize ROS
	rosdep update # update ROS dependencies
fi

#======================================
#---------- SETUP WORKSPACE -----------
#======================================
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


#======================================
#-------- FINALIZE INSTALLS -----------
#======================================
echo "[Part 3/3] Finalizing installs... " # notify user installations are being finalized

# Add Workspace Variables to Allow Command Line Capabilities
bash ~/yumi_ws/src/yumi/setup_ws/setupWSVariables.bash ~/yumi_ws/src/yumi # setup command line variables for running YuMi easier
if [ $testing -eq 0 ]; then # if this is the first time for ROS install
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
clear # clear the terminal window
echo "Workspace setup successfully." # notify user the setup was successful


#======================================
#---------- USER DIRECTIONS -----------
#======================================

#--------------------------------------
#-- Need to add in Wiki page to repo --
#------ Note Create: 2016-05-24 -------
#--------------------------------------

# Give directions to user on how to run YuMi files
echo ""
echo "The following commands can be used to run the YuMi simulation from command line."
echo "NOTE: Ensure roscore is running in a seperate terminal window"
echo "  - For Demo Only:"
echo "      In a terminal window, run: yumi_demo"
echo "  - For Running on an existing YuMi connected to this computer:"
echo "  NOTE: YuMi must have been setup in RobotStudio according to the wiki or it will not work"
echo "      In one terminal window, run: yumi_server 192.168.125.1"
echo "      In another terminal window, run: yumi"
echo "    NOTE: Wait a few seconds before running the second command"
echo ""


