#!/bin/bash
# PROGRAMMER: Frederick Wachter
# DATE CREATED: 2016-05-24
# LAST MODIFIED: 2016-05-24
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
echo "[Part 1/2] Verifying installs... " # notify the user that installs will potentially be made

sudo apt-get update # update potential install list

# ROS Installs
sudo apt-get install ros-indigo-desktop-full -y # install indigo desktop

# MoveIt! Installs
#sudo apt-get install python-wstool -y -- DO I NEED THIS? --

# RViz Installs
#sudo apt-get install ros-indigo-rviz -y -- DO I NEED THIS? --

# Install ROS and related packages if not alredy installed
# Initialize ROS if not already initialized

exit # add breakpoint

sudo rosdep init # initialize ROS


#======================================
#---------- SETUP WORKSPACE -----------
#======================================
echo "[Part 2/2] Setting up workspace... " # notify user the setup has started

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

# Add Workspace Variables to Allow Command Line Capabilities
bash ~/yumi_ws/src/yumi/setup_ws/setupWSVariables.bash ~/yumi_ws/src/yumi # setup command line variables for running YuMi easier
echo "source ~/yumi_ws/devel/setup.bash" >> ~/.bashrc # source workspace for future terminal windows on startup
echo "# End of Addition" >> ~/.bashrc # indicate that there are no more additions for YuMi
source ~/.bashrc # ensure all changes has been soruced

echo "Finished setting up workspace." # notify user the setup has finished

#----------------------------------------------
#-- Add in checks to ensure correct install? --
#---------- Note Create: 2016-05-24 -----------
#----------------------------------------------
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
echo "  - For Demo Only:"
echo "      In a terminal window, run: yumi_demo"
echo "  - For Running on an existing YuMi connected to this computer:"
echo "  NOTE: YuMi must have been setup in RobotStudio according to the wiki or it will not work"
echo "      In one terminal window, run: yumi_server 192.168.125.1"
echo "      In another terminal window, run: yumi"
echo "    NOTE: Wait a few seconds before running the second command"
echo ""


