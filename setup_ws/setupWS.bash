#!/bin/bash
# PROGRAMMER: Frederick Wachter
# DATE CREATED: 2016-05-24
# LAST MODIFIED: 2016-05-26
# PURPOSE: Setup workspace to be able to run YuMi files

# How to set up the workspace:
#  - Clone this repo into the home directory: cd && git clone https://www.github.com/ethz-asl/yumi
#      (or move the file to the home directory: mv <path>/yumi ~/yumi)
#  - Run the following command: bash ~/yumi/setup_ws/setupWS.bash

#=========================================
#---------- PRE-SETUP CHECKING -----------
#=========================================
cd ~ # go to the home directory

#----- Check if yumi_ws already exists in the home directory -----
# REFERENCE: http://stackoverflow.com/questions/59838/check-if-a-directory-exists-in-a-shell-script
if [ -d "yumi_ws" ]; then
    echo "[ERROR]: Setup failed. YuMi workspace directory already exists (~/yumi_ws). Cannot create workspace."
    echo "  - Possible fixes: Delete existing directory or fix existing directory."
    exit
fi

echo "Please read the wiki before continuing in order to ensure the workspace is properly setup. The Wiki and more information about this repo can be found at:" # tell user to visit Wiki
echo "  https://www.github.com/ethz-asl/yumi/wiki" # webpage for the wiki of this repo
echo "" # add in a space after comments

#----- Ask the user if they are sure they would like to continue the setup -----
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

#----- Ask user if ROS Ingigo is installed -----
# REFERENCE: http://stackoverflow.com/questions/1885525/how-do-i-prompt-a-user-for-confirmation-in-bash-script
read -p "Is ROS Indigo installed on this machine (y/n)? " response # get response from user
case "$response" in 
	y|Y ) existROS=1;; # ROS has already been setup
	n|N ) existROS=0;; # Set flag to install ROS
	* ) echo "invalid response. Please use 'y' for Yes and 'n' for No" && exit;; # invalid input
esac

#----- If ROS Indigo has not been setup yet -----
if [ $existROS -eq 0 ]; then
	sudo apt-get update # update potential install list

	sudo apt-get install ros-indigo-desktop-full -y # install indigo desktop

	sudo rosdep init # initialize ROS
	rosdep update # update ROS dependencies
fi

#----- Ensure that MoveIt! is Downloaded -----
sudo apt-get install ros-indigo-moveit-full -y # install MoveIt!
source /opt/ros/indigo/setup.bash # setup the environment

echo "Verified installs." # notify user that the installs have been verified


#======================================
#---------- SETUP WORKSPACE -----------
#======================================
clear # clear the terminal window
echo "[Part 2/3] Setting up workspace... " # notify user the workspace setup has started

#----- Create Workspace -----
mkdir yumi_ws # create workspace folder
mkdir yumi_ws/src # create folder to contain all files in workspace

#----- Add GitHub repo's neccessary to run YuMi files -----
mv yumi yumi_ws/src # move already cloned YuMi repo into workspace
git clone https://github.com/ros-industrial/abb.git yumi_ws/src/abb_driver # clone the GitHub repo for the ABB driver
git clone https://github.com/ros-industrial/industrial_core.git yumi_ws/src/industrial_driver # clone the GitHub repo for the ROS-Industrial driver

#----- Build Workspace -----
cd ~/yumi_ws # go to the YuMi workspace
catkin_make # build the workspace

echo "Finished setting up the workspace." # notify user that the workspace has been setup


#======================================
#-------- FINALIZE INSTALLS -----------
#======================================
clear # clear the terminal window
echo "[Part 3/3] Finalizing installs... " # notify user installations are being finalized

#----- Variables -----
addYuMiCommands=0 # 0) Don't add YuMi command line tools | 1) Add YuMi commands line tools (yumi, yumi_demo, yumi_server)
addYuMiSource=0 # 0) Don't add YuMi workspace source command to bashrc | 1) Add YuMi workspace source command to bashrc
addBashrcHeader=1 # 0) Header for additions to bashrc file has already been added | 1) Header for additions to bashrc file has not been added yet
addBashrcFooter=0 # 0) No additions have been made to bashrc file, footer is not needed | 1) Additions have been made to bashrc file, add footer to additions

#----- Add Workspace Variables to Allow Command Line Capabilities -----
# YuMi command line tools
read -p "Add YuMi quick commands to command line? (recommended) (y/n)? " response # get response from user
case "$response" in 
	y|Y ) addYuMiCommands=1;; # add in command line alias to run YuMi easier 
	n|N ) echo "Not adding command line tools for YuMi... ";; # don't add in command line alias
	* ) echo "Invalid input. Not adding command line tools for YuMi... ";; # don't add in comamnd line alias
esac
if [ $addYuMiCommands -eq 1 ]; then # if the commands should be added
	echo "Adding command line tools for YuMi... " # notify user that command line tools for YuMi are being added

	bash ~/yumi_ws/src/yumi/setup_ws/setupWSVariables.bash ~/yumi_ws/src/yumi # run file to add command for YuMi
	addBashrcHeader=0 # set flag to inficate a header has already been added to the bashrc file
	addBashrcFooter=1 # set flag to add in footer to bashrc file
fi 

#----- ROS Sourcing -----
if [ $existROS -eq 0 ]; then # if this is the first time for ROS install
	echo "Adding in ROS source... " # notify user that sourcing of ROS is being added to bashrc

	# If a header hasnt been added to the bashrc file yet
	if [ $addBashrcHeader -eq 1 ]; then
		echo "" >> ~/.bashrc # add in a blank ine before addition
		echo "# From: YuMi Github Repo" >> ~/.bashrc # add in a header for added section
		echo "# Purpose: Sourcing for ROS and/or YuMi workspace" >> ~/.bashrc # adding comment to describe purpose of addition
		addBashrcHeader=0 # set flag to inficate a header has already been added to the bashrc file
	fi

	echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc # add sourcing of ROS
	addBashrcFooter=1 # set flag to add in footer to bashrc file
fi

#----- YuMi Workspace Sourcing -----
read -p "Automatically add workspace source to bashrc? (recommended) (y/n)? " response # get response from user
case "$response" in 
	y|Y ) addYuMiSource=1;; # add in command line alias to run YuMi easier 
	* ) echo "Not adding source to bashrc file... ";; # don't add in comamnd line alias
esac
if [ $addYuMiSource -eq 1 ]; then
	echo "Adding in YuMi workspace source... " # notify user that sourcing of the YuMi workspace is being added to bashrc

	# If a header hasnt been added to the bashrc file yet
	if [ $addBashrcHeader -eq 1 ]; then
		echo "" >> ~/.bashrc # add in a blank ine before addition
		echo "# From: YuMi Github Repo" >> ~/.bashrc # add in a header for added section
		echo "# Purpose: Sourcing for ROS and/or YuMi workspace" >> ~/.bashrc # adding comment to describe purpose of addition
		addBashrcHeader=0 # set flag to indicate a header has already been added to the bashrc file
	fi

	echo "source ~/yumi_ws/devel/setup.bash" >> ~/.bashrc # add sourcing of the YuMi workspace
	addBashrcFooter=1 # set flag to add in footer to bashrc file
fi

#----- Adding footer to added section -----
if [ $addBashrcFooter -eq 1 ]; then
	echo "# End of Addition" >> ~/.bashrc # indicate that there are no more additions for YuMi
	source ~/.bashrc # ensure all changes has been soruced
fi

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

#----- Give directions to user on how to run YuMi files -----
echo "" # add in a blank space before instructions
echo "Please use the following command before continuing:"
echo "cd && source ~/.bashrc" # go to the home directory and ensure bashrc has been source after changes
echo ""
echo "Please refer to the repo Wiki page for futher instructions"
echo "Wiki location: www.github.com/ethz-asl/yumi/wiki"
echo "" # add in a blank space after instructions


