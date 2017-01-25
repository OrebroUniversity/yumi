Urdf & Gazebo files for the ABB YuMi (IRB 14000) robot

### Dependencies
This package depends on ros-industrial which is not released on kinetic yet. So skip step 2 if you have already install ros-industrial.

#### Step 1
Install all of these:

> sudo apt-get install ros-kinetic-control-toolbox ros-kinetic-controller-interface ros-kinetic-controller-manager ros-kinetic-joint-limits-interface ros-kinetic-transmission-interface ros-kinetic-moveit-core ros-kinetic-moveit-planners ros-kinetic-moveit-ros-planning 

#### Step 2
Install ros-industrial from source. 

#### Step 3
Source the workspace containing ros-industrial and then catkin_make the workspace containing the clone of this package.
