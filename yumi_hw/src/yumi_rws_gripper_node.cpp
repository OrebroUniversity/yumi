#include <ros/ros.h>
#include <ros/console.h>

#include <yumi_hw/yumi_gripper_node.h>

int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "yumi_gripper_node");
    YumiGripperNode gripperNode;
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();

    //ros::spin();
    return 0;
}

 
