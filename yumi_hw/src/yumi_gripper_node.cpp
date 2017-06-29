#include <ros/ros.h>
#include <ros/console.h>

#include <yumi_hw/yumi_gripper_node.h>


/*! Main method of the yumi_rws_gripper_node file. (testing doxygen)
 *
 */
int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "yumi_gripper_node");
    YumiGripperNode gripperNode;
    ros::AsyncSpinner spinner(6); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();

    return 0;
}

 
