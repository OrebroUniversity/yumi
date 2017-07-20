#include <ros/ros.h>
#include <ros/console.h>

#include <yumi_hw/yumi_gripper_rws_node.h>


/*! Main method of the yumi_rws_gripper_node file. (testing doxygen)
 *
 */
int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "yumi_gripper_node");
    YumiGripperRWSNode gripperNode;

    ros::Rate r(25);
    while(ros::ok()) {
	gripperNode.publish();
	ros::spinOnce();
	r.sleep();
    }
    return 0;
}

 
