#include <ros/ros.h>
#include <ros/console.h>

#include <yumi_collision/yumi_collision_node.h>

int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "yumi_collision_node");
    YumiCollisionNode collisionNode;
    ros::AsyncSpinner spinner(1); // Use 1 threads
    spinner.start();
    ros::waitForShutdown();

    //ros::spin();
    return 0;
}

 
