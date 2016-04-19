#include<yumi_hw/yumi_hw.h>

#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "yumi_hw_ifce");

    /*
    //ros::NodeHandle param("~");
    //robot = new IIWARobot(param);

    //signal(SIGTERM, &handler);
    ros::NodeHandle nh;
    controller_manager::ControllerManager cm(robot,nh);
    ros::AsyncSpinner spinner(8);
    spinner.start();

    while (true)
    {
	robot->write();
	robot->read();
	cm.update(robot->get_time(), robot->get_period());
    }

    spinner.stop();
    */
}

