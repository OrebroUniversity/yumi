

#include <yumi_test_controllers.h>

int main( int argc, char* argv[] )
{
	ros::init(argc, argv, "test_grippers_control");
	ros::NodeHandle nh;

	ROS_INFO("test_grippers_control node started!");

	ros::spin();

	return EXIT_SUCCESS;
}
