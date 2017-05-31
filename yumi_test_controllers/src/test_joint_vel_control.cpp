
#include <yumi_test_controllers.h>


sensor_msgs::JointState joints_state;
trajectory_msgs::JointTrajectory traj;


int numSubscribersConnected = 0;
int num_joints = 14;
bool command_sent = false;

ros::Publisher pub;
ros::Subscriber sub;




void vel_controller_callback(const ros::SingleSubscriberPublisher& pub)
{
	ROS_INFO("New subscriber!");
    numSubscribersConnected++;
}




void joint_states_callback(const sensor_msgs::JointState &msg)
{
	ROS_INFO("Joint states update!");
	joints_state.name = msg.name;
	joints_state.position = msg.position;


	if(!command_sent)
	{
		trajectory_msgs::JointTrajectoryPoint p;
		for (int i = 0; i < num_joints; i++)
		{
	        p.positions.push_back(joints_state.position[i]);
	        // p.velocities.push_back(0.0);
	        // p.accelerations.push_back(0.0);
	        p.time_from_start = ros::Duration(1.0);
		}

		p.positions[3] = -1.0;

		traj.points.push_back(p);
		traj.header.stamp = ros::Time::now();

		cout << "Publishing command" << endl;
		pub.publish(traj);

		command_sent = true;
	}
}




int main( int argc, char* argv[] )
{

	ros::init(argc, argv, "test_joint_vel_control");
	ros::NodeHandle nh;


    ROS_INFO("test_join_vel_control node started!");

    traj.joint_names.push_back("yumi_joint_1_l");
    traj.joint_names.push_back("yumi_joint_1_r");
    traj.joint_names.push_back("yumi_joint_2_l");
    traj.joint_names.push_back("yumi_joint_2_r");
    traj.joint_names.push_back("yumi_joint_3_l");
    traj.joint_names.push_back("yumi_joint_3_r");
    traj.joint_names.push_back("yumi_joint_4_l");
    traj.joint_names.push_back("yumi_joint_4_r");
    traj.joint_names.push_back("yumi_joint_5_l");
    traj.joint_names.push_back("yumi_joint_5_r");
    traj.joint_names.push_back("yumi_joint_6_l");
    traj.joint_names.push_back("yumi_joint_6_r");
    traj.joint_names.push_back("yumi_joint_7_l");
    traj.joint_names.push_back("yumi_joint_7_r");


    sub = nh.subscribe("/yumi/joint_states", 1000, joint_states_callback);

    pub = nh.advertise<trajectory_msgs::JointTrajectory>("/yumi/joint_trajectory_vel_controller/command", 100, vel_controller_callback);

	ros::spin();


	return EXIT_SUCCESS;
}

