
#include <yumi_test_controllers.h>


sensor_msgs::JointState joints_state;
trajectory_msgs::JointTrajectory left_traj;
trajectory_msgs::JointTrajectory right_traj;


int numSubscribersConnected = 0;
int num_joints = 14;
int num_joints_arm = 7;
bool left_command_sent = false;
bool right_command_sent = false;

ros::Publisher left_pub;
ros::Publisher right_pub;
ros::Subscriber sub;




void left_vel_controller_callback(const ros::SingleSubscriberPublisher& pub)
{
	ROS_INFO("New subscriber!");
    numSubscribersConnected++;
}


void right_vel_controller_callback(const ros::SingleSubscriberPublisher& pub)
{
	ROS_INFO("New subscriber!");
    numSubscribersConnected++;
}




void joint_states_callback(const sensor_msgs::JointState &msg)
{
	// ROS_INFO("Joint states update!");
	joints_state.name = msg.name;
	joints_state.position = msg.position;


	if(!left_command_sent)
	{
		ROS_INFO("Left arm command!");
		trajectory_msgs::JointTrajectoryPoint p;
		for (int i = 0; i < num_joints; i+=2)
		{
	        p.positions.push_back(joints_state.position[i]);
	        // p.velocities.push_back(0.0);
	        // p.accelerations.push_back(0.0);
	        p.time_from_start = ros::Duration(1.0);
		}

		int joint_test_idx = 2;
		p.positions[joint_test_idx] = p.positions[joint_test_idx] - 0.5;
		// p.velocities[joint_test_idx] = -1.0;

		left_traj.points.push_back(p);
		left_traj.header.stamp = ros::Time(0.0);

		cout << "Publishing command" << endl;
		left_pub.publish(left_traj);

		left_command_sent = true;
	}

	if(!right_command_sent)
	{
		ROS_INFO("Right arm command!");
		trajectory_msgs::JointTrajectoryPoint p;
		for (int i = 1; i < num_joints; i+=2)
		{
	        p.positions.push_back(joints_state.position[i]);
	        // p.velocities.push_back(0.0);
	        // p.accelerations.push_back(0.0);
	        p.time_from_start = ros::Duration(1.0);
		}

		int joint_test_idx = 4;
		p.positions[joint_test_idx] = p.positions[joint_test_idx] - 0.5;
		// p.velocities[joint_test_idx] = -1.0;

		right_traj.points.push_back(p);
		right_traj.header.stamp = ros::Time(0.0);

		cout << "Publishing command" << endl;
		right_pub.publish(right_traj);

		right_command_sent = true;
	}
}




int main( int argc, char* argv[] )
{

	ros::init(argc, argv, "test_joint_vel_control");
	ros::NodeHandle nh;


    ROS_INFO("test_join_vel_control node started!");

    left_traj.joint_names.push_back("yumi_joint_1_l");
    left_traj.joint_names.push_back("yumi_joint_2_l");
    left_traj.joint_names.push_back("yumi_joint_3_l");
    left_traj.joint_names.push_back("yumi_joint_4_l");
    left_traj.joint_names.push_back("yumi_joint_5_l");
    left_traj.joint_names.push_back("yumi_joint_6_l");
	left_traj.joint_names.push_back("yumi_joint_7_l");
    
	right_traj.joint_names.push_back("yumi_joint_1_r");
	right_traj.joint_names.push_back("yumi_joint_2_r");
	right_traj.joint_names.push_back("yumi_joint_3_r");
	right_traj.joint_names.push_back("yumi_joint_4_r");
	right_traj.joint_names.push_back("yumi_joint_5_r");
	right_traj.joint_names.push_back("yumi_joint_6_r");
	right_traj.joint_names.push_back("yumi_joint_7_r");


    sub = nh.subscribe("/yumi/joint_states", 1000, joint_states_callback);

    left_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/yumi/joint_traj_vel_controller_l/command", 100, left_vel_controller_callback);
	right_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/yumi/joint_traj_vel_controller_r/command", 100, right_vel_controller_callback);

	ros::spin();


	return EXIT_SUCCESS;
}

