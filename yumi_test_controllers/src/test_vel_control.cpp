
#include <yumi_test_controllers.h>


sensor_msgs::JointState joints_state;
trajectory_msgs::JointTrajectory left_traj;
trajectory_msgs::JointTrajectory right_traj;
std_msgs::Float64MultiArray left_command;
std_msgs::Float64MultiArray right_command;


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
	ROS_INFO("Joint states update!");
	joints_state.name = msg.name;
	joints_state.position = msg.position;


	if(!left_command_sent)
	{
		std::vector<double> left_vel = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

		left_command.layout.dim.push_back(std_msgs::MultiArrayDimension());
		left_command.layout.dim[0].size = left_vel.size();
		left_command.layout.dim[0].stride = 1;
		left_command.layout.dim[0].label = "v";
		
		left_command.data.clear();
		left_command.data.resize(num_joints_arm);

		for (int i = 0; i < num_joints_arm; i++)
		{
	        left_command.data[i] = left_vel[i];
		}

		cout << "Publishing command" << endl;
		left_pub.publish(left_command);

		left_command_sent = true;
	}

	if(!right_command_sent)
	{
		std::vector<double> right_vel = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

		right_command.layout.dim.push_back(std_msgs::MultiArrayDimension());
		right_command.layout.dim[0].size = right_vel.size();
		right_command.layout.dim[0].stride = 1;
		right_command.layout.dim[0].label = "v";
		
		right_command.data.clear();
		right_command.data.resize(num_joints_arm);
		// right_command.data.insert(right_command.data.end(), right_vel.begin(), right_vel.end());

		for (int i = 0; i < num_joints_arm; i++)
		{
	        right_command.data[i] = right_vel[i];
		}

		cout << "Publishing command" << endl;
		right_pub.publish(right_command);

		right_command_sent = true;
	}
}




int main( int argc, char* argv[] )
{

	ros::init(argc, argv, "test_joint_vel_control");
	ros::NodeHandle nh;


    ROS_INFO("test_join_vel_control node started!");

    // left_traj.joint_names.push_back("yumi_joint_1_l");
    // left_traj.joint_names.push_back("yumi_joint_2_l");
    // left_traj.joint_names.push_back("yumi_joint_7_l");
    // left_traj.joint_names.push_back("yumi_joint_3_l");
    // left_traj.joint_names.push_back("yumi_joint_4_l");
    // left_traj.joint_names.push_back("yumi_joint_5_l");
    // left_traj.joint_names.push_back("yumi_joint_6_l");
    
	// right_traj.joint_names.push_back("yumi_joint_1_r");
	// right_traj.joint_names.push_back("yumi_joint_2_r");
	// right_traj.joint_names.push_back("yumi_joint_7_r");
	// right_traj.joint_names.push_back("yumi_joint_3_r");
	// right_traj.joint_names.push_back("yumi_joint_4_r");
	// right_traj.joint_names.push_back("yumi_joint_5_r");
	// right_traj.joint_names.push_back("yumi_joint_6_r");


    sub = nh.subscribe("/yumi/joint_states", 1000, joint_states_callback);

    left_pub = nh.advertise<std_msgs::Float64MultiArray>("/yumi/joint_group_vel_controller_l/command", 100, left_vel_controller_callback);
	right_pub = nh.advertise<std_msgs::Float64MultiArray>("/yumi/joint_group_vel_controller_r/command", 100, right_vel_controller_callback);

	ros::spin();


	return EXIT_SUCCESS;
}

