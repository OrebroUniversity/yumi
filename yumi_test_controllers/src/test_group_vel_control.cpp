
#include <yumi_test_controllers.h>


sensor_msgs::JointState joints_state;
trajectory_msgs::JointTrajectory left_traj;
trajectory_msgs::JointTrajectory right_traj;
std_msgs::Float64MultiArray left_command;
std_msgs::Float64MultiArray right_command;


int numSubscribersConnected = 0;
int num_joints = 14;
int num_joints_arm = 7;


auto last_call_time = std::chrono::high_resolution_clock::now();
auto curr_call_time = std::chrono::high_resolution_clock::now();;

double sine_period = 1.0;
double sine_freq = 1 / sine_period;
double sine_amp = 0.3;


ros::Publisher left_pub;
ros::Publisher right_pub;
ros::Publisher vel_signal_pub;
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

	curr_call_time = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed = curr_call_time - last_call_time;

	double sin_val = sine_amp * sin(elapsed.count() * 2 * M_PI * sine_freq);

	vel_signal_pub.publish(sin_val);
	cout << "Time since last joint state received = " << elapsed.count() << endl;
	cout << "Sine value = " << sin_val << endl;

	std::vector<double> left_multiplier = {0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0};
	left_command.layout.dim.push_back(std_msgs::MultiArrayDimension());
	left_command.layout.dim[0].size = left_multiplier.size();
	left_command.layout.dim[0].stride = 1;
	left_command.layout.dim[0].label = "v";	
	left_command.data.clear();
	left_command.data.resize(num_joints_arm);


	std::vector<double> right_multiplier = {0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0};
	right_command.layout.dim.push_back(std_msgs::MultiArrayDimension());
	right_command.layout.dim[0].size = right_multiplier.size();
	right_command.layout.dim[0].stride = 1;
	right_command.layout.dim[0].label = "v";	
	right_command.data.clear();
	right_command.data.resize(num_joints_arm);


	for (int i = 0; i < num_joints_arm; i++)
	{
		left_command.data[i] = sin_val * left_multiplier[i];
		right_command.data[i] = sin_val * right_multiplier[i];
	}

	cout << "Publishing command" << endl;
	left_pub.publish(left_command);
	right_pub.publish(right_command);


	if(elapsed.count() >= sine_period)
	{
		last_call_time = std::chrono::high_resolution_clock::now();
	}
}




int main( int argc, char* argv[] )
{

	ros::init(argc, argv, "test_joint_vel_control");
	ros::NodeHandle nh;

    sub = nh.subscribe("/yumi/joint_states", 1000, joint_states_callback);

	vel_signal_pub = nh.advertise<std_msgs::Float64>("/yumi/vel_signal", 250);
    left_pub = nh.advertise<std_msgs::Float64MultiArray>("/yumi/joint_group_vel_controller_l/command", 1000);
	right_pub = nh.advertise<std_msgs::Float64MultiArray>("/yumi/joint_group_vel_controller_r/command", 1000);

	ros::spin();


	return EXIT_SUCCESS;
}

