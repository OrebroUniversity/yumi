
#include <signal.h>


#include <ros/ros.h>

#include <boost/interprocess/smart_ptr/unique_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>


#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <controller_manager/controller_manager.h>

#include "simple_message/message_handler.h"
#include "simple_message/message_manager.h"
#include "simple_message/messages/joint_message.h"
#include "simple_message/smpl_msg_connection.h"
#include "simple_message/socket/tcp_socket.h"
#include "simple_message/socket/tcp_client.h"
#include <sensor_msgs/JointState.h>

#include <yumi_hw/YumiGrasp.h>
#include <urdf/model.h>

#include <std_msgs/Float64.h>

#define MSG_TYPE_GRIPPER_COMMAND 8008
#define MSG_TYPE_GRIPPER_STATE 8009

#define DEFAULT_STATE_PORT 13002
#define DEFAULT_COMMAND_PORT 13000

#define LEFT_GRIPPER 1
#define RIGHT_GRIPPER 2

/**
  * Overrides message handler: keeps joint states thread-safe.
  */
class YumiGripperStateHandler : public industrial::message_handler::MessageHandler 
{
    using industrial::message_handler::MessageHandler::init;
    
    private:
		float gripper_positions[2];
		boost::mutex data_buffer_mutex;

    public:
		bool getGripperStates(float &left, float &right) 
		{ 
			boost::mutex::scoped_lock lock(data_buffer_mutex);
			left = gripper_positions[0];
			right = gripper_positions[1];
		}

		bool init(industrial::smpl_msg_connection::SmplMsgConnection* connection)
		{
			return init((int)MSG_TYPE_GRIPPER_STATE, connection);
		}

    protected:

		bool internalCB(industrial::simple_message::SimpleMessage& in)
		{
			boost::mutex::scoped_lock lock(data_buffer_mutex);

			bool ret;	    
			if(in.getMessageType() != MSG_TYPE_GRIPPER_STATE) 
			{
				ret = false;
			}
			else 
			{		
				industrial::byte_array::ByteArray data = in.getData();
				industrial::shared_types::shared_real value_from_msg;

				data.unload(value_from_msg);
				gripper_positions[0] = value_from_msg; 
				data.unload(value_from_msg);
				gripper_positions[1] = value_from_msg;
			}

			// Reply back to the controller if the sender requested it.
			if (industrial::simple_message::CommTypes::SERVICE_REQUEST == in.getCommType())
			{
				ROS_INFO("Reply requested, sending");
				industrial::simple_message::SimpleMessage reply;
				reply.init(MSG_TYPE_GRIPPER_STATE, industrial::simple_message::CommTypes::SERVICE_REPLY, 
					ret ? industrial::simple_message::ReplyTypes::SUCCESS: industrial::simple_message::ReplyTypes::FAILURE) ;
				this->getConnection()->sendMsg(reply);
			}

			return ret;
		}

};

/**
  * Keep a connection to the robot and send and receive joint states
  */
class YumiGripperStateInterface {
    private:
		boost::thread RapidCommThread_;
		
		///industrial connection
		industrial::tcp_client::TcpClient default_tcp_connection_; 
		industrial::smpl_msg_connection::SmplMsgConnection* connection_;
		
		industrial::tcp_client::TcpClient default_tcp_connection_command; 
		industrial::smpl_msg_connection::SmplMsgConnection* connection_command;

		industrial::message_manager::MessageManager manager_;
		YumiGripperStateHandler gripper_handler;

		bool stopComm_;

		virtual void RapidCommThreadCallback()
		{
			while(!stopComm_) 
			{
				manager_.spinOnce();
			}
			return;
		}

    public:
		YumiGripperStateInterface() 
		{
			this->connection_ = NULL;
			stopComm_ = true;
		}

		~YumiGripperStateInterface() 
		{
			stopThreads();
		}
		
		void stopThreads() 
		{
			stopComm_ = true;
			RapidCommThread_.join();
		}

		void startThreads() 
		{
			if(!stopComm_) 
			{
				boost::thread( boost::bind(&YumiGripperStateInterface::RapidCommThreadCallback, this) );
			}
		}

		void getCurrentJointStates(float &left, float &right) 
		{
			gripper_handler.getGripperStates(right, left);	    
		}

		void setGripperEfforts(float left, float right) 
		{
			industrial::simple_message::SimpleMessage grasp_msg;
			industrial::byte_array::ByteArray data;
			industrial::shared_types::shared_real value_to_msg;
		
			value_to_msg = left;
			data.load(value_to_msg);
		
			value_to_msg = right;
			data.load(value_to_msg);

			std::cout << "Gripper commands (left and right):" << std::endl;
			std::cout << left << std::endl;
			std::cout << right << std::endl;

			grasp_msg.init(MSG_TYPE_GRIPPER_COMMAND, industrial::simple_message::CommTypes::TOPIC, industrial::simple_message::ReplyTypes::INVALID, data);
			connection_command->sendMsg(grasp_msg);
		}

		bool init(std::string ip = "", int port = DEFAULT_STATE_PORT, int port_command = DEFAULT_COMMAND_PORT) 
		{
			/* Initialize connection */
			char* ip_addr = strdup(ip.c_str());  // connection.init() requires "char*", not "const char*"
			ROS_INFO("Robot state connecting to IP address: '%s:%d'", ip_addr, port);
			default_tcp_connection_.init(ip_addr, port);
			default_tcp_connection_command.init(ip_addr, port_command);
			free(ip_addr);

			connection_ = &default_tcp_connection_;
			connection_ -> makeConnect();

			connection_command = &default_tcp_connection_command;
			connection_command -> makeConnect();

			/* Initialize message manager */
			manager_.init(connection_);

			/* Initialize message handler */
			gripper_handler.init(connection_);

			/* Register handler to manager */
			manager_.add(&gripper_handler,false);

			stopComm_ = false;
		}
	
};



bool g_quit = false;

void quitRequested(int sig)
{
  g_quit = true;
}

class YumiGrippersHW : public hardware_interface::RobotHW
{
    public:
        YumiGrippersHW(std::string name)
        {
            nh_.param<std::string>("joint_state_topic", gripper_state_topic, "gripper_states");
            nh_.param<std::string>("grasp_request_topic", grasp_request_topic, "do_grasp");
            nh_.param<std::string>("grasp_release_topic", grasp_release_topic, "release_grasp");
            nh_.param<double>("publish_period", js_rate, 0.1);
            nh_.param("port_stream", port_s, DEFAULT_STATE_PORT);
            nh_.param("port_command", port_c,DEFAULT_COMMAND_PORT);
            nh_.param("grasp_force", default_force, 5);
            nh_.param("ip", ip, std::string("192.168.125.1") );

            heartbeat_ = nh_.createTimer(ros::Duration(js_rate), &YumiGrippersHW::publishState, this);
            gripper_status_publisher_ = ros::NodeHandle().advertise<sensor_msgs::JointState>(gripper_state_topic, 10); 

			left_gripper_cmd = 13;
			right_gripper_cmd = 13;

            gripper_interface.init(ip, port_s, port_c);
            gripper_interface.startThreads();

			left_gripper_cmd_sub = nh_.subscribe("/gripper_l_pos_cmd", 100, &YumiGrippersHW::leftGripperCmdCallback, this);
			right_gripper_cmd_sub = nh_.subscribe("/gripper_r_pos_cmd", 100, &YumiGrippersHW::rightGripperCmdCallback, this);

            ROS_INFO_STREAM("Creating a YumiGrippers HW interface");
            hardware_interface::JointStateHandle state_handle_l("gripper_l_joint", &pos[0], &vel[0], &eff[0]);
            grippers_state_interface.registerHandle(state_handle_l);
            hardware_interface::JointStateHandle state_handle_r("gripper_r_joint", &pos[1], &vel[1], &eff[1]);
            grippers_state_interface.registerHandle(state_handle_r);
            registerInterface(&grippers_state_interface);

            hardware_interface::JointHandle effort_handle_l(grippers_state_interface.getHandle("gripper_l_joint"), &cmd[0]);
            grippers_effort_interface.registerHandle(effort_handle_l);
            hardware_interface::JointHandle effort_handle_r(grippers_state_interface.getHandle("gripper_r_joint"), &cmd[1]);
            grippers_effort_interface.registerHandle(effort_handle_r);
            registerInterface(&grippers_effort_interface);
        }

        ~YumiGrippersHW()
        { }



        void read(ros::Time time, ros::Duration period) 
        {
            // data_buffer_mutex.lock();
            float left, right;
            gripper_interface.getCurrentJointStates(left, right);

			//to mm
			left *= 1e-3;
			right *= 1e-3;

            pos[0] = left;
            pos[1] = right;

            // data_buffer_mutex.unlock();
        }


        void write(ros::Time time, ros::Duration period)
        {

            // enforceLimits(period);

			left_gripper_cmd = cmd[0];
			right_gripper_cmd = cmd[1];

            // ROS_INFO_STREAM("Gripper commands (left and right):");
            // ROS_INFO_STREAM(left_gripper_cmd);
            // ROS_INFO_STREAM(right_gripper_cmd);

            gripper_interface.setGripperEfforts((int)left_gripper_cmd, (int)right_gripper_cmd);

            // data_buffer_mutex.unlock();
        }


    private:
		ros::NodeHandle nh_;

		ros::Publisher gripper_status_publisher_;
		ros::ServiceServer request_grasp_;
		ros::ServiceServer request_release_;
		YumiGripperStateInterface gripper_interface;
		ros::Subscriber left_gripper_cmd_sub;
		ros::Subscriber right_gripper_cmd_sub;

		float left_gripper_cmd;
		float right_gripper_cmd;

		std::string gripper_state_topic, grasp_request_topic, grasp_release_topic, ip;
		int port_s, port_c;
		double js_rate;
		int default_force;
        boost::mutex data_buffer_mutex;
		ros::Timer heartbeat_;

        hardware_interface::JointStateInterface grippers_state_interface;
        hardware_interface::EffortJointInterface grippers_effort_interface;
        double cmd[2];
        double pos[2];
        double vel[2];
        double eff[2];


		void publishState(const ros::TimerEvent& event)
		{
			float left, right;
			gripper_interface.getCurrentJointStates(left, right);

			//to mm
			left *= 1e-3;
			right *= 1e-3;

			sensor_msgs::JointState js;
			js.header.stamp = ros::Time::now();
			js.name.push_back("gripper_l_joint");
			js.position.push_back(left);

			js.name.push_back("gripper_r_joint");
			js.position.push_back(right);
			
			gripper_status_publisher_.publish(js);

		}

		void leftGripperCmdCallback(const std_msgs::Float64::ConstPtr& msg)
		{
			ROS_INFO("Left gripper. I heard: [%f]", msg->data);

			left_gripper_cmd = msg -> data;
			gripper_interface.setGripperEfforts(left_gripper_cmd, right_gripper_cmd);
		}


		void rightGripperCmdCallback(const std_msgs::Float64::ConstPtr& msg)
		{
			ROS_INFO("Right gripper. I heard: [%f]", msg->data);

			right_gripper_cmd = msg -> data;
			gripper_interface.setGripperEfforts(left_gripper_cmd, right_gripper_cmd);
		}



};



int main( int argc, char** argv )
{
    signal(SIGTERM, quitRequested);
    signal(SIGINT, quitRequested);
    signal(SIGHUP, quitRequested);

    /* Init ROS node */
    ros::init(argc, argv, "grippers_hw_interface", ros::init_options::NoSigintHandler);

    /* ROS spinner */
    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::NodeHandle grippers_nh ("~");

    std::string name = "/yumi_grippers";

    YumiGrippersHW* yumi_grippers = new YumiGrippersHW(name);
    ROS_INFO("YummiGrippersHW interface is up!");

    controller_manager::ControllerManager manager(yumi_grippers);

    struct timespec ts = {0, 0};
    ros::Time last(ts.tv_sec, ts.tv_nsec), now(ts.tv_sec, ts.tv_nsec);
    ros::Duration period(1.0);

    ros::Duration(2.0).sleep();


    while( !g_quit )
    {
        // get the time / period
        if (!clock_gettime(CLOCK_MONOTONIC, &ts))
        {
            now = ros::Time::now();	
            now.sec = ts.tv_sec;
            now.nsec = ts.tv_nsec;
            period = now - last;
            last = now;
        } 
        else
        {
            ROS_FATAL("Failed to poll realtime clock!");
            break;
        }

        /* Read the state from YuMi */
        yumi_grippers -> read(now, period);
        
        /* Update the controllers */
        manager.update(now, period);

        /* Write the command to YuMi */
        yumi_grippers -> write(now, period);

    }


    return EXIT_SUCCESS;
}