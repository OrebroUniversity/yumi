#ifndef YUMI_GRIPPER_NODE_H
#define YUMI_GRIPPER_NODE_H

#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include "simple_message/message_handler.h"
#include "simple_message/message_manager.h"
#include "simple_message/messages/joint_message.h"
#include "simple_message/smpl_msg_connection.h"
#include "simple_message/socket/tcp_socket.h"
#include "simple_message/socket/tcp_client.h"
#include <sensor_msgs/JointState.h>

#include <yumi_hw/YumiGrasp.h>

#define MSG_TYPE_GRIPPER_COMMAND 8008
#define MSG_TYPE_GRIPPER_STATE 8009

#define DEFAULT_STATE_PORT 12002
#define DEFAULT_COMMAND_PORT 12000

#define LEFT_GRIPPER 1
#define RIGHT_GRIPPER 2

/**
  * Overrides message handler: keeps joint states thread-safe.
  */
class YumiGripperStateHandler : public industrial::message_handler::MessageHandler {
    using industrial::message_handler::MessageHandler::init;
    
    private:
	float gripper_positions[2];
	boost::mutex data_buffer_mutex;

    public:
	bool getGripperStates(float &left, float &right) { 
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
	    if(in.getMessageType() != MSG_TYPE_GRIPPER_STATE) {
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
	    while(!stopComm_) {
		manager_.spinOnce();
	    }
	    return;
	}

    public:
	YumiGripperStateInterface() { 
	    this->connection_ = NULL;
	    stopComm_ = true;
	}

	~YumiGripperStateInterface() { 
	    stopThreads();
	}
	
	void stopThreads() {
	    stopComm_ = true;
	    RapidCommThread_.join();
	}

	void startThreads() {
	    if(!stopComm_) {
		boost::thread(boost::bind(&YumiGripperStateInterface::RapidCommThreadCallback,this ));
	    }
	}

	void getCurrentJointStates(float &left, float &right) {
	    gripper_handler.getGripperStates(right,left);	    
	}

	void setGripperEfforts(float left, float right) {

	    industrial::simple_message::SimpleMessage grasp_msg;
	    industrial::byte_array::ByteArray data;
	    industrial::shared_types::shared_real value_to_msg;
	   
	    value_to_msg = left;
	    data.load(value_to_msg);
	   
	    value_to_msg = right;
	    data.load(value_to_msg);
	   
	    grasp_msg.init(MSG_TYPE_GRIPPER_COMMAND, industrial::simple_message::CommTypes::TOPIC, 
		    industrial::simple_message::ReplyTypes::INVALID, data) ;
	    connection_command->sendMsg(grasp_msg);

	}

	bool init(std::string ip = "", int port = DEFAULT_STATE_PORT, int port_command = DEFAULT_COMMAND_PORT) {
	    //initialize connection 
	    char* ip_addr = strdup(ip.c_str());  // connection.init() requires "char*", not "const char*"
	    ROS_INFO("Robot state connecting to IP address: '%s:%d'", ip_addr, port);
	    default_tcp_connection_.init(ip_addr, port);
	    default_tcp_connection_command.init(ip_addr, port_command);
	    free(ip_addr);

	    connection_ = &default_tcp_connection_;
	    connection_->makeConnect();

	    connection_command = &default_tcp_connection_command;
	    connection_command->makeConnect();

	    //initialize message manager
	    manager_.init(connection_);

	    //initialize message handler
	    gripper_handler.init(connection_);

	    //register handler to manager
	    manager_.add(&gripper_handler,false);

	    stopComm_ = false;
	}
	
};

class YumiGripperNode
{
    public:
	YumiGripperNode() {

	    ROS_INFO("YumiGrippers: starting node");
	    nh_ = ros::NodeHandle("~");

	    //read in parameters
	    nh_.param<std::string>("joint_state_topic", gripper_state_topic,"joint_states");
	    nh_.param<std::string>("grasp_request_topic", grasp_request_topic,"do_grasp");
	    nh_.param<std::string>("grasp_release_topic", grasp_release_topic,"release_grasp");
	    nh_.param<double>("publish_period", js_rate, 0.1);
	    nh_.param("port_stream", port_s, DEFAULT_STATE_PORT);
	    nh_.param("port_command", port_c,DEFAULT_COMMAND_PORT);
	    nh_.param("grasp_force", default_force, 5);
	    nh_.param("ip", ip, std::string("192.168.125.1") );

	    heartbeat_ = nh_.createTimer(ros::Duration(js_rate), &YumiGripperNode::publishState, this);
	    request_grasp_ = nh_.advertiseService(grasp_request_topic, &YumiGripperNode::request_grasp, this);;
	    request_release_ = nh_.advertiseService(grasp_release_topic, &YumiGripperNode::request_release, this);;
	    gripper_status_publisher_ = ros::NodeHandle().advertise<sensor_msgs::JointState>(gripper_state_topic, 10); 

	    gripper_interface.init(ip,port_s);
	    gripper_interface.startThreads();


	}

	virtual ~YumiGripperNode() {

	}

    private:
	ros::NodeHandle nh_;

	ros::Publisher gripper_status_publisher_;
	ros::ServiceServer request_grasp_;
	ros::ServiceServer request_release_;
	YumiGripperStateInterface gripper_interface;

	std::string gripper_state_topic, grasp_request_topic, grasp_release_topic, ip;
	int port_s, port_c;
	double js_rate;
	int default_force;

	ros::Timer heartbeat_;

	bool request_grasp(yumi_hw::YumiGrasp::Request  &req,
		yumi_hw::YumiGrasp::Response &res ) {

	    float left=0, right=0;
	    if(req.gripper_id == LEFT_GRIPPER) {
		left = default_force;
	    }
	    if(req.gripper_id == RIGHT_GRIPPER) {
		right = default_force;
	    }
	    gripper_interface.setGripperEfforts(left,right);
	    return true;
	}
	
	bool request_release(yumi_hw::YumiGrasp::Request  &req,
		yumi_hw::YumiGrasp::Response &res ) {

	    float left=0, right=0;
	    if(req.gripper_id == LEFT_GRIPPER) {
		left = -default_force;
	    }
	    if(req.gripper_id == RIGHT_GRIPPER) {
		right = -default_force;
	    }
	    gripper_interface.setGripperEfforts(left,right);
	    return true;
	}

	//callbacks
	void publishState(const ros::TimerEvent& event) {

	    float left, right;
	    gripper_interface.getCurrentJointStates(left,right);

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
};



#endif
