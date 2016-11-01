#ifndef __YUMI_HW_RAPID_H 
#define __YUMI_HW_RAPID_H

#include "yumi_hw/yumi_hw.h"

#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include "simple_message/message_handler.h"
#include "simple_message/message_manager.h"
#include "simple_message/messages/joint_message.h"
#include "simple_message/smpl_msg_connection.h"
#include "simple_message/socket/tcp_socket.h"
#include "simple_message/socket/tcp_client.h"

#ifndef N_YUMI_JOINTS
#define N_YUMI_JOINTS 14
#endif

/**
  * Overrides message handler: keeps joint states thread-safe.
  */
class YumiJointStateHandler : public industrial::message_handler::MessageHandler {
    using industrial::message_handler::MessageHandler::init;
    
    private:
	float joint_positions[N_YUMI_JOINTS];
	float joint_command[N_YUMI_JOINTS];
	bool first_iteration;
	boost::mutex data_buffer_mutex, t_m;
	boost::condition_variable joint_state_received, joint_commands_set;
	bool b_joint_state_received, b_joint_commands_set;
	int mode;

    public:
	bool getJointStates(float (&jnts)[N_YUMI_JOINTS]) {
	    boost::mutex::scoped_lock lock(data_buffer_mutex);
	    while(!b_joint_state_received) {
		joint_state_received.wait(lock);
	    }
	    b_joint_state_received = false;
	    memcpy(&jnts,&joint_positions,sizeof(jnts));
	}

	bool setJointCommands(float (&jnts)[N_YUMI_JOINTS], int mode_) {
	    boost::mutex::scoped_lock lock(data_buffer_mutex);
	    memcpy(&joint_command,&jnts,sizeof(jnts));
	    b_joint_commands_set=true;
	    mode = mode_;
	    joint_commands_set.notify_all();
	}

	bool init(industrial::smpl_msg_connection::SmplMsgConnection* connection)
	{
	    first_iteration = true;
	    b_joint_state_received = false;
	    b_joint_commands_set = false;

	    return init((int)industrial::simple_message::StandardMsgTypes::JOINT, connection);
	}

    protected:

	
	bool internalCB(industrial::simple_message::SimpleMessage& in)
	{
	    boost::mutex::scoped_lock lock(data_buffer_mutex);
	    
	    //ROS_INFO("Received a message");
	    industrial::joint_message::JointMessage joint_msg;
	    bool rtn = true;

	    if (!joint_msg.init(in))
	    {
		ROS_ERROR("Failed to initialize joint message");
		return false;
	    }

	    //ROS_INFO("Message parsed");
	    industrial::shared_types::shared_real joint_value_from_msg;

	    for(int i=0; i<N_YUMI_JOINTS; i++) {
		//std::cerr<<i<<":";
		if (joint_msg.getJoints().getJoint(i, joint_value_from_msg))
		{
		    joint_positions[i] = joint_value_from_msg;
		    //std::cerr<<joint_positions[i]<<" ";
		}
		else
		{
		    //std::cerr<<"X";
		    rtn = false;
		}
	    }
	    //std::cerr<<"\n";

	    // Reply back to the controller if the sender requested it.
	    if (industrial::simple_message::CommTypes::SERVICE_REQUEST == joint_msg.getMessageType())
	    {
		//ROS_INFO("Reply requested, sending");
		industrial::simple_message::SimpleMessage reply;
		joint_msg.toReply(reply, rtn ? industrial::simple_message::ReplyTypes::SUCCESS : industrial::simple_message::ReplyTypes::FAILURE);
		this->getConnection()->sendMsg(reply);
	    }

	    b_joint_state_received=true;
	    joint_state_received.notify_all();

	    while(!b_joint_commands_set) {
		joint_commands_set.wait(lock);
	    }
	    b_joint_commands_set = false;

	    //if first call back, then mirror state to command
	    if(first_iteration) {
		ROS_INFO("Mirroring to command");
		memcpy(&joint_command,&joint_positions,sizeof(joint_command));
		first_iteration = false;	
	    }
	    
	    //TODO: format trajectory request message
	    industrial::shared_types::shared_real joint_value_to_msg;

	    for(int i=0; i<N_YUMI_JOINTS; i++) {
		joint_value_to_msg = joint_command[i];
//		std::cerr<<joint_command[i]<<" ";
		if (!joint_msg.getJoints().setJoint(i, joint_value_to_msg))
		{
		    rtn = false;
		}
	    }
	    if (!joint_msg.getJoints().setJoint(N_YUMI_JOINTS, mode))
	    {
		rtn = false;
	    }
//	    std::cerr<<"\n";

	    //TODO: send back on conncetion 
	    industrial::simple_message::SimpleMessage next_point;
	    joint_msg.toRequest(next_point);
	    this->getConnection()->sendMsg(next_point);

	    //ROS_INFO("Done processing");
	    
	    return rtn;
	}

};

/**
  * Keep a connection to the robot and send and receive joint states
  */
class YumiRapidInterface {
    private:
	boost::thread RapidCommThread_;
	
	///industrial connection
	industrial::tcp_client::TcpClient default_tcp_connection_; //?
	//industrial::tcp_client::RobotStatusRelayHandler default_robot_status_handler_; //?

	industrial::smpl_msg_connection::SmplMsgConnection* connection_;
	industrial::message_manager::MessageManager manager_;
	YumiJointStateHandler js_handler;

	bool stopComm_;

	virtual void RapidCommThreadCallback()
	{
	    while(!stopComm_) {
		manager_.spinOnce();
	    }
	    return;
	}

    public:
	YumiRapidInterface() { 
	    this->connection_ = NULL;
	    stopComm_ = true;
	}

	~YumiRapidInterface() { 
	    stopThreads();
	}
	
	void stopThreads() {
	    stopComm_ = true;
	    RapidCommThread_.join();
	}

	void startThreads() {
	    if(!stopComm_) {
		boost::thread(boost::bind(&YumiRapidInterface::RapidCommThreadCallback,this ));
	    }
	}

	void getCurrentJointStates(float (&joints)[N_YUMI_JOINTS]) {
	    js_handler.getJointStates(joints);	    
	}

	void setJointTargets(float (&joints)[N_YUMI_JOINTS], int mode) {
	    js_handler.setJointCommands(joints, mode);
	}


	bool init(std::string ip = "", int port = industrial::simple_socket::StandardSocketPorts::STATE) {
	    //initialize connection 
	    char* ip_addr = strdup(ip.c_str());  // connection.init() requires "char*", not "const char*"
	    ROS_INFO("Robot state connecting to IP address: '%s:%d'", ip_addr, port);
	    default_tcp_connection_.init(ip_addr, port);
	    free(ip_addr);

	    connection_ = &default_tcp_connection_;
	    connection_->makeConnect();

	    ROS_INFO("Connection established");
	    //initialize message manager
	    manager_.init(connection_);

	    //initialize message handler
	    js_handler.init(connection_);

	    //register handler to manager
	    manager_.add(&js_handler,false);

	    ROS_INFO("Callbacks and handlers set up");
	    stopComm_ = false;
	}
	
};

class YumiHWRapid : public YumiHW
{

public:
  YumiHWRapid() : YumiHW() {
      isInited = false;
      isSetup = false;
      firstRunInPositionMode = true;
      sampling_rate_ = 0.1;
  }
  
  ~YumiHWRapid() { 
      robot_interface.stopThreads();
  }

  float getSampleTime(){return sampling_rate_;};
	
  void setup(std::string ip_ = "", int port_ = industrial::simple_socket::StandardSocketPorts::STATE) {
      ip = ip_;
      port = port_;
      isSetup = true;
  }

  // Init, read, and write, with FRI hooks
  bool init()
  {
    if (isInited) return false;
    if(!isSetup) {
	ROS_ERROR("IP and port of controller are not set up!");
	return false;
    }

    robot_interface.init(ip,port);
    robot_interface.startThreads();
    isInited = true;

    return true;
  }

  ///copies the last received joint state out to the controller manager
  void read(ros::Time time, ros::Duration period)
  {
    if(!isInited) return;  

    //ROS_INFO("reading joints");
    data_buffer_mutex.lock();
    robot_interface.getCurrentJointStates(readJntPosition);

    for (int j = 0; j < n_joints_; j++)
    {
      joint_position_prev_[j] = joint_position_[j];
      joint_position_[j] = readJntPosition[j];
      //joint_effort_[j] = readJntEffort[j]; //TODO: read effort 
      joint_velocity_[j] = filters::exponentialSmoothing((joint_position_[j]-joint_position_prev_[j])/period.toSec(), joint_velocity_[j], 0.04); //exponential smoothing
      if(firstRunInPositionMode) {
	  joint_position_command_[j] = readJntPosition[j];
      }
    }
    firstRunInPositionMode = false;
    data_buffer_mutex.unlock();
    //ROS_INFO("read joints");

    return;
  }

  ///caches the most recent joint commands into the robot interface
  void write(ros::Time time, ros::Duration period)
  {
    if(!isInited) return;  
    enforceLimits(period);

    //ROS_INFO("writing joints");
    data_buffer_mutex.lock();
    switch (getControlStrategy())
    {
      case JOINT_POSITION:
        for (int j = 0; j < n_joints_; j++)
        {
          newJntPosition[j] = joint_position_command_[j];
        }
        break;
      
      case JOINT_VELOCITY:
	//std::cerr<<"ASS = ";
	for (int j = 0; j < n_joints_; j++)
	{
	  //std::cerr<<joint_velocity_command_[j]<<"*"<<period.toSec()<<" + "<<joint_position_[j]<<" ::: ";  
	  newJntPosition[j] = joint_velocity_command_[j]; //*period.toSec() + joint_position_[j]; 
	}
	//std::cerr<<std::endl;
	firstRunInPositionMode = true;
	break;
      //case JOINT_EFFORT:
      //break;
      default: 
	break;
    }

    robot_interface.setJointTargets(newJntPosition, getControlStrategy());
    data_buffer_mutex.unlock();
    //ROS_INFO("wrote joints");

    return;
  }

private:

  ///
  YumiRapidInterface robot_interface; 
  ///
  float last_comm[N_YUMI_JOINTS];
  ///
  std::string hintToRemoteHost_;
  ///
  bool isInited, isSetup, firstRunInPositionMode;
  ///
  float sampling_rate_;
  ///
  boost::mutex data_buffer_mutex;
  ///command buffers
  float newJntPosition[N_YUMI_JOINTS];
  ///data buffers
  float readJntPosition[N_YUMI_JOINTS];

  std::string ip;
  int port;
    

};

#endif
