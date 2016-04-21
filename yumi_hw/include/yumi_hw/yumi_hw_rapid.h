#ifndef __YUMI_HW_RAPID_H 
#define __YUMI_HW_RAPID_H

#include "yumi_hw/yumi_hw.h"
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include "simple_message/message_handler.h"
#include "simple_message/messages/joint_message.h"

#define N_YUMI_JOINTS 14

/**
  * Overrides message handler: keeps joint states thread-safe.
  */
class YumiJointStateHandler : public industrial::message_handler::MessageHandler {
    using industrial::message_handler::MessageHandler::init;
    
    private:
	float joint_positions[N_YUMI_JOINTS];
	boost::mutex data_buffer_mutex;

    public:
	void getJointStates(float &jnts[N_YUMI_JOINTS]) {
	    data_buffer_mutex.lock();
	    for(int i=0; i<N_YUMI_JOINTS; i++) {
		jnts[i] = joint_positions[i];
	    }
	    data_buffer_mutex.unlock();
	}
	bool init(SmplMsgConnection* connection)
	{
	    return init((int)StandardMsgTypes::JOINT, connection);
	}


    protected:
	bool internalCB(SimpleMessage& in)
	{
	    JointMessage joint_msg;
	    bool rtn = true;

	    if (!joint_msg.init(in))
	    {
		ROS_ERROR("Failed to initialize joint message");
		return false;
	    }

	    industrial::shared_types::shared_real joint_value_from_msg;

	    data_buffer_mutex.lock();
	    for(int i=0; i<N_YUMI_JOINTS; i++) {
		if (joint_msg.getJoints().getJoint(i, joint_value_from_msg))
		{
		    joint_positions[i] = joint_value_from_msg;
		}
		else
		{
		    rtn = false;
		}
	    }
	    data_buffer_mutex.unlock();

	    // Reply back to the controller if the sender requested it.
	    if (CommTypes::SERVICE_REQUEST == joint_msg.getMessageType())
	    {
		SimpleMessage reply;
		joint_msg.toReply(reply, rtn ? ReplyTypes::SUCCESS : ReplyTypes::FAILURE);
		this->getConnection()->sendMsg(reply);
	    }

	    //TODO: if first call back, then mirror state to command

	    //TODO: format trajectory request message

	    //TODO: send back on conncetion 
	    return rtn;
	}

};

/**
  * Keep a connection to the robot and send and receive joint states
  */
class YumiRapidInterface {
    private:
	boost::shared_ptr<std::thread> RapidCommThread_;
	bool stopComm_;

	void RapidCommThreadCallback()
	{
	    bool isFirstRun = true;
	    stopComm_ = false;

	    //TODO: initialize connection 
	    //TODO: initialize message manager
	    //TODO: initialize message handler
	    //TODO: register handler to manager
	    //TODO: while ok, do stuff

	    while(!stopComm_) {

#if 0
		data_buffer_mutex.lock();
		//here update read out values
		memcpy(readJntPosition, device_->getMsrMsrJntPosition(), 7*sizeof(float));
		memcpy(readJntEffort, device_->getMsrJntTrq(), 7*sizeof(float));
		device_->getCurrentCmdJntPosition(last_comm);
		//TODO? signal to main loop to do a control step?
		//here update new commands
		memcpy(locJntPosition, newJntPosition, 7*sizeof(float));
		memcpy(locJntStiff, newJntStiff, 7*sizeof(float));
		memcpy(locJntDamp, newJntDamp, 7*sizeof(float));
		memcpy(locJntAddTorque, newJntAddTorque, 7*sizeof(float));

		//first iteration, copy current positions to commands
		if(isFirstRun) {
		    memcpy(locJntPosition, readJntPosition, 7*sizeof(float));
		    isFirstRun=false;
		}
		data_buffer_mutex.unlock();
#endif
	    }
	    return;
	}
    public:
	YumiRapidInterface() { 
	    RapidCommThread_.reset( new std::thread( &YumiRapidInterface::RapidCommThreadCallback,this ) );
	}
	~YumiRapidInterface() { 
	    stopComm_ = true; 
	    RapidCommThread_.get()->join();
	}
	//TODO
	void stopThreads();
	void getCurrentJountStates(float &joints[N_YUMI_JOINTS]) const;
	void setJointTargets(const float &joints[N_YUMI_JOINTS]);
  
};

class YumiHWRapid : public YumiHW
{

public:
  YumiHWRapid() : YumiHW() {}
  
  ~YumiHWRapid() { 
  }

  float getSampleTime(){return sampling_rate_;};

  // Init, read, and write, with FRI hooks
  bool init(std::string robot_ip)
  {
    return true;
  }

  ///copies the last received joint state out to the controller manager
  void read(ros::Time time, ros::Duration period)
  {
    data_buffer_mutex.lock();
    for (int j = 0; j < n_joints_; j++)
    {
      joint_position_prev_[j] = joint_position_[j];
      joint_position_[j] = readJntPosition[j];
      //joint_effort_[j] = readJntEffort[j]; //TODO: read effort 
      joint_velocity_[j] = filters::exponentialSmoothing((joint_position_[j]-joint_position_prev_[j])/period.toSec(), joint_velocity_[j], 0.2); //exponential smoothing
    }
    data_buffer_mutex.unlock();
    return;
  }

  ///caches the most recent joint commands into the robot interface
  void write(ros::Time time, ros::Duration period)
  {
    enforceLimits(period);

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
	for (int j = 0; j < n_joints_; j++)
	{
	  newJntPosition[j] = joint_velocity_command_[j]*period.toSec();// + last_comm[j]; //FIXME: this should have the previous joint position in as well
	}
	break;
      //case JOINT_EFFORT:
      //break;
      default: 
	break;
    }
    data_buffer_mutex.unlock();
    return;
  }

private:

  ///
  YumiJointStateStreamer robot_interface; 
  ///
  float last_comm[N_YUMI_JOINTS];
  ///
  std::string hintToRemoteHost_;
  ///
  bool ip_set_ = false;
  ///
  float sampling_rate_;
  ///
  boost::mutex data_buffer_mutex;
  ///command buffers
  float newJntPosition[N_YUMI_JOINTS];
  ///data buffers
  float readJntPosition[N_YUMI_JOINTS];
    

};

#endif
