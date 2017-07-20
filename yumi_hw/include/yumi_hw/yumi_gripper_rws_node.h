#ifndef YUMI_GRIPPER_NODE_H
#define YUMI_GRIPPER_NODE_H

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <abb_rws_interface/rws_interface_yumi.h>

class YumiGripperRWSNode
{
    public:
		YumiGripperRWSNode(): closed_distance_(0) 
		{

			ROS_INFO("YumiGrippers: starting node");
			nh_ = ros::NodeHandle("~");

			//read in parameters
			nh_.param("ip", ip, std::string("192.168.125.1") );
			nh_.param("port", port, std::string("80") );
			nh_.param<std::string>("joint_state_topic", gripper_state_topic,"gripper_states");

			request_grasp_left_ = nh_.advertiseService("grasp_left", &YumiGripperRWSNode::grasp_left, this);
			request_release_left_ = nh_.advertiseService("release_left", &YumiGripperRWSNode::release_left, this);;
			gripper_status_publisher_ = ros::NodeHandle().advertise<sensor_msgs::JointState>(gripper_state_topic, 10);
		        left_side = abb::rws_interface::LEFT_SIDE;	
		        right_side = abb::rws_interface::RIGHT_SIDE;	
			initRWS();

		}

		virtual ~YumiGripperRWSNode() 
		{
		}

		void publish() {
		    abb::rws_interface::DualSGData sgdata;
		    rws_interface_->getData(&sgdata);

		    //get gripper status from rws and publish it
		    left_gripper_ = sgdata.left.getTargetPosition()*1e-3;
		    right_gripper_ = sgdata.right.getTargetPosition()*1e-3;

		    sensor_msgs::JointState js;
		    js.header.stamp = ros::Time::now();
		    js.name.push_back("gripper_l_joint");
		    js.position.push_back(left_gripper_);

		    js.name.push_back("gripper_r_joint");
		    js.position.push_back(right_gripper_);

		    gripper_status_publisher_.publish(js);
		}


		float left_gripper_;
		float right_gripper_;



    private:
		ros::NodeHandle nh_;

		ros::Publisher gripper_status_publisher_;
		ros::ServiceServer request_grasp_left_;
		ros::ServiceServer request_release_left_;
		
		abb::rws_interface::Side left_side, right_side;
		std::string gripper_state_topic, ip, port;
		float closed_distance_;
		/* RWS */
		// RWS interface which uses TCP communication for starting the EGM joint mode on YuMi
		boost::shared_ptr<abb::rws_interface::RWSInterfaceYuMi> rws_interface_;

		bool grasp_left(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
		{
		    rws_interface_->doSGGrip(left_side);
		    return true;
		}
		bool release_left(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
		{
		    rws_interface_->doSGMoveTo(0, false, left_side);
		    return true;
		}
	
		bool initRWS() {
		    ROS_INFO_STREAM(ros::this_node::getName() << ": starting RWS connection with IP & PORT: " << ip << " / " << port);

		    rws_interface_.reset(new abb::rws_interface::RWSInterfaceYuMi(ip, port));
		    ros::Duration(1.0).sleep();

		    // Check that RAPID is running on the robot and that robot is in AUTO mode
		    if(!rws_interface_->isRAPIDRunning())
		    {
			ROS_ERROR_STREAM(ros::this_node::getName() << ": robot unavailable, make sure that the RAPID program is running on the flexpendant.");
			return false;
		    }

		    ros::Duration(1.0).sleep();

		    if(!rws_interface_->isModeAuto())
		    {
			ROS_ERROR_STREAM(ros::this_node::getName() << ": robot unavailable, make sure to set the robot to AUTO mode on the flexpendant.");
			return false;
		    }

		    ros::Duration(1.0).sleep();
		    rws_interface_->doSGInitialize();
		    ros::Duration(1.0).sleep();
		    rws_interface_->doSGCalibrate();
		    ros::Duration(1.0).sleep();

		}
};



#endif
