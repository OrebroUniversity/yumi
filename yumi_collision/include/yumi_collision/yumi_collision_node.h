#ifndef YUMI_COLLISION_NODE_H
#define YUMI_COLLISION_NODE_H

#include <ros/ros.h>
#include <gazebo_msgs/ContactsState.h>
#include <std_msgs/Empty.h>

class YumiCollisionNode
{
public:
	YumiCollisionNode() {

		ROS_INFO("YumiCollision: starting node");
		nh_ = ros::NodeHandle("~");
		n_ = ros::NodeHandle();

		sub1 = nh_.subscribe("/yumi/link_1_r/contact", 100,  &YumiCollisionNode::collisionCallback, this);
		sub2 = nh_.subscribe("/yumi/link_1_r/contact", 100,  &YumiCollisionNode::collisionCallback, this);
		sub3 = nh_.subscribe("/yumi/link_1_r/contact", 100,  &YumiCollisionNode::collisionCallback, this);
		sub4 = nh_.subscribe("/yumi/link_1_r/contact", 100,  &YumiCollisionNode::collisionCallback, this);
		sub5 = nh_.subscribe("/yumi/link_1_r/contact", 100,  &YumiCollisionNode::collisionCallback, this);
		sub6 = nh_.subscribe("/yumi/link_1_r/contact", 100,  &YumiCollisionNode::collisionCallback, this);
		sub7 = nh_.subscribe("/yumi/link_1_r/contact", 100,  &YumiCollisionNode::collisionCallback, this);
		sub8 = nh_.subscribe("/gripper_r/fl/contact", 100,  &YumiCollisionNode::collisionCallback, this);
		sub9 = nh_.subscribe("/gripper_r/fr/contact", 100,  &YumiCollisionNode::collisionCallback, this);

		collision_publisher_ = n_.advertise<std_msgs::Empty>("yumi/collision",10);
	}

	~YumiCollisionNode() { }

	void collisionCallback(const gazebo_msgs::ContactsState cs){

		if(!(cs.states.empty())){	

			collision_publisher_.publish(empty_msg_);
		}
	}

private:
	ros::NodeHandle nh_;
	ros::NodeHandle n_;

	std_msgs::Empty empty_msg_;

	ros::Publisher collision_publisher_;

	ros::Subscriber sub1;
	ros::Subscriber sub2;
	ros::Subscriber sub3;
	ros::Subscriber sub4;
	ros::Subscriber sub5;
	ros::Subscriber sub6;
	ros::Subscriber sub7;
	ros::Subscriber sub8;
	ros::Subscriber sub9;
	//callbacks
};



#endif
