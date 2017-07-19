#!/usr/bin/env python

import roslib
import rospy
from std_msgs.msg import Float64, Empty
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryActionResult
from yumi_hw.srv import YumiGrasp
import threading


grippers_indices = [0, 1]
left_arm_indices = [0, 2, 4, 6, 8, 10, 12, 14]
right_arm_indices = [1, 3, 5, 7, 9, 11, 13, 15]
right_arm_pub = None
left_arm_pub = None

grippers_state_pub = None
gripper_r_cmd_subs = None
gripper_l_cmd_subs = None
gripper_r_cmd_pub = None
gripper_l_cmd_pub = None
gripper_dograsp_service = None
gripper_releasegrasp_service = None

gripper_srv_force = 5.0



def joint_states_callback(data):
    global right_arm_pub, left_arm_pub, grippers_indices, left_arm_indices, right_arm_indices
    
    left_arm_msg = JointState()
    left_arm_msg.header.stamp = rospy.Time.now()
    left_arm_msg.header.frame_id = '/world'
    left_arm_msg.name = [data.name[i] for i in left_arm_indices]
    left_arm_msg.position = [data.position[i] for i in left_arm_indices]
    left_arm_msg.velocity = [data.velocity[i] for i in left_arm_indices]
    left_arm_msg.effort = [data.effort[i] for i in left_arm_indices]
    left_arm_pub.publish(left_arm_msg)

    right_arm_msg = JointState()
    right_arm_msg.header.stamp = rospy.Time.now()
    right_arm_msg.header.frame_id = '/world'
    right_arm_msg.name = [data.name[i] for i in right_arm_indices]
    right_arm_msg.position = [data.position[i] for i in right_arm_indices]
    right_arm_msg.velocity = [data.velocity[i] for i in right_arm_indices]
    right_arm_msg.effort = [data.effort[i] for i in right_arm_indices]
    right_arm_pub.publish(right_arm_msg)

    grippers_state_msg = JointState()
    grippers_state_msg.header.stamp = rospy.Time.now()
    grippers_state_msg.header.frame_id = '/world'
    grippers_state_msg.name = [data.name[i] for i in grippers_indices]
    grippers_state_msg.position = [data.position[i] for i in grippers_indices]
    grippers_state_pub.publish(grippers_state_msg)


def gripper_l_cmd_callback(data):
    global gripper_l_cmd_pub
    gripper_l_cmd_msg = -data
    gripper_l_cmd_pub.publish(gripper_l_cmd_msg)


def gripper_r_cmd_callback(data):
    global gripper_r_cmd_pub
    gripper_r_cmd_msg = -data
    gripper_r_cmd_pub.publish(gripper_r_cmd_msg)


def request_grasp_handle(req):
    global gripper_l_cmd_pub, gripper_r_cmd_pub
    if (req.gripper_id == req.LEFT_GRIPPER):
        gripper_l_cmd_msg = -gripper_srv_force
        gripper_l_cmd_pub.publish(gripper_l_cmd_msg)
        
    elif (req.gripper_id == req.RIGHT_GRIPPER):
        gripper_r_cmd_msg = -gripper_srv_force
        gripper_r_cmd_pub.publish(gripper_r_cmd_msg)
        
    return []


def request_release_handle(req):
    global gripper_l_cmd_pub, gripper_r_cmd_pub
    if (req.gripper_id == req.LEFT_GRIPPER):
        gripper_l_cmd_msg = gripper_srv_force
        gripper_l_cmd_pub.publish(gripper_l_cmd_msg)
        
    elif (req.gripper_id == req.RIGHT_GRIPPER):
        gripper_r_cmd_msg = gripper_srv_force
        gripper_r_cmd_pub.publish(gripper_r_cmd_msg)
        
    return []



def listener():
    global right_arm_pub, left_arm_pub, grippers_state_pub, gripper_l_cmd_subs, gripper_l_cmd_pub, gripper_r_cmd_subs, gripper_r_cmd_pub, gripper_dograsp_service
    rospy.init_node('gazebo_joints_remap', anonymous=False)
    rospy.Subscriber("/yumi/joint_states", JointState, joint_states_callback)
    left_arm_pub = rospy.Publisher("/joint_states", JointState, queue_size=1)
    right_arm_pub = rospy.Publisher("/joint_states", JointState, queue_size=1)

    grippers_state_pub = rospy.Publisher("/yumi/gripper_states", JointState, queue_size=1)
    gripper_l_cmd_pub = rospy.Publisher("/yumi/gripper_effort_controller_l/command", Float64, queue_size=1)
    gripper_r_cmd_pub = rospy.Publisher("/yumi/gripper_effort_controller_r/command", Float64, queue_size=1)

    gripper_l_cmd_subs = rospy.Subscriber("/yumi/gripper_l_effort_cmd", Float64, gripper_l_cmd_callback)
    gripper_r_cmd_subs = rospy.Subscriber("/yumi/gripper_r_effort_cmd", Float64, gripper_r_cmd_callback)

    gripper_dograsp_service = rospy.Service('/yumi/yumi_gripper/do_grasp', YumiGrasp, request_grasp_handle)
    gripper_releasegrasp_service = rospy.Service('/yumi/yumi_gripper/release_grasp', YumiGrasp, request_release_handle)

    rospy.spin()


if __name__ == '__main__':
    listener()
