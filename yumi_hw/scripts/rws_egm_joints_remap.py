#!/usr/bin/env python

import roslib
import rospy
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryActionResult
import threading


grippers_indices = [14, 15]
left_arm_indices = [0, 2, 4, 6, 8, 10, 12]
right_arm_indices = [1, 3, 5, 7, 9, 11, 13]
right_arm_pub = None
left_arm_pub = None



def callback(data):
    global right_arm_pub, left_arm_pub, grippers_indices, left_arm_indices, right_arm_indices
    '''print "I hear: ", data
    print "Grippers: ", [data.name[i] for i in grippers_indices]
    print "Left arm joints: ", [data.name[i] for i in left_arm_indices]
    print "Right arm joints: ", [data.name[i] for i in right_arm_indices]'''
    if(len(data.name) > 2):
        left_arm_msg = JointState()
        # left_arm_msg.header.stamp = rospy.Time.now()
        left_arm_msg.header = data.header
        left_arm_msg.header.frame_id = '/world'
        left_arm_msg.name = [data.name[i] for i in left_arm_indices]
        left_arm_msg.position = [data.position[i] for i in left_arm_indices]
        left_arm_msg.velocity = [data.velocity[i] for i in left_arm_indices]
        left_arm_msg.effort = [data.effort[i] for i in left_arm_indices]
        left_arm_pub.publish(left_arm_msg)

        right_arm_msg = JointState()
        # right_arm_msg.header.stamp = rospy.Time.now()
        right_arm_msg.header = data.header
        right_arm_msg.header.frame_id = '/world'
        right_arm_msg.name = [data.name[i] for i in right_arm_indices]
        right_arm_msg.position = [data.position[i] for i in right_arm_indices]
        right_arm_msg.velocity = [data.velocity[i] for i in right_arm_indices]
        right_arm_msg.effort = [data.effort[i] for i in right_arm_indices]
        right_arm_pub.publish(right_arm_msg)




def listener():
    global right_arm_pub, left_arm_pub
    rospy.init_node('egm_joints_remap', anonymous=False)
    
    startup_time = rospy.get_param('startup_time', 5)
    rospy.loginfo("EGM joint remapper will sleep for %s seconds", str(startup_time))
    rospy.sleep(startup_time)
    rospy.loginfo("\x1b[6;30;42m" + "RWS/EGM joint remapper started. Now you can start planning with MoveIt!" + "\x1b[0m")

    rospy.Subscriber("/yumi/joint_states", JointState, callback)
    left_arm_pub = rospy.Publisher("/joint_states", JointState, queue_size=1)
    right_arm_pub = rospy.Publisher("/joint_states", JointState, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    listener()