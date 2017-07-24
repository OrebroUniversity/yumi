#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


print("============ Starting tutorial setup")
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()


print("============ Setting move groups ============")
group_left = moveit_commander.MoveGroupCommander("left_arm")
# group_left.set_planner_id("ESTkConfigDefault")

group_right = moveit_commander.MoveGroupCommander("right_arm")

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory, queue_size=10)


print("============ Left arm reference frame: %s ============" % group_left.get_planning_frame())
print("============ Left arm end effector link: %s ============" % group_left.get_end_effector_link())

print("============ Robot Groups: ============")
print(robot.get_group_names())

print("============ Printing robot state ============")
print(robot.get_current_state())
print("============")


print("============ Generating plan_left ============")
group_variable_values = group_left.get_current_joint_values()
group_variable_values[0] = 1.0
group_left.set_joint_value_target(group_variable_values)
plan_left = group_left.plan()

print("============ Waiting while RVIZ displays plan_left... ============")
rospy.sleep(10)

print("============ Visualizing plan_left ============")
display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan_left)
display_trajectory_publisher.publish(display_trajectory)

print("============ Waiting while plan_left is visualized (again)... ============")
rospy.sleep(5)

group_left.go(wait=True)





print("============ Generating plan_left ============")
group_variable_values = group_right.get_current_joint_values()

group_variable_values[0] = 1.0
group_right.set_joint_value_target(group_variable_values)

plan_right = group_right.plan()

print("============ Waiting while RVIZ displays plan_left... ============")
rospy.sleep(10)


print("============ Visualizing plan_left ============")
display_trajectory = moveit_msgs.msg.DisplayTrajectory()

display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan_right)
display_trajectory_publisher.publish(display_trajectory)

print("============ Waiting while plan_left is visualized (again)... ============")
rospy.sleep(5)

group_right.go(wait=True)






moveit_commander.roscpp_shutdown()