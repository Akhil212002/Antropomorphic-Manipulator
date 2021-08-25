#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg._JointTrajectoryPoint import JointTrajectoryPoint
import time

def moveright():
	move_pub = rospy.Publisher('/right_arm_controller/command', JointTrajectory, queue_size=1, latch=True)
	rospy.init_node('arm_move', anonymous=True)
	rate = rospy.Rate(10)
	arm_cmd = JointTrajectory()

	arm_cmd.joint_names = ["right_clavicle_joint","right_shoulder_joint","right_elbow","right_grip_x_joint","right_grip_y_joint","right_grip_z_joint","right_effector"]

	joint_point = JointTrajectoryPoint()
	joint_point.positions = [-1.86, 1.07, -0.55, -1.43, -0.39, 0.00, 0]
	joint_point.velocities = [0.5, 0.5, 0.5, 0.5,0.5,0.5,0.5]
	joint_point.accelerations = [0.5, 0.5, 0.5, 0.5,0.5, 0.5, 0.5]
	joint_point.effort = []
	joint_point.time_from_start = rospy.rostime.Duration(secs=1, nsecs=0)
	arm_cmd.points = [joint_point]

	move_pub.publish(arm_cmd)
	print('Right Message Published!!!')
	rate.sleep()


def moveleft():
	move_pub = rospy.Publisher('/left_arm_controller/command', JointTrajectory, queue_size=1, latch=True)
	rospy.init_node('arm_move', anonymous=True)
	rate = rospy.Rate(10)
	arm_cmd = JointTrajectory()

	arm_cmd.joint_names = ["left_clavicle_joint","left_shoulder_joint","left_elbow","left_grip_x_joint","left_grip_y_joint","left_grip_z_joint","left_effector"]

	joint_point = JointTrajectoryPoint()
	joint_point.positions = [-1.75, 3.14,-0.55, 0.16, -1.62, 2.44, 0.60]
	joint_point.velocities = [0.5, 0.5, 0.5, 0.5,0.5,0.5,0.5]
	joint_point.accelerations = [0.5, 0.5, 0.5, 0.5,0.5,0.5,0.5]
	joint_point.effort = []
	joint_point.time_from_start = rospy.rostime.Duration(secs=1, nsecs=0)
	arm_cmd.points = [joint_point]

	move_pub.publish(arm_cmd)
	print('Left Message Published!!!')
	rate.sleep()

if __name__ == '__main__':
	try:
		moveright()
		moveleft()
	except rospy.ROSInterruptException:
		pass
