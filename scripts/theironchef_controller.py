#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Empty, String, Bool
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState

class TheIronChefController:
	def __init__(self):
		rospy.init_node('TheIronChefController')
		self.current_gantry_position = [0.0, 0.0, 0.0]
		self.current_arm_position = [0.0, 0.0, 0.0]

		self.reset_pub = rospy.Publisher('/TheIronChef/Reset', Empty, queue_size=1)
		self.home_pub = rospy.Publisher('/TheIronChef/Home', Empty, queue_size=1)
		self.move_gantry_pub = rospy.Publisher('/TheIronChef/move_gantry', Point, queue_size=10);
		self.move_arm_pub = rospy.Publisher('/TheIronChef/move_arm', Point, queue_size=10)
		self.electromagnet_pub = rospy.Publisher('/TheIronChef/Electromagnet_Switch', Bool, queue_size=1)

		rospy.sleep(0.1)

	def reset_robot(self):
		empty_msg = Empty()
		self.reset_pub.publish(empty_msg)
		self.reset_pub.publish(empty_msg)

	def home_arm(self):
		return self.move_arm([90, 6, 70])

	def home_robot(self):
		empty_msg = Empty()
		self.home_pub.publish(empty_msg)
		self.home_pub.publish(empty_msg)
		done_homing_msg = rospy.wait_for_message('/TheIronChef/done_homing', Bool)
		return done_homing_msg.data && self.home_arm()

	def move_gantry(self, desired_gantry_position):
		point_msg = Point()
		point_msg.x = desired_gantry_position[0]
		point_msg.y = desired_gantry_position[1]
		point_msg.z = desired_gantry_position[2]
		self.move_gantry_pub.publish(point_msg)
		self.move_gantry_pub.publish(point_msg)
		done_moving_gantry_msg = rospy.wait_for_message('/TheIronChef/done_moving_gantry', Bool)
		return done_moving_gantry_msg.data

	def move_arm(self, desired_arm_angles):
		point_msg.x = desired_arm_angles[0]
		point_msg.y = desired_arm_angles[1]
		point_msg.z = desired_arm_angles[2]
		self.move_arm_pub.publish(point_msg)
		self.move_arm_pub.publish(point_msg)
		done_moving_arm_msg = rospy.wait_for_message('TheIronChef/done_moving_arm', Bool)
		return done_moving_arm_msg.data

	def get_current_position(self):
		joint_state_msg = rospy.wait_for_message('/TheIronChef/joint_states', JointState)
		return [joint_state_msg.position[0], joint_state_msg.position[1], joint_state_msg.position[2],
				joint_state_msg.position[3], joint_state_msg.position[4], joint_state_msg.position[5]]

	def electromagnet_switch(self, switch):
		bool_msg = Bool()
		if(switch == "on"):
			bool_msg.data = True
		else:
			bool_msg.data = False
		self.electromagnet_pub.publish(bool_msg)
		self.electromagnet_pub.publish(bool_msg)
		electromagnet_status_msg = rospy.wait_for_message('/TheIronChef/electromagnet_status', Bool)
		return electromagnet_status_msg.data