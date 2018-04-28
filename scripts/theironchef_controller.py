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

		self.reset_pub = rospy.Publisher('/TheIronChef/Reset', Empty, queue_size=10)
		self.home_pub = rospy.Publisher('/TheIronChef/Home', Empty, queue_size=10)
		self.home_arm_pub = rospy.Publisher('/TheIronChef/HomeArm', Empty, queue_size=10)
		self.home_gantry_pub = rospy.Publisher('/TheIronChef/HomeGantry', Empty, queue_size=10)
		self.home_x_gantry_pub = rospy.Publisher('/TheIronChef/HomeXGantry', Empty, queue_size=10)
		self.home_y_gantry_pub = rospy.Publisher('/TheIronChef/HomeYGantry', Empty, queue_size=10)
		self.home_z_gantry_pub = rospy.Publisher('/TheIronChef/HomeZGantry', Empty, queue_size=10)
		self.move_gantry_pub = rospy.Publisher('/TheIronChef/move_gantry', Point, queue_size=10);
		self.move_arm_pub = rospy.Publisher('/TheIronChef/move_arm', Point, queue_size=10)
		self.electromagnet_pub = rospy.Publisher('/TheIronChef/Electromagnet_Switch', Bool, queue_size=10)

		rospy.sleep(0.1)

	def reset_robot(self):
		empty_msg = Empty()

		self.reset_pub.publish(empty_msg)
		self.reset_pub.publish(empty_msg)

	def home_arm(self):
		empty_msg = Empty()

		self.home_arm_pub.publish(empty_msg)
		self.home_arm_pub.publish(empty_msg)

		try:
			done_homing_msg = rospy.wait_for_message('/TheIronChef/done_homing', Bool)
			return done_homing_msg.data
		except:
			return False

	def home_gantry(self):
		empty_msg = Empty()

		self.home_gantry_pub.publish(empty_msg)
		self.home_gantry_pub.publish(empty_msg)

		try:
			done_homing_msg = rospy.wait_for_message('/TheIronChef/done_homing', Bool)
			return done_homing_msg.data
		except:
			return False

	def home_robot(self):
		empty_msg = Empty()

		self.home_pub.publish(empty_msg)
		self.home_pub.publish(empty_msg)

		try:
			done_homing_msg = rospy.wait_for_message('/TheIronChef/done_homing', Bool)
			return done_homing_msg.data
		except:
			return False

	def home_x_gantry(self):
		empty_msg = Empty()

		self.home_x_gantry_pub.publish(empty_msg)
		self.home_x_gantry_pub.publish(empty_msg)

		try:
			done_homing_msg = rospy.wait_for_message('/TheIronChef/done_homing', Bool)
			return done_homing_msg.data
		except:
			return False

	def home_y_gantry(self):
		empty_msg = Empty()

		self.home_y_gantry_pub.publish(empty_msg)
		self.home_y_gantry_pub.publish(empty_msg)

		try:
			done_homing_msg = rospy.wait_for_message('/TheIronChef/done_homing', Bool)
			return done_homing_msg.data
		except: 
			return False

	def home_z_gantry(self):
		empty_msg = Empty()

		self.home_z_gantry_pub.publish(empty_msg)
		self.home_z_gantry_pub.publish(empty_msg)

		try:
			done_homing_msg = rospy.wait_for_message('/TheIronChef/done_homing', Bool)
			return done_homing_msg.data
		except:
			return False

	def move_gantry(self, desired_gantry_position):
		point_msg = Point()
		point_msg.x = desired_gantry_position[0]
		point_msg.y = desired_gantry_position[1]
		point_msg.z = desired_gantry_position[2]

		self.move_gantry_pub.publish(point_msg)
		self.move_gantry_pub.publish(point_msg)

		try:
			done_moving_gantry_msg = rospy.wait_for_message('/TheIronChef/done_moving_gantry', Bool)
			return done_moving_gantry_msg.data
		except:
			return False

	def move_arm(self, desired_arm_angles):
		point_msg = Point()
		point_msg.x = desired_arm_angles[0]
		point_msg.y = desired_arm_angles[1]
		point_msg.z = desired_arm_angles[2]

		self.move_arm_pub.publish(point_msg)
		self.move_arm_pub.publish(point_msg)

		try:
			done_moving_arm_msg = rospy.wait_for_message('TheIronChef/done_moving_arm', Bool)
			return done_moving_arm_msg.data
		except:
			return False

	def get_current_position(self):
		try:
			joint_state_msg = rospy.wait_for_message('/TheIronChef/joint_states', JointState, 10)
			return [joint_state_msg.position[0], joint_state_msg.position[1], joint_state_msg.position[2],
					joint_state_msg.position[3], joint_state_msg.position[4], joint_state_msg.position[5]]
		except:
			return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

	def get_current_gantry_position(self):
		try:
			joint_state_msg = rospy.wait_for_message('/TheIronChef/joint_states', JointState, 10)
			return [joint_state_msg.position[0], joint_state_msg.position[1], joint_state_msg.position[2]]
		except:
			return [0.0, 0.0, 0.0]

	def get_current_arm_angles(self):
		try:
			joint_state_msg = rospy.wait_for_message('/TheIronChef/joint_states', JointState, 10)
			return [joint_state_msg.position[3], joint_state_msg.position[4], joint_state_msg.position[5]]
		except:
			return [0.0, 0.0, 0.0]

	def electromagnet_switch(self, switch):
		bool_msg = Bool()
		if(switch == "on"):
			bool_msg.data = True
		else:
			bool_msg.data = False

		self.electromagnet_pub.publish(bool_msg)
		self.electromagnet_pub.publish(bool_msg)

		try:
			electromagnet_status_msg = rospy.wait_for_message('/TheIronChef/electromagnet_status', Bool, 3)
			return electromagnet_status_msg.data
		except:
			return bool_msg.data