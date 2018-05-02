#!/usr/bin/env python

import time
import rospy
import math
import numpy
import argparse
from theironchef_controller import TheIronChefController
import theironchef_stations as tic_stations
#from theironchef_ik import TheIronChefIK
#from theironchef_cv import TheIronChefCV

#All measurements in meters and degrees
    
class TheIronChefPlanner:

    def __init__(self):
        self.tic_controller = TheIronChefController()
        #self.tic_ik = TheIronChefIK()
        #self.tic_cv = TheIronChefCV()

        self.done_moving_gantry_flag = False
        self.done_moving_arm_flag = False
        self.done_homing_robot_flag = False
        self.done_homing_arm_flag = False
        self.done_homing_gantry_flag = False
        self.electromagnet_status = False

    def reset_robot(self):
        print("Resetting robot")
        self.tic_controller.reset_robot()
        print("Reset robot")

    def home_robot(self):
        print("Homing Robot")

        self.done_homing_robot_flag = self.tic_controller.home_robot()
        while(self.done_homing_robot_flag != True):
           self.done_homing_robot_flag = self.tic_controller.home_robot() 

        print("Done Homing Robot")

    def home_arm(self):
        print("Homing Arm")

        self.done_homing_arm_flag = self.tic_controller.home_arm()
        while(self.done_homing_arm_flag != True):
            self.done_homing_arm_flag = self.tic_controller.home_arm()

        print("Done Homing Arm")

    def home_gantry(self):
        print("Homing Gantry")

        self.done_homing_gantry_flag = self.tic_controller.home_gantry()
        while(self.done_homing_gantry_flag != True):
            self.done_homing_gantry_flag = self.tic_controller.home_gantry()

        print("Done Homing Gantry")

    def move_gantry(self, desired_gantry_coords):
        self.print_current_gantry_position()

        print("Desired Gantry Position: [" + str(desired_gantry_coords[0]) + ", " + str(desired_gantry_coords[1]) + ", " + str(desired_gantry_coords[2]) + "]")

        self.done_moving_gantry_flag = self.tic_controller.move_gantry(desired_gantry_coords)
        while(self.done_moving_gantry_flag != True):
            self.done_moving_gantry_flag = self.tic_controller.move_gantry(desired_gantry_coords)

        self.print_current_gantry_position()

    def move_x_gantry_to_center(self):
        self.move_gantry([0.5, 0.0, 0.0])

    def move_arm(self, desired_arm_angles):
        self.print_current_arm_angles()

        print("Desired Arm Angles: [" + str(desired_arm_angles[0]) + ", " + str(desired_arm_angles[1]) + ", " + str(desired_arm_angles[2]) + "]")

        self.done_moving_arm_flag = self.tic_controller.move_arm(desired_arm_angles)
        while(self.done_moving_arm_flag != True):
            self.done_moving_arm_flag = self.tic_controller.move_arm(desired_arm_angles)

        self.print_current_arm_angles()

    def dispense_syringe(self, speed, time):
        self.print_current_arm_angles()

        desired_arm_angles = [self.current_arm_angles[0], self.current_arm_angles[1], speed]

        print("Desired Arm Angles: [" + str(desired_arm_angles[0]) + ", " + str(desired_arm_angles[1]) + ", " + str(desired_arm_angles[2]) + "]")

        self.done_moving_arm_flag = self.tic_controller.move_arm(desired_arm_angles)
        time.sleep(time)

        self.print_current_arm_angles()
    
    def print_current_gantry_position(self):
        self.current_gantry_position = self.tic_controller.get_current_gantry_position()
        print("Current Gantry Position: [" + str(self.current_gantry_position[0]) + ", " + str(self.current_gantry_position[1]) + ", " + str(self.current_gantry_position[2]) + "]")

    def print_current_arm_angles(self):
        self.current_arm_angles = self.tic_controller.get_current_arm_angles()
        print("Current Arm Angles: [" + str(self.current_arm_angles[0]) + ", " + str(self.current_arm_angles[1]) + ", " + str(self.current_arm_angles[2]) + "]")

    def turn_on_electromagnet(self):
        print("Turning on electromagnet.")
        self.electromagnet_status = self.tic_controller.electromagnet_switch("on")
        while(self.electromagnet_status != True):
            self.electromagnet_status = self.tic_controller.electromagnet_switch("on")
        print("Done turning on electromagnet.")

    def turn_off_electromagnet(self):
        print("Turning off electromagnet.")
        self.electromagnet_status = self.tic_controller.electromagnet_switch("off")
        while(self.electromagnet_status != False):
            self.electromagnet_status = self.tic_controller.electromagnet_switch("off")
        print("Done turning off electromagnet.")

    def cook_steak(self):

        gripper_coords = tic_stations.get_attachment_station_coords("gripper")

        self.move_gantry(gripper_coords)

        self.turn_on_electromagnet()

        self.move_gantry([gripper_coords[0], 0.0, 0.0])

        self.move_gantry([gripper_coords[0] - 0.05, 0.0, 0.0])
    
        steak_coords = tic_stations.get_station_coords("steaks")

        self.move_gantry([gripper_coords[0] - 0.05, steak_coords[1], 0.0])

        self.move_gantry(steak_coords)

        # Close gripper around steak
        self.move_arm([90, 6, 20])

        self.move_gantry([steak_coords[0], steak_coords[1], 0.0])

        self.move_arm([90, 90, 20])

        griddle_coords = tic_stations.get_station_coords("griddle")

        self.move_gantry([griddle_coords[0], griddle_coords[1], 0.0])

        self.move_gantry(griddle_coords)

        self.move_arm([90, 90, 70])

        self.move_gantry(gripper_coords)

        self.turn_off_electromagnet()