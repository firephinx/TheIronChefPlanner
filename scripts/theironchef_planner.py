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

        self.attachment_liftup_height = 0.01
        self.gripper1_y_pickup_offset = 0.1
        self.gripper2_y_pickup_offset = 0.1
        self.syringe_y_pickup_offset = -0.1

        self.steak_push_y_offset = 0.1
        self.steak_push_z_offset = 0.03

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

    def home_z_gantry(self):
        print("Homing Z Gantry")

        self.done_homing_gantry_flag = self.tic_controller.home_z_gantry()
        while(self.done_homing_gantry_flag != True):
            self.done_homing_gantry_flag = self.tic_controller.home_z_gantry()

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

        print("Turning on electromagnet again just to make sure.")
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

        print("Turning off electromagnet again just to make sure.")
        self.electromagnet_status = self.tic_controller.electromagnet_switch("off")
        while(self.electromagnet_status != False):
            self.electromagnet_status = self.tic_controller.electromagnet_switch("off")
        print("Done turning off electromagnet.")

    def rack_gripper_1(self):
        gripper_1_coords = tic_stations.get_attachment_station_coords("gripper1")

        self.move_arm([179, 87, 87])

        self.move_gantry([0.0, gripper_1_coords[1] + self.gripper1_y_pickup_offset, 0.0])

        self.move_gantry([gripper_1_coords[0], gripper_1_coords[1] + self.gripper1_y_pickup_offset, gripper_1_coords[2] - self.attachment_liftup_height])

        self.move_gantry([gripper_1_coords[0], gripper_1_coords[1] + self.gripper1_y_pickup_offset, gripper_1_coords[2] - self.attachment_liftup_height])

        self.move_gantry([gripper_1_coords[0], gripper_1_coords[1], gripper_1_coords[2] - self.attachment_liftup_height])

        self.move_gantry(gripper_1_coords)

        self.turn_off_electromagnet()

        self.home_z_gantry()

    def rack_gripper_2(self):
        gripper_2_coords = tic_stations.get_attachment_station_coords("gripper2")

        self.move_arm([87, 87, 100])

        self.move_gantry([0.0, gripper_2_coords[1] + self.gripper2_y_pickup_offset, 0.0])

        self.move_gantry([gripper_2_coords[0], gripper_2_coords[1] + self.gripper2_y_pickup_offset, 0.0])

        self.move_gantry([gripper_2_coords[0], gripper_2_coords[1] + self.gripper2_y_pickup_offset, gripper_2_coords[2] - self.attachment_liftup_height])

        self.move_gantry([gripper_2_coords[0], gripper_2_coords[1], gripper_2_coords[2] - self.attachment_liftup_height])

        self.move_gantry(gripper_2_coords)

        self.turn_off_electromagnet()

        self.home_z_gantry()

    def rerack_gripper_1(self):
        gripper_1_coords = tic_stations.get_attachment_station_coords("gripper1")

        self.move_arm([179, 87, 87])

        self.move_gantry([gripper_1_coords[0], gripper_1_coords[1] + self.gripper1_y_pickup_offset, gripper_1_coords[2] - self.attachment_liftup_height])

        self.move_gantry([gripper_1_coords[0], gripper_1_coords[1] + self.gripper1_y_pickup_offset, gripper_1_coords[2] - self.attachment_liftup_height])

        self.move_gantry([gripper_1_coords[0], gripper_1_coords[1], gripper_1_coords[2] - self.attachment_liftup_height])

        self.move_gantry(gripper_1_coords)

        self.turn_off_electromagnet()

        self.home_z_gantry()

    def rerack_gripper_2(self):
        gripper_2_coords = tic_stations.get_attachment_station_coords("gripper2")

        self.move_arm([87, 87, 100])

        self.move_gantry([gripper_2_coords[0], gripper_2_coords[1] + self.gripper2_y_pickup_offset, 0.0])

        self.move_gantry([gripper_2_coords[0], gripper_2_coords[1] + self.gripper2_y_pickup_offset, gripper_2_coords[2] - self.attachment_liftup_height])

        self.move_gantry([gripper_2_coords[0], gripper_2_coords[1], gripper_2_coords[2] - self.attachment_liftup_height])

        self.move_gantry(gripper_2_coords)

        self.turn_off_electromagnet()

        self.home_z_gantry()

    def cook_steak(self):

        # # Pick up gripper 1
        # gripper_1_coords = tic_stations.get_attachment_station_coords("gripper1")

        # self.move_arm([179, 87, 87])

        # self.move_gantry(gripper_1_coords)

        # self.turn_on_electromagnet()

        # self.move_gantry([gripper_1_coords[0], gripper_1_coords[1], gripper_1_coords[2] - self.attachment_liftup_height])

        # self.move_gantry([gripper_1_coords[0], gripper_1_coords[1] + self.gripper1_y_pickup_offset, gripper_1_coords[2] - self.attachment_liftup_height])

        # self.home_z_gantry()

        # self.move_arm([87, 87, 87])
    
        # # Pick up the steak
        # steak_coords = tic_stations.get_station_coords("steaks")

        # self.move_gantry([steak_coords[0], steak_coords[1], 0.0])

        # self.move_gantry(steak_coords)

        # # Close gripper around steak
        # self.move_arm([87, 87, 6])

        # self.home_z_gantry()

        # # Move the steak over the griddle
        # griddle_coords = tic_stations.get_station_coords("griddle")

        # self.move_gantry([griddle_coords[0], griddle_coords[1], 0.0])

        # self.move_gantry(griddle_coords)

        # self.move_arm([87, 87, 87])

        # self.move_gantry([griddle_coords[0], griddle_coords[1], griddle_coords[2] + self.steak_push_z_offset])

        # self.move_gantry([griddle_coords[0], griddle_coords[1] - self.steak_push_y_offset, griddle_coords[2] + self.steak_push_z_offset])

        # self.home_z_gantry()

        # # Rerack gripper 1
        # self.rerack_gripper_1()

        # griddle_coords = tic_stations.get_station_coords("griddle")

        # # Pick up gripper 2
        # self.move_arm([87, 87, 100])
        # gripper_2_coords = tic_stations.get_attachment_station_coords("gripper2")

        # self.move_gantry(gripper_2_coords)

        # self.turn_on_electromagnet()

        # self.move_gantry([gripper_2_coords[0], gripper_2_coords[1], gripper_2_coords[2] - self.attachment_liftup_height])

        # self.move_gantry([gripper_2_coords[0], gripper_2_coords[1] + self.gripper2_y_pickup_offset, gripper_2_coords[2] - self.attachment_liftup_height])

        # self.home_z_gantry()

        # Flip the steak
        steak_flip_initial_coords = tic_stations.get_station_coords("steak_flip_initial")

        self.move_gantry([steak_flip_initial_coords[0], steak_flip_initial_coords[1], 0.0])

        self.move_arm([179, 179, 87])

        self.move_gantry(steak_flip_initial_coords)

        steak_flip_done_coords = tic_stations.get_station_coords("steak_flip_done")

        self.move_gantry(steak_flip_done_coords)

        self.move_arm([179, 179, 6])

        self.home_z_gantry()

        steak_flip_drop_coords = tic_stations.get_station_coords("steak_flip_drop")

        self.move_gantry([steak_flip_drop_coords[0], steak_flip_drop_coords[1], 0.0])

        self.move_arm([179, 150, 87])

        self.move_gantry([steak_flip_initial_coords[0], steak_flip_initial_coords[1], 0.0])

        # Wait for the steak to cook
        time.sleep(10)

        # Pick up the steak
        self.move_arm([179, 179, 87])

        self.move_gantry(steak_flip_initial_coords)

        self.move_gantry(steak_flip_done_coords)

        self.move_arm([179, 179, 6])

        self.home_z_gantry()

        # Move the steak to the plate
        plate_coords = tic_stations.get_station_coords("plate")

        self.move_gantry([steak_flip_done_coords[0], plate_coords[1], 0.0])

        self.move_gantry([plate_coords[0], plate_coords[1], 0.0])

        self.move_gantry(plate_coords)

        self.move_arm([179, 130, 87])

        self.home_z_gantry()

        # Rerack gripper 2
        self.move_gantry([gripper_2_coords[0], plate_coords[1], 0.0])

        self.rerack_gripper_2()

        self.home_robot()