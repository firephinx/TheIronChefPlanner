#!/usr/bin/env python

import rospy
import math
import numpy
import argparse
from theironchef_controller import TheIronChefController
#from theironchef_ik import TheIronChefIK
#from theironchef_cv import TheIronChefCV

#All measurements in meters and degrees
    
def get_attachment_station_coords(station):
    if (station == "gripper"):
        coords = [0.0, 0.0, 0.0]
    elif (station == "syringe"):
        coords = [0.0, 0.0, 0.0]
    else:
        coords = [0.0, 0.0, 0.0]
    # Move the gantry to the coordinates
    return coords

def get_station_coords(station):
    if(station == "steaks"):
        coords = [0.0, 0.0, 0.0]
    elif(station == "griddle"):
        coords = [0.0, 0.0, 0.0]
    else:
        coords = [0.0, 0.0, 0.0]
    return coords

if __name__ == '__main__':

    tic_controller = TheIronChefController()
    #tic_ik = TheIronChefIK()
    #tic_cv = TheIronChefCV()

    tic_controller.reset_robot()
    tic_controller.home_robot()

    gripper_coords = get_attachment_station_coords("gripper")

    done_moving_robot_base_flag = tic_controller.move_gantry(gripper_coords)
    while(done_moving_robot_base_flag != True):
        done_moving_robot_base_flag = tic_controller.move_gantry(gripper_coords)

    cur_gantry_pos = tic_controller.get_current_position()
    print("Gantry Position: [" + str(cur_gantry_pos[0]) + ", " + str(cur_gantry_pos[1]) + ", " + str(cur_gantry_pos[2]) + "]")

    steak_coords = get_station_coords("steaks")

    done_moving_robot_base_flag = tic_controller.move_gantry(steak_coords)
    while(done_moving_robot_base_flag != True):
        done_moving_robot_base_flag = tic_controller.move_gantry(steak_coords)

    cur_gantry_pos = tic_controller.get_current_position()
    print("Gantry Position: [" + str(cur_gantry_pos[0]) + ", " + str(cur_gantry_pos[1]) + ", " + str(cur_gantry_pos[2]) + "]")

    griddle_coords = get_station_coords("griddle")

    done_moving_robot_base_flag = tic_controller.move_gantry(griddle_coords)
    while(done_moving_robot_base_flag != True):
        done_moving_robot_base_flag = tic_controller.move_gantry(griddle_coords)

    cur_gantry_pos = tic_controller.get_current_position()
    print("Gantry Position: [" + str(cur_gantry_pos[0]) + ", " + str(cur_gantry_pos[1]) + ", " + str(cur_gantry_pos[2]) + "]")

