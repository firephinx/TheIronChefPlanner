#!/usr/bin/env python

def get_attachment_station_coords(station):
    if (station == "gripper"):
        coords = [0.7, 0.0, 0.075]
    elif (station == "syringe"):
        coords = [0.0, 0.0, 0.0]
    else:
        coords = [0.0, 0.0, 0.0]
    # Move the gantry to the coordinates
    return coords

def get_station_coords(station):
    if(station == "steaks"):
        coords = [0.73, 0.37, 0.01]
    elif(station == "griddle"):
        coords = [0.8, 0.33, 0.05]
    else:
        coords = [0.0, 0.0, 0.0]
    return coords