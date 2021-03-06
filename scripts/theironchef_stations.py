#!/usr/bin/env python

def get_attachment_station_coords(station):
    if (station == "gripper1"):
        coords = [0.48, 0.05, 0.161]
    elif (station == "gripper2"):
        coords = [0.28, 0.05, 0.13]
    elif (station == "syringe"):
        coords = [0.405, 0.745, 0.213]
    else:
        coords = [0.0, 0.0, 0.0]
    # Move the gantry to the coordinates
    return coords

def get_station_coords(station):
    if(station == "steaks"):
        coords= [0.4625, 0.375, 0.14]
    elif(station == "griddle"):
        coords = [0.73, 0.375, 0.05]
    elif(station == "steak_flip_initial"):
        coords = [0.73, 0.7, 0.166]
    elif(station == "steak_flip_done"):
        coords = [0.73, 0.33, 0.166]
    elif(station == "steak_flip_drop"):
        coords = [0.73, 0.6, 0.05]
    elif(station == "plate"):
        # coords = [1.11, 0.75, 0.1]
        coords = [1.11, 0.35, 0.1]
    elif(station == "oil"):
        coords = [0.9, 0.375, 0.08]
    else:
        coords = [0.0, 0.0, 0.0]
    return coords