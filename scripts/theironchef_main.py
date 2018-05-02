#!/usr/bin/env python

from theironchef_planner import TheIronChefPlanner

if __name__ == '__main__':
	tic_planner = TheIronChefPlanner()

	tic_planner.reset_robot()
    tic_planner.home_robot()

    tic_planner.move_x_gantry_to_center()

	tic_planner.cook_steak()