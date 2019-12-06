'''
Filename: perspectives.py
Description: My implementation of states from a cartesian view (x,y,h) and 
    robot view (rho,alpha,beta) and transformations between them.
Author: Andi Frank
E-mail: a2frank@eng.ucsd.edu
Purpose: CSE 276A - Intro to Robotics; Fall 2019
'''

##############################################################
#                       IMPORTS
##############################################################
import sys, copy
from time import sleep, monotonic
from math import sin, cos, tan, atan2, pi
from numpy.linalg import norm
import numpy as np
from helpers import sign
import cartesian_pose as cart
import bicycle_model as bike




#################################################################
#       Going btw bike.BicyclePose and cart.CartesianPose objects      #
#################################################################


def cartesian_to_bicycle(robot_cartesian, goal_cartesian):
    '''
    Calculate BicyclePose representation from CartesianPoses robot_cartesian and goal_cartesian.
    '''
    diff = goal_cartesian - robot_cartesian
    rho  = diff.norm()
    rho_angle = diff.theta()
    alpha = rho_angle - robot_cartesian.h
    beta  = goal_cartesian.h - rho_angle
    return bike.BicyclePose(rho,alpha,beta)

def bicycle2robot(bicycle_pose, goal_cartesian):
    '''
    Calculate robot_cartesian from bicycle model and goal_cartesian
    '''
    # If bicycle_pose is a BicycleModel object, extract BicyclePose
    if isinstance(bicycle_pose, bike.BicycleModel):
        bicycle_pose = bicycle_pose.current_pose

    picar_x = goal_cartesian.x - bicycle_pose.rho * cos(goal_cartesian.h - bicycle_pose.beta)
    picar_y = goal_cartesian.y - bicycle_pose.rho * sin(goal_cartesian.h - bicycle_pose.beta)
    picar_h = goal_cartesian.h - bicycle_pose.beta - bicycle_pose.alpha
    return cart.CartesianPose(picar_x, picar_y, picar_h)

def bicycle2goal(bicycle_pose, robot_cartesian):
    '''
    Calculate goal_cartesian from bicycle model and robot_cartesian
    '''
    # If bicycle_pose is a BicycleModel object, extract BicyclePose
    if isinstance(bicycle_pose, bike.BicycleModel):
        bicycle_pose = bicycle_pose.current_pose

    goal_x = robot_cartesian.x + bicycle_pose.rho * cos(robot_cartesian.h + bicycle_pose.alpha)
    goal_y = robot_cartesian.y + bicycle_pose.rho * sin(robot_cartesian.h + bicycle_pose.alpha)
    goal_h = robot_cartesian.h + bicycle_pose.alpha + bicycle_pose.beta
    return cart.CartesianPose(goal_x, goal_y, goal_h) 
                



    
def test():
    print("Test needs to be updated before it will run.")
    # robot_cartesian = cart.CartesianPose(4,8,3*pi/4)
    # goal_cartesian = cart.CartesianPose(9,3,pi/4)
    # print("Robot in cartesian ref frame: {}\nGoal in cartesian ref frame: {}".format(
    #     robot_cartesian,goal_cartesian))


    # # print(goal_cart.wrt(robot_cartesian))

    # robot_bicyclepose = cartesian_to_bicycle(robot_cartesian, robot_in_cartesian_ref_frame=robot_cartesian)
    # goal_bicyclepose = cartesian_to_bicycle(goal_cartesian, robot_in_cartesian_ref_frame=robot_cartesian)

    # print("Cartesian coords: {}\nBM coords: {}\n\n".format(
    #     goal_cartesian, goal_bicyclepose))

    # rc = bicycle_to_cartesian(robot_bicyclepose,robot_in_cartesian_ref_frame=robot_cartesian)
    # gc = bicycle_to_cartesian(goal_bicyclepose,robot_in_cartesian_ref_frame=robot_cartesian)

    # print("Original robot position: {}\nRecalculated robot position: {}\n".format(
    #     robot_cartesian, rc))
    # print("Original goal position: {}\nRecalculated goal position: {}\n".format(
    #     goal_cartesian, gc))



if __name__=="__main__":
    test()
