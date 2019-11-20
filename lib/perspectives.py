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

def bicycle_to_cartesian(bp, robot_in_cartesian_ref_frame=None):
    '''
    Calculate the cartesian coordinates of the goal point
    '''
    if robot_in_cartesian_ref_frame is None:
        robot_in_cartesian_ref_frame = cart.CartesianPose(0,0,0)

    # Find (x,y,h) coordinates w.r.t. robot
    goal_x_in_robot_ref_frame = bp.rho * cos(bp.alpha) 
    goal_y_in_robot_ref_frame = bp.rho * sin(bp.alpha) 
    goal_h_in_robot_ref_frame = bp.alpha - bp.beta


    goal_in_robot_ref_frame = cart.CartesianPose(goal_x_in_robot_ref_frame,
                                            goal_y_in_robot_ref_frame,
                                            goal_h_in_robot_ref_frame)

    # Rotate point along with robot back to correct heading while we are in robot ref frame
    goal_rotated = goal_in_robot_ref_frame.rotate(robot_in_cartesian_ref_frame.h)

    # Now that our translation and cartesian axes are aligned, we can translate to our cartesian position
    translation_vector = copy.copy(robot_in_cartesian_ref_frame)
    translation_vector.h = 0
    goal = goal_rotated + translation_vector

    return goal


def cartesian_to_bicycle(goal, robot_in_cartesian_ref_frame=None):
    '''
    Calculate the bike.BicycleModel given a goal cart.CartesianPose
    '''
    if robot_in_cartesian_ref_frame is None:
        robot_in_cartesian_ref_frame = cart.CartesianPose(0,0,0)

    goal_wrt_robot = goal.wrt(robot_in_cartesian_ref_frame)

    # Now get rho, alpha, beta
    rho   = goal_wrt_robot.norm()
    alpha = goal_wrt_robot.theta()
    beta  = goal_wrt_robot.h

    return bike.BicyclePose(rho=rho, alpha=alpha, beta=beta)


    

    
def test():
    robot_cartesian = cart.CartesianPose(4,8,3*pi/4)
    goal_cartesian = cart.CartesianPose(9,3,pi/4)
    print("Robot in cartesian ref frame: {}\nGoal in cartesian ref frame: {}".format(
        robot_cartesian,goal_cartesian))


    # print(goal_cart.wrt(robot_cartesian))

    robot_bicyclepose = cartesian_to_bicycle(robot_cartesian, robot_in_cartesian_ref_frame=robot_cartesian)
    goal_bicyclepose = cartesian_to_bicycle(goal_cartesian, robot_in_cartesian_ref_frame=robot_cartesian)

    print("Cartesian coords: {}\nBM coords: {}\n\n".format(
        goal_cartesian, goal_bicyclepose))

    rc = bicycle_to_cartesian(robot_bicyclepose,robot_in_cartesian_ref_frame=robot_cartesian)
    gc = bicycle_to_cartesian(goal_bicyclepose,robot_in_cartesian_ref_frame=robot_cartesian)

    print("Original robot position: {}\nRecalculated robot position: {}\n".format(
        robot_cartesian, rc))
    print("Original goal position: {}\nRecalculated goal position: {}\n".format(
        goal_cartesian, gc))



if __name__=="__main__":
    test()
