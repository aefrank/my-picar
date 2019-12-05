'''
Filename: bicycle_model.py
Description: My implementation of the 3DoF relationship between
    a picar and its goal pose.
Author: Andi Frank
E-mail: a2frank@eng.ucsd.edu
Purpose: CSE 276A - Intro to Robotics; Fall 2019
'''

##############################################################
#                       IMPORTS                             #
##############################################################
from math import sin, cos, atan2, pi
from numpy.linalg import norm
import numpy as np
from helpers import within_pi, angle_a2b
import cartesian_pose as cp




class BicyclePose():
    '''
    - Keep track of a relative state from the perspective of the robot and 
        allow easily readable access to (rho,alpha,beta).
    - Calculate (rho,alpha,beta) from robot and goal cp.CartesianPoses. 
    - Update (rho,alpha,beta) from v and gamma.
    '''

    def __init__(self,  rho=None, alpha=None, beta=None,  
                        x=None, y=None, h=None,
                        goal_cartesian=None, robo_cartesian=None):
        '''
        Can initialize with a (rho,alpha,beta), from a cp.CartesianPose object,
            from an (x,y,h), or with a combination of the above that fully
            defines a bicycle model.
        '''

        # Check that input specs are well-defined
        check_init_spec(rho=rho, alpha=alpha, beta=beta, 
                            goal_cartesian=goal_cartesian,
                            x=x, y=y, h=h)

        
        if rho is not None:
            self.rho = rho 
        if alpha is not None:
            self.alpha = alpha
        if beta is not None:
            self.beta = beta

        if robo_cartesian is None:
            robo_cartesian = cp.CartesianPose(0,0,0)
        if goal_cartesian is None:
            if x is None or y is None or h is None:
                robo_cartesian = cp.CartesianPose(0,0,0)
            else:
                goal_cartesian = cp.CartesianPose(x,y,h)
        if rho is None: 
            rho   = RHO(robo_cartesian, goal_cartesian)
        if alpha is None:
            alpha = ALPHA(robo_cartesian, goal_cartesian)
        if beta is None:
            beta  = BETA(robo_cartesian, goal_cartesian)
        
        self.rho   = rho
        self.alpha = alpha
        self.beta  = beta


    def __str__(self):
        return "BicyclePose: ({:>6.3f}, {:>6.3f}, {:>6.3f})".format(self.rho, self.alpha, self.beta)

# class BicycleModelControllers():
#     '''
#     Class to hold PID controllers for Bicycle Model
#     '''
#     def __init__(self, rho_controller, alpha_controller, beta_controller):
#         self.rho   = rho_controller
#         self.alpha = alpha_controller
#         self.beta  = beta_controller



class BicycleModel():

    def __init__(self, bicycle_pose=None, rho=None, alpha=None, beta=None, L=1):
        if bicycle_pose is None:
            bicycle_pose = BicyclePose(rho=rho, alpha=alpha, beta=beta)
        self.current_pose = bicycle_pose
        self.L = L

    @staticmethod
    def from_cartesian( x=None, y=None, h=None,
                        goal_cartesian=None, 
                        robo_cartesian=None):
        bicycle_pose = BicyclePose(x=x, y=y, h=h,
                               goal_cartesian=goal_cartesian, 
                               robo_cartesian=robo_cartesian)
        return BicycleModel(bicycle_pose)

    def next_pose(self, speed, steer, direction, dt, current_pose=None):
        if current_pose==None:
            current_pose = self.current_pose
        drhodt   = dRHOdt  (                       alpha=current_pose.alpha,   speed=speed,                direction=direction)
        dalphadt = dALPHAdt(rho=current_pose.rho,  alpha=current_pose.alpha,   speed=speed, steer=steer,   direction=direction)
        dbetadt  = dBETAdt (rho=current_pose.rho,  alpha=current_pose.alpha,   speed=speed,                direction=direction)

        return current_pose + dt*BicyclePose(drhodt,dalphadt,dbetadt)




#########################################
#          CALCULATE PARAMETERS         #
#########################################

def RHO(robot,goal):
    '''
    Magnitude of the vector from (robot location) to (goal location).

    Robot and goal poses should be given as cp.CartesianPose objects.
    '''
    difference = goal - robot
    return difference.norm()

def ALPHA(robot,goal):
    '''
    Angle from the robot's heading to the vector from (robot location) to (goal location).

    Robot and goal poses should be given as cp.CartesianPose objects.
    '''
    difference = goal - robot
    return within_pi(difference.theta())

def BETA(robot,goal): 
    '''
    Angle from the vector from (robot location) to (goal location) to the goal heading angle.

    Robot and goal poses should be given as cp.CartesianPose objects.
    '''
    difference = goal - robot
    return angle_a2b(a=difference.theta, b=difference.h)


### BicyclePose Derivatives based on current parameter and control values

def dRHOdt(alpha, speed, direction=1):
    v = sign(direction)*abs(speed)
    return -v*cos(alpha)

def dALPHAdt(rho, alpha, speed, steer, direction=1):
    v = sign(direction)*abs(speed)
    return within_pi(v*sin(alpha)/rho - steer)

def dBETAdt(rho, alpha, speed, direction=1):
    v = sign(direction)*abs(speed)
    return within_pi(-v*cos(alpha)/rho)



### CartesianPose Derivatives based on current parameter and control values for Picar

def dHdt(speed, steer, L, direction=1):
    v = sign(direction)*abs(speed)
    return within_pi(v*tan(steer_angle)/L) 

def dXdt(speed, heading, direction=1):
    v = sign(direction)*abs(speed)
    return speed*cos(heading)

def dYdt(speed, heading, direction=1):
    v = sign(direction)*abs(speed)
    return speed*sin(heading)


# Not really a bicycle_model thing, more of a decision about how we wanted to
# calculate controls BASED ON BicycleModel -- put in my_picar instead
#
# ##############################
# #     CALCULATE CONTROLS     #
# ##############################

# def V(rho, rho_controller, dt=1):
#     '''
#     Calculate velocity at next timestep based on rho.
#     rho_controller must be a my_pid.PID object
#     '''
#     return rho_controller.input(rho, dt=dt)


# def GAMMA(v, alpha, beta, alpha_controller, beta_controller, L=1, dt=1):
#     '''
#     Calculate steering angle gamma at next timestep based on alpha and beta.
#     {alpha,beta}_controller must be a my_pid.PID object
#     '''
#     a = alpha_controller.input(alpha, dt=dt) 
#     b = beta_controller .input(beta,  dt=dt)  

#     # Desired change in heading angle
#     dh = a + b
#     omega = dh/dt

#     # Calculate steering angle corresponding to angular velocity
#        # based on car geometry and linear velocity
#     gamma = atan(omega*L/v)

#     # Bound between [-pi, pi]
#     return within_pi(gamma)
 

# def should_i_back_up(alpha):
#     # If alpha is greater than pi/2, it's easier to go backward
#     # Inspired by code.py example from Homework 1.
#     # https://d1b10bmlvqabco.cloudfront.net/attach/k0uju462t062l4/j12evy3w52o5kl/k1vnoghb1697/code.pdf
#     if abs(alpha) > (pi/2 + 1e-4):
#         return True
#     else:
#         return False








### Init helpers

def check_init_spec(rho, alpha, beta, goal_cartesian, x, y, h):
    '''
    Throw errors if init specification is not well-defined.
    Otherwise, return success exit code True
    '''
    wd = well_defined(  rho=rho, alpha=alpha, beta=beta, 
                        goal_cartesian=goal_cartesian,
                        x=x, y=y, h=h)
    if   (wd == 1):
        raise InputError("BicycleModel specification overdefined.")
    elif (wd ==-1):
        raise InputError("BicycleModel specification underdefined.")

    return True



def well_defined(rho, alpha, beta, goal_cartesian, x, y, h):
    '''
    CHECK FOR UNDER- OR OVERDEFINED BicycleModel SPECIFICATION:
        Check if given either
            2) ALL three (rho,alpha,beta) AND also other arguments    [overdefined]
        or NOT ALL three (rho,alpha,beta) AND:
            3) both    goal_cartesian  AND  any (x,y,h)                      [overdefined]
            4) neither goal_cartesian  NOR  all three (x,y,h)                [underdefined]

    Returns -1 for underdefined and 1 for overdefined, or 0 if well defined.
    '''

    # Save the existance of inputs
    R = (rho   is not None)     # rho exists
    A = (alpha is not None)     # alpha exists
    B = (beta  is not None)     # beta exists
    all_RAB = (R and A and B)   # all of (rho,alpha,beta) exists

    W = (goal_cartesian is not None)
    
    X = (x   is not None)       # x exists
    Y = (y is not None)         # y exists
    H = (h  is not None)    # h exists
    all_XYH = (X and Y and H)   # all of (x,y,h) exists
    any_XYH = (X  or Y  or H)   # any of (x,y,h) exists


    # If you have input all (rho,alpha,beta)
    if (all_RAB):
        # But also input either cp.CartesianPose or any of (x,y,h)
        if (W or any_XYH):
            # You are overdefined
            return 1
    # If you haven't input all of (rho,alpha,beta)
    else:
        # If we need to calculate rho,   then we need (x,y) of the goal state
        if not R and not (W or (X and H)):
            return -1
        # If we need to calculate alpha, then we need (x,y) of the state
        if not A and not (W or (X and Y)):
            return -1
        # If we need to calculate, beta, then we need (x,y,h) of the state
        if not H and not (W or (all_XYH)):
            return -1

    # Specification is well-defined
    return 0