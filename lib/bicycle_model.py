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
from math import sin, cos, tan, atan2, pi
from numpy.linalg import norm
import numpy as np

from helpers import within_pi, angle_a2b, sign, InputError
import cartesian_pose as cp




class BicyclePose():
    '''
    - Keep track of a relative state from the perspective of the robot and 
        allow easily readable access to (rho,alpha,beta).
    - Calculate (rho,alpha,beta) from robot and goal cp.CartesianPoses. 
    - Update (rho,alpha,beta) from v and gamma.
    '''

    def __init__(self,  rho=0, alpha=0, beta=0,  
                        x=None, y=None, h=None,
                        goal_cartesian=None, robo_cartesian=None):
        '''
        Can initialize with a (rho,alpha,beta), from a goal cp.CartesianPose object,
            from an goal (x,y,h), or with a combination of the above that fully
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

    @staticmethod
    def from_cartesian( x=0, y=0, h=0, goal_cartesian=None, robo_cartesian=None):
        return BicyclePose(x=x, y=y, h=h,
                                goal_cartesian=goal_cartesian, 
                                robo_cartesian=robo_cartesian)



    #############################################
    #         OVERRIDE BUILT-IN METHODS         #
    #############################################

    def __str__(self):
        return "BicyclePose: ({:>6.3f}, {:>6.3f}, {:>6.3f})".format(self.rho, self.alpha, self.beta)

    def __add__(self, cp):
        return BicyclePose (self.rho+cp.rho, within_pi(self.alpha+cp.alpha), within_pi(self.beta+cp.beta))

    def __radd__(self, cp):
        return BicyclePose (self.rho+cp.rho, within_pi(self.alpha+cp.alpha), within_pi(self.beta+cp.beta))

    def __iadd__(self, cp):
        self.rho = self.rho+cp.rho
        self.alpha = within_pi(self.alpha+cp.alpha)
        self.beta = within_pi(self.beta+cp.beta)
        return self

    def __rsub__(self, cp):
        return BicyclePose (self.rho-cp.rho, within_pi(self.alpha-cp.alpha), within_pi(self.beta-cp.beta))

    def __sub__(self, cp):
        return BicyclePose (self.rho-cp.rho, within_pi(self.alpha-cp.alpha), within_pi(self.beta-cp.beta))

    def __isub__(self, cp):
        self.rho = self.rho-cp.rho
        self.alpha = within_pi(self.alpha-cp.alpha)
        self.beta = within_pi(self.beta-cp.beta)
        return self

    def __neg__(self):
        return BicyclePose(-self.rho, -self.alpha, -self.beta)

    def __mul__(self, k):
        return BicyclePose(k*self.rho, within_pi(k*self.alpha), within_pi(k*self.beta))

    def __pow__(self, p):
        return BicyclePose(self.rho**p, within_pi(self.alpha**p), within_pi(self.beta**p))




def next_pose(current_pose, speed, steer, dt, direction=1):
    drhodt   = dRHOdt  (                       alpha=current_pose.alpha,   speed=speed,                direction=direction)
    dalphadt = dALPHAdt(rho=current_pose.rho,  alpha=current_pose.alpha,   speed=speed, steer=steer,   direction=direction)
    dbetadt  = dBETAdt (rho=current_pose.rho,  alpha=current_pose.alpha,   speed=speed,                direction=direction)
    return current_pose + BicyclePose(drhodt,dalphadt,dbetadt)*dt




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
    return within_pi(v*tan(steer)/L) 

def dXdt(speed, heading, direction=1):
    v = sign(direction)*abs(speed)
    return v*cos(heading)

def dYdt(speed, heading, direction=1):
    v = sign(direction)*abs(speed)
    return v*sin(heading)





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