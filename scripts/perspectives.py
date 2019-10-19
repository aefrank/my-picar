'''
Filename: perspectives.py
Description: My implementation of states from a world view (x,y,h) and 
    robot view (rho,alpha,beta) and transformations between them.
Author: Andi Frank
E-mail: a2frank@eng.ucsd.edu
Purpose: CSE 276A - Intro to Robotics; Fall 2019
'''

##############################################################
#                       IMPORTS
##############################################################
import sys
from time import sleep, monotonic
from math import sin, cos, tan, atan2, pi
from numpy.linalg import norm
import numpy as np
from helpers import sign



##############################################################
#                       STATE CLASSES
##############################################################

class XYHstate():
    '''
    Keep track of a state in the world reference frame and allow easily 
    readable access to (x,y,h).
    '''
    def __init__(self,x=0, y=0, h=0):
        self.x = x
        self.y = y
        self.h = 0

    def __add__(self, ws):
        return XYHstate (self.x+ws.x, self.y+ws.y, self.h+ws.h)

    def __radd__(self, ws):
        return XYHstate (self.x+ws.x, self.y+ws.y, self.h+ws.h)

    def __iadd__(self,ws):
        self.x = self.x+ws.x
        self.y = self.y+ws.y
        self.h = self.h+ws.h
        return self

    def __rsub__(self,ws):
        return XYHstate (self.x-ws.x, self.y-ws.y, self.h-ws.h)

    def __sub__(self,ws):
        return XYHstate (self.x-ws.x, self.y-ws.y, self.h-ws.h)

    def __isub__(self,ws):
        self.x = self.x-ws.x
        self.y = self.y-ws.y
        self.h = self.h-ws.h
        return self

    def __neg__(self):
        return XYHstate(-self.x, -self.y, -self.h)



    def norm(self):
        return pow( pow(self.x,2) + pow(self.y,2), 0.5)

    def pos(self):
        return np.array([self.x,self.y])

    def theta(self):
        """
        Angle of the vector from the origin to this State, w.r.t. the x-axis.
        Between -pi/2 and pi/2.
        """
        return atan2(self.y, self.x)


    def rotate(self, angle):
        c = cos(angle)
        s = sin(angle)

        x = self.x*c - self.y*s
        y = self.x*s + self.y*c
        h = self.h + angle

        return XYHstate(x,y,h)

    def wrt(self, origin):
        rotated = rotate(self, origin.h)
        translated = rotated - (origin.x, origin.y, 0)
        return translated




class RABstate():
    '''
    - Keep track of a relative state from the perspective of the robot and 
        allow easily readable access to (rho,alpha,beta).
    - Calculate (rho,alpha,beta) from robot and goal WorldStates. 
    - Update (rho,alpha,beta) from v and gamma.
    '''


    def __init__(self, rho=None, alpha=None, beta=None, xyh=None, x=None, y=None, h=None,
                         robo_xyh=None):
        # Check that input specs are well-defined
        self.check_init_spec(rho=rho, alpha=alpha, beta=beta, xyh=xyh,
                        x=x, y=y, h=h)

        if xyh is None:
            xyh = XYHstate(x,y,h) # <-- might still be none here

        if robo_xyh is None:
            robo_xyh = XYHstate(0,0,0)

        # Transform xyh such that robo_xyh is at the origin pose
        xyh = xyh - robo_xyh # <-- and if so will still be none here
        print("DEBUG AT LINE 120 IN perspectives.py")

        if rho is None:
            rho = xyh.norm()

        if alpha is None:
            alpha = xyh.theta() - robot_xyh.h

        if beta is None:
            beta = xyh.h - xyh.theta()

        self.rho   = rho
        self.alpha = alpha
        self.beta  = beta


    def __str__(self):
        return "({:3f}, {:3f}, {:3f})".format(self.rho, self.alpha, self.beta)

    @classmethod
    def well_defined(cls, rho, alpha, beta, xyh, x, y, h):
        '''
        CHECK FOR UNDER- OR OVERDEFINED RABstate SPECIFICATION:
            Check if given either
                2) ALL three (rho,alpha,beta) AND also other arguments    [overdefined]
            or NOT ALL three (rho,alpha,beta) AND:
                3) both    xyh  AND  any (x,y,h)                      [overdefined]
                4) neither xyh  NOR  all three (x,y,h)                [underdefined]

        Returns -1 for underdefined and 1 for overdefined, or 0 if well defined.
        '''

        # Save the existance of inputs
        R = (rho   is not None)     # rho exists
        A = (alpha is not None)     # alpha exists
        B = (beta  is not None)     # beta exists
        all_RAB = (R and A and B)   # all of (rho,alpha,beta) exists

        XYHstate = (xyh is not None)
        
        X = (x   is not None)       # x exists
        Y = (y is not None)         # y exists
        H = (h  is not None)    # h exists
        all_XYH = (X and Y and H)   # all of (x,y,h) exists
        any_XYH = (X  or Y  or H)   # any of (x,y,h) exists


        # If you have input all (rho,alpha,beta)
        if (all_RAB):
            # But also input either XYHstate or any of (x,y,h)
            if (XYHstate or any_XYH):
                # You are overdefined
                return 1
        # If you haven't input all of (rho,alpha,beta)
        else:
            # If we need to calculate rho,   then we need (x,y) of the goal state
            if not R and not (XYHstate or (X and H)):
                return -1
            # If we need to calculate alpha, then we need (x,y) of the state
            if not A and not (XYHstate or (X and Y)):
                return -1
            # If we need to calculate, beta, then we need (x,y,h) of the state
            if not H and not (XYHstate or (all_XYH)):
                return -1

        # Specification is well-defined
        return 0

    @classmethod
    def check_init_spec(cls, rho, alpha, beta, xyh, x, y, h):
        '''
        Throw errors if init specification is not well-defined.
        Otherwise, return a 
        '''
        sc = cls.well_defined(rho=rho, alpha=alpha, beta=beta, xyh=xyh,
                    x=x, y=y, h=h)
        if   (sc == 1):
            raise InputError("RABstate specification overdefined.")
        elif (sc ==-1):
            raise InputError("RABstate specification underdefined.")



    def to_XYH(self, origin=None):
        if origin is None:
            origin = XYHstate(0,0,0)

        x_wrt_robot = self.rho * cos(self.alpha) 
        y_wrt_robot = self.rho * sin(self.alpha) 
        h_wrt_robot = self.alpha + self.beta
        
        # Assuming the robot is at the origin facing the x-axis,
        # these are the coordinates of the point
        xyh_wrt_robot = XYHstate(x_wrt_robot, y_wrt_robot, h_wrt_robot)

        return my_xyh.wrt(origin)