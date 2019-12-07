'''
Filename: cartesian_pose.py
Description: My implementation of a 3DoF picar robot in standard coordinates (x,y,h)
Author: Andi Frank
E-mail: a2frank@eng.ucsd.edu
Purpose: CSE 276A - Intro to Robotics; Fall 2019
'''

##############################################################
#                       IMPORTS                             #
##############################################################
from math import sin, cos, atan2, pi, sqrt
from numpy.linalg import norm
import numpy as np
from helpers import within_pi


class CartesianPose():
    '''
    Keep track of a state in a given reference frame and allow easily 
    readable access to (x,y,h).
    '''
    def __init__(self,x=0, y=0, h=0):
        self.x = x
        self.y = y
        self.h = within_pi(h)

        # In case 'h' is confusing
        self.heading = self.h



    ##########################################
    #         GET DERIVED PARAMETERS         #
    ##########################################

    def pos(self):
        return np.array([self.x, self.y])

    def theta(self):
        """
        DO NOT CONFUSE WITH HEADING!!
        Angle of the vector from the origin to a CartesianPose, w.r.t. the world x-axis.
        Between -pi and pi.
        """
        return atan2(self.y, self.x)

    def wrt(self, new_origin_state):
        return wrt(self, new_origin_state)

    def norm(self):
        return norm(self)

    def rotate(self, angle):
        return rotate(self, angle)


    #############################################
    #         OVERRIDE BUILT-IN METHODS         #
    #############################################

    def __str__(self):
        return "CartesianPose: ({:>6.3f}, {:>6.3f}, {:>6.3f})".format(self.x, self.y, self.h)

    def __add__(self, cp):
        return CartesianPose (self.x+cp.x, self.y+cp.y, within_pi(self.h+cp.h))

    def __radd__(self, cp):
        return CartesianPose (self.x+cp.x, self.y+cp.y, within_pi(self.h+cp.h))

    def __iadd__(self, cp):
        self.x = self.x+cp.x
        self.y = self.y+cp.y
        self.h = within_pi(self.h+cp.h)
        return self

    def __rsub__(self, cp):
        return CartesianPose (self.x-cp.x, self.y-cp.y, within_pi(self.h-cp.h))

    def __sub__(self, cp):
        return CartesianPose (self.x-cp.x, self.y-cp.y, within_pi(self.h-cp.h))

    def __isub__(self, cp):
        self.x = self.x-cp.x
        self.y = self.y-cp.y
        self.h = within_pi(self.h-cp.h)
        return self

    def __neg__(self):
        return CartesianPose(-self.x, -self.y, -self.h)

    def __mul__(self, k):
        return CartesianPose(k*self.x, k*self.y, within_pi(k*self.h))

    def __pow__(self, p):
        return CartesianPose(self.x**p, self.y**p, within_pi(self.h**p))

    
    def dist(self, pose2):
        sq = (self - pose2)**2
        sm = sq.x + sq.y
        return sqrt(sm)



####################################################
#         CALCULATIONS AND TRANSFORMATIONS         #
####################################################

def norm(cp):
    return pow( pow(cp.x,2) + pow(cp.y,2), 1/2)

def rotate(cp, angle):
    '''
    2D rotate <angle> radians around the origin.
    '''
    c = cos(angle)
    s = sin(angle)

    x = cp.x*c - cp.y*s
    y = cp.x*s + cp.y*c
    h = within_pi(cp.h + angle)

    return CartesianPose(x,y,h)

def wrt(cp, new_origin_state):
    '''
    Returns the coordinates of this point with respect to a new origin.
    Both cp and the new origin should be w.r.t. the current origin in world coordinates.
    '''
    # Translate new origin to (0,0,h)
    translated = cp - CartesianPose(new_origin_state.x, new_origin_state.y, 0)
    # Rotate new origin to (0,0,0)
    rotated = rotate(translated, -new_origin_state.h)
    return rotated

 