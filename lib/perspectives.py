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
import sys, copy
from time import sleep, monotonic
from math import sin, cos, tan, atan2, pi
from numpy.linalg import norm
import numpy as np
from helpers import sign
# from perspectives import WorldState, BMState



##############################################################
#                       STATE CLASSES
##############################################################

class WorldState():
    '''
    Keep track of a state in the world reference frame and allow easily 
    readable access to (x,y,h).
    '''
    def __init__(self,x=0, y=0, h=0, xyh=None):
        if xyh is not None:
            self.x = xyh[0]
            self.x = xyh[1]
            self.x = xyh[2]
        else:
            self.x = x
            self.y = y
            self.h = h

    def __str__(self):
        return "WorldState: ({:.3f}, {:.3f}, {:.3f})".format(self.x, self.y, self.h)

    def __add__(self, ws):
        return WorldState (self.x+ws.x, self.y+ws.y, self.h+ws.h)

    def __radd__(self, ws):
        return WorldState (self.x+ws.x, self.y+ws.y, self.h+ws.h)

    def __iadd__(self,ws):
        self.x = self.x+ws.x
        self.y = self.y+ws.y
        self.h = self.h+ws.h
        return self

    def __rsub__(self,ws):
        return WorldState (self.x-ws.x, self.y-ws.y, self.h-ws.h)

    def __sub__(self,ws):
        return WorldState (self.x-ws.x, self.y-ws.y, self.h-ws.h)

    def __isub__(self,ws):
        self.x = self.x-ws.x
        self.y = self.y-ws.y
        self.h = self.h-ws.h
        return self

    def __neg__(self):
        return WorldState(-self.x, -self.y, -self.h)



    def norm(self):
        return pow( pow(self.x,2) + pow(self.y,2), 0.5)

    def pos(self):
        return np.array([self.x,self.y])

    def theta(self):
        """
        NOT HEADING!!
        Angle of the vector from the origin to this State, w.r.t. the x-axis.
        Between -pi and pi.
        """
        return atan2(self.y, self.x)


    def rotate(self, angle):
        '''
        2D rotate <angle> radians around the origin.
        '''
        c = cos(angle)
        s = sin(angle)

        x = self.x*c - self.y*s
        y = self.x*s + self.y*c
        h = self.h + angle

        return WorldState(x,y,h)

    def wrt(self, new_origin_state):
        '''
        Returns the coordinates of this point with respect to a new origin.
        Both self and the new origin should be w.r.t. the current origin in world coordinates.
        '''
        translated = self - WorldState(new_origin_state.x, new_origin_state.y, 0)
        rotated = translated.rotate(-new_origin_state.h)
        return rotated






class BicycleModelState():
    '''
    - Keep track of a relative state from the perspective of the robot and 
        allow easily readable access to (rho,alpha,beta).
    - Calculate (rho,alpha,beta) from robot and goal WorldStates. 
    - Update (rho,alpha,beta) from v and gamma.
    '''



    def __init__(self, rho=None, alpha=None, beta=None, ws=None, x=None, y=None, h=None,
                         robo_ws=None):
        # Check that input specs are well-defined
        self.check_init_spec(rho=rho, alpha=alpha, beta=beta, ws=ws,
                        x=x, y=y, h=h)

        if robo_ws is None:
            robo_ws = WorldState(0,0,0)

        if ws is None:
            ws = WorldState(x,y,h)

        if rho is None: 
            relative_ws = ws - robo_ws
            rho = relative_ws.norm()
        
        if alpha is None:
            if relative_ws is None:
                relative_ws = ws - robo_ws
            alpha = relative_ws.theta()

        if beta is None:
            if relative_ws is None:
                relative_ws = ws - robo_ws
            beta = relative_ws.h - relative_ws.theta()
        

        self.rho   = rho
        self.alpha = alpha
        self.beta  = beta


    def __str__(self):
        return "BicycleModelState: ({:.3f}, {:.3f}, {:.3f})".format(self.rho, self.alpha, self.beta)

    @classmethod
    def well_defined(cls, rho, alpha, beta, ws, x, y, h):
        '''
        CHECK FOR UNDER- OR OVERDEFINED BicycleModelState SPECIFICATION:
            Check if given either
                2) ALL three (rho,alpha,beta) AND also other arguments    [overdefined]
            or NOT ALL three (rho,alpha,beta) AND:
                3) both    ws  AND  any (x,y,h)                      [overdefined]
                4) neither ws  NOR  all three (x,y,h)                [underdefined]

        Returns -1 for underdefined and 1 for overdefined, or 0 if well defined.
        '''

        # Save the existance of inputs
        R = (rho   is not None)     # rho exists
        A = (alpha is not None)     # alpha exists
        B = (beta  is not None)     # beta exists
        all_RAB = (R and A and B)   # all of (rho,alpha,beta) exists

        W = (ws is not None)
        
        X = (x   is not None)       # x exists
        Y = (y is not None)         # y exists
        H = (h  is not None)    # h exists
        all_XYH = (X and Y and H)   # all of (x,y,h) exists
        any_XYH = (X  or Y  or H)   # any of (x,y,h) exists


        # If you have input all (rho,alpha,beta)
        if (all_RAB):
            # But also input either WorldState or any of (x,y,h)
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

    @classmethod
    def check_init_spec(cls, rho, alpha, beta, ws, x, y, h):
        '''
        Throw errors if init specification is not well-defined.
        Otherwise, return a 
        '''
        sc = cls.well_defined(rho=rho, alpha=alpha, beta=beta, ws=ws,
                    x=x, y=y, h=h)
        if   (sc == 1):
            raise InputError("BicycleModelState specification overdefined.")
        elif (sc ==-1):
            raise InputError("BicycleModelState specification underdefined.")


    @staticmethod 
    def from_world(ws, robot_in_world_ref_frame=None):
        if robot_in_world_ref_frame is None:
            robot_in_world_ref_frame = WorldState(0,0,0)

        # # Translate first, since our translation vector is in world coordinates
        # translated = WorldState(ws.x-robot_in_world_ref_frame.x, ws.y-robot_in_world_ref_frame.y, ws.h)

        # # Then rotate robot to heading=0
        # rotated = translated.rotate(-robot_in_world_ref_frame.h)

        wrt_robot = ws.wrt(robot_in_world_ref_frame)

        # Now get rho, alpha, beta
        rho   = wrt_robot.norm()
        alpha = wrt_robot.theta()
        beta  = wrt_robot.h

        return BicycleModelState(rho=rho, alpha=alpha, beta=beta)




    def to_world(self, robot_in_world_ref_frame=None):
        if robot_in_world_ref_frame is None:
            robot_in_world_ref_frame = WorldState(0,0,0)

        # Find (x,y,h) coordinates w.r.t. robot
        x_in_robot_ref_frame = self.rho * cos(self.alpha) 
        y_in_robot_ref_frame = self.rho * sin(self.alpha) 
        h_in_robot_ref_frame = self.alpha - self.beta


        xyh_in_robot_ref_frame = WorldState(x_in_robot_ref_frame,
                                            y_in_robot_ref_frame,
                                            h_in_robot_ref_frame)


        # Rotate point along with robot back to correct heading while we are in robot ref frame
        xyh_rotated = xyh_in_robot_ref_frame.rotate(robot_in_world_ref_frame.h)

        # Now that our translation and world axes are aligned, we can translate to our world position
        translation_vector = copy.copy(robot_in_world_ref_frame)
        translation_vector.h = 0
        xyh = xyh_rotated + translation_vector

        return xyh


def test():
    Rw = WorldState(4,8,3*pi/4)
    Gw = WorldState(9,3,pi/4)
    print("Robot in world ref frame: {}\nGoal in world ref frame: {}".format(Rw,Gw))


    # print(Gw.wrt(Rw))

    Rbm = BicycleModelState.from_world(Rw, robot_in_world_ref_frame=Rw)
    Gbm = BicycleModelState.from_world(Gw, robot_in_world_ref_frame=Rw)

    print("World coords: {}\nBM coords: {}\n\n".format(Gw, Gbm))

    rw = Rbm.to_world(robot_in_world_ref_frame=Rw)
    gw = Gbm.to_world(robot_in_world_ref_frame=Rw)

    print("Original robot position: {}\nRecalculated robot position: {}\n".format(Rw, rw))
    print("Original goal position: {}\nRecalculated goal position: {}\n".format(Gw, gw))



if __name__=="__main__":
    test()
