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
            self.y = xyh[1]
            self.h = xyh[2]
        else:
            self.x = x
            self.y = y
            self.h = h

        # In case 'h' is confusing
        self.heading = self.h


    #############################################
    #         OVERRIDE BUILT-IN METHODS         #
    #############################################

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

    def __mul__(self, k):
        return WorldState(k*self.x, k*self.y, k*self.h)



    ###################################
    #         UTILITY METHODS         #
    ###################################

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



    ##################################################################################
    ##################################################################################
    ##################################################################################
    ##################################################################################
    ##################################################################################



class BicycleModel():
    '''
    - Keep track of a relative state from the perspective of the robot and 
        allow easily readable access to (rho,alpha,beta).
    - Calculate (rho,alpha,beta) from robot and goal WorldStates. 
    - Update (rho,alpha,beta) from v and gamma.
    '''


    #########################################
    #          Instantiation Methods        #
    #########################################


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
            rho = BicycleModel.RHO(robo_ws,ws)
        
        if alpha is None:
            alpha = BicycleModel.ALPHA()

        if beta is None:
            beta = BicycleModel.BETA()
        

        self.rho   = rho
        self.alpha = alpha
        self.beta  = beta

    @classmethod
    def check_init_spec(cls, rho, alpha, beta, ws, x, y, h):
        '''
        Throw errors if init specification is not well-defined.
        Otherwise, return a 
        '''
        sc = cls.well_defined(rho=rho, alpha=alpha, beta=beta, ws=ws,
                    x=x, y=y, h=h)
        if   (sc == 1):
            raise InputError("BicycleModel specification overdefined.")
        elif (sc ==-1):
            raise InputError("BicycleModel specification underdefined.")

    @classmethod
    def well_defined(cls, rho, alpha, beta, ws, x, y, h):
        '''
        CHECK FOR UNDER- OR OVERDEFINED BicycleModel SPECIFICATION:
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

    
    #########################################
    #       Overload Built-In Methods       #
    #########################################


    def __str__(self):
        return "BicycleModel: ({:.3f}, {:.3f}, {:.3f})".format(self.rho, self.alpha, self.beta)



    #####################################################
    #       Going btw BicycleModel and WorldState       #
    #####################################################

    def goal_worldstate(self, robot_in_world_ref_frame=None):
        '''
        Calculate the world coordinates of the goal point
        '''
        if robot_in_world_ref_frame is None:
            robot_in_world_ref_frame = WorldState(0,0,0)

        # Find (x,y,h) coordinates w.r.t. robot
        goal_x_in_robot_ref_frame = self.rho * cos(self.alpha) 
        goal_y_in_robot_ref_frame = self.rho * sin(self.alpha) 
        goal_h_in_robot_ref_frame = self.alpha - self.beta


        goal_in_robot_ref_frame = WorldState(goal_x_in_robot_ref_frame,
                                             goal_y_in_robot_ref_frame,
                                             goal_h_in_robot_ref_frame)

        # Rotate point along with robot back to correct heading while we are in robot ref frame
        goal_rotated = goal_in_robot_ref_frame.rotate(robot_in_world_ref_frame.h)

        # Now that our translation and world axes are aligned, we can translate to our world position
        translation_vector = copy.copy(robot_in_world_ref_frame)
        translation_vector.h = 0
        goal = goal_rotated + translation_vector

        return xyh


    #************************************ BEGIN STATIC METHODS *****************************

    @staticmethod 
    def from_goal_worldstate(goal, robot_in_world_ref_frame=None):
        '''
        Calculate the BicycleModel given a goal WorldState
        '''
        if robot_in_world_ref_frame is None:
            robot_in_world_ref_frame = WorldState(0,0,0)

        goal_wrt_robot = goal.wrt(robot_in_world_ref_frame)

        # Now get rho, alpha, beta
        rho   = goal_wrt_robot.norm()
        alpha = goal_wrt_robot.theta()
        beta  = goal_wrt_robot.h

        return BicycleModel(rho=rho, alpha=alpha, beta=beta)


    

    ###############################################################################
    #          Calculate BicycleModel parameters from robot and goal poses        #
    ###############################################################################

    @staticmethod 
    def RHO(robot,goal):
        '''
        Magnitude of the vector from (robot location) to (goal location).

        Robot and goal poses should be given as WorldState objects.
        '''
        difference = goal - robot
        return difference.norm()

    @staticmethod 
    def ALPHA(robot,goal):
        '''
        Angle from the robot's heading to the vector from (robot location) to (goal location).

        Robot and goal poses should be given as WorldState objects.
        '''
        difference = goal - robot
        return under_pi(difference.theta())

    @staticmethod 
    def BETA(robot,goal): 
        '''
        Angle from the vector from (robot location) to (goal location) to the goal heading angle.

        Robot and goal poses should be given as WorldState objects.
        '''
        difference = goal - robot
        return angle_a2b(a=difference.theta, b=difference.h)

    @staticmethod 
    # Bicycle model coordinates
    def dRHO(v,alpha):
        return -v*cos(alpha)

    @staticmethod 
    def dALPHA(v,gamma,rho,alpha):
        return angle_a2b(a=gamma, b=v*sin(alpha)/rho)

    @staticmethod 
    def dBETA(v,rho,alpha):
        return under_pi(-v*cos(alpha)/rho)



    ##############################
    #     Calculate Controls     #
    ##############################

    @staticmethod 
    def V(rho, rho_controller, dt=1):
        return rho_controller.input(rho, dt=dt)


    @staticmethod 
    def GAMMA(v, alpha, beta, alpha_controller, beta_controller, L=1, dt=1):
        a = alpha_controller.input(alpha, dt=dt) 
        b = beta_controller .input(beta,  dt=dt)  

        # Weighted change in gamma = desired dh
        dh = a + b

        # Turn gamma is measured from the front wheel, not the back wheel; 
        # Get front wheel turn gamma
        s = abs(v) # make sure we have unsigned speed; direction is already handles
        gamma = atan(dh*L/s)

        # Bound between [-pi, pi]
        return under_pi(gamma)

    @staticmethod 
    def should_i_back_up(alpha):
        # If alpha is greater than pi/2, it's easier to go backward
        # Inspired by code.py example from Homework 1.
        # https://d1b10bmlvqabco.cloudfront.net/attach/k0uju462t062l4/j12evy3w52o5kl/k1vnoghb1697/code.pdf
        if abs(alpha) > (pi/2 + 1e-4):
            return True
        else:
            return False


    


def test():
    Rw = WorldState(4,8,3*pi/4)
    Gw = WorldState(9,3,pi/4)
    print("Robot in world ref frame: {}\nGoal in world ref frame: {}".format(Rw,Gw))


    # print(Gw.wrt(Rw))

    Rbm = BicycleModel.from_world(Rw, robot_in_world_ref_frame=Rw)
    Gbm = BicycleModel.from_world(Gw, robot_in_world_ref_frame=Rw)

    print("World coords: {}\nBM coords: {}\n\n".format(Gw, Gbm))

    rw = Rbm.to_world(robot_in_world_ref_frame=Rw)
    gw = Gbm.to_world(robot_in_world_ref_frame=Rw)

    print("Original robot position: {}\nRecalculated robot position: {}\n".format(Rw, rw))
    print("Original goal position: {}\nRecalculated goal position: {}\n".format(Gw, gw))



if __name__=="__main__":
    test()
