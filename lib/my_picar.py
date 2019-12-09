'''
Filename: my_picar.py
Description: Class and functions to represent a physical picar in the program.
Author: Andi Frank
E-mail: a2frank@eng.ucsd.edu
Purpose: CSE 276A - Intro to Robotics; Fall 2019
'''

##############################################################
#                       IMPORTS
##############################################################

import sys
from time import sleep, monotonic
from math import sin, cos, tan, atan, atan2, pi
from numpy.linalg import norm
import numpy as np

# Find SunFounder_PiCar submodule
sys.path.append("../lib/SunFounder_PiCar")

# Custom libraries
from helpers import sign, angle_a2b, within_pi, clip, InputError
from my_pid import myPID
import perspectives 
from cartesian_pose import CartesianPose
from bicycle_model  import BicyclePose, dXdt, dYdt, dHdt
from my_picar_utils import PicarHardwareInterface as Hardware 
from my_picar_utils import MyPicarController as Controller
from my_picar_utils import HardwareUnitConverter as Converter 
from my_picar_utils import MyWorldFrame as World 




##############################################################
#                   WORLD CLASS
##############################################################

class Picar:
    '''
    Class to represent Picar robot.
    '''
    
    def __init__(self,  hw = None, configfile="config", L=0.14, 
                        max_speed_hw=90, max_steer_hw=40, unit_converter=None, 
                        virtual=False, verbose=False, virtual_verbose=False):
        
        self.verbose = verbose

        # Hardware
        if hw is None:
            if unit_converter is None:
                unit_converter = Converter(speed_slope=1, angle_slope=1) # default to hardware units = world units
            hw = Hardware(max_steer=max_steer_hw, max_speed=max_speed_hw,
                       unit_converter=unit_converter, configfile=configfile, 
                       virtual=virtual, virtual_verbose=virtual_verbose)
        self.hw = hw
        self.L = L

        # Control State
        self._current_speed = 0
        self._current_steer = 0
        self._current_direction = 1 # Forward

        # Model 
        self._world_pose   = CartesianPose(0,0,0)
        self._goal_pose = CartesianPose(0,0,0)
        self._bicycle_pose = BicyclePose(0,0,0)




    def rho(self):
        return self._bicycle_pose.rho

    def alpha(self): 
        return self._bicycle_pose.alpha

    def beta(self): 
        return self._bicycle_pose.beta

    def x(self):
        return self._world_pose.x

    def y(self):
        return self._world_pose.y

    def h(self):
        return self._world_pose.h
    
    def world_pose(self):
        return self._world_pose

    def goal_pose(self):
        return self._goal_pose 

    def bicycle_pose(self):
        return self._bicycle_pose


    def set_pose(self, world_pose=None, goal_pose=None, bicycle_pose=None):
        '''
        Coordinates keeping self._world_pose, self._goal_pose, and self._bicycle_pose consistent.

        Valid Specifications:
        B undefined; G defined, W defined -> self.B=cartesian2bicycle(G,W); self.G=G, self.W=W
        G undefined; B defined, W defined -> self.G=bicycle2goal(B,W);      self.B=B, self.W=W
        W undefined; B defined, G defined -> self.W=bicycle2robot(B,G);     self.B=B, self.G=G

        B undefined, G undefined; W defined -> self.B=cartesian2bicycle(self.G,W); self.W=W
        B undefined, W undefined; G defined -> self.B=cartesian2bicycle(G,self.W); self.G=G
        NOTE: G undefined, W undefined; B defined -> UNDERDEFINED: should we preserve self.G and update self.W or opposite?
        '''
        W = (world_pose is not None)
        G = (goal_pose is not None)
        B = (bicycle_pose is not None)

        # Make sure inputs do not over- or under-define state.
        # Underdefined if neither G nor W is input
        if not G and not W:
            raise InputError("Pose underdefined. Must input at least one of goal_pose or world_pose.")
        # 3 non-None inputs: overdefined
        elif (not B) + (not G) + (not W) > 2:
            raise InputError("Pose overdefined. You must input at most two arguments.")

        # If bicycle pose is not input, it will be recalculated from the other inputs/current poses
        if not B:
            if W:
                self._world_pose = world_pose
            if G:
                self._goal_pose = goal_pose
            self._bicycle_pose = perspectives.cartesian2bicycle( robot_cartesian=self._world_pose, goal_cartesian=self._goal_pose)
        # If bicycle pose is input, we are calculating either W or G
        else:
            self._bicycle_pose = bicycle_pose
            if W:
                self._world_pose   = world_pose
                self._goal_pose    = perspectives.bicycle2goal(bicycle_pose=self._bicycle_pose, robot_cartesian=self._world_pose)
            elif G:
                self._goal_pose    = goal_pose
                self._world_pose   = perspectives.bicycle2robot(bicycle_pose=self._bicycle_pose, goal_cartesian=self._goal_pose)

                
    def next_pose(self, speed, steer, direction, dt):
        dxdt = dXdt(speed=speed, heading=self.h(), direction=direction)
        dydt = dYdt(speed=speed, heading=self.h(), direction=direction)
        dhdt = dHdt(speed=speed, steer=steer, direction=direction, L=self.L)
        return self._world_pose+CartesianPose(dxdt,dydt,dhdt)*dt

    ################################################
    ######### Highest level picar control ##########
    ################################################

    def drive(self, speed, steer=0, direction=None):
        '''
        Sets picar steer and speed in WORLD UNITS by calling 
        self.turn() and self.set_speed().
        '''
        controls = self.hw.send_controls(speed=speed, steer=steer, direction=direction)
        
        # Update current controls
        if speed is not None:
            if np.ndim(controls) == 1:
                self._current_speed = controls[0]
            else:
                self._current_speed = controls
        if steer is not None:
            if np.ndim(controls) == 1:
                self._current_steer = controls[1]
            else:
                self._current_steer = controls

        return self._current_speed, self._current_steer


    def halt(self):
        '''
        Stop picar and turn steering forward.
        '''
        self._current_speed, self._current_steer = self.hw.send_controls(speed=0, steer=0, direction=1)

    def turn_straight(self, overshoot=10, delay=0.05):
        '''
        NOTE: CAUSES BIIIG DELAYS WHEN USED IN LOOP BECAUSE OF delay ARG -- ONLY USE FOR MANUAL TUNING
        Often when "turning straight", the picar stops a bit short of fully straight
        forward. This is my attempt to remedy that by overshooting then correcting.

        Tune the overshoot [deg] and delay [sec] if it's too jittery.
        '''
        self.turn(overshoot)
        sleep(delay)
        self.turn(-overshoot)
        sleep(delay)
        self.turn(0)
    
    def turn(self, steer):
        '''
        Sets picar steering to steer in WORLD UNITS
        '''
        # Send control, then update _current_speed by control signal 
        # actually sent to hardware (potentially clipped by hardware)
        self._current_steer = self.hw.send_controls(steer=self._current_steer)

    def set_speed(self, speed, direction=None):
        '''
        Sets picar speed in WORLD UNITS.
        Positive speeds are forward and negative speeds are backward.
        ''' 
        # Send control, then update _current_speed by control signal 
        # actually sent to hardware (potentially clipped by hardware)
        self._current_speed = self.hw.send_controls(speed=int(speed), direction=direction)


    def set_direction(self, direction):
        self.hw.send_controls(direction=self._current_direction)

    

    # State queries -- controls
    def get_direction(self):
        return self._current_direction

    def get_speed(self):
        return self._current_speed

    def get_steer(self):
        return self._current_steer







def test():
    pc = Picar(verbose=True,virtual=True,virtual_verbose=False)
    waypoints = np.array( [
        [0,0,0],
        [1,-1,-pi/2]
        ])
    # pc.traverse(waypoints)

if __name__=="__main__":
    test()






