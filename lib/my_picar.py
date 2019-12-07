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
from helpers import sign, angle_a2b, within_pi, clip
from my_pid import myPID
import perspectives 
from cartesian_pose import CartesianPose
from bicycle_model  import BicyclePose, BicycleModel
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

        # Control State
        self.current_speed = 0
        self.current_steer = 0
        self.current_direction = 1 # Forward

        # Model 
        self.bicycle_model = BicycleModel(bicycle_pose=BicyclePose(0,0,0), L=L)
        self.world_pose = CartesianPose(0,0,0)


    # Highest level picar control
    def drive(self, speed, steer=0, direction=None):
        '''
        Sets picar steering and speed in WORLD UNITS by calling 
        self.turn() and self.set_speed().
        '''
        controls = self.hw.send_controls(speed=speed, steer=steer, direction=direction)
        
        # Update current controls
        if speed is not None:
            if np.ndim(controls) == 1:
                self.current_speed = controls[0]
            else:
                self.current_speed = controls
        if steer is not None:
            if np.ndim(controls) == 1:
                self.current_steer = controls[1]
            else:
                self.current_steer = controls


    def halt(self):
        '''
        Stop picar and turn steering forward.
        '''
        self.current_speed, self.current_steer = self.hw.send_controls(speed=0, steer=0, direction=1)

    
    def turn(self, steer):
        '''
        Sets picar steering to steer in WORLD UNITS
        '''
        # Send control, then update current_speed by control signal 
        # actually sent to hardware (potentially clipped by hardware)
        self.current_steer = self.hw.send_controls(steer=self.current_steer)

    def set_speed(self, speed, direction=None):
        '''
        Sets picar speed in WORLD UNITS.
        Positive speeds are forward and negative speeds are backward.
        ''' 
        # Send control, then update current_speed by control signal 
        # actually sent to hardware (potentially clipped by hardware)
        self.current_speed = self.hw.send_controls(speed=int(speed), direction=direction)


    def set_direction(self, direction):
        self.hw.send_controls(direction=self.current_direction)

    

    # State queries -- controls
    def get_direction(self):
        return self.current_direction

    def get_speed(self):
        return self.current_speed

    def get_steering(self):
        return self.current_steering

    # # State queries -- pose
    # def set_my_cartesian(self, pose=None, xyh=None):
    #     '''
    #     Set a new cartesian pose for the picar. Recalculate the BicyclePose based on this
    #     new picar pose and the current goal pose.

    #     Goal pose itself is not updated.
    #     '''
    #     if pose is None:
    #         pose = CartesianPose(*xyh)
    #     self._my_cartesian_pose = pose
    #     self._my_bicycle_model = pr.cartesian_to_bicycle(self._goal_cartesian_pose, robot_in_cartesian_ref_frame=pose)

    # def set_goal_cartesian(self, goal=None, xyh=None):
    #     '''
    #     Set a new goal pose for the picar. Recalculate the BicyclePose based on this
    #     new goal pose and the current picar pose.

    #     Picar pose itself is not updated.
    #     '''
    #     if goal is None:
    #         goal = CartesianPose(*xyh)
    #     self._goal_cartesian_pose = goal
    #     self._my_bicycle_model = pr.cartesian_to_bicycle(goal, robot_in_cartesian_ref_frame=self._my_cartesian_pose)

    # def set_bicycle_pose(self, pose=None, rab=None):
    #     '''
    #     Set a new bicycle pose for the scenario. Assume the picar's cartesian pose is unchanged, and 
    #     recalculate the cartesian Goal Pose based on this new bicycle pose and the current picar pose.

    #     Picar pose itself is not updated.
    #     '''
    #     if pose is None:
    #         pose = BicyclePose(*rab)
    #         self._my_bicycle_model.pose = pose
    #         self._goal_cartesian_pose = pr.bicycle_to_cartesian(pose, 
    #             robot_in_cartesian_ref_frame=self._my_cartesian_pose)






def test():
    pc = Picar(verbose=True,virtual=True,virtual_verbose=False)
    waypoints = np.array( [
        [0,0,0],
        [1,-1,-pi/2]
        ])
    # pc.traverse(waypoints)

if __name__=="__main__":
    test()






