'''
Filename: my_picar_utils.py
Description: Picar utilities to help organize interfacing with the picar through the my_picar.Picar class.
Author: Andi Frank
E-mail: a2frank@eng.ucsd.edu
Purpose: CSE 276A - Intro to Robotics; Fall 2019
'''


##############################################################
#                       IMPORTS
##############################################################

import sys, os
from time import sleep, monotonic
from math import sin, cos, tan, atan, atan2, pi
from numpy.linalg import norm
import numpy as np

# My libraries
from helpers import sign, angle_a2b, within_pi, clip, InputError
from my_pid import myPID
from cartesian_pose import CartesianPose
from bicycle_model import BicyclePose, BicycleModel, dHdt, dXdt, dYdt
from costmap import Rect, Map

# Find SunFounder_PiCar submodule
sys.path.append("SunFounder_PiCar")




class PicarHardwareInterface():
    '''
    Handle low level interfacing with hardware via SunFounder_PiCar picar.front_wheels.Front_Wheels
    and picar.back_wheels.Back_Wheels classes (or their virtual_wheels analogs).
    '''

    def __init__(self, max_steer=40, max_speed=90,
                       unit_converter=None, configfile="config", 
                       virtual=False, virtual_verbose=False, initialize=True):
         # Use virtual libraries if requested (so we don't get GPIO errors for not running on a Pi)
        if not virtual:
            import picar
            from picar.front_wheels import Front_Wheels
            from picar.back_wheels import Back_Wheels
            picar.setup()
            self.fw = Front_Wheels(db=configfile)
            self.bw = Back_Wheels(db=configfile)
        else:
            from virtual_wheels import Virtual_Front_Wheels as Front_Wheels
            from virtual_wheels import Virtual_Back_Wheels as Back_Wheels
            self.fw = Front_Wheels(db=configfile, verbose=virtual_verbose)
            self.bw = Back_Wheels(db=configfile, verbose=virtual_verbose)


        if unit_converter is None:
            unit_converter = HardwareUnitConverter(speed_slope=1, angle_slope=1) # Assume 1:1 unit relationship by default
        self.unit_converter = unit_converter # HardwareUnitConverter class

        self.fw.max_turn = max_steer         # Max turn radius

        if initialize:
            self.initialize()


    # PUBLIC METHODS: All inputs assumed to be in WORLD UNITS

    def initialize(self):
        # Initialize hardware
        self.fw.ready()
        self.bw.ready()
        self.bw.forward()


    def turn_wheels(self, world_steer):
        '''
        Input steer in WORLD ANGLE UNITS.
        Make it so inputs steer are relative to 0 degrees being straight forward.
        '''
        steer = self.unit_converter.speed_world2hardware(world_steer)
        self.fw.turn(steer + self.fw._straight_angle)


    def turn_wheels_straight(self, overshoot=10, delay=0.05):
        '''
        Often when "turning straight", the picar stops a bit short of fully straight
        forward. This is my attempt to remedy that by overshooting then correcting.

        Tune the overshoot [deg] and delay [sec] if it's too jittery.
        '''
        self.turn_wheels(overshoot)
        sleep(delay)
        self.turn_wheels(-overshoot)
        sleep(delay)
        self.fw.turn_straight()
        self._steering = 0


    def stop_motors(self):
        '''
        Low level, minimal latency method to stop picar motors. Leaves steering/direction as is.
        '''
        self.bw.stop()
        self._speed= 0


    


    def send_controls(self, speed=None, steer=None, direction=None):
        '''
        Send current speed and steer_angle control signals to hardware.
        Note: direction=-1 will flip BOTH STEER AND SPEED INPUTS such that
                the steering is opposite, and the car will move in the desired direction
                THIS MIGHT NOT BE WISE but i think it is. COULD BE A BUG LOOK HERE IF 
                TURN DIRECTION IS MESSED UP WHEN BACKWARDS.
        '''
        # Send controls that have been defined
        controls_sent = []

        if direction is not None:
            self._send_direction(direction)
        else:
            direction = 1

        return_vals = None
        if steer is not None:
            steer = direction*int(steer) # reverse turn direction if we are going backward
            steer = self.unit_converter.angle_world2hardware(steer) # to HW units
            steer = self._send_steer(steer) 
            steer = self.unit_converter.angle_hardware2world(steer) # back to world units
            controls_sent.append(steer)

        if speed is not None:
            speed = int(abs(speed))
            speed = self.unit_converter.speed_hardware2world(speed) # to HW units
            speed = self._send_speed(speed)
            speed = self.unit_converter.speed_hardware2world(speed) # back to world units
            controls_sent.insert(0,speed)  # By convention, we want speed THEN steer

        return controls_sent     # Return control values that were sent to hardware in world units, 
                                 # which may have been modified if they exceeded hw limits


    # PRIVATE METHODS: All inputs assumed to be in PICAR UNITS

    def _send_speed(self, speed):
        '''
        Input speed in HARDWARE UNITS.
        Sends control signal to hardware.
        '''
        # Set picar speed
        if speed==0:
            self.stop_motors()
        else: 
            # Bound by max speed
            if abs(speed) > self.max_speed:
                speed = sign(speed)*self.max_speed
            self.bw.speed = speed

        return speed # Returns value of speed, which MAY HAVE BEEN CHANGED if over max speed


    def _send_steer(self, steer):
        '''
        Input steer in HARDWARE UNITS.
        Sends control signal to hardware.
        '''
        # Set picar steering angle
        if steer==0:
            self.turn_wheels_straight()
        else:
            # Bound by max steer
            if abs(steer) > self.max_steer:
                steer = sign(steer)*self.max_steer
                steer_changed = steer
            self.turn_wheels(steer)

        return steer # Returns value of steer, which MAY HAVE BEEN CHANGED if over max steer

    def _send_direction(self, direction):
        # Set back wheel direction
        if direction == 1:
            self.bw.forward()
        elif direction == -1:
            self.bw.backward()
        elif direction == 0:
            # If direction is 0, don't change bw's settings.
            pass
        else:
            # If any other edge case, halt picar
            self.stop_motors()
            self.turn_wheels_straight()
            print('Picar object has invalid direction field {}. Picar halting.'.format(direction))
            return None # failure flag

        return direction
        

        



def test_PicarHardwareInterface():
    '''
    Test PicarHardwareInterface constructor and methods
    '''
    hw = PicarHardwareInterface(virtual=True)
    hw.turn_wheels(-20)
    hw.turn_wheels_straight()
    hw.stop_motors()
    hw.apply_controls(1,-1,1)

class MyPicarController():
    '''
    Basic PID control for Picar speed and steering angle based on a BicycleModel.bicycle_model representation
    of the relative goal pose.
    '''
    def __init__(self, rho_controller=None, alpha_controller=None, beta_controller=None,
                        kpr=1, kpa=0, kpb=0, kdr=0, kda=0, kdb=0, kir=0, kia=0, kib=0):
        if rho_controller is None:
            rho_controller   = myPID(Kp=kpr, Ki=kir, Kd=kdr)
        if alpha_controller is None:
            alpha_controller = myPID(Kp=kpa, Ki=kia, Kd=kda)
        if beta_controller is None:
            beta_controller  = myPID(Kp=kpb, Ki=kib, Kd=kdb)      

        self.rho_controller   = rho_controller
        self.alpha_controller = alpha_controller
        self.beta_controller  = beta_controller

    def SPEED(self, rho, dt=1):
        return self.rho_controller.input(rho, dt=dt)

    def STEER(self, speed, alpha, beta, L, dt=1): 
        '''
        Note: Returns steer direction assuming only forward motion is possible.
        '''
        a = self.alpha_controller.input(alpha, dt=dt) 
        b = self.beta_controller .input(beta,  dt=dt)  

        # Weighted change in gamma = desired dh
        dh = a + b

        # Steer angle is measured from the front wheel, not the back wheel; 
        s = abs(speed) # make sure we have unsigned speed; direction should be handled separately
        steer = atan(dh*L/s)

        # Bound between [-pi, pi]
        return within_pi(steer)

    def DIRECTION(self, alpha, dt=1):
        # If alpha is greater than pi/2, it's easier to go backward
        # Inspired by code.py example from Homework 1.
        # https://d1b10bmlvqabco.cloudfront.net/attach/k0uju462t062l4/j12evy3w52o5kl/k1vnoghb1697/code.pdf
        if abs(alpha) > (pi/2 + 1e-4):
            return True
        else:
            return False

def test_MyPicarController():
    pc = MyPicarController()
    pc = MyPicarController(rho_controller=myPID(1,2,3))
    s = pc.SPEED(rho=4.2)
    pc.STEER(speed=s, alpha=pi/2, beta=-pi/4, L=2)
    pc.DIRECTION(alpha=pi/2)



class HardwareUnitConverter():
    '''
    Handle conversions between world units and picar internal units. ASSUMES LINEAR RELATIONSHIPS.
    '''
    def __init__(self, speed_slope, angle_slope, time_slope=1, speed_intercept=0, angle_intercept=0):
        '''
        World = m*Picar + b
        '''
        self.world_speed_scale  = speed_slope       # [m/s / hardware_speed_unit] (i.e. (world speed units)/(picar speed units))
        self.world_angle_scale  = angle_slope       # [deg/hardware_angle_unit]
        self.world_time_scale   = time_slope        # [sec/hardware_time_unit]     (most likely hardware_time_unit will be sec)
        self.world_speed_offset = speed_intercept   # [m/s]
        self.world_angle_offset = angle_intercept   # [deg]


    def speed_hardware2world(self, hardware_speed):
        return self.world_speed_scale*hardware_speed + self.world_speed_offset     # [m/s]

    def angle_hardware2world(self, hardware_angle):
        return self.world_angle_scale*hardware_angle + self.world_angle_offset     # [deg]

    def time_hardware2world(self, hardware_time):
        return self.world_time_scale*hardware_time     # [sec]

    def speed_world2hardware(self, world_speed):
        return (world_speed - self.world_speed_offset)/self.world_speed_scale   # [picar speed units] 
                                                                                # (i.e. [picar length units/picar time units])
    def angle_world2hardware(self, world_angle):
        return (world_angle - self.world_angle_offset)/self.world_angle_scale   # [picar angle units] 

    def time_world2hardware(self, world_time):
        return world_time/self.world_time_scale                                 # [picar time units]

def test_HardwareUnitConverter():
    X = [-1, -5.6, 7, 92.3]
    uc = HardwareUnitConverter(speed_slope=3, angle_slope=4, time_slope=1, 
                                speed_intercept=0, angle_intercept=0)
    for x in X:
        uc.angle_hardware2world(x)
        uc.time_hardware2world(x)
        uc.speed_hardware2world(x)
        uc.angle_hardware2world(x)
        uc.time_hardware2world(x)
        uc.speed_hardware2world(x)


class MyWorldFrame():
    '''
    Specifically for Picar operation environments in CSE 276A.
    Describes the picar pose, goal pose, environment, etc. from the world reference frame 
        in cartesian coordinates in world units. 
    Assumes a rectangular environment.
    '''

    def __init__(self,  xlim=None, ylim=None, 
                        obstacles=None, obstacle_val=1, resolution=1, costmap=None,
                        picar_pose=CartesianPose(0,0,0), goal_pose=CartesianPose(0,0,0), waypoints=None,
                        picar_wheelbase=0.14, qr_codes=None):
        '''
        @TODO: Environment stuff (lims, costmap, obstacles, etc.) yet to be implemented -- just placeholder arguments for now.
        '''
        if xlim is None:
            xlim = [0,10]

        if ylim is None:
            ylim = xlim     # Default to square map
        
        if costmap is None:
            costmap = Map(origin=[0,0], xlim=xlim, ylim=ylim,
                          resolution=resolution, fill=0)
        if obstacles is not None:
            for obs in obstacles: # obstacles should be a list of Rects
                costmap.fill_rect(obs, fill=obstacle_val)

        if waypoints is None:
            waypoints = [goal_pose]

        self.xlim = xlim
        self.ylim = ylim
        self.costmap    = costmap
        self.obstacles  = obstacles
        self.qr_codes   = qr_codes
        self.picar_pose = picar_pose
        self.goal_pose  = goal_pose
        self.waypoints  = waypoints
        self.picar_wheelbase = picar_wheelbase

    def next_picar_pose(self, speed, steer, dt, direction=1, picar_pose=None):
        if picar_pose is None:
            picar_pose = self.picar_pose
        dx = dXdt(speed=speed, heading=picar_pose.h, direction=direction)*dt
        dy = dYdt(speed=speed, heading=picar_pose.h, direction=direction)*dt
        dh = dHdt(speed=speed, steer=steer, direction=direction, L=self.picar_wheelbase)*dt
        return picar_pose + CartesianPose(dx,dy,dh)




        


def test_MyWorldFrame():
    wf = MyWorldFrame()
    dt = 0.1
    T = 1
    for dt in range(int(T/dt)):
        wf.picar_pose = wf.next_picar_pose(speed=0.5, steer=pi/4, dt=dt, picar_pose=wf.picar_pose)
        print(wf.picar_pose)


def main():
    # test_PicarHardwareInterface()
    # test_MyPicarController()
    # test_HardwareUnitConverter()
    test_MyWorldFrame()


if __name__ == "__main__":
    main()