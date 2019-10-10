'''
Filename: Picar.py
Description: My implementation of the PiCar as a class.
Author: Andi Frank
E-mail: a2frank@eng.ucsd.edu
Purpose: CSE 276A - Intro to Robotics; Fall 2019
'''

##############################################################
#                       IMPORTS
##############################################################
import sys, time
import helpers

# Find SunFounder_PiCar submodule
sys.path.append("../SunFounder_PiCar")
import picar



##############################################################
#                   PICAR CLASS
##############################################################

class Picar:

    def __init__(self, max_turn=45, speed=50, configfile="config", loop_delay=0.001, L=0.145):
        picar.setup()

        self.L = L

        self.config_file = configfile
        self.fw = picar.front_wheels.Front_Wheels(db=configfile)
        self.bw = picar.back_wheels.Back_Wheels(db=configfile)
        self.fw.max_turn = max_turn # [deg]  In picar's world; in reality 45 maps to around 40, etc
        
        self.speed = speed

        self.fw.ready()
        self.bw.ready()

        self.loop_delay = loop_delay # [sec]


    def turn(self, angle):
        '''
        Input angle in degrees.

        Make it so inputs angles are relative to 0 degrees being straight forward.
        '''
        self.fw.turn(angle + fw._straight_angle)

    def turn_straight(self, overshoot=10, delay=0.05):
        '''
        Often when "turning straight", the picar stops a bit short of fully straight
        forward. This is my attempt to remedy that by overshooting then correcting.

        Tune the overshoot [deg] and delay [sec] if it's too jittery.
        '''
        self.turn(overshoot)
        time.sleep(delay)
        self.turn(-overshoot)
        time.sleep(delay)
        fw.turn_straight()



