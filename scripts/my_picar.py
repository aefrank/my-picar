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
import sys
from time import sleep
from math import sin, cos, tan, atan2, pi
from numpy.linalg import norm
import numpy as np
from helpers import sign

# Find SunFounder_PiCar submodule
sys.path.append("../SunFounder_PiCar")
import picar


def clamp(angle):
    '''
    Clamp angle from 0 to pi.
    '''
    return (angle+pi) % 2*pi - pi 


def angle_a2b(a,b):
    '''
    Shortest distance (angular) between two angles.
    It will be in range [-pi, pi].
    Credit:
    http://blog.lexique-du-net.com/index.php?post/Calculate-the-real-difference-between-two-angles-keeping-the-sign
    '''
    phi = b-a            
    while phi > pi:  
        phi = phi - 2*pi
    while phi < -pi:
        phi = phi + 2*pi
    return phi
    

    

##############################################################
#                       PID CONTROLLER
##############################################################

def P(kp=1, error=0):
    return kp*error

def I(ki=1, error=0, integral=0, dt=1):
    integral += new_error*dt
    return ki*integral

def D(kd=1, error=0, last_error=0, dt=1): 
    derivative = (error-last_error)/dt
    return kd*derivative



##############################################################
#                       ROBOT FRAME
##############################################################

def RHO(s,g):
    rho = g-s
    rho[2] = 0
    return rho

def ALPHA(s=[0,0,0],g=None,rho=None):
    if ((g is None) and (rho is None)) or ((g is not None) and (rho is not None)):
        raise InputError("You must input either g or rho.")
    if rho is None:
        rho = RHO(s,g)
    return angle_a2b( a=s[2], b=atan2(rho[1],rho[0]))

def BETA(s=None,g=[0,0,0],rho=None): 
    if ((s is None) and (rho is None)) or ((s is not None) and (rho is not None)):
        raise InputError("You must input either s or rho.")
    if rho is None:
        rho = RHO(s,g)
    return angle_a2b( a=atan2(rho[1],rho[0]), b=g[2] )


def dRHO(v,alpha):
    return -v*cos(alpha)

def dALPHA(v,gamma,rho,alpha):
    return angle_a2b(a=gamma, b=v*sin(alpha)/norm(rho))

def dBETA(v,rho,alpha):
    return angle_a2b(a=0, b=-v*cos(alpha)/norm(rho))




##############################################################
#                   PICAR CLASS
##############################################################

class Picar:

    def __init__(self, max_turn=45, speed=50, configfile="config", loop_delay=0.001, L=0.145,
                    kpr=1, kpa=0, kpb=0, max_speed_world=0.4, max_turn_world=0.626):
        picar.setup()

        self.L = L

        self.config_file = configfile
        self.fw = picar.front_wheels.Front_Wheels(db=configfile)
        self.bw = picar.back_wheels.Back_Wheels(db=configfile)
        self.fw.max_turn = max_turn # [deg]  In picar's world; in reality 45 maps to around 40, etc
        
        self.speed = speed

        self.fw.ready()
        self.bw.ready()
        self.bw.forward()

        self.loop_delay = loop_delay # [sec]

        self.Kpr = kpr
        self.Kpa = kpa
        self.Kpb = kpb

        self.MAX_SPEED = max_speed_world
        self.MAX_TURNING_ANGLE = max_turn_world


    def map_speed(self, spd):
        return int(213*spd)

    def map_turn(self, angle):
        angle = angle * 180 / pi    # picar deals in degrees
        if angle > 0:
            angle = int(angle*1.27 - 0.55)
        else:
            angle = int(angle*0.88 - 8.29)
        return angle


    ##############################################################
    #                       WORLD FRAME
    ##############################################################
    def V(self,rho):
        v = P(self.Kpr,norm(rho))
        if v > self.MAX_SPEED:
            v = self.MAX_SPEED
        return v

    def GAMMA(self,alpha,beta):
        gamma = angle_a2b(a=0, b=P(self.Kpa,alpha)+P(self.Kpb,beta))
        if abs(gamma) > self.MAX_TURNING_ANGLE:
            gamma = sign(gamma)*self.MAX_TURNING_ANGLE
        return gamma

    def dX(self,v,theta):
        return v*cos(theta)

    def dY(self,v,theta):
        return v*sin(theta)

    def dTHETA(self,v,gamma):
        return angle_a2b(a=0, b=v*tan(gamma)/self.L)


    def travel(self, waypoints):
        dt = self.loop_delay
        t = 0
        self.s = waypoints[0]
        g = waypoints[1]
        self.rho = RHO(self.s,g)


        # Stop when close enough
        while ( norm(self.rho)>0.1 and abs(self.s[2]-g[2])<pi/6 ):
            
            # Current world state
            # x = self.s[0]
            # y = self.s[1]
            theta = self.s[2]

            # Current ego-centric state
            self.rho = RHO(self.s,g)
            self.alpha = ALPHA(s=self.s,rho=self.rho)
            self.beta = BETA(g=g,rho=self.rho)

            # Calculate controls
            self.speed = self.V(self.rho)
            self.turn_angle = self.GAMMA(self.alpha,self.beta)

            # Send controls to hardware
            self.turn( self.map_turn(self.turn_angle) )
            self.bw.speed = self.map_speed(self.speed)

            # Calculate change in world state
            dx = self.dX(self.speed,theta)
            dy = self.dY(self.speed,theta)
            dtheta = self.dTHETA(self.speed,self.turn_angle)*dt

            # Update world state
            self.s = self.s + dt*np.array([dx, dy, dtheta])
            print("x: {:.4f}\ty: {:.4f}\tth: {:.4f}\tt: {:.3f}".format(self.s[0],self.s[1],self.s[2],t))

            sleep(self.loop_delay)
            t = t+dt





    
    ##############################################################
    #                   LOW LEVEL CONTROLS
    ##############################################################

    def turn(self, angle):
        '''
        Input angle in degrees.

        Make it so inputs angles are relative to 0 degrees being straight forward.
        '''
        self.fw.turn(angle + self.fw._straight_angle)

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






if __name__=="__main__":
    main()


