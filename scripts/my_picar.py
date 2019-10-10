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
from time import sleep, monotonic
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
    rho[2] = atan2(rho[1],rho[0])
    return rho

def ALPHA(s,r):
    return angle_a2b( a=s[2], b=r[2] )

def BETA(r,g):
    return angle_a2b( a=r[2], b=g[2] )

# def ALPHA(s,g):
#     rho = RHO(s,g)
#     return angle_a2b( a=s[2], b=rho[2] )
# 
# def BETA(s,g): 
#     rho = RHO(s,g)
#     return angle_a2b( a=rho[2], b=g[2] )


def dRHO(v,alpha):
    return -v*cos(alpha)

def dALPHA(v,gamma,rho,alpha):
    return angle_a2b(a=gamma, b=v*sin(alpha)/norm(rho[:2]))

def dBETA(v,rho,alpha):
    return angle_a2b(a=0, b=-v*cos(alpha)/norm(rho[:2]))




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
        
        # Deal with offsets around 0
        if abs(angle) < 5:
            return 0
        if abs(angle) < 10:
            return angle

        if angle > 0:
            angle = int(angle*1.25)
        else:
            angle = int(angle*1.16)
        return angle


    ##############################################################
    #                       WORLD FRAME
    ##############################################################
    def V(self,rho):
        v = P(self.Kpr,norm(rho[:2]))
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
        breakflag = False
        start = 0
        self.s = waypoints[start]
        g = waypoints[start+1]
        
        self.rho    = RHO  (self.s,g)
        self.alpha  = ALPHA(self.s,g)
        self.beta   = BETA (self.s,g)

        try:
            #Initialize times
            dt = self.loop_delay
            t = 0
            mono_time = monotonic()

            # Stop when close enough
            while ( norm(self.rho[:2])>0.1 or abs(self.s[2]-g[2])<pi/6 ) and not breakflag:

                # Calculate controls
                self.speed      = self.V (self.rho)
                self.turn_angle = self.GAMMA (self.alpha, self.beta)

                # Send controls to hardware
                self.turn (     self.map_turn(self.turn_angle) )
                self.bw.speed = self.map_speed(self.speed)

                # Calculate change in world state
                dx      = self.dX(     self.speed, self.s[2] )
                dy      = self.dY(     self.speed, self.s[2] )
                dtheta  = self.dTHETA( self.speed, self.turn_angle )

                # Update world state
                self.s = self.s + np.array([dx, dy, dtheta])*dt
                # Keep theta in [-pi, pi]
                self.s[2] = angle_a2b(a=0, b=self.s[2])

                # Update ego-centric state
                self.rho   = RHO   (self.s, g) 
                self.alpha = ALPHA (self.s, self.rho)   #(self.s,g) 
                self.beta  = BETA  (self.rho, g)        #(self.s,g)

                if t % 1 < dt:
                    print("World:  x: {:.2f}\ty: {:.2f}\tth: {:.2f}\tt: {:.3f}".format(
                        self.s[0], self.s[1], self.s[2]*180/pi, t) )
                    print("World:  dx: {:.2f}\tdy: {:.2f}\tdth: {:.2f}\tdt: {:.3f}".format(
                        dx, dy, dtheta*180/pi,dt ) )
                    print("Robot:  v: {:.2f}\tgam: {:.2f}\trho: {:.2f}\ta: {:.2f}\tb: {:.2f}".format(
                        self.speed, self.turn_angle*180/pi, norm(self.rho[:2]), self.alpha*180/pi, self.beta*180/pi) )
                    print("Robot:  da: {:.2f}\tdb: {:.2f}\tdrho: {:.2f}\trho_angle: {:.2f}".format(
                        dALPHA (self.speed, self.turn_angle, self.rho, self.alpha) * 180/pi,
                        dBETA  (self.speed, self.rho, self.alpha) *180/pi,
                        dRHO   (self.speed,self.alpha), 
                        self.rho[2] * 180/pi
                        ) )
                    print("Goal :  x: {:.4f}\ty: {:.4f}\tth: {:.4f}\t".format(
                        g[0],g[1],g[2]*180/pi) )
                    print()

                # Loop delay
                sleep(self.loop_delay)
                dt = monotonic() - mono_time
                mono_time = mono_time + dt
                t = t+dt

                #if t>2:
                #    breakflag = True

            print("Goal reached, halting.")
            return

        except Exception as e:
            raise e
        finally:
            self.halt()    



    
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
        sleep(delay)
        self.turn(-overshoot)
        sleep(delay)
        self.fw.turn_straight()


    def stop_motors(self):
        self.bw.stop()


    def halt(self):
        self.stop_motors()
        self.turn_straight()



if __name__=="__main__":
    main()


