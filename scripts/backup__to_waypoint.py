'''
Filename: to_waypoint.py
Description: Controlled motion from s0=[x0 y0 h0] to s_goal=[xg yg hg] by basic PID controller.
Author: Andi Frank
E-mail: a2frank@eng.ucsd.edu
Purpose: CSE 276A - Intro to Robotics; Fall 2019
'''

##############################################################
#                       IMPORTS
##############################################################
from time import sleep
from math import pi, atan2, sin, cos, tan
import numpy as np
from numpy.linalg import norm
import helpers
from my_picar import Picar

# Proportional gain
Kpr = 0
Kpa = 0
Kpb = 0

# Integral gain
Kir = 0
Kia = 0
Kib = 0

# Derivative gain
Kdr = 0
Kda = 0
Kdb = 0

# PiCar wheel axis offset
L = 0.145 # [m]

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
#                       WORLD FRAME
##############################################################
def V(rho,kpr=Kpr):
    return P(kpr,rho)

def GAMMA(alpha,beta,kpa=Kpa,kpb=Kpb):
    return P(kpa,alpha)+P(kpb,beta)

def dX(v,theta):
    return v*cos(theta)

def dY(v,theta):
    return v*sin(theta)

def dTHETA(v,gamma,L=1):
    return v*tan(gamma)/L

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
    return atan2(rho[1],rho[0]) - s[2]

def BETA(s=None,g=[0,0,0],rho=None): 
    if ((s is None) and (rho is None)) or ((s is not None) and (rho is not None)):
        raise InputError("You must input either s or rho.")
    if rho is None:
        rho = RHO(s,g)
    return g[2] - atan2(rho[1],rho[0])


def dRHO(v,alpha):
    return -v*cos(alpha)

def dALPHA(v,gamma,rho,alpha):
    return v*sin(alpha)/norm(rho) - gamma

def dBETA(v,rho,alpha):
    return -v*cos(alpha)/norm(rho)




def main():

    waypoints = np.asarray(
                [ [      0,     0,      0],
                  [     -1,     0,      0],
                  [     -1,     1,   pi/2],
                  [     -2,     1,      0],
                  [     -2,     2,  -pi/2],
                  [     -1,     1,  -pi/4],
                  [     0,      0,      0]
                ]
            )

    pc = Picar(L=L)

    s = waypoints[0]
    g = waypoints[1]
    for i in range(10):
        
        theta = s[2]
        rho = RHO(s,g)
        alpha = ALPHA(s=s,rho=rho)
        beta = BETA(g=g,rho=rho)

        v = V(rho)
        gamma = GAMMA(alpha,beta)

        dx = dX(v,theta)
        dy = dY(v,theta)
        dtheta = dTHETA(v,pc.L,gamma)

        sleep(pc.loop_delay)



if __name__=="__main__":
    main()


