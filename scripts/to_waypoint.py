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
import sys
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


def main():
    # Proportional gain
    Kpr = 0
    Kpa = 0
    Kpb = 0

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

    if len(sys.argv) > 1:
        Kpr = float(sys.argv[1])
    if len(sys.argv) > 2:
        Kpa = float(sys.argv[2])
    if len(sys.argv) > 3:
        Kpb = float(sys.argv[3])

    pc = Picar(kpr=float(Kpr), kpa=float(Kpa), kpb=float(Kpb), loop_delay=0.001)

    pc.travel(waypoints)


if __name__=="__main__":
    main()


