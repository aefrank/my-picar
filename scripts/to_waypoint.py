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
import argparse as ap

# Add library folder to path
sys.path.append("../lib")
import helpers
from my_picar import Picar as picar


def main():

    # Parse input arguments
    parser = ap.ArgumentParser(description="Move Picar around waypoints.")
    # parser.add_argument("-v", "--verbose", action="store_true", default=False) #@TODO
    parser.add_argument("-s",  "--simulate", help="Use virtual_picar module to avoid errors from not running on RPi.", 
                                                        action="store_true",    default=False)
    parser.add_argument("--kpr", type=float, help="Proportional gain for rho.",     default=0)
    parser.add_argument("--kpa", type=float, help="Proportional gain for alpha.",   default=0)
    parser.add_argument("--kpb", type=float, help="Proportional gain for beta.",    default=0)
    parser.add_argument("--kir", type=float, help="Integral gain for rho.",         default=0)
    parser.add_argument("--kia", type=float, help="Integral gain for alpha.",       default=0)
    parser.add_argument("--kib", type=float, help="Integral gain for beta.",        default=0)
    parser.add_argument("--kdr", type=float, help="Derivative gain for rho.",       default=0)
    parser.add_argument("--kda", type=float, help="Derivative gain for alpha.",     default=0)
    parser.add_argument("--kdb", type=float, help="Derivative gain for beta.",      default=0)
    parser.add_argument("--waypoints", type=str, help=".txt file of waypoints.") #@TODO
    args = parser.parse_args()

    print(args)



    # Proportional gain
    Kpr = args.kpr
    Kpa = args.kpa
    Kpb = args.kpb

    # Integral gain
    Kir = args.kir
    Kia = args.kia
    Kib = args.kib

    # Derivative gain
    Kdr = 0
    Kda = 0
    Kdb = 0

    if args.waypoints is None:
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
    else:
        # @TODO: READ IN WAYPOINT FILE
        # waypoints = args.waypoints
        raise InputError("Waypoint file handling not implemented yet.")


    pc = picar(kpr=args.kpr, kpa=args.kpa, kpb=args.kpb, 
               kir=args.kir, #kia=args.kia, kib=args.kib, 
               kdr=args.kdr, #kda=args.kda, kdb=args.kdb, 
               virtual=args.simulate
            ) #, loop_delay=0.001) deprecated

    pc.travel(waypoints)


if __name__=="__main__":
    main()


