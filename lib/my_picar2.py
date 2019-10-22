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
from math import sin, cos, tan, atan, atan2, pi
from numpy.linalg import norm
import numpy as np
from helpers import sign, angle_a2b, shortest_rotation, clip
from my_pid import PID

# Find SunFounder_PiCar submodule
sys.path.append("../lib/SunFounder_PiCar")



kspeed = 215



##############################################################
#                  PARAMETER FUNCTIONS
##############################################################

def RHO(s,g):
    # offset = (g-s).norm()
    rho = g-s
    rho[2] = atan2(rho[1],rho[0])
    return rho

def ALPHA(s,g):
    rho = RHO(s,g)
    return angle_a2b( a=s[2], b=rho[2] )

def BETA(s,g): 
    rho = RHO(s,g)
    return angle_a2b( a=rho[2], b=g[2] )


def dRHO(v,alpha):
    '''
    Calculates change in the MAGNITUDE ONLY of rho.
    '''
    return -v*cos(alpha)

def dALPHA(v,gamma,rho,alpha):
    return angle_a2b(a=gamma, b=v*sin(alpha)/norm(rho[:2]))

def dBETA(v,rho,alpha):
    return shortest_rotation(-v*cos(alpha)/norm(rho[:2]))


def dX(v,h):
    return v*cos(h)

def dY(v,h):
    return v*sin(h)

def dTHETA(v,gamma,L):
    return shortest_rotation(v*tan(gamma)/L)



##############################################################
#                   PICAR CLASS
##############################################################

class Picar:
    '''
    Class to represent Picar parameters and outputs.
    '''
    
    # @TODO: Clean up input parameters to the important ones; try to group Ks
    def __init__(self, max_turn=45, speed=50, configfile="config", L=0.145,
                    kpr=1, kpa=0, kpb=0, kdr=0, kir=0,
                    max_picar_speed=80, max_picar_turn=40,
                    virtual=False, min_dt=0.005, verbose=False):

        self.verbose = verbose

        # Use virtual libraries if requested (so we don't get GPIO errors for not running on a Pi)
        if not virtual:
            import picar
            from picar.front_wheels import Front_Wheels
            from picar.back_wheels import Back_Wheels
            picar.setup()
        else:
            from virtual_picar import Virtual_Front_Wheels as Front_Wheels
            from virtual_picar import Virtual_Back_Wheels as Back_Wheels

        # Wheel base [m]
        self.L = L        

        # Wheel objects
        self.fw = Front_Wheels(db=configfile)
        self.bw = Back_Wheels(db=configfile)
        self.fw.max_turn = max_turn # [deg]  In picar's world; in reality 45 maps to around 40, etc
        # Initialize wheels
        self.speed = speed      # from 0-100
        self.turn_angle = 0     # initalize at forward
        self.fw.ready()
        self.bw.ready()
        self.bw.forward()

        # PID controllers for parameters
        self.rhoPID = PID(Kp=kpr, Ki=kir, Kd=kdr)
        self.alphaPID = PID(Kp=kpa)
        self.betaPID = PID(Kp=kpb)

        # Set maximum values of control signals -- IN PICAR UNITS (e.g. degrees, not radians)
        self.MAX_PICAR_SPEED = max_picar_speed
        self.MAX_PICAR_TURN = max_picar_turn

        # Minimum loop delay
        self.min_dt = min_dt




    ##############################################################
    #               APPLY CALIBRATION MAPS
    ##############################################################

    # Maps --> use calibration to map real world speed/angle to the control signals for the Picar motors
    # @TODO: Address 'kspeed' parameter
    def speed_world2picar(self, spd, m=215, b=0):
        '''
        Map real-world speed in meters-per-second to 0-100 picar speed based on calibration.
        '''
        spd = int(m*abs(spd) + b)
        spd = min( max(spd,0), self.MAX_PICAR_SPEED) # bound speed between 0 and MAX_PICAR_SPEED
        return spd

    # The reverse; go from control signal to real world
    def speed_picar2world(self, spd, m=215, b=0):
        '''
        Map picar 0-100 speed to real-world speed in meters-per-second based on calibration.
        '''
        return (spd+b)/m

    def turnangle_world2picar(self, angle, m_right=1, b_right=0, m_left=None, b_left=None):
        '''
        Map real-world turn-angle in radians to picar turn-angle in degrees based on calibration.
        '''
        ang = -angle * 180 / pi    # picar deals in degrees, and treats left (CCW) as negative

        if m_left is None:
            m_left = m_right
        if b_left is None:
            b_left = b_right

        # Turn right
        if ang > 0:
            ang = int(m_right*ang + b_right)
        else:
            ang = int(m_left*ang + b_left)

        # bound angle between -max and +max
        if abs(ang) > self.MAX_PICAR_TURN:
            ang = sign(ang)*self.MAX_PICAR_TURN

        return ang

    def turnangle_picar2world(self, angle, m_right=1, b_right=0, m_left=None, b_left=None):
        '''
        Map picar turn-angle in degrees to real-world turn-angle in radians based on calibration.
        '''
        ang = -angle * pi / 180    # change to radians, and treat left turn (CCW) as positive

        if m_left is None:
            m_left = m_right
        if b_left is None:
            b_left = b_right

        # Turn right
        if ang > 0:
            ang = int( (ang + b_right)/m_right )
        else:
            ang = int( (ang + b_left) /m_left )

        return ang


    ##############################################################
    #                    CALCULATE CONTROLS
    ##############################################################

    def get_drive_direction(self):
        # If alpha is greater than pi/2, it's easier to go backward
        # Inspired by code.py example from Homework 1.
        # https://d1b10bmlvqabco.cloudfront.net/attach/k0uju462t062l4/j12evy3w52o5kl/k1vnoghb1697/code.pdf
        if abs(self.alpha) > (pi/2.0 + 1e-4):
            # car should drive backward
            return -1
        else:
            # car should drive forward
            return 1

    def set_drive_direction(self, direction=None):
        # Set bw direction to forward or backward.
        # Inspired by code.py example from Homework 1.
        # https://d1b10bmlvqabco.cloudfront.net/attach/k0uju462t062l4/j12evy3w52o5kl/k1vnoghb1697/code.pdf
        if direction==None:
            direction = self.get_drive_direction()

        if direction==1:
            # drive forward
            self.bw.forward()
        elif direction==-1:
            # drive backward
            self.bw.backward()
        else:
            raise InputError("Direction must be -1 (backward) or 1 (forward).")


    def V(self, dt=1):
        '''
        Use PID control to calculate velocity.
        '''
        rho = norm(self.rho)
        v = self.rhoPID.input(rho)
        # Bound from [0, MAX_PICAR_SPEED]
        print(v)
        v = min(max(v,0), self.MAX_PICAR_SPEED)
        return v

    # TODO? Change steering based on rho? e.g. if rho is changing quickly err toward alpha
    def GAMMA(self, v, dt=1):
        '''
        Use PID control to calculate turn angle.
        '''
        a = self.alphaPID.input(self.alpha, dt=dt) 
        b = self.betaPID .input(self.beta,  dt=dt)  

        # Weighted change in angle = desired dh
        dh = a + b

        # Check if no turn is required
        if dh<1e-4:
            return 0

        # Switch direction of d_heading if we are driving backward
        dh = dh*self.get_drive_direction()

        # Turn angle is measured from the front wheel, not the back wheel; 
        # Get front wheel turn angle
        s = abs(v) # make sure we have unsigned speed; direction is already handles
        gamma = atan(dh*self.L/s)

        # Clip to max
        bound = self.turnangle_picar2world( self.MAX_PICAR_TURN );
        gamma = clip(gamma, -bound, bound)

        # Bound between [-pi, pi]
        return shortest_rotation(gamma)

    



    ##############################################################
    #                    TRAVERSE WAYPOINTS
    ##############################################################

    '''
    The big important function (as of now).
    '''
    def traverse(self, waypoints):
        # Breakflag; as of now it never gets set to True, but keeping around in
        # case we want it later so we don't have to reimplement.
        # NOTE: while loop DOES check for it right now.
        breakflag = False

        ##############################################################
        #                       INITIALIZE
        ##############################################################
        # IMPORTANT: Initialize printable variables (so they don't cause errors
        #   when the try-catch statement tries to catch and error and print out
        #   variables that haven't been initialized yet in 'finally:')

        # Initialize world parameters
        self.s = waypoints[0]
        g = waypoints[1]
        dx=0
        dy=0
        dh=0

        # Initialize robot-centric reference
        self.rho    = RHO  (self.s,g)
        self.alpha  = ALPHA(self.s,g)
        self.beta   = BETA (self.s,g)

        
        # Initialize times
        dt = self.min_dt
        t = 0

        ##############################################################
        #                        CONTROL LOOP
        ##############################################################
        try:
            stopwatch = monotonic()
            
            # For each waypoint
            for i in range(len(waypoints)-1):
                self.rho    = RHO  (self.s,g)
                self.alpha  = ALPHA(self.s,g)
                self.beta   = BETA (self.s,g)
                
                # Stop when close enough
                while ( not (    norm(self.rho[:2])<0.1                 # within delta (10cm) of goal
                            and  abs(angle_a2b(self.s[2],g[2])) < pi/6  # heading is almost correct
                            )
                        and not breakflag
                    ):

                    # Calculate controls
                    speed           = self.V (dt=dt)
                    control_speed   = self.speed_world2picar(speed)             
                    # Re-map controls to real world -> this will account for bounding
                    #   the speed/angle to the picar's working range.
                    self.speed      = self.speed_picar2world(control_speed)

                    turn_angle = self.GAMMA (v=self.speed, dt=dt)
                    control_angle   = self.turnangle_world2picar(turn_angle)
                    # Re-map controls to real world -> this will account for bounding
                    #   the speed/angle to the picar's working range.
                    self.turn_angle = self.turnangle_picar2world(control_angle)

                    
                    # Send controls to hardware
                    self.set_drive_direction()
                    self.turn (       control_angle )
                    self.bw.speed   = control_speed

                    # Re-map controls to real world -> this will account for bounding
                    #   the speed/angle to the picar's working range.
                    self.turn_angle = self.turnangle_picar2world(control_angle)

                    # Calculate change in world state
                    dx = dX(     self.speed, self.s[2] )
                    dy = dY(     self.speed, self.s[2] )
                    dh = dTHETA( self.speed, self.turn_angle, self.L )

                    # Update world state
                    self.s += np.array([dx, dy, dh])*dt
                    # Keep h in [-pi, pi]
                    self.s[2] = angle_a2b(a=0, b=self.s[2])

                    # Update ego-centric state
                    self.rho    = RHO(self.s,g)# Don't do this one it gets messed up with direction -> += dRHO  (v=self.speed, alpha=self.alpha) 
                    self.alpha  = ALPHA(self.s,g)
                    self.beta   = BETA (self.s,g)

                    # If verbose, print status every half second
                    if self.verbose and t % 0.5 < dt:
                        self.print_status(g=g,dx=dx,dy=dy,dh=dh)

                    # Timekeeping
                    sleep(self.min_dt) # Wait for minimum loop time
                    dt = monotonic() - stopwatch # Catch elapsed time this loop
                    stopwatch = stopwatch + dt # Update stopwatch
                    t = t+dt # Update elapsed time since start of control loop

                # while loop concluded without error -- we are at the goal!
                if self.verbose:
                    print("\n\nGoal reached, halting.")
                    print("-------------------------------------------------------") 
                    self.print_status(g=g,dx=dx,dy=dy,dh=dh)
                    print("-------------------------------------------------------") 
                    self.print_errors(g=g)

        # I can't remember why I needed this except clause but I think it didn't work right without it
        except Exception as e:
            raise e

        # Print state and halt picar before exiting
        finally:
            if self.verbose:
                self.print_status(g=g,dx=dx,dy=dy,dh=dh)
            self.halt()   
            sleep(0.01)



    


    ##############################################################
    #                   HELPER FUNCTIONS
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
        '''
        Stop picar motors, leave steering as is.
        '''
        self.bw.stop()


    def halt(self):
        '''
        Stop picar and turn steering forward.
        '''
        self.stop_motors()
        self.turn_straight()


    def print_status(self, dt=-1, t=-1, g=[0, 0, 0], dx=0, dy=0, dh=0):
        print("\n-------------------------------------------------------") 
        print("World:  x: {:.2f}\ty: {:.2f}\tth: {:.2f}\tt: {:.3f}".format(
            self.s[0], self.s[1], self.s[2]*180/pi, t) )
        print("World:  dx: {:.2f}\tdy: {:.2f}\tdth: {:.2f}\tdt: {:.3f}".format(
            dx, dy, dh*180/pi, dt ) )
        print("Robot:  v: {:.2f}\tgam: {:.2f}\trho: {:.2f}\ta: {:.2f}\tb: {:.2f}".format(
            self.speed, self.turn_angle*180/pi, norm(self.rho[:2]), self.alpha*180/pi, self.beta*180/pi) )
        print("Robot:  da: {:.2f}\tdb: {:.2f}\tRHO: [{:.2f}, {:.2f}]\trho_angle: {:.2f}".format(
            dALPHA (self.speed, self.turn_angle, self.rho, self.alpha) * 180/pi,
            dBETA  (self.speed, self.rho, self.alpha) *180/pi,
            self.rho[0], self.rho[1], 
            self.rho[2] * 180/pi
            ) )
        print("Goal :  x: {:.4f}\ty: {:.4f}\tth: {:.4f}\t".format(
            g[0],g[1],g[2]*180/pi) )
        print()
            

    def print_errors(self, goal):
        print("-------------------------------------------------------") 
        print("Distance from goal: {:.2f}m\tHeading error: {:.2f}".format(
            norm(self.rho[:2]), 
            angle_a2b( self.s[2], goal[2]) * 180/pi)
            )
        print('\n')



if __name__=="__main__":
    main()


