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
from helpers import sign, angle_a2b, bound_angle
from my_pid import PID

# Find SunFounder_PiCar submodule
sys.path.append("../lib/SunFounder_PiCar")



kspeed = 215



##############################################################
#                  PARAMETER FUNCTIONS
##############################################################

def RHO(s,g):
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
    return bound_angle(-v*cos(alpha)/norm(rho[:2]))


def dX(v,h):
    return v*cos(h)

def dY(v,h):
    return v*sin(h)

def dTHETA(v,gamma,L):
    return bound_angle(v*tan(gamma)/L)



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
                    max_speed_world=0.4, max_turn_world=0.626,
                    virtual=False, min_dt=0.005):

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

        # Set maximum values of control signals
        self.MAX_SPEED = max_speed_world
        self.MAX_TURNING_ANGLE = max_turn_world

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
        spd = min( max(spd,0), self.MAX_SPEED) # bound speed between 0 and MAX_SPEED
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
        if abs(ang) > self.MAX_TURNING_ANGLE:
            ang = sign(ang)*self.MAX_TURNING_ANGLE

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
    #                       WORLD FRAME
    ##############################################################

    def V(self, rho=None, dt=1):
        if rho is None:
            rho = norm(self.rho[:2])

        #  # Get linear direction and magnitude of rho
        if abs(self.alpha) > pi:
            rho_sign = -1 
        else:
            rho_sign = 1
        rho *= rho_sign
         
        v = self.rhoPID.input(rho)

        # Don't go below 0
        v = max(v,0)

        # Or above max
        if v > self.MAX_SPEED:
            v = self.MAX_SPEED
        return v

    # Change steering based on rho? e.g. if rho is changing quickly err toward alpha
    def GAMMA(self, rho=None, alpha=None, beta=None, dt=1):
        if rho is None:
            rho = norm(self.rho[:2])
        if alpha is None:
            alpha = self.alpha
        if beta is None:
            beta = self.beta

        a = self.alphaPID.input(alpha, dt=dt) 
        b = self.betaPID .input(beta,  dt=dt)  
        # gamma = rho*a + b+2*self.Kpb/rho -b*(self.last_rho-rho)
        gamma = a + b
        gamma = angle_a2b(a=0, b=gamma)

        # Stop at max
        if abs(gamma) > self.MAX_TURNING_ANGLE:
            gamma = sign(gamma)*self.MAX_TURNING_ANGLE
        return gamma

    



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


    '''
    The big important function (as of now).
    '''
    def travel(self, waypoints):
        breakflag = False


        ##############################################################
        #                       INITIALIZE
        ##############################################################
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

        # Initialize printable variables (so they don't cause errors when
        #   the try-catch statement tries to catch and error and print out
        #   variables that haven't been initialized yet in 'finally:')
        # Initialize times
        dt = self.min_dt
        t = 0

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
                    self.speed      = self.V (rho=norm(self.rho[:2]),dt=dt)
                    control_speed   = self.speed_world2picar(self.speed)
                    self.turn_angle = self.GAMMA (self.alpha, self.beta,dt=dt)
                    control_angle   = self.turnangle_world2picar(self.turn_angle)

                    # Send controls to hardware
                    self.turn (       control_angle )
                    self.bw.speed   = control_speed

                    # Re-map controls to real world -> this will account for bounding
                    #   the speed/angle to the picar's working range.
                    self.speed      = self.speed_picar2world(control_speed)
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

                    # Print status every half second
                    if t % 0.5 < dt:
                        self.print_status(g=g,dx=dx,dy=dy,dh=dh)

                    # Timekeeping
                    sleep(self.min_dt) # Wait for minimum loop time
                    dt = monotonic() - stopwatch
                    stopwatch = stopwatch + dt
                    t = t+dt

                # While loop concluded without error -- we are at the goal!
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
            self.print_status(g=g,dx=dx,dy=dy,dh=dh)
            self.halt()   
            sleep(0.01)



    
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



if __name__=="__main__":
    main()


