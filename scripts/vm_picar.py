'''
Filename: vm_picar.py
Description: My implementation of the PiCar as a class FOR USE ON A VM.
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
from virtual_wheels import Virtual_Front_Wheels as Front_Wheels
from virtual_wheels import Virtual_Back_Wheels  as Back_Wheels


# Find SunFounder_PiCar submodule
sys.path.append("../lib/SunFounder_PiCar")
# import picar



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
    

    
kspeed = 215











##############################################################
#                       PID CONTROLLER
##############################################################

class PID():

    def __init__(Kp=0, Ki=0, Kd=0, integral_error=0, last_error=0, dt=1, 
                    integral_max=None, integral_active_region=None):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.integral_error = integral_error
        self.integral_max = integral_max
        self.integral_active_region = abs(integral_active_region)
        self.last_error = last_error

        self.dt = dt

    def P(self, error=0):
        '''
        Proportional term
        '''
        return self.Kp*error



    def I(self, error=0):
        '''
        Integral term
        '''
        # If we have defined an active region for the integral controller,
        # only use integral control when error is within that region.
        # Helps control integral windup.
        if (integral_active_region is not None):

            # Check if you are outside integral_active_region
            if (abs(error) > integral_active_region):
                # You're in the inactive region. Return no control signal.
                return 0

        # If you've made it here, you either have no defined active region,
        # or you are within that active region. Proceed as normal.

        # Update integral_error
        self.integral_error += error

        # If integral_max is defined, limit integral windup
        if self.integral_max is not None:
            # Don't let integral pass limit
            if abs(self.integral_error) > self.integral_max:
                self.integral_error = sign(self.integral_error)*self.integral_max

        return self.Ki*integral*self.dt



    def D(self, error=0): 
        '''
        Derivative term
        '''
        d_error = error - self.last_error
        # Update previous error
        self.last_error = error

        return self.Kd*d_error*self.dt



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


# Magnitude only
def dRHO(v,alpha):
    return -v*cos(alpha)

def dALPHA(v,gamma,rho,alpha):
    return angle_a2b(a=0, b=v*sin(alpha)/norm(rho[:2])) - gamma

def dBETA(v,rho,alpha):
    return angle_a2b(a=0, b=-v*cos(alpha)/norm(rho[:2]))




##############################################################
#                   PICAR CLASS
##############################################################



    # def halt(self, verbose=False):
    #     self.stop()
    #     self.turn_forward()
    #     if verbose:
    #         print("Wheels forward.")

    

class Virtual_Picar:
    '''
    Class to represent Virtual_Picar parameters and outputs.
    '''
    
    # @TODO: Clean up input parameters to the important ones; try to group Ks
    def __init__(self, max_turn=45, speed=50, configfile="config", delay=0.005, L=0.145,
                    kpr=1, kpa=0, kpb=0, kdr=0, kir=0,
                    max_speed_world=0.4, max_turn_world=0.626):
        
        #picar.setup()
        print("Picar setup.")

        self.L = L

        self.config_file = configfile
        #self.fw = picar.front_wheels.Front_Wheels(db=configfile)
        #self.bw = picar.back_wheels.Back_Wheels(db=configfile)
        self.fw = Front_Wheels(db=configfile)
        self.bw = Back_Wheels (db=configfile)

        self.fw.max_turn = max_turn # [deg]  In picar's world; in reality 45 maps to around 40, etc
        
        self.speed = speed
        self.turn_angle = 0

        self.fw.ready()
        self.bw.ready()
        self.bw.forward()

        self.delay = delay # [sec]

        self.Kpr = kpr
        self.Kpa = kpa
        self.Kpb = kpb

        self.Kdr = kdr
        self.Kir = kir

        self.MAX_SPEED = max_speed_world
        self.MAX_TURNING_ANGLE = max_turn_world

        self.integral_rho = 0
        self.last_rho = 0



    # Maps --> use calibration to map real world speed/angle to the control signals for the Virtual_Picar motors
    # @TODO: Address 'kspeed' parameter
    def map_speed(self, spd):
        s = int(kspeed*spd)
        return min(s,100)

    # The reverse; go from control signal to real world
    def inverse_map_speed(self,spd):
        return spd/kspeed

    def map_turn(self, angle):
        angle = -angle * 180 / pi    # picar deals in degrees, and treats left (CCW) as negative
        
        # # Deal with offsets around 0
        # if abs(angle) < 5:
        #     return 0
        # if abs(angle) < 10:
        #     return angle

        if angle > 0:
            angle = max(min(int(angle*1.1),35),0)
        else:
            angle = max(int(angle*0.8), -35)
        return int(angle)


    ##############################################################
    #                       WORLD FRAME
    ##############################################################

    def V(self, rho, dt=1):
        #  # Get linear direction and magnitude of rho
        if abs(self.alpha) > pi:
            rho_sign = -1 
        else:
            rho_sign = 1
        rho *= rho_sign
         
        v = self.Kpr*rho # @TODO: Integrate PID class

        # Don't go below 0
        v = max(v,0)

        # Or above max
        if v > self.MAX_SPEED:
            v = self.MAX_SPEED
        return v

    # Change steering based on rho? e.g. if rho is changing quickly err toward alpha
    def GAMMA(self,alpha,beta,dt):
        rho = norm(self.rho[:2])
        a = self.Kpa*alpha # @TODO: Integrate PID class
        b = self.Kpb*beta # @TODO: Integrate PID class
        # gamma = rho*a + b+2*self.Kpb/rho -b*(self.last_rho-rho)
        gamma = a + b
        gamma = angle_a2b(a=0, b=gamma)

        # Stop at max
        if abs(gamma) > self.MAX_TURNING_ANGLE:
            gamma = sign(gamma)*self.MAX_TURNING_ANGLE
        return gamma

    def dX(self,v,h):
        return v*cos(h)

    def dY(self,v,h):
        return v*sin(h)

    def dTHETA(self,v,gamma):
        return angle_a2b(a=0, b=v*tan(gamma)/self.L)





    '''
    The big important function (as of now).
    '''
    def travel(self, waypoints):
        breakflag = False

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

        try:
            #Initialize times
            dt = self.delay
            t = 0
            mono_time = monotonic()
            
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
                    control_speed   = self.map_speed(self.speed)
                    self.turn_angle = self.GAMMA (self.alpha, self.beta,dt=dt)

                    # Send controls to hardware
                    self.turn (     self.map_turn(self.turn_angle) )
                    self.bw.speed = control_speed
                    self.speed    = self.inverse_map_speed(control_speed) 
                    sleep(self.delay)

                    # Calculate change in world state
                    dx      = self.dX(     self.speed, self.s[2] )
                    dy      = self.dY(     self.speed, self.s[2] )
                    dh  = self.dTHETA( self.speed, self.turn_angle )

                    # Update world state
                    self.s += np.array([dx, dy, dh])*dt
                    # Keep h in [-pi, pi]
                    self.s[2] = angle_a2b(a=0, b=self.s[2])

                    # Update ego-centric state
                    self.rho  = RHO(self.s,g)# Don't do this one it gets messed up with direction -> += dRHO  (v=self.speed, alpha=self.alpha) 
                    self.alpha  = ALPHA(self.s,g)
                    self.beta   = BETA (self.s,g)

                    # Print status every half second
                    if t % 0.5 < dt:
                        print("World:  x: {:.2f}\ty: {:.2f}\tth: {:.2f}\tt: {:.3f}".format(
                            self.s[0], self.s[1], self.s[2]*180/pi, t) )
                        print("World:  dx: {:.2f}\tdy: {:.2f}\tdth: {:.2f}\tdt: {:.3f}".format(
                            dx, dy, dh*180/pi,dt ) )
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

                    # Timekeeping
                    sleep(self.delay)
                    dt = monotonic() - mono_time
                    mono_time = mono_time + dt
                    t = t+dt

                print("Goal reached, halting.")
                print("-------------------------------------------------------") 

                print("World:  x: {:.2f}\ty: {:.2f}\tth: {:.2f}\tt: {:.3f}".format(
                    self.s[0], self.s[1], self.s[2]*180/pi, t) )
                print("World:  dx: {:.2f}\tdy: {:.2f}\tdth: {:.2f}\tdt: {:.3f}".format(
                    dx, dy, dh*180/pi,dt ) )
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
                print("-------------------------------------------------------") 
                print("Distance from goal: {:.2f}m\tHeading error: {:.2f}".format(
                    norm(self.rho[:2]), 
                    angle_a2b( self.s[2], g[2]) * 180/pi)
                    )
                print('\n')

        # I can't remember why I needed this except clause but I think it didn't work right without it
        except Exception as e:
            raise e

        # Print state and halt picar before exiting
        finally:
            print("\n-------------------------------------------------------") 
            print("World:  x: {:.2f}\ty: {:.2f}\tth: {:.2f}\tt: {:.3f}".format(
                self.s[0], self.s[1], self.s[2]*180/pi, t) )
            print("World:  dx: {:.2f}\tdy: {:.2f}\tdth: {:.2f}\tdt: {:.3f}".format(
                dx, dy, dh*180/pi,dt ) )
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
            print("-------------------------------------------------------") 
            print("Distance from goal: {:.2f}m\tHeading error: {:.2f}".format(
                norm(self.rho[:2]), 
                angle_a2b( self.s[2], g[2]) * 180/pi)
                )
            print('\n')
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


