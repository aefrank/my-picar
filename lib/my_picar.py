'''
Filename: my_picar.py
Description: Class and functions to represent a physical picar in the program.
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
from helpers import sign, angle_a2b, under_pi, clip
from my_pid import PID
from perspectives import WorldState, BicycleModel

# Find SunFounder_PiCar submodule
sys.path.append("../lib/SunFounder_PiCar")



# kspeed = 215



##############################################################
#                  PARAMETER FUNCTIONS
##############################################################
# @TODO: Most of these should be stowed away in perspectives.BicycleModel

# Bicycle model coordinates
def RHO(robot,goal):
    '''
    Magnitude of the vector from (robot location) to (goal location).
    '''
    difference = goal - robot
    return difference.norm()

def ALPHA(robot,goal):
    '''
    Angle from the robot's gamma to the vector from (robot location) to (goal location).
    '''
    difference = goal - robot
    return under_pi(difference.theta())

def BETA(robot,goal): 
    '''
    Angle from the vector from (robot location) to (goal location) to the goal gamma gamma.
    '''
    difference = goal - robot
    return gamma_a2b(a=difference.theta, b=difference.h)


# Bicycle model coordinates
def dRHO(v,alpha):
    return -v*cos(alpha)

def dALPHA(v,gamma,rho,alpha):
    return gamma_a2b(a=gamma, b=v*sin(alpha)/rho)

def dBETA(v,rho,alpha):
    return under_pi(-v*cos(alpha)/rho)


# World Coordinates
def dX(v,h):
    return v*cos(h)

def dY(v,h):
    return v*sin(h)

def dH(v,gamma,L):
    return under_pi(v*tan(gamma)/L)


# Controls
def V(rho, rho_controller, dt=1):
    return rho_controller.input(rho, dt=dt)


def GAMMA(v, alpha, beta, alpha_controller, beta_controller, L=1, dt=1):
    a = alpha_controller.input(alpha, dt=dt) 
    b = beta_controller .input(beta,  dt=dt)  

    # Weighted change in gamma = desired dh
    dh = a + b

    # Turn gamma is measured from the front wheel, not the back wheel; 
    # Get front wheel turn gamma
    s = abs(v) # make sure we have unsigned speed; direction is already handles
    gamma = atan(dh*L/s)

    # Bound between [-pi, pi]
    return under_pi(gamma)

def should_i_back_up(alpha):
    # If alpha is greater than pi/2, it's easier to go backward
    # Inspired by code.py example from Homework 1.
    # https://d1b10bmlvqabco.cloudfront.net/attach/k0uju462t062l4/j12evy3w52o5kl/k1vnoghb1697/code.pdf
    if abs(alpha) > (pi/2 + 1e-4):
        return True
    else:
        return False



##############################################################
#                   PICAR CLASS
##############################################################

class Picar:
    '''
    Class to represent Picar parameters and outputs.
    '''
    
    # @TODO: Clean up input parameters to the important ones; try to group Ks
    def __init__(self, max_turn=45, configfile="config", L=0.145,
                    kpr=1, kpa=0, kpb=0, kdr=0, kir=0,
                    max_picar_speed=80, max_picar_turn=40,
                    virtual=False, min_dt=0.005, verbose=False, virtual_verbose=False):

        self.verbose = verbose

        # Use virtual libraries if requested (so we don't get GPIO errors for not running on a Pi)
        if not virtual:
            import picar
            from picar.front_wheels import Front_Wheels
            from picar.back_wheels import Back_Wheels
            picar.setup()
            self._fw = Front_Wheels(db=configfile)
            self._bw = Back_Wheels(db=configfile)
        else:
            from virtual_wheels import Virtual_Front_Wheels as Front_Wheels
            from virtual_wheels import Virtual_Back_Wheels as Back_Wheels
            self._fw = Front_Wheels(db=configfile, verbose=virtual_verbose)
            self._bw = Back_Wheels(db=configfile, verbose=virtual_verbose)

        # Wheel base [m]
        self._L = L        
        # Max turn radius
        self._fw.max_turn = max_turn # [deg]  In picar's world; in reality 45 maps to around 40, etc
        # Initialize wheels
        self._v= 0      # from 0-100
        self._gamma = 0     # initalize at forward
        self._direction = 1
        self._fw.ready()
        self._bw.ready()
        self._bw.forward()

        # PID controllers for parameters
        self._rhoPID   = PID(Kp=kpr, Ki=kir, Kd=kdr)
        self._alphaPID = PID(Kp=kpa)
        self._betaPID  = PID(Kp=kpb)

        # Set maximum values of control signals -- IN PICAR UNITS (e.g. degrees, not radians)
        self.MAX_PICAR_SPEED = max_picar_speed
        self.MAX_PICAR_TURN = max_picar_turn

        # Minimum loop delay
        self.min_dt = min_dt

        # Initialize WorldState and BicycleModel
        self._my_worldstate   = WorldState(0,0,0)
        self._goal_worldstate = WorldState(0,0,0)
        self._BM = BicycleModel(0,0,0)



###############################################################################
###############################################################################
###########              USER-ACCESSIBLE METHODS                ###############
###############################################################################
###############################################################################

    def turn(self, gamma, units="world"):
        if units == "world":
            picar_turn = self._gamma_to_picar_turn(gamma)
            if abs(picar_turn) > self.MAX_PICAR_TURN:
                picar_turn = sign(picar_turn)*self.MAX_PICAR_TURN
                gamma = self._picar_turn_to_gamma(picar_turn)
        elif units == "picar":
            picar_turn = gamma
        else:
            raise InputError("Invalid units arg {}. units must be 'world' or 'picar'.".format(units))
        
        if self._gamma != gamma:
            self._turn_wheels(int(picar_turn))



    def set_speed(self, v, units="world"):
        direction = sign(v)
        picar_speed = self._v_to_picar_speed(abs(v))
        if picar_speed > self.MAX_PICAR_SPEED:
            picar_speed = self.MAX_PICAR_SPEED
            v = self._picar_turn_to_gamma(picar_speed)
        if self._v != v*direction:
            self._set_v(v*direction)


    def drive(self, v, gamma=0):
        self.turn(self, gamma)
        self._bw.speed = v

    def halt(self):
        '''
        Stop picar and turn steering forward.
        '''
        self._stop_motors()
        self._turn_wheels_straight()


    def get_drive_direction(self):
        return self._direction

    def set_drive_direction(self, direction):
        self._set_direction(direction)


    def get_speed(self):
        return self._v

    def get_gamma(self):
        return self._gamma


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
        self._my_worldstate   = WorldState(xyh=waypoints[0])
        self._goal_worldstate = WorldState(xyh=waypoints[1])
        
        # Initialize times
        dt = self.min_dt
        t = 0

        # Initialize controls
        picar_speed = 0
        picar_turn = 0

        ##############################################################
        #                  LOOP THROUGH WAYPOINTS
        ##############################################################
        try:
            # For each waypoint
            for i in range(len(waypoints)-1):

                ##############################################################
                #             INITIALIZE NEW GOAL POSE
                ##############################################################

                # OPTIONAL:
                # Update initial my_worldstate to ideal location -- exactly at the last waypoint
                #   You may want to comment this out to keep the model at the world state
                #   of its last timestep on the previous goal loop.
                self._my_worldstate = WorldState(xyh=waypoints[i]) 

                # Record goal world state
                self._goal_worldstate = WorldState(xyh=waypoints[i+1])

                # Calculate initial BicycleModel state
                self._BM = BicycleModel.from_world(self._goal_worldstate, self._my_worldstate)
                

                ##############################################################
                #                        CONTROL LOOP
                ##############################################################

                # Record start time
                stopwatch = monotonic()
                # Stop when close enough in terms of distance and gamma
                while ( 
                        not ((self._BM.rho<0.1) and (abs(self._my_worldstate.h)<pi/6)) 
                    and not breakflag
                    ):

                    # Calculate controls from current BicycleModel state (rho,alpha,beta)
                    self._v, self._gamma = self._get_v_gamma(dt=dt)
                    picar_speed, picar_turn = self._calculate_hardware_controls(
                            v=self._v, gamma=self._gamma, dt=dt, adjust_inputs=True )
                    # Send control signals to hardware
                    self._transmit_controls(picar_speed, picar_turn)

                    # Update world state
                    self._my_worldstate = self._next_worldstate(self._my_worldstate, dt=dt)

                    # Calculate new ego-centric state
                    self._BM = BicycleModel.from_world(self._goal_worldstate, self._my_worldstate)

                    # If verbose, print status every half second
                    if self.verbose and t % 0.5 < dt:
                        self.print_status(t=t, dt=dt, picar_speed=picar_speed, picar_turn=picar_turn)

                    # Timekeeping
                    sleep(self.min_dt) # Wait for minimum loop time
                    dt = monotonic() - stopwatch # Catch elapsed time this loop
                    stopwatch = stopwatch + dt # Update stopwatch
                    t = t+dt # Update elapsed time since start of control loop



                ##############################################################
                #                        GOAL REACHED
                ##############################################################

                # while loop concluded without error -- we are at the goal!
                if self.verbose:
                    self.print_goal()


        ##############################################################
        #                       EXCEPTION HANDLING
        ##############################################################

        # I can't remember why I needed this except clause but I think it didn't work right without it
        except Exception as e:
            raise e

        # Print state and halt picar before exiting
        finally:
            if self.verbose:
                self.print_status(t=t, dt=dt, picar_speed=picar_speed, picar_turn=picar_turn)
            self.halt()   
            sleep(0.01)






###############################################################################
###############################################################################
###########                  PRIVATE METHODS                    ###############
###############################################################################
###############################################################################


    ##############################################################
    #               APPLY CALIBRATION MAPS
    ##############################################################

    # Maps --> use calibration to map real world speed/gamma to the control signals for the Picar motors
    def _v_to_picar_speed(self, v, m=215, b=0):
        '''
        Map real-world speed in meters-per-second to 0-100 picar speed based on calibration.
        '''
        picar_speed = int(m*abs(v) + b)
        picar_speed = min( max(picar_speed,0), self.MAX_PICAR_SPEED) # bound speed between 0 and MAX_PICAR_SPEED
        return picar_speed


    # The reverse; go from control signal to real world
    def _picar_speed_to_v(self, picar_speed, m=215, b=0):
        '''
        Map picar 0-100 speed to real-world speed in meters-per-second based on calibration.
        '''
        v = (picar_speed+b)/m
        return v



    def _gamma_to_picar_turn(self, gamma, m_right=1, b_right=0, m_left=None, b_left=None):
        '''
        Map real-world turn-gamma in radians to picar turn-gamma in degrees based on calibration.
        '''
        ang = -gamma * 180 / pi    # picar deals in degrees, and treats left (CCW) as negative

        if m_left is None:
            m_left = m_right
        if b_left is None:
            b_left = b_right

        # Turn right
        if ang > 0:
            ang = int(m_right*ang + b_right)
        else:
            ang = int(m_left*ang + b_left)

        # bound gamma between -max and +max
        if abs(ang) > self.MAX_PICAR_TURN:
            ang = sign(ang)*self.MAX_PICAR_TURN

        return ang

    def _picar_turn_to_gamma(self, picar_turn, m_right=1, b_right=0, m_left=None, b_left=None):
        '''
        Map picar turn-gamma in degrees to real-world turn-gamma in radians based on calibration.
        '''
        ang = -picar_turn * pi / 180    # change to radians, and treat left turn (CCW) as positive

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
    #                    CONTROLS
    ##############################################################

    def _V(self, dt=1):
        '''
        Use PID control to calculate velocity.
        '''
        v = V(self._BM.rho, self._rhoPID, dt=dt)

        # Bound from [0, MAX_PICAR_SPEED]
        v = min(max(v,0), self.MAX_PICAR_SPEED)
        return v


    # TODO? Change steering based on rho? e.g. if rho is changing quickly err toward alpha
    def _GAMMA(self, v, dt=1):
        '''
        Use PID control to calculate turn gamma.
        '''
        gamma = GAMMA(v=v, alpha=self._BM.alpha, beta=self._BM.beta,
                            alpha_controller=self._alphaPID, beta_controller=self._betaPID,
                            L=self._L, dt=dt)

        # Check if no turn is required
        if gamma<3:
            return 0

        gamma *= self.get_drive_direction()

        # Clip to max
        bound = self._picar_turn_to_gamma( self.MAX_PICAR_TURN );
        gamma = clip(gamma, -bound, bound)

        # Bound between [-pi, pi]
        return under_pi(gamma)



    def _get_drive_direction_from_alpha(self):
        return should_i_back_up(self._BM.alpha)
    
    def _set_direction(self, direction):
        # Set bw direction to forward or backward.
        # Inspired by code.py example from Homework 1.
        # https://d1b10bmlvqabco.cloudfront.net/attach/k0uju462t062l4/j12evy3w52o5kl/k1vnoghb1697/code.pdf

        if direction==1:
            # drive forward
            self._bw.forward()
            self._direction = 1
        elif direction==-1:
            # drive backward
            self._bw.backward()
            self._direction = -1
        else:
            raise InputError("Direction must be -1 (backward) or 1 (forward).")


    def _transmit_controls(self, control_v, control_gamma, direction=1):

        self._set_direction(direction)
        self.turn(control_gamma)
        self._set_v(control_v)


    def _transmit_v_gamma_to_hardware(self, v=None, gamma=None, dt=1, adjust_inputs=False):
        if v == None:
            v = self._v
        if gamma == None:
            gamma == self._gamma

        picar_speed   = self._v_to_picar_speed(v) 
        # Re-calculate speed in case it was clipped in _speed_world2picar
        v               = self._picar_speed_to_v(picar_speed) 

        if adjust_inputs:
            # Use adjusted speed to calculate new world gamma
            gamma       = self._GAMMA (v=v, dt=dt)

        picar_turn = self._gamma_to_picar_turn(gamma)

        direction = self._get_drive_direction_from_alpha()
        picar_speed = v*direction
        picar_turn = gamma*direction

        return picar_speed, picar_turn



    def _calculate_hardware_controls(self, v=None, gamma=None, dt=1, adjust_inputs=False):
        '''
        Calculate speed and turn gamma signals for hardware, but do not send to hardware.

        NOTE: If adjust_inputs is set to True, we base gamma control off of the speed control
        instead of the raw gamma input to account for any bounding/rounding and give a better 
        estimate of the realworld response of the picar.
        '''
        if v == None:
            v = self._v
        if gamma == None:
            gamma == self._gamma

        picar_speed     = self._v_to_picar_speed(v)  
        
        if adjust_inputs:
            # Use hardware-limited speed to calculate control signal for gamma
            adjusted_v  = self._picar_speed_to_v(picar_speed)
            gamma       = self._GAMMA (v=v, dt=dt)

        picar_turn   = self._gamma_to_picar_turn(gamma)

        direction   = self._get_drive_direction_from_alpha()
        picar_speed     *= direction
        picar_turn *= direction

        # if adjust_inputs:
        #     # Re-map controls to real world -> this will account for bounding
        #     #   the speed/gamma to the picar's working range.
        #     self._v= speed  # we already did this adjustment for speed, since we 
        #                         # wanted the best estimate for calculating gamma
        #     self._gamma = self._picar_turn_to_gamma(picar_turn)
        
        return picar_speed, picar_turn



    def _get_v_gamma(self, dt=1):
        v     = self._V (dt=dt)
        gamma = self._GAMMA (v=v, dt=dt)

        return v, gamma


    def _next_worldstate(self, old_worldstate=None, dt=1):
        # If no old_worldstate input, take worldstate from parent object
        if old_worldstate is None:
            old_worldstate = self._my_worldstate

        # Calculate change in world state
        dx = dX( v=self._v, h=old_worldstate.h )
        dy = dY( v=self._v, h=old_worldstate.h )
        dh = dH( v=self._v, gamma=self._gamma, L=self._L )

        # Update world state
        new_worldstate = old_worldstate + WorldState(xyh=[dx, dy, dh])*dt
        # Keep h in [-pi, pi]
        new_worldstate.h = under_pi(new_worldstate.h)

        return new_worldstate









    


    ##############################################################
    #                   HELPER FUNCTIONS
    ##############################################################

    def print_status(self, t=-1, dt=-1, picar_speed=-1, picar_turn=-1):
        '''
        Print the values of current important state variables to monitor progress.
        '''
        print("\n----------------------------------------------------------------") 
        if (t != -1) or (dt != -1):
            print("Time [s]:\tt:   {:>6.2f}\tdt:  {:>2.6f}".format(t,dt))
        print("Goal:\t\tx:   {:>6.2f}\ty:     {:>6.2f}\tgamma: {:>6.2f}\t".format(
            self._goal_worldstate.x, self._goal_worldstate.y, self._goal_worldstate.h*180/pi) )
        print("--  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --")
        print("WorldState:\tx:   {:>6.2f}\ty:     {:>6.2f}\tgamma: {:>6.2f}".format(
            self._my_worldstate.x, self._my_worldstate.y, self._my_worldstate.h*180/pi) )
        print("BicycleModel:\trho: {:>6.2f}\talpha: {:>6.2f}\tbeta:    {:>6.2f}".format(
            self._BM.rho, self._BM.alpha*180/pi, self._BM.beta*180/pi) )
        print("World controls:\tv:   {:>6.2f}\tgamma: {:>6.2f}".format(self._v, self._gamma))
        print("Picar controls:\tv:   {:>6.2f}\tgamma: {:>6.2f}".format(picar_speed, picar_turn))
            

    def print_errors(self):
        '''
        Print error from goal position and gamma.
        '''
        print("----------------------------------------------------------------") 

        print("Distance from goal: {:.2f}m\tHeading error: {:.2f}".format(
            norm(self._BM.rho), 
            gamma_a2b( self._my_worldstate.h, self._goal_worldstate.h) * 180/pi)
            )
        print('\n')

    def print_goal(self):
        print("\n\nGoal reached, halting.")
        print("-------------------------------------------------------") 
        self.print_status(t=t, dt=dt, picar_speed=picar_speed, picar_turn=picar_turn)
        print("-------------------------------------------------------") 
        self.print_errors()


    ##############################################################
    #                    LOW LEVEL FUNCTIONS
    ##############################################################


    def _turn_wheels(self, gamma):
        '''
        Input gamma in degrees.

        Make it so inputs gammas are relative to 0 degrees being straight forward.
        '''
        self._fw.turn(gamma + self._fw._straight_angle)

    def _turn_wheels_straight(self, overshoot=10, delay=0.05):
        '''
        Often when "turning straight", the picar stops a bit short of fully straight
        forward. This is my attempt to remedy that by overshooting then correcting.

        Tune the overshoot [deg] and delay [sec] if it's too jittery.
        '''
        self._turn_wheels(overshoot)
        sleep(delay)
        self._turn_wheels(-overshoot)
        sleep(delay)
        self._fw.turn_straight()
        self._gamma = 0

    def _set_v(self, speed):
        self._direction = sign(speed)
        if self._direction == 1:
            self._bw.forward()
        elif self._direction == -1:
            self._bw.backward()
        else:
            self.halt()
            return

        self._v= abs(speed)
        self._bw.speed = abs(speed)
        


    def _stop_motors(self):
        '''
        Stop picar motors, leave steering as is.
        '''
        self._bw.stop()
        self._v= 0







def test():
    pc = Picar(verbose=True,virtual=True,virtual_verbose=False)
    waypoints = np.array( [
        [0,0,0],
        [1,-1,-pi/2]
        ])
    pc.traverse(waypoints)

if __name__=="__main__":
    test()






