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

# Bicycle model coordinates
def RHO(robot,goal):
    '''
    Magnitude of the vector from (robot location) to (goal location).
    '''
    difference = goal - robot
    return difference.norm()

def ALPHA(robot,goal):
    '''
    Angle from the robot's heading to the vector from (robot location) to (goal location).
    '''
    difference = goal - robot
    return under_pi(difference.theta())

def BETA(robot,goal): 
    '''
    Angle from the vector from (robot location) to (goal location) to the goal heading angle.
    '''
    difference = goal - robot
    return angle_a2b(a=difference.theta, b=difference.h)


# Bicycle model coordinates
def dRHO(v,alpha):
    return -v*cos(alpha)

def dALPHA(v,gamma,rho,alpha):
    return angle_a2b(a=gamma, b=v*sin(alpha)/rho)

def dBETA(v,rho,alpha):
    return under_pi(-v*cos(alpha)/rho)


# World Coordinates
def dX(v,h):
    return v*cos(h)

def dY(v,h):
    return v*sin(h)

def dH(v,gamma,L):
    return under_pi(v*tan(gamma)/L)



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
            from virtual_wheels import Virtual_Front_Wheels as Front_Wheels
            from virtual_wheels import Virtual_Back_Wheels as Back_Wheels

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

        # Initialize WorldState and BicycleModel
        self.my_worldstate   = WorldState(0,0,0)
        self.goal_worldstate = WorldState(0,0,0)
        self.BM = BicycleModel(0,0,0)




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
    #                    CONTROLS
    ##############################################################

    def get_drive_direction(self):
        # If alpha is greater than pi/2, it's easier to go backward
        # Inspired by code.py example from Homework 1.
        # https://d1b10bmlvqabco.cloudfront.net/attach/k0uju462t062l4/j12evy3w52o5kl/k1vnoghb1697/code.pdf
        if abs(self.BM.alpha) > (pi/2.0 + 1e-4):
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
        v = self.rhoPID.input(self.BM.rho, dt=dt)
        # Bound from [0, MAX_PICAR_SPEED]
        v = min(max(v,0), self.MAX_PICAR_SPEED)
        return v

    # TODO? Change steering based on rho? e.g. if rho is changing quickly err toward alpha
    def GAMMA(self, v, dt=1):
        '''
        Use PID control to calculate turn angle.
        '''
        a = self.alphaPID.input(self.BM.alpha, dt=dt) 
        b = self.betaPID .input(self.BM.beta,  dt=dt)  

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
        return under_pi(gamma)

    
    def transmit_controls(self, speed, angle, direction=1):
        self.set_drive_direction(direction)
        self.turn(angle)
        self.bw.speed = speed

    
    # def actual_controls(self, control_speed, control_angle):
    #     '''
    #     Based on the control signals actually output to the system (which may have differed
    #     from the ideal ones due to hardware limitations), calculate the true control signals
    #     to store for continued calculation.
    #     '''
    #     return self.speed_picar2world(control_speed), self.turnangle_picar2world(control_angle)

    def calculate_controls(self, dt=1, adjust_inputs=False):
        '''
        Calculate speed and turn angle signals, but do not send to hardware.

        NOTE: If adjust_inputs is set to True, we adjust self.speed and self.turn_angle
        to account for any bounding/rounding to give a better estimate of the realworld
        response of the picar.
        '''
        speed           = self.V (dt=dt)
        control_speed   = self.speed_world2picar(speed)  
        speed      = self.speed_picar2world(control_speed)

        turn_angle      = self.GAMMA (v=speed, dt=dt)
        control_angle   = self.turnangle_world2picar(turn_angle)

        if adjust_inputs:
            # Re-map controls to real world -> this will account for bounding
            #   the speed/angle to the picar's working range.
            self.speed = speed
            self.turn_angle = self.turnangle_picar2world(control_angle)
        
        return control_speed, control_angle


    def next_worldstate(self, old_worldstate=None, dt=1):
        # If no old_worldstate input, take worldstate from parent object
        if old_worldstate is None:
            old_worldstate = self.my_worldstate

        # Calculate change in world state
        dx = dX( v=self.speed, h=old_worldstate.h )
        dy = dY( v=self.speed, h=old_worldstate.h )
        dh = dH( v=self.speed, gamma=self.turn_angle, L=self.L )

        # Update world state
        new_worldstate = old_worldstate + WorldState(xyh=[dx, dy, dh])*dt
        # Keep h in [-pi, pi]
        new_worldstate.h = under_pi(new_worldstate.h)

        return new_worldstate





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
        self.my_worldstate   = WorldState(xyh=waypoints[0])
        self.goal_worldstate = WorldState(xyh=waypoints[1])
        
        # Initialize times
        dt = self.min_dt
        t = 0

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
                self.my_worldstate = WorldState(xyh=waypoints[i]) 

                # Record goal world state
                self.goal_worldstate = WorldState(xyh=waypoints[i+1])

                # Calculate initial BicycleModel state
                self.BM = BicycleModel.from_world(self.goal_worldstate, self.my_worldstate)
                

                ##############################################################
                #                        CONTROL LOOP
                ##############################################################

                # Record start time
                stopwatch = monotonic()
                # Stop when close enough in terms of distance and heading
                while ( 
                        not ((self.BM.rho<0.1) and (abs(self.my_worldstate.h)<pi/6)) 
                    and not breakflag
                    ):

                    # Calculate controls from current BicycleModel state (rho,alpha,beta)
                        # NOTE: With adjust_inputs=True, the calculate_controls() method also 
                        # modifies the ideal self.speed and self.turn_angle to reflect any rounding,  
                        # bounding to max value, etc. done as a result of transforming the ideal  
                        # inputs to real-world control signal outputs.
                    control_speed, control_angle = self.calculate_controls(dt=dt, adjust_inputs=True)
                    # Send control signals to hardware
                    self.transmit_controls(control_speed, control_angle)

                    # Update world state
                    self.my_worldstate = self.next_worldstate(self.my_worldstate, dt=dt)

                    # Calculate new ego-centric state
                    self.BM = BicycleModel.from_world(self.goal_worldstate,self.my_worldstate)

                    # If verbose, print status every half second
                    if self.verbose and t % 0.5 < dt:
                        self.print_status(t=t, dt=dt, control_speed=control_speed, control_angle=control_angle)
                        pass

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
                    print("\n\nGoal reached, halting.")
                    print("-------------------------------------------------------") 
                    self.print_status(t=t, dt=dt, control_speed=control_speed, control_angle=control_angle)
                    print("-------------------------------------------------------") 
                    self.print_errors()


        ##############################################################
        #                       EXCEPTION HANDLING
        ##############################################################

        # I can't remember why I needed this except clause but I think it didn't work right without it
        except Exception as e:
            raise e

        # Print state and halt picar before exiting
        finally:
            if self.verbose:
                # self.print_status(g=g,dx=dx,dy=dy,dh=dh)
                pass
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


    def print_status(self, t=-1, dt=-1, control_speed=-1, control_angle=-1):
        '''
        Print the values of current important state variables to monitor progress.
        '''
        print("\n----------------------------------------------------------------") 
        if (t != -1) or (dt != -1):
            print("Time:\t\tt:   {:>6.2f}\tdt:  {:>2.6f}".format(t,dt))
        print("Goal:\t\tx:   {:>6.2f}\ty:     {:>6.2f}\theading: {:>6.2f}\t".format(
            self.goal_worldstate.x, self.goal_worldstate.y, self.goal_worldstate.h*180/pi) )
        print("--  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --")
        print("WorldState:\tx:   {:>6.2f}\ty:     {:>6.2f}\theading: {:>6.2f}".format(
            self.my_worldstate.x, self.my_worldstate.y, self.my_worldstate.h*180/pi) )
        print("BicycleModel:\trho: {:>6.2f}\talpha: {:>6.2f}\tbeta:    {:>6.2f}".format(
            self.BM.rho, self.BM.alpha*180/pi, self.BM.beta*180/pi) )
        print("World controls:\tv:   {:>6.2f}\tgamma: {:>6.2f}".format(self.speed, self.turn_angle))
        print("Picar controls:\tv:   {:>6.2f}\tgamma: {:>6.2f}".format(control_speed, control_angle))
        print()
            

    def print_errors(self):
        '''
        Print error from goal position and heading.
        '''
        print("----------------------------------------------------------------") 
        print("Distance from goal: {:.2f}m\tHeading error: {:.2f}".format(
            norm(self.BM.rho), 
            angle_a2b( self.my_worldstate.h, self.goal_worldstate.h) * 180/pi)
            )
        print('\n')




def test():
    pc = Picar(verbose=True,virtual=True)
    waypoints = np.array( [
        [0,0,0],
        [1,-1,-pi/2]
        ])
    pc.traverse(waypoints)

if __name__=="__main__":
    test()






