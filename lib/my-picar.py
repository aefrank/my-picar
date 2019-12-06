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
from helpers import sign, angle_a2b, within_pi, clip
from my_pid import myPID
import perspectives 
from cartesian_pose import CartesianPose
from bicycle_model import BicyclePose, BicycleModel

# Find SunFounder_PiCar submodule
sys.path.append("../lib/SunFounder_PiCar")




##############################################################
#                  CALCULATE CONTROLS
##############################################################

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

##############################################################
#                  CALCULATE NEXT POSE
##############################################################    

def calculate_next_pose(old_pose, v, gamma, dt=1):
    '''
    Calculate the next CartesianPose state of an object with a certain speed and steering angle.
    '''
    # Calculate change in world state
    dx = dX( v=v, h=old_pose.h )
    dy = dY( v=v, h=old_pose.h )
    dh = dH( v=v, gamma=gamma, L=L )

    # Update world state
    new_pose = old_pose + CartesianPose(xyh=[dx, dy, dh])*dt
    # Keep h in [-pi, pi]
    new_pose.h = within_pi(new_pose.h)

    return new_pose



# @TODO: Make sure Picar class is ONLY dealing with the world of the Picar -> make other classes/functions to deal with control, etc.

##############################################################
#                   PICAR CLASS
##############################################################

class Picar:
    '''
    Class to represent Picar parameters and outputs.
    '''
    
    # @TODO: Clean up input parameters to the important ones; try to group Ks
    def __init__(self, max_turn=45, configfile="config", L=0.145,
                    # kpr=1, kpa=0, kpb=0, kdr=0, kda=0, kdb=0, kir=0, kia=0, kib=0,
                    # controllers=lambda x: x, min_dt=0.005, 
                    max_picar_speed=80, max_picar_turn=40,
                    virtual=False, verbose=False, virtual_speedverbose=False):

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
            self._fw = Front_Wheels(db=configfile, verbose=virtual_speedverbose)
            self._bw = Back_Wheels(db=configfile, verbose=virtual_speedverbose)

        # Wheel base [m]
        self._L = L        
        # Max turn radius
        self._fw.max_turn = max_picar_turn
        # Initialize wheels
        self._speed= 0         # from 0-100
        self._steering = 0     # initalize at no turn angle
        self._direction = 1    # wheels set forward
        self._fw.ready()
        self._bw.ready()
        self._bw.forward()

        # Controllers for calculating control signals
        # self.controller = MyPicarController(v_controllers, gamma_controllers)
        # self._rhomyPID   = myPID(Kp=kpr, Ki=kir, Kd=kdr)
        # self._alphamyPID = myPID(Kp=kpa, Ki=kia, Kd=kda)
        # self._betamyPID  = myPID(Kp=kpb, Ki=kib, Kd=kdb)

        # Set maximum values of control signals -- IN PICAR UNITS (e.g. degrees, not radians)
        self.MAX_PICAR_SPEED = max_picar_speed
        self.MAX_PICAR_TURN = max_picar_turn

        # Minimum loop delay
        # self.min_dt = min_dt

        # Initialize WorldState and BicycleModel
        self._my_cartesian_pose   = CartesianPose(0,0,0)
        self._goal_cartesian_pose = CartesianPose(0,0,0)
        self._my_bicycle_model = BicycleModel(rho=0, alpha=0, beta=0)



###############################################################################
###############################################################################
###########              USER-ACCESSIBLE METHODS                ###############
###############################################################################
###############################################################################

    # Highest level picar control
    def drive(self, picar_speed, picar_angle=0, direction=None):
        '''
        Sets picar steering and speed in PICAR UNITS by calling 
        self.turn() and self.set_speed().
        '''
        self.turn(picar_angle, apply=False) # Wait till both settings are changed before sending to hardware
        self.set_speed(picar_speed, direction=direction, apply=True)

    def halt(self):
        '''
        Stop picar and turn steering forward.
        '''
        self.drive(picar_speed=0, picar_angle=0, direction=1)
        # self._stop_motors()
        # self._turn_wheels_straight()

    
    # Fine-tuned picar control
    def turn(self, picar_angle, apply=True):
        '''
        Sets picar steering to PICAR ANGLE in PICAR UNITS
        '''
        if abs(picar_turn) > self.MAX_PICAR_TURN:
            picar_turn = sign(picar_turn)*self.MAX_PICAR_TURN
        self._steering = int(picar_angle)        
        if apply:
            self._apply_current_controls()

    def set_speed(self, picar_speed, direction=None, apply=True):
        '''
        Sets picar speed to PICAR SPEED in PICAR UNITS.
        Positive speeds are forward and negative speeds are backward.
        '''
        if direction is not None:
            self._direction = direction
        else:
            self._direction = sign(picar_speed)
        picar_speed = abs(picar_speed)
        if picar_speed > self.MAX_PICAR_SPEED:
            picar_speed = self.MAX_PICAR_SPEED
        self._speed = int(picar_speed)        

        if apply:
            self._apply_current_controls()


    def set_direction(self, direction, apply=True):
        self._direction = direction

        if apply:
            self._apply_current_controls()
    

    # State queries -- controls
    def get_direction(self):
        return self._direction

    def get_speed(self):
        return self._speed

    def get_steering(self):
        return self._steering

    # State queries -- pose
    def set_my_cartesian(self, pose=None, xyh=None):
        '''
        Set a new cartesian pose for the picar. Recalculate the BicyclePose based on this
        new picar pose and the current goal pose.

        Goal pose itself is not updated.
        '''
        if pose is None:
            pose = CartesianPose(*xyh)
        self._my_cartesian_pose = pose
        self._my_bicycle_model = pr.cartesian_to_bicycle(self._goal_cartesian_pose, robot_in_cartesian_ref_frame=pose)

    def set_goal_cartesian(self, goal=None, xyh=None):
        '''
        Set a new goal pose for the picar. Recalculate the BicyclePose based on this
        new goal pose and the current picar pose.

        Picar pose itself is not updated.
        '''
        if goal is None:
            goal = CartesianPose(*xyh)
        self._goal_cartesian_pose = goal
        self._my_bicycle_model = pr.cartesian_to_bicycle(goal, robot_in_cartesian_ref_frame=self._my_cartesian_pose)

    def set_bicycle_pose(self, pose=None, rab=None):
        '''
        Set a new bicycle pose for the scenario. Assume the picar's cartesian pose is unchanged, and 
        recalculate the cartesian Goal Pose based on this new bicycle pose and the current picar pose.

        Picar pose itself is not updated.
        '''
        if pose is None:
            pose = BicyclePose(*rab)
            self._my_bicycle_model.pose = pose
            self._goal_cartesian_pose = pr.bicycle_to_cartesian(pose, 
                robot_in_cartesian_ref_frame=self._my_cartesian_pose)





###############################################################################
###############################################################################
###########                  PRIVATE METHODS                    ###############
###############################################################################
###############################################################################


    ##############################################################
    #               APPLY CALIBRATION MAPS
    ##############################################################

    # Maps --> use calibration to map real world speed/gamma to the control signals for the Picar motors
    def _speed_world_to_picar(self, v_world, m=215, b=0):
        '''
        Map real-world speed in meters-per-second to 0-100 picar speed based on calibration.
        '''
        sgn = sign(v_world)
        v_picar = int(m*abs(v_world) + b)
        v_picar = min( max(v_picar,0), self.MAX_PICAR_SPEED) # bound speed between 0 and MAX_PICAR_SPEED
        return sgn*v_picar


    # The reverse; go from control signal to real world
    def _speed_picar_to_world(self, v_picar, m=215, b=0):
        '''
        Map picar +/- 0-100 speed to real-world speed in meters-per-second based on calibration.
        '''
        sgn = sign(v_picar)
        v_world = (abs(v_picar)+b)/m
        return sgn*v_world



    def _steering_world_to_picar(self, gamma_world, m_right=1, b_right=0, m_left=None, b_left=None):
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


    def _steering_picar_to_world(self, picar_turn, m_right=1, b_right=0, m_left=None, b_left=None):
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
        self._steering = 0


    def _stop_motors(self):
        '''
        Low level, minimal latency method to stop picar motors. Leaves steering/direction as is.
        '''
        self._bw.stop()
        self._speed= 0

    def _apply_current_controls(self):
        '''
        Send current v and gamma control signals to hardware.
        '''
        # Capture current control values
        direction = sign(self._direction)
        gamma = direction*int(self._steering) # reverse turn direction if we are going backward
        speed = int(abs(self._speed))

        # Set back wheel direction
        if direction == 1:
            self._bw.forward()
        elif direction == -1:
            self._bw.backward()
        elif direction == 0:
            # If direction is 0, don't change _bw's settings.
            pass
        else:
            # If any other edge case, halt picar
            self.halt()
            print('Picar object has invalid direction field {}. Picar halting.'.format(self._direction))

        # Set picar steering angle
        if gamma==0:
            self._turn_wheels_straight()
        else:
            self._turn_wheels(gamma)

        # Set picar speed
        if speed==0:
            self.stop_motors()
        else: 
            self._bw.speed = speed


    


class MyPicarController():

    def __init__(self, picar, rho_controller, alpha_controller, beta_controller, dt=1):
        self.picar = picar
        self.rho_controller   = rho_controller
        self.alpha_controller = alpha_controller
        self.beta_controller  = beta_controller

    ##############################################################
    #                  CALCULATE CONTROLS
    ##############################################################

    def calculate_speed(self, dt=1):
        '''
        Use PID control to calculate PICAR_SPEED and bound by picar limitations.
        '''
        # Calculate world speed
        world_speed = V(self.picar.get_rho(), self.rho_controller, dt=dt)

        # Translate to picar speed
        picar_speed = self.picar._speed_world_to_picar(world_speed)

        # Bound from [0, MAX_PICAR_SPEED]
        picar_speed = min(max(picar_speed,0), self.picar.MAX_PICAR_SPEED)
        
        return picar_speed


    def calculate_steering(self, speed, dt=1):
        '''
        Use PID control to calculate PICAR_ANGLE and bound by picar limitations.
        '''
        picar_angle = GAMMA(v=speed, alpha=self.picar.get_alpha(), beta=self.picar.get_beta(),
                            alpha_controller=self.alpha_controller, beta_controller=self.beta_controller,
                            L=self.picar.get_L(), dt=dt)

        # Check if no turn is required
        if picar_angle<3:
            return 0

        picar_angle *= self.picar.get_drive_direction()

        # Clip to max
        bound = self.picar._steering_picar_to_world( self.picar.MAX_PICAR_TURN )
        picar_angle = clip(picar_angle, -bound, bound)

        # Bound between [-pi, pi]
        return within_pi(picar_angle)


    def _DIRECTION(self):
        return should_i_back_up(self.picar.get_alpha())
    

    def _calculate_controls(self, dt=1):
        picar_speed     = self._SPEED (dt=dt)
        picar_angle = self._STEERING (v=picar_speed, dt=dt)
        return picar_speed, picar_angle




def test():
    pc = Picar(verbose=True,virtual=True,virtual_speederbose=False)
    waypoints = np.array( [
        [0,0,0],
        [1,-1,-pi/2]
        ])
    pc.traverse(waypoints)

if __name__=="__main__":
    test()






