'''
Filename: my_pid.py
Description: PID controller class and functions.
Author: Andi Frank
E-mail: a2frank@eng.ucsd.edu
Purpose: CSE 276A - Intro to Robotics; Fall 2019
'''

from helpers import sign


##############################################################
#                       PID CONTROLLER
##############################################################

class myPID():

    def __init__(self, Kp=0, Ki=0, Kd=0, integral_error=0, last_error=0, dt=1, 
                    integral_max=None, integral_active_region=None):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.integral_error = integral_error
        self.integral_max = integral_max
        self.integral_active_region = integral_active_region
        self.last_error = last_error

        self.dt = dt # Default dt

    def P(self, error=0, dt=None):
        '''
        Proportional term
        '''
        if dt is None:
            dt = self.dt

        return self.Kp*error



    def I(self, error=0, dt=None):
        '''
        Integral term
        '''

        if dt is None:
            dt = self.dt

        # If we have defined an active region for the integral controller,
        # only use integral control when error is within that region.
        # Helps control integral windup.
        if (self.integral_active_region is not None):

            # Check if you are outside integral_active_region
            if (abs(error) > abs(self.integral_active_region)):
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

        return self.Ki*self.integral_error*self.dt



    def D(self, error=0, dt=None): 
        '''
        Derivative term
        '''
        if dt is None:
            dt = self.dt

        d_error = error - self.last_error
        # Update previous error
        self.last_error = error

        return self.Kd*d_error*self.dt


    def input(self, error=0, dt=None):
        '''
        Input an error signal and return a control signal.
        '''
        if dt is None:
            dt = self.dt

        return self.P(error=error,dt=dt) + self.I(error=error,dt=dt) + self.D(error=error,dt=dt)


        