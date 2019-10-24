'''
Filename: virtual_picar.py
Description: Virtual picar.Front_Wheels and picar.Back_Wheels interfaces
	for testing code when you don't have the picar.
Author: Andi Frank
E-mail: a2frank@eng.ucsd.edu
Purpose: CSE 276A - Intro to Robotics; Fall 2019
'''

class Virtual_Front_Wheels():

    def __init__(self, db="config"):
        self.max_turn = 0
        self._straight_angle = 0

    def ready(self, verbose=False):
        if verbose:
            print("Front wheels readied.")

    def turn(self, angle, verbose=False):
        if verbose:
            print("Turning {}".format(angle))

    def turn_straight(self, verbose=False):
        if verbose:
            print("Turning front wheels straight.")


class Virtual_Back_Wheels():

    def __init__(self, db="config"):
        self.max_turn = 0
        self.direction = 1
        self.speed = 0

    def ready(self, verbose=False):
        if verbose:
            print("Back wheels readied.")

    def forward(self, verbose=False):
        self.direction = 1
        if verbose:
            print("Back wheels set to forward.")

    def backward(self, verbose=False):
        self.direction = -1
        if verbose:
            print("Back wheels set to backward.")

    def speed(self, spd, verbose=False):
        self.speed = spd
        if verbose:
            print("Speed set to {}.".format(spd))

    def stop(self, verbose=False):
        self.speed = 0
        if verbose:
            print("Back wheels stopped.")