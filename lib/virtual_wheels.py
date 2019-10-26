'''
Filename: virtual_picar.py
Description: Virtual picar.Front_Wheels and picar.Back_Wheels interfaces
	for testing code when you don't have the picar.
Author: Andi Frank
E-mail: a2frank@eng.ucsd.edu
Purpose: CSE 276A - Intro to Robotics; Fall 2019
'''

class Virtual_Front_Wheels():

    def __init__(self, db="config", verbose=False):
        self.max_turn = 0
        self._straight_angle = 0
        self.verbose = verbose

    def ready(self):
        if self.verbose:
            print("Front wheels readied.")

    def turn(self, angle):
        if self.verbose:
            print("Turning {}".format(angle))

    def turn_straight(self):
        if self.verbose:
            print("Turning front wheels straight.")


class Virtual_Back_Wheels():

    def __init__(self, db="config", verbose=False):
        self.max_turn = 0
        self.direction = 1
        self.speed = 0
        self.verbose = verbose

    def ready(self):
        if self.verbose:
            print("Back wheels readied.")

    def forward(self):
        self.direction = 1
        if self.verbose:
            print("Back wheels set to forward.")

    def backward(self):
        self.direction = -1
        if self.verbose:
            print("Back wheels set to backward.")

    def speed(self, spd):
        self.speed = spd
        if self.verbose:
            print("Speed set to {}.".format(spd))

    def stop(self):
        self.speed = 0
        if self.verbose:
            print("Back wheels stopped.")