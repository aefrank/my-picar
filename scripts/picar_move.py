'''
Filename: picar_move.py
Description: First attempt at getting the PiCar to run a custom program.
Author: Andi Frank
E-mail: a2frank@eng.ucsd.edu
Purpose: CSE 276A: Intro to Robotics; Fall 2019

Inspiration drawn from:
    https://github.com/korzen/PiCar_ROS/

'''

import time
import SunFounder_PiCar.picar as pc


class PiCar():

    def __init__(self, turning_max=30, fwd_speed=50, rev_speed=50):
        self._fw = pc.front_wheels.Front_Wheels()
        self._bw = pc.back_weels.Back_Wheels()

        self.speed = [fwd_speed, rev_speed]
        self.turning_max = turning_max
        self.ready()

    def ready(self):
        self._fw.ready()
        self._bw_ready()

    def forward(speed=None):
        self._bw.forward()
        if speed is None:
            speed = self.speed[1]
        self._bw.speed = speed

    def backward(speed=None):
        self._bw.backward()
        if speed is None:
            speed = self.speed[0]
        self._bw.speed = speed

    def stop():
        self._bw.stop()


pc.setup()
fw = pc.front_wheels.Front_Wheels()
bw = pc.back_wheels.Back_Wheels()

fw.turning_max = 30
fwd_speed = 50
bkw_speed = 50

bw.speed = fwd_speed

fw.ready()
bw.ready()

DELAY = .2

try:
    bw.forward()
    bw.speed = fwd_speed
    time.sleep(DELAY*10)
    bw.backward()
    time.sleep(DELAY*10)
    
#    for i in range(10):
#        bw.speed = 50
#        print("Backward, speed = {}".format(i))
#        time.sleep(DELAY)
#    bw.forward()
#    for i in range(10,0,-1):
#        bw.speed = 50
#        print("Forward, speed = {}".format(i))
#        time.sleep(DELAY)
except KeyboardInterrupt:
    print("KeyboardInterrupt, motor stop.")
    bw.stop()
finally:
    print("Finish, motor stop.")
    bw.stop()


