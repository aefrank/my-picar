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

pc.setup()
fw = pc.front_wheels.Front_Wheels(db="config")
bw = pc.back_wheels.Back_Wheels(db="config")

fw.turning_max = 30
fwd_speed = 50
bkw_speed = 50

bw.speed = fwd_speed

fw.ready()
bw.ready()

DELAY = .2

try:
    bw.forward()
    fw.turn_left()
    time.sleep(1)
    bw.speed = fwd_speed
    time.sleep(DELAY*10)
    fw.turn_right()
    time.sleep(1)
    bw.backward()
    time.sleep(DELAY*10)
    fw.turn_straight()
    bw.stop()
    fw.ready()
    bw.ready()
    
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


