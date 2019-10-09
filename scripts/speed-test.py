'''
Filename: picar_move.py
Description: First attempt at getting the PiCar to run a custom program.
Author: Andi Frank
E-mail: a2frank@eng.ucsd.edu
Purpose: CSE 276A: Intro to Robotics; Fall 2019

Inspiration drawn from:
    https://github.com/korzen/PiCar_ROS/

'''

import sys
import time
import SunFounder_PiCar.picar as pc

TURN_MAX = 45

pc.setup()
fw = pc.front_wheels.Front_Wheels(db="config")
bw = pc.back_wheels.Back_Wheels(db="config")

fw.turning_max = TURN_MAX
fwd_speed = 50
bkw_speed = 50

bw.speed = fwd_speed

fw.ready()
bw.ready()

DELAY = .2

def sign(n):
    if n > 0:
        return 1
    if n < 0:
        return -1
    else:
        return 0

def turn(fw, angle):
    fw.turn(angle+fw._straight_angle)

straighten_delay = 0.05
def turn_straight(fw, overshoot=10):
    turn(fw, overshoot)
    time.sleep(straighten_delay)
    turn(fw, -overshoot)
    time.sleep(straighten_delay)
    fw.turn_straight()

def wheelangle_test(fw, turn_angle=TURN_MAX):
    print("Wheel angle test. Input angle = {}".format(turn_angle))
    try:
        turn(fw, turn_angle)
        #fw.turn_right()
        while(1):
            pass
    except KeyboardInterrupt:
        print("KeyboardInterrupt, turn straight.")
        turn_straight(fw)
    finally:
        print("Exiting, turn straight.")
        turn_straight(fw)

def turnrad_test(fw, bw, turn_angle=TURN_MAX, speed=50, duration=10):
    print("Turn radius test. Turn angle = {}. Speed = {}.".format(turn_angle, speed))
    try:
        turn(fw,turn_angle)
        bw.forward()
        bw.speed = speed
        time.sleep(duration)
    except KeyboardInterrupt:
        print("KeyboardInterrupt, motor stop.")
        turn_straight(fw)
        bw.stop()
    finally:
        print("Finish, motor stop.")
        fw.turn_straight()
        bw.stop()


def speed_test(fw, bw, speed=50, duration=10):
    print("Speed test. Speed={}.".format(speed))
    try:
        fw.turn_straight()
        bw.forward()
        bw.speed = speed
        time.sleep(duration)
        bw.stop()
    except KeyboardInterrupt:
        print("KeyboardInterrupt, motor stop.")
        bw.stop()
    finally:
        print("Finish, motor stop.")
        bw.stop()

# speed_test(fw, bw, fwd_speed)
if len(sys.argv) < 3:
    turnrad_test(fw,bw,turn_angle=int(sys.argv[1]))
else:
    turnrad_test(fw,bw,turn_angle=int(sys.argv[1]), speed=int(sys.argv[2]))
# wheelangle_test(fw,turn_angle=int(sys.argv[1]))

#try:
#    turn(fw, 20)
#    time.sleep(1)
#    turn(fw, -30)
#    #fw.turn_right()
#    while(1):
#        pass
#except KeyboardInterrupt:
#    print("KeyboardInterrupt, turn straight and stop.")
#    bw.stop()
#    turn_straight(fw)
#finally:
#    print("Exiting, turn straight and stop.")
#    bw.stop()
#    turn_straight(fw)


