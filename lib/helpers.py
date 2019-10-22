'''
Filename: helpers.py
Description: Module for common helper functions I don't want to have to define every file.
Author: Andi Frank
E-mail: a2frank@eng.ucsd.edu
Purpose: Convenience
'''

from math import pi, sin, cos, tan, atan2

def sign(n):
    if n > 0:
        return 1
    if n < 0:
        return -1
    else: # should only be in 0 case
        return 0

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