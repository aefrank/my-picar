'''
Filename: helpers.py
Description: Module for common helper functions I don't want to have to define every file.
Author: Andi Frank
E-mail: a2frank@eng.ucsd.edu
Purpose: Convenience
'''

def sign(n):
    if n > 0:
        return 1
    if n < 0:
        return -1
    else: # should only be in 0 case
        return 0
