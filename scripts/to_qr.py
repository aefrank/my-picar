'''
Filename: to_qr.py
Description: Drive picar forward until it centers a QR code at an acceptable distance.
Author: Andi Frank
E-mail: a2frank@eng.ucsd.edu
Purpose: HW2, CSE 276A - Intro to Robotics; Fall 2019
'''

import sys

# Add library folder to path
sys.path.append("../lib")
import helpers
from my_picar import Picar as picar

# Picam libraries
import cv2
from pyzbar.pyzbar import decode
import numpy as np
import imutils

VERBOSE = True

# From calibration
CAMERA_MATRIX = [	[668.26209391,  	 		0.,         299.10258721],
 					[  0.,         	 646.96066863, 			231.98017084],
 					[  0.,           			0.,           		  1.]]
# NEW_CAMERA_MATRIX, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
DISTORTION = [[-0.48698219, -0.51573954,  0.01664705, -0.00720086,  2.29695665]]


def print_qr(QR):
	for qr in QR:
		string = "{}\nROI:\t\t{}\nCenter (x,y):\t({},{})".format(
			qr.data.decode('UTF-8'), qr.rect, 
			qr.rect.left+qr.rect.width/2, qr.rect.top-qr.rect.height/2)
		print(string)
	print()

def preprocess(frame):
	# convert the frame to grayscale, normalize brightness, and increase contrast
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame = cv2.normalize(frame,frame,0,255,cv2.NORM_MINMAX)
    frame = cv2.addWeighted( frame, 1.5, np.zeros(frame.shape, frame.dtype), 0, 0)

    return frame

def show(frame, mirrorlr=False, mirrorud=False):
	# Flip image if necessary
    if mirrorlr:
        frame = cv2.flip(frame,1)
    if mirrorud:
        frame = cv2.flip(frame,0)

    # Display frame
    cv2.imshow('PiCam view', frame)

def to_qr(picar=None, dev=0, verbose=False, mirrorlr=False, mirrorud=False):
	''' 
	Drive Picar to a QR code and center it.
	'''

	# Create video capture object 
	cam = cv2.VideoCapture(dev)
	exit = False

	# Control loop
	try:
		# Until exit flag
		while not exit:
			# Get raw frame 
			frame_available, frame = cam.read()
			if frame_available:
				# Preprocess
				frame = preprocess(frame)

				# Search for QR codes
				qr = decode(frame)

				# If codes are detected...
				if len(qr) > 0:

					# If multiple codes detected, only consider the first.
					# Additionally, print out QR code information if verbose
					if verbose:
						if len(qr) > 1:
							print("Multiple QR codes detected! Only first will be considered.")
						print_qr(qr)
					qr = qr[0]

					
					# Get horizontal error
					qr_center_x = qr.rect.left+qr.rect.width/2

					# PID control
					frame_center_x = frame.shape[1]/2
					frame_center_y = frame.shape[0]/2
					x_error = frame_center_x - qr_center_x
					print("Frame Center (x,y): ({},{})".format(frame_center_x,frame_center_y))
					print("x error: {}".format(x_error))
					picar.turn(x_error/frame_center_x*picar.MAX_PICAR_TURN, units='picar')
					




				# Show frame
				show(frame)

			# Exit on 'q'
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break

	# Release camera before exiting
	finally:
		cam.release()
		cv2.destroyAllWindows()




def main():
	pc = picar(virtual=True, virtual_verbose=True)
	to_qr(picar=pc, dev=1, verbose=VERBOSE)
	

if __name__ == '__main__':
	main()