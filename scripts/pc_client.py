'''
Filename: pc_client.py
Description: Interface to transmit commands to Picar RPi server from client laptop
	through the web.
Author: Andi Frank
Date: Oct 24, 2019
E-mail: a2frank@eng.ucsd.edu
Purpose: CSE 276A - Intro to Robotics; Fall 2019
Comments: Much of the functionality of this file is modified from:
	github.com/quanvuong/ucsd-cse276A/hw1/main.py
	github.com/sunfounder/SunFounder_PiCar-V/client/client.py
'''

##############################################################
#                       IMPORTS
##############################################################

# System
import sys
if sys.version_info.major < 3 or sys.version_info.minor < 4:
    raise RuntimeError('At least Python 3.4 is required')
sys.path.append("../lib")

# Client
import time, http.client
from PyQt5 import QtCore, uic, QtWidgets  
import icons_rc
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QPixmap
from urllib.request import urlopen
import requests

# Math
import numpy as np
from scipy import stats
from itertools import count
import cv2
import pyzbar
import traceback
from helpers import InputError


HOST      = '172.20.10.2'
PORT 	  = '8005'
autologin = 1

# BASE_URL is variant use to save the format of host and port
base_url = 'http://' + HOST + ':'+ PORT + '/'

external_url = 'https://api.github.com/events'

##############################################################
#                    CLIENT HANDLER
##############################################################


class Picar_Client():

	def __init__(self, base_url, verbose=False):
		self.base_url = base_url
		self.verbose = verbose


	def __request__(self, url, times=10, timeout=5):
		''' Attempt to connect to <url> <times> times.
		Returns 0 on success, -1 on failure.

		Modified from github.com/sunfounder/SunFounder_PiCar-V/client/client.py
		'''

		if self.verbose:
			print("URL: {}".format(url))
			print("Requesting connection...", end='')
		# Attempt to connect <times> times
		for i in range(times):
			try:
				if i>0 and self.verbose:
					print("\nRetrying...", end='\t\t')
				# Send request
				requests.get(url, timeout=timeout) 
				# If no error is raised, we are successfully connected.
				if self.verbose:
					print(" connected.")
					print( "\nConnected to URL: '{}' ".format(url))
				# Return success code
				return 0
			except requests.ConnectionError:
				print(" CONNECTION ERROR")
			except:
				print()
				traceback.print_exc()
				sys.exit(0)

		# All connection attempts exhausted, abort
		if self.verbose:
			print("CONNECTION ERROR\n") 
			print("Unable to connect to URL: '{}' ".format(url))
		# Return failure code
		return -1



	def run_action(self, cmd, timeout=5):
		"""Ask server to do something, use in running mode

		Post requests to server, server will do what client want to do according to the url.
		This function for running mode

		Args:
			# ============== Back wheels =============
			'bwready' | 'forward' | 'backward' | 'stop'

			# ============== Front wheels =============
			'fwready' | 'fwleft' | 'fwright' |  'fwstraight'

			# ================ Camera =================
			'camready' | 'camleft' | 'camright' | 'camup' | 'camdown'

		Modified from github.com/sunfounder/SunFounder_PiCar-V/client/client.py
		"""
		# set the url include action information
		url = self.base_url + 'run/?action=' + cmd
		if self.verbose:
			print('url: %s'% url)
		# post request with url
		self.__request__(url, timeout)


	def run_speed(self, speed, timeout=5):
		"""Ask server to set speed, use in running mode

		Post requests to server, server will set speed according to the url.
		This function for running mode.

		Args:
			'0'~'100'

		Modified from github.com/sunfounder/SunFounder_PiCar-V/client/client.py

		"""
		# Check that speed is within bounds
		if speed < 0 or speed > 100:
			raise InputError("'Input argument speed' {} must be between 0 and 100.".format(speed))
		# Make sure speed is an int; cast to int if possible
		speed = int(speed)

		# Set set-speed url
		url = self.base_url + 'run/?speed=' + str(speed)
		if self.verbose:
			print('url: %s'% url)
		# Set speed
		self.__request__(url, timeout)


	def check_connection(self, timeout=10):
		"""Spot-test if connection is okay.

		Post one request to server. If connection is successful,
			server will return http response 'OK'.

		Args:
			none

		Returns:
			if connection is ok, return True
			if connection is not ok, return False

		Raises:
			none

		Modified from github.com/quanvuong/ucsd-cse276A/hw1/main.py	
		"""
		cmd = 'connection_test'
		url = self.base_url + cmd
		if self.verbose:
			print('url: %s'% url)
		# if server find there is 'connection_test' in request url, server will response 'OK'
		try:
			r=requests.get(url, timeout=timeout)
			if r.text == 'OK':
				if self.verbose:
					print("Connection spot-check succesful.")
				return True
		except:
			if self.verbose:
				print("Connection spot-check failed.")
				traceback.print_exc()
			return False



def test():
	client = Picar_Client(base_url, verbose=True)
	print("Object created.")
	client.__request__(external_url)
	print("__request__() executed.")
	# client.check_connection()
	# print("check_connection() executed")
	# client.run_action('fwready')
	# print("run_action() executed.")
	client.run_speed(40)
	print("run_speed() executed.")
	


if __name__=="__main__":
	test()