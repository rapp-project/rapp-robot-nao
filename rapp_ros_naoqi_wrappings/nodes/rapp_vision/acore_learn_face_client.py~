#!/usr/bin/env python
__author__ = "Jan Figat"

# Importing core system functionality
import signal
import os, sys
import rospy
import rosparam

# Importing services
from rapp_robot_agent.srv import *

# Needed for encoding a file
import base64

# Importing OpenCV library
#import cv #cv2.cv as cv #Use OpenCV-2.4.3
#import cv2

# Importing others
import numpy as np
import math
import Image
import time #for time measurement


class AcoreCameraClient():
	
	def __init__(self):
		print "[Camera client] - Acore camera Client initialization"
	

	# Handling a communication with service "rapp_capture_image"
	def captureCameraFrame(self,request):
		print "[Camera client] - Waits for server"
		rospy.wait_for_service('rapp_capture_image')
		try:
			print "[Camera client] - captureCameraFrame"
			captureImage = rospy.ServiceProxy('rapp_capture_image', GetImage)
			resp_capture_image = captureImage(request)
			print "[Camera client] - Image captured"
			return resp_capture_image
		except rospy.ServiceException, e:
			print "[Camera client] - Calling service [/rapp_capture_image] Failed: %s"%e
			exit(1)

class AcoreLearnFaceClient():
	
	def __init__(self):
		print "[LearnFace client] - Acore Learn Face Client initialization"	

	# Handling a communication with service "rapp_learn_face"
	def learnNewFace(self,requestName):
		print "[LearnFace client] - Waits for server"
		rospy.wait_for_service('rapp_learn_face')
		try:
			print "[LearnFace client] - learnNewFace - Tries to learn new face"
			learnFace = rospy.ServiceProxy('rapp_learn_face', LearnFace)
			resp_learn_face = learnFace(requestName)
			print "[LearnFace client] - learnNewFace - done"
			return resp_learn_face
		except rospy.ServiceException, e:
			print "[LearnFace client] - Calling service [/rapp_learn_face] Failed: %s"%e
			exit(1)
	
			
######################################################################
## Functions defined by rapp dynamic agent programmer ##
######################################################################

	# A method that i used to attach files (recorded audio and an Rapp image) 
	# into a message and then sends it to defined email address.
	

# Testng SIGINT signal handler
def signal_handler(signal, frame):
	print "[LearnFace client] - signal SIGINT caught"
	print "[LearnFace client] - system exits"
	sys.exit(0)

# Main entry point
def main():
	signal.signal(signal.SIGINT, signal_handler)
	print "[LearnFace client] - Press Ctrl + C to exit system correctly"
	
	# Testing ROS params
	say_param="Core agent"
	rospy.set_param("say_param", say_param)
	xy=rospy.get_param("say_param")	

	print "[LearnFace client] - Requesting %s" % (xy)
	client_LearnFace = AcoreLearnFaceClient()
	
	# Testing [/rapp_learn_face] service
	print "[LearnFace client] - Testing [/rapp_learn_face] service"
	response_learnFace = client_LearnFace.learnNewFace("Janek")
	print response_learnFace
	
	return


if __name__ == "__main__":
	try:
		main()
	except Exception,e:
		print "__name__ - Error %s" % str(e)
