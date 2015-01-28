#!/usr/bin/env python

__author__ = "Jan Figat"
# Importing core system functionality
import signal
import os, sys
import rospy
import rosparam

# Importing services
from rapp_core_agent.srv import *

# Needed for encoding a file
import base64

class Constants:
	# Testing on NAO
	test_image_name = "camera_frame"

class AcorePhotoCaptureClient():
	
	def __init__(self):
		print "[PhotoCapture client] - Acore PhotoCapture Client initialization"
	

	# Handling a communication with service "rapp_say"
	def capturePhoto(self,request,camImageResolution,cameraId,camImageFormat):
		print "[PhotoCapture client] - Waits for server"
		rospy.wait_for_service('rapp_photocapture')
		try:
			photoCapture = rospy.ServiceProxy('rapp_photocapture', PhotoCapture)
			resp_photocapture = photoCapture(request,camImageResolution,cameraId,camImageFormat)
			# return resp_photocapture.imagePath ## dziala 		
			return resp_photocapture.isCaptured, resp_photocapture.imagePath
		except rospy.ServiceException, e:
			print "[PhotoCapture client] - Calling service [/rapp_photocapture] Failed: %s"%e
			exit(1)
	
	

# Testng SIGINT signal handler
def signal_handler(signal, frame):
	print "[PhotoCapture server] - signal SIGINT caught"
	print "[PhotoCapture server] - system exits"
	sys.exit(0)

# Main entry point
def main():
	signal.signal(signal.SIGINT, signal_handler)
	print "[PhotoCapture server] - Press Ctrl + C to exit system correctly"
	
	# Testing ROS params
	say_param="Core agent"
	rospy.set_param("say_param", say_param)
	xy=rospy.get_param("say_param")	
	
	print "[PhotoCapture client] - Requesting %s" % (xy)
	client = AcorePhotoCaptureClient()
	
	# Testing [/rapp_photocapture] service
	response_photocapture = client.capturePhoto( str(Constants.test_image_name),3,0,"png" ) # captures photo from top camera with the highest resolution, and saves it in "png" format
	print "[PhotoCapture client] - captured image path:\t[ %s ]"%(response_photocapture)#imagePath
		
	return


if __name__ == "__main__":
	try:
		main()
	except Exception,e:
		print "__name__ - Error %s" % str(e)
