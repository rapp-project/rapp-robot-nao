#!/usr/bin/env python

__author__ = "Jan Figat"

########################
# Imports
########################

# Importing services
from rapp_robot_agent.srv import *

# Importing core system functionality
import signal
import sys, os
import rospy
import time

# Importing core functionality from Naoqi
from naoqi import (ALProxy, ALBroker, ALModule)

# Used for finding a text in a file
import mmap

# Needed for encoding a file
import base64

#######################################

# Global variables to store the EmailRecognition module instance and proxy to ALMemory Module
SaveImage = None
prox_memory = None

# Constants
class Constants:
	
	SOUND_DETECTED = "onSoundDetected"
	EVENT_SOUND = "SoundDetected"
	NAO_IP = "nao.local"
	PORT = 9559
	# Testing on NAO
	captured_img_dest="/home/nao/recordings/cameras/"
	resolutionMap = {
			'160 x 120': 0,
			'320 x 240': 1,
			'640 x 480': 2,
			'1280 x 960': 3
	}
	cameraMap = {
			'Default': -1,
			'Top': 0,
			'Bottom': 1
	}

	
	

#######################################

# EmailRecognitionModule class definition
class PhotoCaptureModule(ALModule):
	""" A simple module able to react to
	sound detection events
	"""
	
	# Constructor of PhotoCaptureModule
	def __init__(self,name):
		ALModule.__init__(self,name)
		
		print "[PhotoCapture server] - Acore email Server initialization"
		
		# Initialization of ROS node
		rospy.init_node('acore_photocapture_server')
		self.moduleName = name
		
		# Initialization of Naoqi modules and ROS services
		self.initALModule()
		
		self.openServices()
		
		print "[PhotoCapture server] - Waits for clients ..."
		
		
	
	# Initialization of Naoqi modules
	def initALModule(self):
		print "[PhotoCapture server] - Initialization of Naoqi modules"
		
		print "[PhotoCapture server] - ALMemory proxy initialization"		
		global prox_memory
		prox_memory = ALProxy("ALMemory")
		if prox_memory is None:
			rospy.logerr("[PhotoCapture server] - Could not get a proxy to ALMemory")
			exit(1)
			
			
		print "[PhotoCapture server] - ALPhotoCapture proxy initialization"
		self.prox_pc = ALProxy("ALPhotoCapture")
		if self.prox_pc is None:
			rospy.logerr("[PhotoCapture server] - Could not get a proxy to ALPhotoCapture")
			exit(1)
	
		
	# Initialization of ROS services
	def openServices(self):
		try:
			print "[PhotoCapture server] - setting services"
			print "[PhotoCapture server] - service - [rapp_photocapture]"
			self.service_pc = rospy.Service('rapp_photocapture', PhotoCapture, self.handle_rapp_photocapture)
			
		except Exception, ex:
			print "[PhotoCapture server] - Exception %s" % str(ex)

		
	#######################################
	
	# Core functionality methods 
	
	#######################################
	
	#########################
	
	# Handling methods - methods that used handling services
	
	#########################
		
	def handle_rapp_photocapture(self,req):
		print "[PhotoCapture server receives]: \t%s" % (req.imageName)
		isCaptured=0
		
		try:
			## set camera parameters:
			self.prox_pc.setResolution(req.cameraImageResolution)
			self.prox_pc.setCameraID(req.cameraId)
			self.prox_pc.setPictureFormat(req.cameraImageFormat)
			## save frame from selected^^ camera
			# without setting up the camera parameters it will use the default parameters (".jpg", 640x480)
			self.prox_pc.takePictures(1, Constants.captured_img_dest, str(req.imageName)) ## takes 1 picture
			isCaptured =1
		except AttributeError, ex:
			print "[PhotoCapture server] - Exception AtrributeError = %s" % str(ex)
		except Exception, ex:
			print "[PhotoCapture server] - Unnamed exception = %s" % str(ex)
					
		response = str(str(Constants.captured_img_dest)+str(req.imageName)+str(".")+str(req.cameraImageFormat))
		return PhotoCaptureResponse(isCaptured, response)
		
	

# Testng SIGINT signal handler
def signal_handler(signal, frame):
	print "[PhotoCapture server] - signal SIGINT caught"
	print "[PhotoCapture server] - system exits"
	sys.exit(0)

def main():
	""" Main entry point
	
	"""
	# It is needed to use a broker to be able to construct NAOQI 
	# modules and subscribe to other modules. The broker must stay
	# alive until  the program exists
	try:
		signal.signal(signal.SIGINT, signal_handler)
		print "[PhotoCapture server] - Press Ctrl + C to exit system correctly"
		
		myBroker = ALBroker("myBroker", "0.0.0.0", 0, Constants.NAO_IP,Constants.PORT)
		global PhotoCapture
		PhotoCapture = PhotoCaptureModule("PhotoCapture")
		rospy.spin()
	
	except AttributeError:
		print "[PhotoCapture server] - PhotoCapture - AttributeError"
		myBroker.shutdown()
		sys.exit(0)
		
	except (KeyboardInterrupt, SystemExit):
		print "[PhotoCapture server] - SystemExit Exception caught"
		myBroker.shutdown()
		sys.exit(0)
		
	except Exception, ex:
		print "[PhotoCapture server] - Exception caught %s" % str(ex)
		myBroker.shutdown()
		sys.exit(0)
		
if __name__ == "__main__":
	try:
		main()
	except Exception,e:
		print "__name__ - Error %s" % str(e)
