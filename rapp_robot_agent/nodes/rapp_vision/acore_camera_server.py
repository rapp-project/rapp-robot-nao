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
import smtplib

# Importing core functionality from Naoqi
from naoqi import (ALProxy, ALBroker, ALModule)

# Importing others
import numpy as np
import math
import Image
#import time #for time measurement

from std_msgs.msg import String
from sensor_msgs.msg import Image as Image_ros
from cv_bridge import CvBridge, CvBridgeError

# Used for finding a text in a file
#import mmap

# Needed for encoding a file
#import base64

#######################################

# Global variables to store the Camera module instance and proxy to ALMemory Module
CameraServer = None
prox_memory = None


# Constants
class Constants:
	NAO_IP = "nao.local"
	PORT = 9559

#######################################


class CameraModule(ALModule):
	""" A simple module able to react to
	sound detection events
	"""
	
	# Constructor of CameraModule
	def __init__(self,name):
		ALModule.__init__(self,name)
		
		print "[Camera server] - Acore Camera Server initialization"
		
		# Initialization of ROS node
		rospy.init_node('acore_camera_server')
		self.moduleName = name
		
		self.bridge = CvBridge()
		
		# Initialization of Naoqi modules and ROS services
		self.initALModule()
		
		#self.setVariables()
		self.openServices()
		
		print "[Camera server] - Waits for clients ..."
		
		
	
	# Initialization of Naoqi modules
	def initALModule(self):
		print "[Camera server] - Initialization of Naoqi modules"
		
		print "[Camera server] - ALMemory proxy initialization"		
		global prox_memory#, prox_camera
		prox_memory = ALProxy("ALMemory")
		if prox_memory is None:
			rospy.logerr("[Camera server] - Could not get a proxy to ALMemory")
			exit(1)
		print "[Camera server] - ALVideoDevice proxy initialization"
		self.prox_camera = ALProxy("ALVideoDevice")
		if self.prox_camera is None:
			rospy.logerr("[Camera server] - Could not get a proxy to ALVideoDevice")
			exit(1)
			
		## Camera parameters
		self.resolution = 3	## k4VGA;
		self.colorSpace = 13	## kBGRColorSpace
		self.fps = 29;# maximum value for the highest camera resolution
		## Subscribe to the camera
		self.nameId = self.prox_camera.subscribe("python_camera", self.resolution, self.colorSpace, self.fps);
	
	# Initialization of ROS services
	def openServices(self):
		try:
			print "[Camera server] - setting services"
			print "[Camera server] - service - [rapp_capture_image]"
			self.service_rdqr = rospy.Service('rapp_capture_image', GetImage, self.handle_rapp_capture_image)
		except Exception, ex:
			print "[Camera server] - Exception (services) %s" % str(ex)

		
	#######################################
	
	# Core functionality methods 
	
	#######################################
	
	
	#########################
	
	# Handling methods - methods that used handling services
	
	#########################
		
	def handle_rapp_capture_image(self,req):
		print "[Camera server receives]: \t%s" % (req.request)
		# Get Frame from Camera
		
		try:
			#Modifying camera parameters
			kCameraSelectID = 18
			kCameraExposureAlgorithmID = 22
			if (req.request=="top"):
				self.prox_camera.setParam(kCameraSelectID,0)# 0 for top, 1 for bottom
			elif (req.request=="bottom"):
				self.prox_camera.setParam(kCameraSelectID,1)# bottom camera
			elif (req.request=="top - adaptive auto exposure 0"):# Average scene Brightness
				self.prox_camera.setParam(kCameraSelectID,0)# camera selection
				self.prox_camera.setParam(kCameraExposureAlgorithmID,0)# 0: Average scene Brightness; 1: Weighted average scene Brightness; 2: Adaptive weighted auto exposure for hightlights; 3: Adaptive weighted auto exposure for lowlights
				print "[Camera server]: Using - Average scene Brightness"
			elif (req.request=="top - adaptive auto exposure 1"):# weighted average scene Brightness
				self.prox_camera.setParam(kCameraSelectID,0)# camera selection
				self.prox_camera.setParam(kCameraExposureAlgorithmID,1)# Weighted average scene Brightness
				print "[Camera server]: Using - Weighted average scene Brightness"
			elif (req.request=="top - adaptive auto exposure 2"):# Adaptive weighted auto exposure for hightlights
				self.prox_camera.setParam(kCameraSelectID,0)# camera selection
				self.prox_camera.setParam(kCameraExposureAlgorithmID,2)# Adaptive weighted auto exposure for hightlights
				print "[Camera server]: Using - Adaptive weighted auto exposure for hightlights"
			elif (req.request=="top - adaptive auto exposure 3"):# Adaptive weighted auto exposure for lowlights
				self.prox_camera.setParam(kCameraSelectID,0)# camera selection
				self.prox_camera.setParam(kCameraExposureAlgorithmID,3)# Adaptive weighted auto exposure for lowlights
				print "[Camera server]: Using - Adaptive weighted auto exposure for lowlights"
			else :
				self.prox_camera.setParam(kCameraSelectID,0)# top camera as default
			#----
			
			# Capture image from selected camera
			self.naoImage = self.prox_camera.getImageRemote(self.nameId)
			while self.naoImage[6]==None:
				self.naoImage = self.prox_camera.getImageRemote(self.nameId) # for avoidance of the black image (empty frame)

			if self.naoImage[6]!=None:
				self.frame_img=Image.fromstring("RGB", (self.naoImage[0], self.naoImage[1]), self.naoImage[6]) ## tuple
								
				self.frame_img= np.array(self.frame_img)##For NAO #conversion from tuple to numpy array
				#print self.frame_img.shape[2]
				
				#self.frame_img= cv2.cv.fromarray(self.frame_img[:,:])##from numpy array to CvMat
				
				self.image_message = self.bridge.cv2_to_imgmsg(self.frame_img,"rgb8")#, encoding="rbg") # form numpy.array to imgmsg for ROS communication
				#self.cv_image = self.bridge.imgmsg_to_cv2(self.image_message,"rgb8")
		except AttributeError, ex:
			print "[Camera server] - Exception AtrributeError = %s" % str(ex)
		except Exception, ex:
			print "[Camera server] - Unnamed exception = %s" % str(ex)

		return self.image_message
	
	
# Testng SIGINT signal handler
def signal_handler(signal, frame):
	print "[Camera server] - signal SIGINT caught"
	print "[Camera server] - system exits"
	sys.exit(0)

def main():
	""" Main entry point
	
	"""
	# It is needed to use a broker to be able to construct NAOQI 
	# modules and subscribe to other modules. The broker must stay
	# alive until  the program exists
	try:
		signal.signal(signal.SIGINT, signal_handler)
		print "[Camera server] - Press Ctrl + C to exit system correctly"
		
		myBroker = ALBroker("myBroker", "0.0.0.0", 0, Constants.NAO_IP,Constants.PORT)
		CameraServer = CameraModule("CameraServer")
		rospy.spin()
	
	except AttributeError:
		print "[Camera server] -  - AttributeError"
		#unsubscribe from camera
		CameraServer.prox_camera.unsubscribe(CameraServer.nameId);
		myBroker.shutdown()
		sys.exit(0)
		
	except (KeyboardInterrupt, SystemExit):
		print "[Camera server] - SystemExit Exception caught"
		#unsubscribe from camera
		CameraServer.prox_camera.unsubscribe(CameraServer.nameId);
		myBroker.shutdown()
		sys.exit(0)
		
	except Exception, ex:
		print "[Camera server] - Exception caught %s" % str(ex)
		#unsubscribe from camera
		CameraServer.prox_camera.unsubscribe(CameraServer.nameId);
		myBroker.shutdown()
		sys.exit(0)
	
if __name__ == "__main__":
	try:
		main()
	except Exception,e:
		print "__name__ - Error %s" % str(e)
