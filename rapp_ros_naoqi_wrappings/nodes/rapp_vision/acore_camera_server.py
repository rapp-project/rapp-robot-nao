#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = "Jan Figat"

########################
# Imports
########################

# Importing services
from rapp_ros_naoqi_wrappings.srv import *

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
		
		rospy.loginfo("[Camera server] - Acore Camera Server initialization")
		
		# Initialization of ROS node
		rospy.init_node('acore_camera_server')
		self.moduleName = name
		
		self.bridge = CvBridge()
		
		# Initialization of Naoqi modules and ROS services
		self.initALModule()
		
		#self.setVariables()
		self.openServices()
		
		rospy.loginfo("[Camera server] - Waiting for clients ...")
		
		
	
	# Initialization of Naoqi modules
	def initALModule(self):
		rospy.loginfo("[Camera server] - Initialization of Naoqi modules")
		rospy.loginfo("[Camera server] - ALMemory proxy initialization")
		
		self.prox_memory = ALProxy("ALMemory")
		if self.prox_memory is None:
			rospy.logerr("[Camera server] - Could not get a proxy to ALMemory")
			exit(1)

		rospy.loginfo("[Camera server] - ALVideoDevice proxy initialization")
		self.prox_camera = ALProxy("ALVideoDevice")
		if self.prox_camera is None:
			rospy.logerr("[Camera server] - Could not get a proxy to ALVideoDevice")
			exit(1)
			
		## Camera parameters
		#self.resolution = 3	## k4VGA;
		#self.colorSpace = 13	## kBGRColorSpace
		#self.fps = 29;# maximum value for the highest camera resolution
		## Subscribe to the camera
		#self.nameId = self.prox_camera.subscribe("python_camera", self.resolution, self.colorSpace, self.fps);
	
	# Initialization of ROS services
	def openServices(self):
		try:
			rospy.loginfo("[Camera server] - setting services")
			rospy.loginfo("[Camera server] - service - [rapp_capture_image]")
			self.service_rdqr = rospy.Service('rapp_capture_image', GetImage, self.handle_rapp_capture_image)
			self.service_rscp = rospy.Service('rapp_set_camera_parameter', SetCameraParam, self.handle_rapp_set_camera_parameter)
			self.service_rscps = rospy.Service('rapp_set_camera_parameters', SetCameraParams, self.handle_rapp_set_camera_parameters)
		except Exception, ex:
			rospy.logerr("[Camera server] - Exception (services) %s", str(ex))

		
	#######################################
	# Core functionality methods 
	#######################################
	
	
	#########################
	# Handling methods - methods that used handling services
	#########################
		
	def handle_rapp_capture_image(self,req):
		rospy.loginfo("[Camera server receives]: camera: %s resolution: %d", req.request, req.resolution)
		
		# Get Frame from Camera
		try:
			#Modifying camera parameters
			#kCameraSelectID = 18
			#kCameraExposureAlgorithmID = 22
			#kCameraResolutionID = 14
			
			## Changes camera parameters
			if (req.resolution in [0,1,2,3]):
				self.resolution = req.resolution;#self.prox_camera.getParam(14);
			else:
				self.resolution = 2;

			#if (req.request.find("bottom") != -1):
			#	self.selectedCamera=1; #bottom
			#else:
			#	self.selectedCamera=0; #top

			self.selectedCamera = req.request; #get camera ID
			self.colorSpace = 13;	## kBGRColorSpace
			#self.fps = 29;# maximum value for the highest camera resolution
			#self.colorSpace = req.colorSpace;#self.prox_camera.getColorSpace(req.request); #get color space
			## is self.colorSpace != 9 ?
			self.fps = self.prox_camera.getParameter(req.request,15);# get frame rate
			#self.resolution = self.prox_camera.getParameter(req.request,14); # get camera resolution


			## Subscribe to the camera
			self.nameId = self.prox_camera.subscribeCamera("pythonCameraID",self.selectedCamera, self.resolution, self.colorSpace, self.fps);
			##
			#----
			
			# Capture image from selected camera
			self.naoImage = self.prox_camera.getImageRemote(self.nameId)
			
			while self.naoImage[6]==None:
				print "-EMPTY-"
				self.naoImage = self.prox_camera.getImageRemote(self.nameId) # for avoidance of the black image (empty frame)

			if self.naoImage[6]!=None:
				self.frame_img=Image.fromstring("RGB", (self.naoImage[0], self.naoImage[1]), self.naoImage[6]) ## tuple
				self.frame_img= np.array(self.frame_img)##For NAO #conversion from tuple to numpy array
				#self.frame_img= cv2.cv.fromarray(self.frame_img[:,:])##from numpy array to CvMat
				self.image_message = self.bridge.cv2_to_imgmsg(self.frame_img,"bgr8")#, encoding="rbg") # form numpy.array to imgmsg for ROS communication
				'''mono8: CV_8UC1, grayscale image
				mono16: CV_16UC1, 16-bit grayscale image
				bgr8: CV_8UC3, color image with blue-green-red color order
				rgb8: CV_8UC3, color image with red-green-blue color order
				bgra8: CV_8UC4, BGR color image with an alpha channel
				rgba8: CV_8UC4, RGB color image with an alpha channel
				'''
				#self.cv_image = self.bridge.imgmsg_to_cv2(self.image_message,"rgb8")
				self.prox_camera.unsubscribe(self.nameId);
				return self.image_message

		except AttributeError, ex:
			print "[Camera server] - Exception AtrributeError = %s" % str(ex)
		except Exception, ex:
			print "[Camera server] - Unnamed exception = %s" % str(ex)

		return None
	
	#########################

	def handle_rapp_set_camera_parameter(self,req):
		print "[Camera server - SetCameraParam receives]: Camera ID -%d;\tParameter ID - %d;\tParameter value - %d;" % (req.cameraId, req.cameraParameterId, req.newValue)
		isSet = False
		try:
			### Subscribe to the camera
			#self.nameId = prox_camera.subscribeCamera("pythonCameraID",self.selectedCamera, self.resolution, self.colorSpace, self.fps);

			#isSet = prox_camera.setParam(req.cameraParameterId, req.newValue)
			isSet = self.prox_camera.setParameter(req.cameraId, req.cameraParameterId, req.newValue) #Modifies camera internal parameter.

			#isSet = prox_camera.setCameraParameter(const std::string& Handle, const int& Id, const int& NewValue) #Sets the value of a specific parameter for the module current active camera. 	Parameters:	Handle – Handle to identify the subscriber. Id – Id of the camera parameter (see Camera parameters). NewValue – New value to set.

			#prox_camera.setParam(17,exposure_time) # setting the exposure time
			#prox_camera.setParam(11,1)#enable auto exposition
			
			#prox_camera.unsubscribe(self.nameId);
		except AttributeError, ex:
			print "[Camera server - SetCameraParam] - Exception AtrributeError = %s" % str(ex)
		except Exception, ex:
			print "[Camera server - SetCameraParam] - Unnamed exception = %s" % str(ex)

		return isSet
###########################
	def handle_rapp_set_camera_parameters(self,req):
		#print "[Camera server - SetCameraParams receives]: Camera ID -%d;\tParameter ID - %d;\tParameter value - %d;" % (req.cameraId, req.cameraParameterIds, req.newValues)
		try:
			isSetList=[]
			for i in range(len(req.cameraParameterIds)):
				isSet = self.prox_camera.setParameter(req.cameraId, req.cameraParameterId[i], req.newValue[i]) #Modifies camera internal parameter.
				isSetList.append(isSet);

		except AttributeError, ex:
			print "[Camera server - SetCameraParams] - Exception AtrributeError = %s" % str(ex)
		except Exception, ex:
			print "[Camera server - SetCameraParams] - Unnamed exception = %s" % str(ex)

		return isSetList#isSet
	
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
		CameraServer.prox_camera.unsubscribe(self.nameId);#CameraServer.nameId);
		myBroker.shutdown()
		sys.exit(0)
		
	except (KeyboardInterrupt, SystemExit):
		print "[Camera server] - SystemExit Exception caught"
		#unsubscribe from camera
		CameraServer.prox_camera.unsubscribe(self.nameId);#CameraServer.nameId);
		myBroker.shutdown()
		sys.exit(0)
		
	except Exception, ex:
		print "[Camera server] - Exception caught %s" % str(ex)
		#unsubscribe from camera
		CameraServer.prox_camera.unsubscribe(self.nameId);#CameraServer.nameId);
		myBroker.shutdown()
		sys.exit(0)
	
if __name__ == "__main__":
	try:
		main()
	except Exception,e:
		print "__name__ - Error %s" % str(e)
