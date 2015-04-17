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
import time #for time measurement
import almath #Aldebaran's library for matrices operation


# Used for finding a text in a file
import mmap

# Needed for encoding a file
import base64

from std_msgs.msg import String
#from sensor_msgs.msg import Image as Image_ros
#from cv_bridge import CvBridge, CvBridgeError
#######################################

# Global variables to store the Transform module instance and proxy to ALMemory Module
Transform = None

# Constants
class Constants:
	NAO_IP = "nao.local"
	NAO_PORT = 9559

#######################################

class Matrix4x4Message:
	#row1 = row2 = row3 = row4 = []
	r11=[];r12=[];r13=[];r14=[]
	r21=[];r22=[];r23=[];r24=[]
	r31=[];r32=[];r33=[];r34=[]
	r41=[];r42=[];r43=[];r44=[]
	
	def get_values(self,table,n):
		self.r11=[];self.r12=[];self.r13=[];self.r14=[]
		self.r21=[];self.r22=[];self.r23=[];self.r24=[]
		self.r31=[];self.r32=[];self.r33=[];self.r34=[]
		self.r41=[];self.r42=[];self.r43=[];self.r44=[]
		for k in range(0,n):
			self.r11.append(table[k][0][0]);self.r12.append(table[k][0][1]);self.r13.append(table[k][0][2]);self.r14.append(table[k][0][3])
			self.r21.append(table[k][1][0]);self.r22.append(table[k][1][1]);self.r23.append(table[k][1][2]);self.r24.append(table[k][1][3])
			self.r31.append(table[k][2][0]);self.r32.append(table[k][2][1]);self.r33.append(table[k][2][2]);self.r34.append(table[k][2][3])
			self.r41.append(table[k][3][0]);self.r42.append(table[k][3][1]);self.r43.append(table[k][3][2]);self.r44.append(table[k][3][3])
		return

	
class Transform_class:
	transformMatrix=[]
	
	#def __init__(self, img_path, img_name):
	#	Frame_image_class.img_path = img_path
	#	Frame_image_class.img_name = img_name	
		
	def get_transform(self,chainName_,space_):
		try:
			Transform_class.transformMatrix=[]
			### Get current camera position in NAO space.
			motionProxy = ALProxy("ALMotion", Constants.NAO_IP, Constants.NAO_PORT)
			#transform = motionProxy.getTransform(chainName_, space_, True)#space_=2 - for robot coordinate system; FRAME_TORSO = 0, FRAME_WORLD = 1, FRAME_ROBOT = 2
			transform = motionProxy.getTransform(chainName_, space_, False)#space_=2 - for robot coordinate system; FRAME_TORSO = 0, FRAME_WORLD = 1, FRAME_ROBOT = 2
			transformList = almath.vectorFloat(transform)
			cameraToRobot = almath.Transform(transformList)

			transformMatrix_= np.asarray([ [cameraToRobot.r1_c1,cameraToRobot.r1_c2,cameraToRobot.r1_c3,cameraToRobot.r1_c4],
													[cameraToRobot.r2_c1,cameraToRobot.r2_c2,cameraToRobot.r2_c3,cameraToRobot.r2_c4],
													[cameraToRobot.r3_c1,cameraToRobot.r3_c2,cameraToRobot.r3_c3,cameraToRobot.r3_c4],#
													[0.0,0.0,0.0,1.0]])
			#transformMatrix_=np.linalg.inv(transformMatrix_) #robotToCamera
				
			Transform_class.transformMatrix.append(transformMatrix_)

										
		except Exception, e:
			print "Error when processing Transform %s to %d:"%(chainName_,space_)
			print str(e)
			exit(1)


# GetTransformModule class definition
class GetTransformModule(ALModule):
	""" A simple module able to react to
	sound detection events
	"""
	
	# Constructor of TransformRcognitionModule
	def __init__(self,name):
		ALModule.__init__(self,name)
		
		print "[Transform server] - Acore Transform Server initialization"
		
		# Initialization of ROS node
		rospy.init_node('acore_transform_server')
		self.moduleName = name
		
		# Initialization of Naoqi modules and ROS services
		self.initALModule()
		
		#self.setVariables()
		self.openServices()
		
		print "[Transform server] - Waits for clients ..."
		
		
	
	# Initialization of Naoqi modules
	def initALModule(self):
		print "[Transform server] - Initialization of Naoqi modules"
		
		print "[Transform server] - ALMemory proxy initialization"		
		global prox_memory
		prox_memory = ALProxy("ALMemory")
		if prox_memory is None:
			rospy.logerr("[Transform server] - Could not get a proxy to ALMemory")
			exit(1)

	
	# Initialization of ROS services
	def openServices(self):
		try:
			print "[Transform server] - setting services"
			print "[Transform server] - service - [rapp_get_transform]"
			self.service_rgt = rospy.Service('rapp_get_transform', GetTransform, self.handle_rapp_get_transform)
		except Exception, ex:
			print "[Transform server] - Exception %s" % str(ex)
		
	#######################################
	
	# Core functionality methods 
	
	#######################################
	
	
	#########################
	
	# Handling methods - methods that used handling services
	
	#########################
	
	def handle_rapp_get_transform(self,req):
		self.cornersMatrix= []
		self.GetTransformClass = Transform_class()	

		try:
			self.GetTransformClass.get_transform(req.chainName, req.space) #computes transformation matrixix
		except AttributeError, ex:
			print "[Transform server] - Exception AtrributeError = %s" % str(ex)
		except Exception, ex:
			print "[Transform server] - Unnamed exception = %s" % str(ex)

		## Copy matrix to class object for the ros communication		
		Matrix4x4Transform = Matrix4x4Message() # for transformMatrix
		Matrix4x4Transform.get_values(self.GetTransformClass.transformMatrix,1)
		
		return GetTransformResponse(Matrix4x4Transform)
			
# Testng SIGINT signal handler
def signal_handler(signal, frame):
	print "[Transform server] - signal SIGINT caught"
	print "[Transform server] - system exits"
	sys.exit(0)

def main():
	""" Main entry point
	
	"""
	# It is needed to use a broker to be able to construct NAOQI 
	# modules and subscribe to other modules. The broker must stay
	# alive until  the program exists
	try:
		signal.signal(signal.SIGINT, signal_handler)
		print "[Transform server] - Press Ctrl + C to exit system correctly"	
		myBroker = ALBroker("myBroker", "0.0.0.0", 0, Constants.NAO_IP,Constants.NAO_PORT)
		GetTransform = GetTransformModule("GetTransform")
		rospy.spin()
	
	except AttributeError:
		print "[Transform server] - GetTransform - AttributeError"
		myBroker.shutdown()
		sys.exit(0)
	
	except (KeyboardInterrupt, SystemExit):
		print "[Transform server] - SystemExit Exception caught"
		myBroker.shutdown()
		sys.exit(0)
	
	except Exception, ex:
		print "[Transform server] - Exception caught %s" % str(ex)
		myBroker.shutdown()
		sys.exit(0)
	
if __name__ == "__main__":
	try:
		main()
	except Exception,e:
		print "__name__ - Error %s" % str(e)
