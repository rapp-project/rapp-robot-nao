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

from std_msgs.msg import String
#from sensor_msgs.msg import Image as Image_ros
#from cv_bridge import CvBridge, CvBridgeError


#######################################

## Global variables to store the RecognizeFaces module instance and proxy to ALMemory Module
#face_memory = "FaceDetected"
object_memory = "PictureDetected"
face_name = None
prox_memory = None


# Constants
class Constants:
	NAO_IP = "nao.local"
	PORT = 9559

#######################################


class RecognizeFacesModule(ALModule):
	""" A simple module able to react to
	sound detection events
	"""
	
	# Constructor of RecognizeFacesModule
	def __init__(self,name):
		ALModule.__init__(self,name)
		
		print "[RecognizeFaces server] - Acore RecognizeFaces Server initialization"
		
		# Initialization of ROS node
		rospy.init_node('acore_recognize_faces_server')
		self.moduleName = name
		
		# Initialization of Naoqi modules and ROS services
		self.initALModule()
		
		#self.setVariables()
		self.openServices()
		
		print "[RecognizeFaces server] - Waits for clients ..."
		
		
	
	# Initialization of Naoqi modules
	def initALModule(self):
		print "[RecognizeFaces server] - Initialization of Naoqi modules"
		
		print "[RecognizeFaces server] - ALMemory proxy initialization"		
		#global prox_memory
		self.prox_memory = ALProxy("ALMemory")
		if self.prox_memory is None:
			rospy.logerr("[RecognizeFaces server] - Could not get a proxy to ALMemory")
			exit(1)
		print "[RecognizeFaces server] - ALFaceDetection proxy initialization"
		self.faceProxy = ALProxy("ALFaceDetection")#, Constants.NAO_IP, Constants.PORT)
		if self.faceProxy is None:
			rospy.logerr("[RecognizeFaces server] - Could not get a proxy to ALFaceDetection")
			exit(1)
		
		
			
		
	# Initialization of ROS services
	def openServices(self):
		try:
			print "[RecognizeFaces server] - setting services"
			print "[RecognizeFaces server] - service - [rapp_recognize_faces]"
			self.service_rlface = rospy.Service('rapp_recognize_faces',RecognizeFaces, self.handle_rapp_face_recognition)
		except Exception, ex:
			print "[RecognizeFaces server] - Exception (services) %s" % str(ex)

		
	#######################################
	
	# Core functionality methods 
	
	#######################################
	
	
	#########################
	
	# Handling methods - methods that used handling services
	
	#########################
		
	def handle_rapp_face_recognition(self,req):
		print "Faces in database:\t%s"%self.faceProxy.getLearnedFacesList()
		print "[RecognizeFaces server receives]: \t%s" % (req.request)
		
		try:			
			self.faceProxy.enableRecognition(req.isRecognitionEnabled)#Enables/disables the face recognition process. The remaining face detection process will be faster if face recognition is disabled.
			self.faceProxy.enableTracking(req.isTrackingEnabled)#Enables/disables face tracking. Enabling tracking usually allows you to follow a face even if the corresponding person is not facing the camera anymore. However, it can lead to more false detections. When active, only one face at a time will be detected.
			self.names = None
			self.isRecognized = False
			self.names = []

			period = 500
			##Subscribe to the ALFaceDetection proxy
			self.faceProxy.subscribe("FaceRecognition", period, 0.0 ) #The module will write in ALMemory with the given period

			for i in range(0, 5):
				#time.sleep(0.05)# in s ## eventual
				self.val = self.prox_memory.getData("FaceDetected",0) # For face detection and recognition
				#print self.prox_memory.getData(object_memory)
				'''#print self.val
				## First Field = TimeStamp.
				#timeStamp = val[0]
				## Second Field = array of face_Info's.
				#faceInfoArray = val[1]'''

				self.names = []
				self.nFacesDetected = 0#-1
				
				if(self.val and isinstance(self.val, list) and len(self.val) > 0):
					'''if(self.nFacesDetected != len(self.val[1]) -1): # an additional array has been placed at the end for time
						self.nFacesDetected = len(self.val[1]) -1  # filtered info and has to be substracted when counting faces
						if(self.nFacesDetected != 0):
							print self.nFacesDetected'''
					if (len(self.val[1]) > 0): # just in case of the ALValue is in the wrong format
						#if self.val[1] != []:
						#print "Human face detected"
						self.timeFilteredResult = self.val[1][len(self.val[1]) -1]
						self.nFacesDetected = len(self.val[1]) -1  # counting faces
						if(self.nFacesDetected != 0):
							print "The number of detected human faces: %d"%self.nFacesDetected
						if( len(self.timeFilteredResult) == 1 ):
							# If a face has been detected for more than 8s but not recognized
							if(self.timeFilteredResult[0] == 4):
								print "The number of detected human faces: %d"%self.nFacesDetected
						elif( len(self.timeFilteredResult) == 2 ):
							# If one or several faces have been recognized
							if(self.timeFilteredResult[0] in [2, 3]):
								for s in self.timeFilteredResult[1]:
									print "Human face recognized: %s"%s
									self.isRecognized = True
									self.names.append(s)

				if self.isRecognized==True:
					break

				'''## For object recognition:
				self.val_object = prox_memory.getData(object_memory)
				self.nPicturesDetected = -1
				if(len(self.val_object) > 1):
					self.nPicturesDetected = len(self.val_object[1])
					labels = []
					for s in self.val_object[1]:
						labels.append(s[0])
					print "Recognized objects:\t%s"%labels
				else:
					if(self.nPicturesDetected != 0):
						self.nPicturesDetected = 0
					print "Nothing was recognized"
				'''

				'''if(len(p) > 0):
            if(len(p[1]) > 0): # just in case of the ALValue is in the wrong format
                # get the ALValue returned by the time filtered recognition:
                #    - [] when nothing new.
                #    - [4] when a face has been detected but not recognized during the first 8s.
                #    - [2, [faceName]] when one face has been recognized.
                #    - [3, [faceName1, faceName2, ...]] when several faces have been recognized.
				'''

				#print "---"
			
			## Unsubscribe the module.
			self.faceProxy.unsubscribe("FaceRecognition")
			
			print "[RecognizeFaces server] - Done \nNumber of detected faces: %s"%self.nFacesDetected
		except AttributeError, ex:
			print "[RecognizeFaces server] - Exception AtrributeError = %s" % str(ex)
		except Exception, ex:
			print "[RecognizeFaces server] - Unnamed exception = %s" % str(ex)
		return RecognizeFacesResponse(self.names,self.isRecognized)
	
	
# Testng SIGINT signal handler
def signal_handler(signal, frame):
	print "[RecognizeFaces server] - signal SIGINT caught"
	print "[RecognizeFaces server] - system exits"
	sys.exit(0)

def main():
	""" Main entry point
	
	"""
	# It is needed to use a broker to be able to construct NAOQI 
	# modules and subscribe to other modules. The broker must stay
	# alive until  the program exists
	try:
		signal.signal(signal.SIGINT, signal_handler)
		print "[RecognizeFaces server] - Press Ctrl + C to exit system correctly"
		
		myBroker = ALBroker("myBroker", "0.0.0.0", 0, Constants.NAO_IP,Constants.PORT)
		RecognizeFacesServer = RecognizeFacesModule("RecognizeFaces")
		rospy.spin()
	
	except AttributeError:
		print "[RecognizeFaces server] - AttributeError"
		myBroker.shutdown()
		sys.exit(0)
		
	except (KeyboardInterrupt, SystemExit):
		print "[RecognizeFaces server] - SystemExit Exception caught"
		
		myBroker.shutdown()
		sys.exit(0)
		
	except Exception, ex:
		print "[RecognizeFaces server] - Exception caught %s" % str(ex)
		myBroker.shutdown()
		sys.exit(0)
	
if __name__ == "__main__":
	try:
		main()
	except Exception,e:
		print "__name__ - Error %s" % str(e)
