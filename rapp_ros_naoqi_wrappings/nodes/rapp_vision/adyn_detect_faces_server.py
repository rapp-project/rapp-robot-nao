#!/usr/bin/env python

__author__ = "Jan Figat"

########################
# Imports
########################

# Importing services
from rapp_ros_naoqi_wrappings.srv import *

# Importing core system functionality
#import signal
import sys, os
import rospy
#import time
import smtplib

# Importing core functionality from Naoqi
from naoqi import (ALProxy, ALBroker, ALModule)

# Importing others
import numpy as np
#import cv2

#import math
#import Image
#import time #for time measurement

from std_msgs.msg import String
#from sensor_msgs.msg import Image as Image_ros
#from cv_bridge import CvBridge, CvBridgeError


#######################################

## Global variables to store the RecognizeFaces module instance and proxy to ALMemory Module
#prox_memory = None


# Constants
class Constants:
	IP = "nao.local"
	PORT = 9559

#######################################


class DetectFacesModule(ALModule):
	""" A simple module able to react to
	sound detection events
	"""
	
	# Constructor of RecognizeFacesModule
	def __init__(self,name):
		ALModule.__init__(self,name)
		
		print "[DetectFaces server] - Adyn DetectFaces Server initialization"
		
		# Initialization of ROS node
		rospy.init_node('adyn_detect_faces_server')
		self.moduleName = name
		
		# Initialization of Naoqi modules and ROS services
		self.initALModule()
		
		#self.setVariables()
		self.openServices()
		
		print "[DetectFaces server] - Waits for clients ..."
		
		
	
	# Initialization of Naoqi modules
	def initALModule(self):
		print "[DetectFaces server] - Initialization of Naoqi modules"
		
		## Create a proxy to ALFaceDetection and ALVideoDevice
		try:
			self.faceProxy = ALProxy("ALFaceDetection", Constants.IP, Constants.PORT)
			self.prox_camera = ALProxy("ALVideoDevice",Constants.IP, Constants.PORT)
		except Exception, e:
			#print "Error when creating face detection proxy:"
			rospy.logerr("[DetectFaces server] - Could not get a proxy to ALFaceDetection or to ALVideoDevice")
			print str(e)
			exit(1)
		
		# ALMemory variable where the ALFaceDetection module outputs its results.
		self.memValue = "FaceDetected"
		# Create a proxy to ALMemory
		try:
			self.memoryProxy = ALProxy("ALMemory", Constants.IP, Constants.PORT)
		except Exception, e:
			print "Error when creating memory proxy:"
			print str(e)
			exit(1)
		
		
			
		
	# Initialization of ROS services
	def openServices(self):
		try:
			print "[DetectFaces server] - setting services"
			print "[DetectFaces server] - service - [rapp_face_detect]"
			self.service_rdface = rospy.Service('rapp_face_detect',FaceDetect, self.handle_rapp_face_detection)
		except Exception, ex:
			print "[DetectFaces server] - Exception (services) %s" % str(ex)

		
	#######################################
	
	# Core functionality methods 
	
	#######################################
	
	
	#########################
	
	# Handling methods - methods that used handling services
	
	#########################
		
	def handle_rapp_face_detection(self,req):
		try:			
			period = 1
			
			##Subscribe to the ALFaceDetection proxy
			self.faceProxy.subscribe("FaceDetection", period, 0.0 ) #The module will write in ALMemory with the given period

			# Setting the parameters for the face detection
			if (self.faceProxy.isTrackingEnabled == True):
				self.faceProxy.setTrackingEnabled(False) #disables face tracking. Enabling tracking usually allows you to follow a face even if the corresponding person is not facing the camera anymore. However, it can lead to more false detections. When active, only one face at a time will be detected.
			if (self.faceProxy.isRecognitionEnabled == True):
				self.faceProxy.setRecognitionEnabled(False) #disables the face recognition process. The remaining face detection process will be faster if face recognition is disabled. Face recognition is enabled by default.
			# Setting the camera parameters
			## Subscribe to the camera
			#self.nameId = prox_camera.subscribeCamera("cameraModule",self.selectedCamera, self.resolution, self.colorSpace, self.fps);
			print "setting camera parameters"
			
			self.prox_camera.setActiveCamera("FaceDetection",req.cameraID) #selecting top camera
			self.prox_camera.setResolution("FaceDetection", req.resolution) #AL::kVGA	2	Image of 640*480px; AL::k4VGA	3	Image of 1280*960px
			self.prox_camera.setFrameRate("FaceDetection", 30) #Fps from 1 to 30
			self.prox_camera.setColorSpace("FaceDetection", 11) #AL::kRGBColorSpace	11; AL::kBGRColorSpace	13
			
			if (resolution==3):# or resolution == k4VGA):
				self.width = 1280
				self.height = 960
			elif (resolution==2):# or resolution == kVGA):
				self.width = 1280/2
				self.height = 960/2
			elif (resolution==1):# or resolution == kQVGA):
				self.width = 1280/4
				self.height = 960/4
			elif (resolution==0):# or resolution == kQQVGA):
				self.width = 1280/8
				self.height = 960/8
			elif (resolution==7):
				self.width = 1280/16
				self.height = 960/16
			elif (resolution==8):
				self.width = 1280/32
				self.height = 960/32

			## Reads the memValue and checks whether faces are detected
			val = self.memoryProxy.getData(self.memValue, 0)
			
			if(val and isinstance(val, list) and len(val) >= 2):
				# We detected faces !

				# Browse the faceInfoArray to get info on each detected face.
				#for faceInfo in faceInfoArray:
				# Browse the faceInfoArray to get info on each detected face.
				for j in range( len(val[1])-1 ): #faceInfoArray = val[1]
					#Face Detected [1]/ First face [0]/ Shape Info [0]/ Alpha [1]
					alpha = val[1][j][0][1];
					beta = val[1][j][0][2];
					sx = val[1][j][0][3];
					sy = val[1][j][0][4];
					
					# sizeX/sizeY are the face size in relation to the image
					sizeX = self.height * sx;
					sizeY = self.width * sy;
					
					# Center of face into the image
					x = self.width/2 - self.width * alpha;
					y = self.height/2 + self.height * beta;
								
					if (j==0): # cleans all lists
						self.centerOfFaceX_list=[]
						self.centerOfFaceY_list=[]
						self.FaceSizeX_list=[]
						self.FaceSizeY_list=[]
						
					# Add an item (information about the face) to the end of the list
					self.centerOfFaceX_list.append(x)
					self.centerOfFaceY_list.append(y)
					self.FaceSizeX_list.append(sizeX)
					self.FaceSizeY_list.append(sizeY)
					
					'''
					try:
						## Select faces on the image
						if (resolution==3):
							self.img = cv2.imread('/home/nao/recordings/cameras/FaceDetection_Top_1280x960_'+str(i)+'.png')
						elif (resolution==2):
							self.img = cv2.imread('/home/nao/recordings/cameras/FaceDetection_Top_640x480_'+str(i)+'.png')
						#cv2.circle(self.img, (int(x),int(y)), int(sizeY/2), (0,0,255), 2)
						
						cv2.rectangle(self.img, (int(x-(sizeX/2)), int(y-(sizeY/2))), (int(x+(sizeX/2)), int(y+(sizeY/2))), (0,255,0), 2);
						#cv2.cross(img, y, x, 10, vpColor::red);
						#displayCross(img, y, x, 10, vpColor::red);
						#displayRectangle(img,y,x,0.0,sizeX,sizeY,vpColor::cyan,1);
						if (resolution==3):
							cv2.imwrite('/home/nao/recordings/cameras/FaceDetection_Top_1280x960_'+str(i)+'Face_'+str(j+1)+'.png',self.img)
						elif (resolution==2):
							cv2.imwrite('/home/nao/recordings/cameras/FaceDetection_Top_640x480_'+str(i)+'Face_'+str(j+1)+'.png',self.img)
					except Exception, e:
						print "Error when opening image"
						print str(e)
						#exit(1)
					'''
				'''
				for j in range( len(val[1])-1 ): #faceInfoArray = val[1]
					print "\t\tFace number %d"%(j+1)
					print "Centre of face: x= %.3f; y= %.3f"%(self.centerOfFaceX_list[j],self.centerOfFaceY_list[j])
					print "Face size in relation to the image: size_x= %.3f; size_y= %.3f"%(self.FaceSizeX_list[j],self.FaceSizeY_list[j])
				'''
				
			## Unsubscribe the module.
			self.faceProxy.unsubscribe("FaceDetection")
			
			print "[DetectFaces server] - Done \nNumber of detected faces: %s"%self.nFacesDetected
		except AttributeError, ex:
			print "[DetectFaces server] - Exception AtrributeError = %s" % str(ex)
		except Exception, ex:
			print "[DetectFaces server] - Unnamed exception = %s" % str(ex)
		return FaceDetectResponse(self.centerOfFaceX_list,self.centerOfFaceY_list,self.FaceSizeX_list,self.FaceSizeY_list)
	
	
# Testng SIGINT signal handler
def signal_handler(signal, frame):
	print "[DetectFaces server] - signal SIGINT caught"
	print "[DetectFaces server] - system exits"
	sys.exit(0)

def main():
	""" Main entry point
	
	"""
	# It is needed to use a broker to be able to construct NAOQI 
	# modules and subscribe to other modules. The broker must stay
	# alive until  the program exists
	try:
		signal.signal(signal.SIGINT, signal_handler)
		print "[DetectFaces server] - Press Ctrl + C to exit system correctly"
		
		myBroker = ALBroker("myBroker", "0.0.0.0", 0, Constants.NAO_IP,Constants.PORT)
		DetectFacesServer = DetectFacesModule("DetectFaces")
		rospy.spin()
	
	except AttributeError:
		print "[DetectFaces server] - AttributeError"
		myBroker.shutdown()
		sys.exit(0)
		
	except (KeyboardInterrupt, SystemExit):
		print "[DetectFaces server] - SystemExit Exception caught"
		myBroker.shutdown()
		sys.exit(0)
		
	except Exception, ex:
		print "[DetectFaces server] - Exception caught %s" % str(ex)
		myBroker.shutdown()
		sys.exit(0)
	
if __name__ == "__main__":
	try:
		main()
	except Exception,e:
		print "__name__ - Error %s" % str(e)