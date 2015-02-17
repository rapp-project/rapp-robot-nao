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

# Global variables to store the LearnFace module instance and proxy to ALMemory Module
face_memory = "FaceDetected"
face_name = None
prox_memory = None


# Constants
class Constants:
	NAO_IP = "nao.local"
	PORT = 9559
	period = 500 # the module will write in ALMemory with the given period

#######################################


class LearnFaceModule(ALModule):
	""" A simple module able to react to
	sound detection events
	"""
	
	# Constructor of LearnFaceModule
	def __init__(self,name):
		ALModule.__init__(self,name)
		
		print "[LearnFace server] - Acore LearnFace Server initialization"
		
		# Initialization of ROS node
		rospy.init_node('acore_learn_face_server')
		self.moduleName = name
		
		#self.bridge = CvBridge()
		
		# Initialization of Naoqi modules and ROS services
		self.initALModule()
		
		#self.setVariables()
		self.openServices()
		
		print "[LearnFace server] - Waits for clients ..."
		
		
	
	# Initialization of Naoqi modules
	def initALModule(self):
		print "[LearnFace server] - Initialization of Naoqi modules"
		
		print "[LearnFace server] - ALMemory proxy initialization"		
		global prox_memory
		prox_memory = ALProxy("ALMemory")
		if prox_memory is None:
			rospy.logerr("[LearnFace server] - Could not get a proxy to ALMemory")
			exit(1)
		print "[LearnFace server] - ALFaceDetection proxy initialization"
		self.faceProxy = ALProxy("ALFaceDetection")#, Constants.NAO_IP, Constants.PORT)
		if self.faceProxy is None:
			rospy.logerr("[LearnFace server] - Could not get a proxy to ALFaceDetection")
			exit(1)
			
		
	# Initialization of ROS services
	def openServices(self):
		try:
			print "[LearnFace server] - setting services"
			print "[LearnFace server] - service - [rapp_learn_face]"
			self.service_rlface = rospy.Service('rapp_learn_face',LearnFace, self.handle_rapp_learn_face)
		except Exception, ex:
			print "[LearnFace server] - Exception (services) %s" % str(ex)

		
	#######################################
	
	# Core functionality methods 
	
	#######################################
	
	
	#########################
	
	# Handling methods - methods that used handling services
	
	#########################
		
	def handle_rapp_learn_face(self,req):
		learned = False
		print "[LearnFace server receives]: \t%s" % (req.name)
		# Learn the req.name's face 
		
		try:
			face_name = req.name # for unsubscribe
			
			
			# Subscribe to the faceProxy (ALFaceDetection)
			self.faceProxy.subscribe(req.name, Constants.period, 0.0 )
			print "[LearnFace server]: The list containing the name of each learned face: %s" %self.faceProxy.getLearnedFacesList() # Gets a list containing the name of each learned face
			# A simple loop that reads the memValue and checks whether faces are detected.
			for i in range(0, 30):
				time.sleep(0.5) #
				self.val = prox_memory.getData(face_memory)

				# Check whether we got a valid output.
				if(self.val and isinstance(self.val, list) and len(self.val) >= 2):
					# We detected faces !
					# For each face, we can read its shape info and ID.
					
					# First Field = TimeStamp.
					timeStamp = self.val[0]
					# Second Field = array of face_Info's.
					faceInfoArray = self.val[1]
					
					try:
						# Browse the faceInfoArray to get info on each detected face.
						for j in range( len(faceInfoArray)-1 ):
							#faceInfo = faceInfoArray[j]
							## First Field = Shape info.
							#faceShapeInfo = faceInfo[0]
							## Second Field = Extra info (empty for now).
							#faceExtraInfo = faceInfo[1]
							#print "  alpha %.3f - beta %.3f" % (faceShapeInfo[1], faceShapeInfo[2])
							#print "  width %.3f - height %.3f" % (faceShapeInfo[3], faceShapeInfo[4])
						
							#### face learning
							#self.faceProxy.forgetPerson(face_name) # Deletes from the database all learned faces corresponding to the specified person
							isLearned=self.faceProxy.learnFace(face_name) # Learns a new face and add it in the database under the specified name.
							if isLearned == False:
								isLearned=self.faceProxy.reLearnFace(face_name)
							
							print i+1, isLearned
							if isLearned == True:
								learned = True
					except Exception, e:
						print "faces detected, but it seems getData is invalid. ALValue ="
						print val
						print "Error msg %s" % (str(e))
						learned = False
				else:
					print "No face detected"
					learned = False

			print "[LearnFace server] - Done \nFace is learned %s"%learned
		except AttributeError, ex:
			print "[LearnFace server] - Exception AtrributeError = %s" % str(ex)
		except Exception, ex:
			print "[LearnFace server] - Unnamed exception = %s" % str(ex)
		return learned
	
	
# Testng SIGINT signal handler
def signal_handler(signal, frame):
	print "[LearnFace server] - signal SIGINT caught"
	print "[LearnFace server] - system exits"
	sys.exit(0)

def main():
	""" Main entry point
	
	"""
	# It is needed to use a broker to be able to construct NAOQI 
	# modules and subscribe to other modules. The broker must stay
	# alive until  the program exists
	try:
		signal.signal(signal.SIGINT, signal_handler)
		print "[LearnFace server] - Press Ctrl + C to exit system correctly"
		
		myBroker = ALBroker("myBroker", "0.0.0.0", 0, Constants.NAO_IP,Constants.PORT)
		LearnFaceServer = LearnFaceModule("LearnFace")
		rospy.spin()
	
	except AttributeError:
		print "[LearnFace server] - AttributeError"
		#unsubscribe the module LearnFace
		if face_name!=None:
			LearnFaceServer.faceProxy.unsubscribe(face_name)
		myBroker.shutdown()
		sys.exit(0)
		
	except (KeyboardInterrupt, SystemExit):
		print "[LearnFace server] - SystemExit Exception caught"
		#unsubscribe the module LearnFace
		if face_name!=None:
			LearnFaceServer.faceProxy.unsubscribe(face_name)
		myBroker.shutdown()
		sys.exit(0)
		
	except Exception, ex:
		print "[LearnFace server] - Exception caught %s" % str(ex)
		#unsubscribe the module LearnFace
		if face_name!=None:
			LearnFaceServer.faceProxy.unsubscribe(face_name)
		myBroker.shutdown()
		sys.exit(0)
	
if __name__ == "__main__":
	try:
		main()
	except Exception,e:
		print "__name__ - Error %s" % str(e)
