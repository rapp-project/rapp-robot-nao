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
from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule

# Used for finding a text in a file
import mmap

# Needed for encoding a file
import base64

#######################################

# Global variables to store
prox_memory = None

# Constants
class Constants:
	NAO_IP = "nao.local"
	PORT = 9559


	
	

#######################################

# TextToSpeechModule class definition
class TextToSpeechModule(ALModule):
	""" A simple module able to react to
	sound detection events
	"""
	
	# Constructor of TextToSpeechModule
	def __init__(self,name):
		ALModule.__init__(self,name)
		
		print "[TextToSpeech server] - Acore TextToSpeech Server initialization"
		
		# Initialization of ROS node
		rospy.init_node('acore_textToSpeech')
		self.moduleName = name
		
		# Initialization of Naoqi modules and ROS services
		self.initALModule()
		
		self.openServices()
		
		print "[TextToSpeech server] - Waits for clients ..."
		
		
	
	# Initialization of Naoqi modules
	def initALModule(self):
		print "[TextToSpeech server] - Initialization of Naoqi modules"
		
		print "[TextToSpeech server] - ALMemory proxy initialization"		
		global prox_memory
		prox_memory = ALProxy("ALMemory")
		if prox_memory is None:
			rospy.logerr("[TextToSpeech server] - Could not get a proxy to ALMemory")
			exit(1)
			
		print "[TextToSpeech server] - ALTextToSpeech proxy initialization"
		self.prox_tts = ALProxy("ALTextToSpeech")
		if self.prox_tts is None:
			rospy.logerr("[TextToSpeech server] - Could not get a proxy to ALTextToSpeech")
			exit(1)
	
	
	
	# Initialization of ROS services
	def openServices(self):
		try:
			print "[TextToSpeech server] - setting services"
			print "[TextToSpeech server] - service - [rapp_say]"
			self.service_rs = rospy.Service('rapp_say', Say, self.handle_rapp_say)
		except Exception, ex:
			print "[TextToSpeech server] - Exception %s" % str(ex)
		
		
	#######################################
	
	# Core functionality methods 
	
	#######################################
	
	
	#########################
	
	# Handling methods - methods that used handling services
	
	#########################
		
	def handle_rapp_say(self,req):
		print "[TextToSpeech server receives]: \t%s\n" % (req.request)
		#self.prox_tts.say("Nao says : %s"% req.request)
		self.prox_tts.say(req.request)
		return SayResponse(req.request)
		
	
# Testng SIGINT signal handler
def signal_handler(signal, frame):
	print "[TextToSpeech server] - signal SIGINT caught"
	print "[TextToSpeech server] - system exits"
	sys.exit(0)

def main():
	""" Main entry point
	
	"""
	# It is needed to use a broker to be able to construct NAOQI 
	# modules and subscribe to other modules. The broker must stay
	# alive until  the program exists
	try:
		signal.signal(signal.SIGINT, signal_handler)
		print "[TextToSpeech server] - Press Ctrl + C to exit system correctly"
		
		myBroker = ALBroker("myBroker", "0.0.0.0", 0, Constants.NAO_IP,Constants.PORT)
		global TextToSpeech
		TextToSpeech = TextToSpeechModule("TextToSpeech")
		rospy.spin()
	
	except AttributeError:
		print "[TextToSpeech server] - TextToSpeech - AttributeError"
		myBroker.shutdown()
		sys.exit(0)
		
	except (KeyboardInterrupt, SystemExit):
		print "[TextToSpeech server] - SystemExit Exception caught"
		myBroker.shutdown()
		sys.exit(0)
		
	except Exception, ex:
		print "[TextToSpeech server] - Exception caught %s" % str(ex)
		myBroker.shutdown()
		sys.exit(0)
		
if __name__ == "__main__":
	try:
		main()
	except Exception,e:
		print "__name__ - Error %s" % str(e)
