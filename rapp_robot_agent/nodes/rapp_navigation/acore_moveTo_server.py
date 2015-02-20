#!/usr/bin/env python

__author__ = "Wojciech Dudek"

########################
# Imports
########################

# Importing services
from rapp_robot_agent.srv import *
# Importing core system functionality
import signal
import sys, os
import rospy
import almath as m
import numpy
# Importing core functionality from Naoqi
from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule

#######################################

# Constants
class Constants:

	NAO_IP = "nao.local"
	PORT = 9559

	

#######################################

# MoveNaoModule class definition
class MoveToNaoModule(ALModule):
	""" A simple module able to react to
	sound detection events
	"""
	
	# Constructor of MoveToNaoModule
	def __init__(self,name):
		ALModule.__init__(self,name)
		
		print "[Move server] - Acore Move Server initialization"
		
		# Initialization of ROS node
		rospy.init_node('acore_move_to')
		self.moduleName = name
		
		# Initialization of Naoqi modules and ROS services
		self.initALModule()
		self.setVariables()
		self.openServices()
		#Subscribe to Naoqi events
		self.subscribeToEvents()
		print "[Move server] - Waits for clients ..."
		
		
	
	# Initialization of Naoqi modules
	def initALModule(self):
		print "[MoveTo server] - Initialization of Naoqi modules"
		
		self.prox_memory = ALProxy("ALMemory")
		if self.prox_memory is None:
			rospy.logerr("[MoveTo server] - Could not get a proxy to ALMemory")
			exit(1)
			
		print "[MoveTo server] - ALMotion proxy initialization"
		self.proxy_motion = ALProxy("ALMotion")
		if self.proxy_motion is None:
			rospy.logerr("[MoveTo server] - Could not get a proxy to ALMotion")
			exit(1)
		print "[MoveTo server] - ALRobotPosture proxy initialization"
		self.proxy_RobotPosture = ALProxy("ALRobotPosture")
		if self.proxy_RobotPosture is None:
			rospy.logerr("[MoveTo server] - Could not get a proxy to ALRobotPosture")
			exit(1)

	
	# Setting variables
	def setVariables(self):
		print "[MoveTo server] - Setting variables"
	
		
	# Initialization of ROS services
	def openServices(self):
		try:
			print "[MoveTo server] - setting services"
			print "[MoveTo server] - service - [rapp_moveTo]"
			self.service_mt = rospy.Service('rapp_moveTo', MoveTo, self.handle_rapp_moveTo)
		except Exception, ex:
			print "[MoveTo server] - Exception %s" % str(ex)
		
	def StiffnessOn(self, proxy):
	# We use the "Body" name to signify the collection of all joints
		pNames = "Body"
		pStiffnessLists = 1.0
		pTimeLists = 1.0
		proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)	
	

	def SetPose(self,pose):

		try:
			self.StiffnessOn(self.proxy_motion)
		except Exception, ex:
			print "[MoveTo server] - Exception %s" % str(ex)
		try:	
			self.proxy_RobotPosture.goToPosture(pose, 0.5)
		except Exception, e:
				print "[MoveTo server] - Exception %s" % str(e)	
		print "[MoveTo server] - Actual Nao pose : %s" % str(pose)

		
	def handle_rapp_moveTo(self,req):

		self.SetPose('StandInit')
		#####################
		## Collision detection
		#####################
		self.proxy_motion.setExternalCollisionProtectionEnabled('All', True)

		#####################
		## Enable arms control by move algorithm
		#####################
		self.proxy_motion.setWalkArmsEnabled(True, True)

		#####################
		## FOOT CONTACT PROTECTION
		#####################
		#~ motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION",False]])
		self.proxy_motion.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])

		#####################
		## get robot position before move
		#####################
		InitRobotPosition = m.Pose2D(self.proxy_motion.getRobotPosition(False))

		print "[MoveTo server] - Destination X = %s Destination Y = %s  Destination Theta = %s" %(req.destination_x, req.destination_y, req.destination_theta)

		print "[MoveTo server] - Nao init position = ", InitRobotPosition

		X = req.destination_x
		Y = req.destination_y
		Theta = req.destination_theta

		self.proxy_motion.post.moveTo(X, Y, Theta)

		# wait is useful because with post moveTo is not blocking function
		self.proxy_motion.waitUntilMoveIsFinished()

		self.SetPose('Stand')
		isDestinationReached = True
	
		#####################
		## get END robot position
		#####################
		EndRobotPosition = m.Pose2D(self.proxy_motion.getRobotPosition(False))
		print "[MoveTo server] - Nao end position = ", EndRobotPosition
	
		Nao_x = EndRobotPosition.x
		Nao_y = EndRobotPosition.y
		return MoveToResponse(isDestinationReached)
		
	def subscribeToEvents(self):

		self.prox_memory.subscribeToEvent("ALMotion/Safety/MoveFailed", self.moduleName, "MoveCallback")

	def MoveCallback(self, strVarName, value,message):
		""" Mandatory docstring.
			comment needed to create a bound method
		"""
		print "\n \n Event:  ALMotion/Safety/MoveFailed callback: "
		print "key %s" % str(strVarName)
		print "value %s" % str(value)
		print ""


# Testng SIGINT signal handler
def signal_handler(signal, frame):
	print "[MoveTo server] - signal SIGINT caught"
	print "[MoveTo server] - system exits"
	sys.exit(0)

def main():
	""" Main entry point
		"""
	try:
		signal.signal(signal.SIGINT, signal_handler)
		print "[MoveTo server] - Press Ctrl + C to exit system correctly"
		
		myBroker = ALBroker("myBroker", "0.0.0.0", 0, Constants.NAO_IP,Constants.PORT)
		
		global NaoMoveTo
		NaoMoveTo = MoveToNaoModule("NaoMoveTo")

		rospy.spin()
	
	except (KeyboardInterrupt, SystemExit):
		print "[MoveTo server] - SystemExit Exception caught"
		#unsubscribe()
		myBroker.shutdown()
		sys.exit(0)
		
	except Exception, ex:
		print "[MoveTo server] - Exception caught %s" % str(ex)
		#unsubscribe()
		myBroker.shutdown()
		sys.exit(0)
		
if __name__ == "__main__":
	try:
		main()
	except Exception,e:
		print "__name__ - Error %s" % str(e)