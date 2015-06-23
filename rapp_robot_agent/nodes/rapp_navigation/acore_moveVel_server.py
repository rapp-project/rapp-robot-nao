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

# MoveVelNaoModule class definition
class MoveVelNaoModule(ALModule):
	""" A simple module able to react to
	sound detection events
	"""
	
	# Constructor of MoveVelNaoModule
	def __init__(self,name):
		ALModule.__init__(self,name)
		
		print "[MoveVel server] - Acore MoveVel Server initialization"
		
		# Initialization of ROS node
		rospy.init_node('acore_move_velocity')
		self.moduleName = name
		
		# Initialization of Naoqi modules and ROS services
		self.initALModule()
		self.setVariables()
		self.openServices()
		self.subscribeToEvents()
		print "[MoveVel server] - Waits for clients ..."
			
	
	# Initialization of Naoqi modules
	def initALModule(self):
		print "[MoveVel server] - Initialization of Naoqi modules"
		
		self.prox_memory = ALProxy("ALMemory")
		if self.prox_memory is None:
			rospy.logerr("[MoveVel server] - Could not get a proxy to ALMemory")
			exit(1)
			
		print "[MoveVel server] - ALMotion proxy initialization"
		self.proxy_motion = ALProxy("ALMotion")
		if self.proxy_motion is None:
			rospy.logerr("[MoveVel server] - Could not get a proxy to ALMotion")
			exit(1)
		print "[MoveVel server] - ALRobotPosture proxy initialization"
		self.proxy_RobotPosture = ALProxy("ALRobotPosture")
		if self.proxy_RobotPosture is None:
			rospy.logerr("[MoveVel server] - Could not get a proxy to ALRobotPosture")
			exit(1)

	
	# Setting variables
	def setVariables(self):
		print "[MoveVel server] - Setting variables"
	
		
	# Initialization of ROS services
	def openServices(self):
		try:
			print "[MoveVel server] - setting services"
			print "[MoveVel server] - service - [rapp_rapp_moveVel]"
			self.service_mv = rospy.Service('rapp_moveVel', MoveVel, self.handle_rapp_moveVel)
		except Exception, ex:
			print "[MoveVel server] - Exception %s" % str(ex)
		
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
			print "[MoveVel server] - Exception %s" % str(ex)
		try:	
			self.proxy_RobotPosture.goToPosture(pose, 0.5)
		except Exception, e:
				print "[MoveVel server] - Exception %s" % str(e)	
		print "[MoveVel server] - Actual pose - %s" % str(pose)
			
	def handle_rapp_moveVel(self,req):

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

		print "[MoveVel server] - Velocity X = %s Velocity Y = %s  Velocity Theta = %s" %(req.velocity_x, req.velocity_y, req.velocity_theta)

		print "[MoveVel server] - Nao init position = ", InitRobotPosition

		X = req.velocity_x
		Y = req.velocity_y
		Theta = req.velocity_theta

		self.proxy_motion.post.moveToward(X, Y, Theta)

		self.SetPose('Stand')
		isDestinationReached = True
	
		#####################
		## get END robot position
		#####################
		EndRobotPosition = m.Pose2D(self.proxy_motion.getRobotPosition(False))
		print "[MoveVel server] - Nao end position = ", EndRobotPosition
		
		Nao_x = EndRobotPosition.x
		Nao_y = EndRobotPosition.y
		return MoveVelResponse(isDestinationReached)
		
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
	print "[MoveVel server] - signal SIGINT caught"
	print "[MoveVel server] - system exits"
	sys.exit(0)

def main():
	""" Main entry point
		"""
	try:
		signal.signal(signal.SIGINT, signal_handler)
		print "[MoveVel server] - Press Ctrl + C to exit system correctly"
		
		myBroker = ALBroker("myBroker", "0.0.0.0", 0, Constants.NAO_IP,Constants.PORT)
		global NaoMoveVel
		NaoMoveVel= MoveVelNaoModule("NaoMoveVel")

		rospy.spin()
	
	except (KeyboardInterrupt, SystemExit):
		print "[MoveVel server] - SystemExit Exception caught"
		#unsubscribe()
		myBroker.shutdown()
		sys.exit(0)
		
	except Exception, ex:
		print "[MoveVel server] - Exception caught %s" % str(ex)
		#unsubscribe()
		myBroker.shutdown()
		sys.exit(0)
		
if __name__ == "__main__":
	try:
		main()
	except Exception,e:
		print "__name__ - Error %s" % str(e)