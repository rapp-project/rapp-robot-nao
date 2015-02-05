#!/usr/bin/env python

__author__ = "Wojciech Dudek"

########################
# Imports
########################

# Importing services
#from rapp_core_agent.srv import *
from rapp_robot_agent.srv import *
# Importing core system functionality
import signal
import sys, os
import rospy
import time
import almath as m 
import numpy
# Importing core functionality from Naoqi
from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule
# from naoqi import ALMotion


#######################################

# Global variables to store the EmailRecognition module instance and proxy to ALMemory Module
prox_memory = None

# Constants
class Constants:

	NAO_IP = "nao.local"
	PORT = 9559

	

#######################################

# EmailRecognitionModule class definition
class MoveNaoModule(ALModule):
	""" A simple module able to react to
	sound detection events
	"""
	
	# Constructor of EmailRcognitionModule
	def __init__(self,name):
		ALModule.__init__(self,name)
		
		print "[Move server] - Acore Move Server initialization"
		
		# Initialization of ROS node
		rospy.init_node('acore_move_server')
		self.moduleName = name
		
		# Initialization of Naoqi modules and ROS services
		self.initALModule()
		
		self.setVariables()
		self.openServices()
		
		print "[Move server] - Waits for clients ..."
		
		
	
	# Initialization of Naoqi modules
	def initALModule(self):
		print "[Move server] - Initialization of Naoqi modules"
		
		print "[Move server] - ALMemory proxy initialization"		
		global prox_memory
		prox_memory = ALProxy("ALMemory")
		if prox_memory is None:
			rospy.logerr("[Move server] - Could not get a proxy to ALMemory")
			exit(1)
			
		print "[Move server] - ALMotion proxy initialization"
		self.proxy_motion = ALProxy("ALMotion")
		if self.proxy_motion is None:
			rospy.logerr("[Move server] - Could not get a proxy to ALMotion")
			exit(1)
		print "[Move server] - ALRobotPosture proxy initialization"
		self.proxy_RobotPosture = ALProxy("ALRobotPosture")
		if self.proxy_RobotPosture is None:
			rospy.logerr("[Move server] - Could not get a proxy to ALRobotPosture")
			exit(1)

	
	# Setting variables
	def setVariables(self):
		print "[Move server] - Setting variables"
		self.poses = ["Stand","StandInit","StandZero","Crouch","Sit","SitRelax","LyingBelly","LyingBack",]
	
		
	# Initialization of ROS services
	def openServices(self):
		try:
			print "[Move server] - setting services"
			print "[Move server] - service - [rapp_follow_marker]"
			self.service_fm = rospy.Service('rapp_follow_marker', FollowMarker, self.handle_rapp_follow_marker)
		except Exception, ex:
			print "[Move server] - Exception %s" % str(ex)
		
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
			print "[Move server] - Exception %s" % str(ex)
		try:	
			self.proxy_RobotPosture.goToPosture(pose, 0.5)
		except Exception, e:
				print "[Move server] - Exception %s" % str(e)	
		print "[Move server] - Actual pose - %s" % str(pose)
	
	def MoveHead(self,proxy_motion, head_yaw,head_pitch):

		head_yaw_radians = head_yaw*m.TO_RAD
		fractionMaxSpeed = 0.1
		proxy_motion.setAngles('HeadYaw',head_yaw_radians,fractionMaxSpeed)

		head_pitch_radians = head_pitch*m.TO_RAD
		proxy_motion.setAngles('HeadPitch',head_pitch_radians,fractionMaxSpeed)

		
	def handle_rapp_follow_marker(self,req):

		self.SetPose('StandInit')

		#####################
		## Enable arms control by move algorithm
		#####################
		self.proxy_motion.setWalkArmsEnabled(True, True)
		#~ motionProxy.setWalkArmsEnabled(False, False)

		#####################
		## FOOT CONTACT PROTECTION
		#####################
		#~ motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION",False]])
		self.proxy_motion.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])

		#####################
		## get robot position before move
		#####################
		InitRobotPosition = m.Pose2D(self.proxy_motion.getRobotPosition(False))

		print "[Move server] - Marker X = %s Marker Y = %s" %(req.marker_x, req.marker_y)

		print "[Move server] - Nao init position = ", InitRobotPosition

		X = req.marker_x
		Y = req.marker_y
		Theta = 0

		self.proxy_motion.post.moveTo(X, Y, Theta)

		# wait is useful because with post moveTo is not blocking function
		self.proxy_motion.waitUntilMoveIsFinished()

		self.SetPose('Stand')
		isMarkerReached = True
	
		#####################
		## get END robot position
		#####################
		EndRobotPosition = m.Pose2D(self.proxy_motion.getRobotPosition(False))
		print "[Move server] - Nao end position = ", EndRobotPosition
		if isMarkerReached:
			self.MoveHead(self.proxy_motion,req.head_yaw,req.head_pitch)
		Nao_x = EndRobotPosition.x
		Nao_y = EndRobotPosition.y
		return FollowMarkerResponse(isMarkerReached,Nao_x,Nao_y)
		

# Testng SIGINT signal handler
def signal_handler(signal, frame):
	print "[Move server] - signal SIGINT caught"
	print "[Move server] - system exits"
	sys.exit(0)

def main():
	""" Main entry point
	
	"""
	# It is needed to use a broker to be able to construct NAOQI 
	# modules and subscribe to other modules. The broker must stay
	# alive until  the program exists
	try:
		signal.signal(signal.SIGINT, signal_handler)
		print "[Move server] - Press Ctrl + C to exit system correctly"
		
		myBroker = ALBroker("myBroker", "0.0.0.0", 0, Constants.NAO_IP,Constants.PORT)
		global FollowMarker
		FollowMarker = MoveNaoModule("FollowMarker")
		rospy.spin()
	
	except (KeyboardInterrupt, SystemExit):
		print "[Move server] - SystemExit Exception caught"
		#EmailRecognition.unsubscribe()
		myBroker.shutdown()
		sys.exit(0)
		
	except Exception, ex:
		print "[Move server] - Exception caught %s" % str(ex)
		#EmailRecognition.unsubscribe()
		myBroker.shutdown()
		sys.exit(0)
		
if __name__ == "__main__":
	try:
		main()
	except Exception,e:
		print "__name__ - Error %s" % str(e)
