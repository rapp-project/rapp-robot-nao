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
class MoveNaoModule(ALModule):
	""" A simple module able to react to
	sound detection events
	"""
	
	# Constructor of MoveNaoModule
	def __init__(self,name):
		ALModule.__init__(self,name)
		
		print "[Move server] - Acore Move Server initialization"
		
		# Initialization of ROS node
		rospy.init_node('acore_move')
		self.moduleName = name
		
		# Initialization of Naoqi modules and ROS services
		self.initALModule()
		self.setVariables()
		self.openServices()
		self.avoideObstacle()
		#Subscribe to Naoqi events
		#self.subscribeToEvents()

		print "[Move server] - Waits for clients ..."
		
		
	
	# Initialization of Naoqi modules
	def initALModule(self):
		print "[Move server] - Initialization of Naoqi modules"
		
		self.prox_memory = ALProxy("ALMemory")
		if self.prox_memory is None:
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
		self.proxy_sonar = ALProxy("ALSonar")
		if self.prox_memory is None:
			rospy.logerr("[Move server] - Could not get a proxy to ALSonar")
			exit(1)
	
	# Setting variables
	def setVariables(self):
		print "[Move server] - Setting variables"
	
		
	# Initialization of ROS services
	def openServices(self):
		try:
			print "[Move server] - setting services"
			print "[Move server] - service - [rapp_MoveTo]"
			self.service_mt = rospy.Service('rapp_moveTo', MoveTo, self.handle_rapp_MoveTo)
		except Exception, ex_mt:
			print "[Move server] - Exception %s" % str(ex)
		try:
			print "[Move server] - service - [rapp_moveVel]"
			self.service_mv = rospy.Service('rapp_moveVel', MoveVel, self.handle_rapp_moveVel)
		except Exception, ex_mv:
			print "[Move server] - Exception %s" % str(ex)
		try:
			print "[Move server] - service - [rapp_moveHead]"
			self.service_mh = rospy.Service('rapp_moveHead', MoveHead, self.handle_rapp_moveHead)
		except Exception, ex_mh:
			print "[Move server] - Exception %s" % str(ex)
		try:
			print "[Move server] - service - [rapp_moveStop]"
			self.service_mh = rospy.Service('rapp_moveStop', MoveStop, self.handle_rapp_moveStop)
		except Exception, ex_mh:
			print "[Move server] - Exception %s" % str(ex)
		try:
			print "[Move server] - service - [rapp_moveGetCollisionStatus]"
			self.service_mh = rospy.Service('rapp_moveGetCollisionStatus', MoveGetCollisionStatus, self.handle_rapp_moveGetCollisionStatus)
		except Exception, ex_mh:
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
		print "[Move server] - Actual Nao pose : %s" % str(pose)

		
	def handle_rapp_MoveTo(self,req):

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
		#~ self.proxy_motion.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION",False]])
		self.proxy_motion.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])

		#####################
		## get robot position before move
		#####################
		InitRobotPosition = m.Pose2D(self.proxy_motion.getRobotPosition(False))

		print "[Move server] - Destination X = %s Destination Y = %s  Destination Theta = %s" %(req.destination_x, req.destination_y, req.destination_theta)

		print "[Move server] - Nao init position = ", InitRobotPosition

		X = req.destination_x
		Y = req.destination_y
		Theta = req.destination_theta

		self.proxy_motion.post.moveTo(X, Y, Theta)

		# wait is useful because with post Move is not blocking function
		self.proxy_motion.waitUntilMoveIsFinished()

		self.SetPose('Stand')
		isDestinationReached = True

		#####################
		## get END robot position
		#####################
		EndRobotPosition = m.Pose2D(self.proxy_motion.getRobotPosition(False))
		print "[Move server] - Nao end position = ", EndRobotPosition
	
		Nao_x = EndRobotPosition.x
		Nao_y = EndRobotPosition.y
		return MoveToResponse(isDestinationReached)
	
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

		self.proxy_motion.post.move(X, Y, Theta)

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

	def handle_rapp_moveHead(self,req):

		fractionMaxSpeed = 0.1
		self.proxy_motion.setAngles('HeadYaw',req.yaw,fractionMaxSpeed)

		self.proxy_motion.setAngles('HeadPitch',req.pitch,fractionMaxSpeed)
		names		= "Head"
		useSensors  = True
		yaw_end = 0
		pitch_end = 0		
		
		while True:
			yaw_end_old = yaw_end
			pitch_end_old = pitch_end
			rospy.sleep(0.2)
			sensorAngles = self.proxy_motion.getAngles(names, useSensors)
			yaw_end=sensorAngles[0]
			pitch_end=sensorAngles[1]
			if yaw_end_old == yaw_end and pitch_end_old == pitch_end: 
				break

		return MoveHeadResponse(yaw_end,pitch_end)

	def handle_rapp_moveStop(self,req):
		self.proxy_motion.stopMove()
		return MoveStopResponse(True)

	def handle_rapp_moveGetCollisionStatus(self,req):
		self.subscribeToEvents()
		return MoveGetCollisionStatusResponse(self.EventKey,self.EventValue)


	def subscribeToEvents(self):
		self.prox_memory.subscribeToEvent("ALMotion/Safety/MoveFailed", self.moduleName, "MoveCallback")
		print "\n \n Subscribe callback: "

		self.prox_memory.subscribeToEvent("Navigation/AvoidanceNavigator/ObstacleDetected", self.moduleName, "ObstacleCallback")
	def MoveCallback(self, strVarName, value,message):
		""" Mandatory docstring.
			comment needed to create a bound method
		"""
		self.EventKey = str(strVarName)
		self.EventValue = value
		print "\n \n Event:  ALMotion/Safety/MoveFailed callback: "
		print "key %s" % str(strVarName)
		print "value %s" % str(value)
		print ""
	def ObstacleCallback(self, strVarName, value,message):

		print "\n \n Event:  Navigation/AvoidanceNavigator/ObstacleDetected callback: "
		print "key %s" % str(strVarName)
		print "value %s" % str(value)
		print ""
	def getSonarData(self):
		self.proxy_sonar.subscribe("obstacleAvoidance")
		self.sonar_data = [0,0]
		self.sonar_data[0] =self.prox_memory.getData("Device/SubDeviceList/US/Right/Sensor/Value")
		self.sonar_data[1] = self.prox_memory.getData("Device/SubDeviceList/US/Left/Sensor/Value")
		return self.sonar_data


	def avoideObstacle(self):
		Obs_flag = 0
		self.proxy_motion.setExternalCollisionProtectionEnabled('All', False)
		self.sonar_data = self.getSonarData()
		#print min(self.sonar_data)
		self.followObstaclesBoudery(1)		


	def followObstaclesBoudery(self,direction):
		k=0.1
		i=0
		print min(self.sonar_data)

	 	while (k >0 and k<1 and i < 5):
	 	 			
	 		sonar_data = self.getSonarData()
	 		#      |  <-sonar_left_distance 
	 		#      |  ___________                 AVOIDE LEFT DIRACTION
	 		#      |  \ <- sonar_right_distance  
	 		#      | / \
	 		#      |/   \
	 		#      *
	 		if sonar_data[1]>sonar_data[0]: # [0] - right,  [1] - left
	 			#avoide_diraction = "left"
	 			theta = 0.1
	 		else:
	 			#avoide_diraction = "right"
	 			theta = -0.1
			self.proxy_motion.moveToward(0, 0, theta)
	 		k = min(sonar_data)
	 		print sonar_data
	 		if k == sonar_data[0]:
	 			print "right sonar : %s" %k
	 		else :
	 			print "left sonar : %s" %k
	 		if k > 1:
	 			i=i+1

	 	print "STOP"

	 	while True:
	 		sonar_data = self.getSonarData()
	 		if theta >0:

		 		if sonar_data[0]>1: # [0] - right,  [1] - left
		 			#avoide_diraction = "left"
			 		self.proxy_motion.moveToward(0.4, 0, -sonar_data[0]/4)

		 		else:
		 			#avoide_diraction = "right"
			 		self.proxy_motion.moveToward(0.4, 0, 0.9/1-sonar_data[0] )
	 		else:

		 		if sonar_data[1]>1: # [0] - right,  [1] - left
		 			#avoide_diraction = "left"
			 		self.proxy_motion.moveToward(0.4, 0, sonar_data[0]/4)
		 		else:
		 			#avoide_diraction = "right"
			 		self.proxy_motion.moveToward(0.4, 0, -0.9/1-sonar_data[0] )
	
	 	# while (k >0 and k<1):
	 	 			
	 	# 	sonar_data = self.getSonarData()
	 	# 	#      |  <-sonar_left_distance 
	 	# 	#      |  ___________                 AVOIDE LEFT DIRACTION
	 	# 	#      |  \ <- sonar_right_distance  
	 	# 	#      | / \
	 	# 	#      |/   \
	 	# 	#      *
	 	# 	if sonar_data[1]>sonar_data[0]: # [0] - right,  [1] - left
	 	# 		#avoide_diraction = "left"
	 	# 		theta = 0.1
	 	# 	else:
	 	# 		#avoide_diraction = "right"
	 	# 		theta = -0.1
			# self.proxy_motion.moveToward(0, 0, theta)
	 	# 	k = min(sonar_data)
	 	# 	print k




	 	self.proxy_motion.moveToward(0, 0, theta)

	 	self.proxy_motion.stopMove()



		### 
		#duza przeszkoda:
		#  obracaj sie w odpowiednia strone az jeden sonar bedzie wolny, idz do przodu az drugi sonar bedzie wolny,
		#  skrec w ta strone z ktorej uwolnil sie sonar, idz prosto
# mala:  Kieruj sie non stop na mete, az oba sonary beda wolne.


# Testng SIGINT signal handler
def signal_handler(signal, frame):
	print "[Move server] - signal SIGINT caught"
	print "[Move server] - system exits"
	sys.exit(0)

def main():
	""" Main entry point
		"""
	try:
		signal.signal(signal.SIGINT, signal_handler)
		print "[Move server] - Press Ctrl + C to exit system correctly"
		
		myBroker = ALBroker("myBroker", "0.0.0.0", 0, Constants.NAO_IP,Constants.PORT)
		
		global NaoMove
		NaoMove = MoveNaoModule("NaoMove")

		rospy.spin()
	
	except (KeyboardInterrupt, SystemExit):
		print "[Move server] - SystemExit Exception caught"
		#unsubscribe()
		myBroker.shutdown()
		sys.exit(0)
		
	except Exception, ex:
		print "[Move server] - Exception caught %s" % str(ex)
		#unsubscribe()
		myBroker.shutdown()
		sys.exit(0)
		
if __name__ == "__main__":
	try:
		main()
	except Exception,e:
		print "__name__ - Error %s" % str(e)