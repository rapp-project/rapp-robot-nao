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
from geometry_msgs import Pose.msgs

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
		
		print "[Navigation server] - Acore Navigation Server initialization"
		
		# Initialization of ROS node
		rospy.init_node('acore_move')
		self.moduleName = name
		
		# Initialization of Naoqi modules and ROS services
		self.initALModule()
		self.setVariables()
		self.openServices()
		self.avoideObstacle()
		#Subscribe to Naoqi events
		self.subscribeToEvents()

		print "[Navigation server] - Waits for clients ..."
		
		
	
	# Initialization of Naoqi modules
	def initALModule(self):
		print "[Navigation server] - Initialization of Naoqi modules"
		
		self.prox_memory = ALProxy("ALMemory")
		if self.prox_memory is None:
			rospy.logerr("[Navigation server] - Could not get a proxy to ALMemory")
			exit(1)
			
		print "[Navigation server] - ALMotion proxy initialization"
		self.proxy_motion = ALProxy("ALMotion")
		if self.proxy_motion is None:
			rospy.logerr("[Navigation server] - Could not get a proxy to ALMotion")
			exit(1)
		print "[Navigation server] - ALRobotPosture proxy initialization"
		self.proxy_RobotPosture = ALProxy("ALRobotPosture")
		if self.proxy_RobotPosture is None:
			rospy.logerr("[Navigation server] - Could not get a proxy to ALRobotPosture")
			exit(1)
		self.proxy_sonar = ALProxy("ALSonar")
		if self.prox_memory is None:
			rospy.logerr("[Navigation server] - Could not get a proxy to ALSonar")
			exit(1)
	
	# Setting variables
	def setVariables(self):
		print "[Navigation server] - Setting variables"
	
		
	# Initialization of ROS services
	def openServices(self):
		try:
			print "[Navigation server] - setting services"
			print "[Navigation server] - service - [rapp_updatePose]"
			self.service_mt = rospy.Service('rapp_updatePose', UpdatePose, self.handle_rapp_UpdatePose)
		except Exception, ex_mt:
			print "[Navigation server] - Exception %s" % str(ex)
		try:
			print "[Navigation server] - service - [rapp_getPose]"
			self.service_mt = rospy.Service('rapp_getPose', GetPose, self.handle_rapp_GetPose)
		except Exception, ex_mt:
			print "[Navigation server] - Exception %s" % str(ex)
		try:
			print "[Navigation server] - service - [rapp_getPlan]"
			self.service_mt = rospy.Service('rapp_getPlan', GetPlan, self.handle_rapp_GetPlan)
		except Exception, ex_mt:
			print "[Navigation server] - Exception %s" % str(ex)


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
			print "[Navigation server] - Exception %s" % str(ex)
		try:	
			self.proxy_RobotPosture.goToPosture(pose, 0.5)
		except Exception, e:
				print "[Navigation server] - Exception %s" % str(e)	
		print "[Navigation server] - Actual Nao pose : %s" % str(pose)

		
	def handle_rapp_updatePose(self,req):

	pub = rospy.Publisher('rapp_updateOdometryPose', Pose, queue_size=10)
	odometryPose = geometry_msgs.Pose
	rospy.init_node('OdometryPoseUpdater', anonymous=True)
	Pose.position = (1,1)
	odometryPose.position = (1,1)
	odometryPose.orientation = (0,0,0,1)#"hello world %s" % rospy.get_time()
	rospy.loginfo(odometryPose)
	pub.publish(odometryPose)

	
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

		self.prox_memory.subscribeToEvent("DangerousObstacleDetected", self.moduleName, "ObstacleCallback")

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

		print "\n \n Event:  DangerousObstacleDetected callback: "
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
		k=0
		print min(self.sonar_data)
		self.proxy_motion.setWalkArmsEnabled(True, True)

	 	while k not in xrange(0,1):
	 	 			
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
	 		
	 		if k = sonar_data[0]:
	 			print "right sonar :" k
	 		else :
	 			print "left sonar :" k

	 	print min(self.sonar_data)
	 	self.proxy_motion.stopMove()



		### 
		#duza przeszkoda:
		#  obracaj sie w odpowiednia strone az jeden sonar bedzie wolny, idz do przodu az drugi sonar bedzie wolny,
		#  skrec w ta strone z ktorej uwolnil sie sonar, idz prosto
# mala:  Kieruj sie non stop na mete, az oba sonary beda wolne.


# Testng SIGINT signal handler
def signal_handler(signal, frame):
	print "[Navigation server] - signal SIGINT caught"
	print "[Navigation server] - system exits"
	sys.exit(0)

def main():
	""" Main entry point
		"""
	try:
		signal.signal(signal.SIGINT, signal_handler)
		print "[Navigation server] - Press Ctrl + C to exit system correctly"
		
		myBroker = ALBroker("myBroker", "0.0.0.0", 0, Constants.NAO_IP,Constants.PORT)
		
		global NaoMove
		NaoMove = MoveNaoModule("NaoMove")

		rospy.spin()
	
	except (KeyboardInterrupt, SystemExit):
		print "[Navigation server] - SystemExit Exception caught"
		#unsubscribe()
		myBroker.shutdown()
		sys.exit(0)
		
	except Exception, ex:
		print "[Navigation server] - Exception caught %s" % str(ex)
		#unsubscribe()
		myBroker.shutdown()
		sys.exit(0)
		
if __name__ == "__main__":
	try:
		main()
	except Exception,e:
		print "__name__ - Error %s" % str(e)