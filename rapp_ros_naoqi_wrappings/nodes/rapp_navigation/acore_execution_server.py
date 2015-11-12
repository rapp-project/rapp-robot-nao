#!/usr/bin/env python
######################
## written by Wojciech Dudek
######################
__author__ = "Wojciech Dudek"

########################
# Imports
########################

# Importing services
from rapp_ros_naoqi_wrappings.srv import *
from rapp_ros_naoqi_wrappings.msg import obstacleData

from navfn.srv import *

# Importing core system functionality
import signal
import sys, os
import rospy
import almath as m
import numpy
#import transformPose

# Importing core functionality from Naoqi
from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule

#from geometry_msgs import PoseStamped
#######################################


# Constants
class Constants:

	NAO_IP = "nao.local"
	PORT = 9559

	

#######################################

class MoveNaoModule(ALModule):
	""" A simple module able to react to
	sound detection events
	"""
	
	# Constructor of MoveNaoModule
	def __init__(self,name):
		ALModule.__init__(self,name)
		
		print "[Execution server] - Acore Execution server initialization"
		
		# Initialization of ROS node
		rospy.init_node('acore_execution_server')
		self.moduleName = name
		
		# Initialization of Naoqi modules and ROS services
		self.initALModule()
		self.setVariables()
		#self.subscribeToEvents()
		self.openServices()


		print "[Execution server] - Waits for clients ..."
				
	# Initialization of Naoqi modules
	def initALModule(self):
		print "[Execution server] - Initialization of Naoqi modules"
		
		self.prox_memory = ALProxy("ALMemory")
		if self.prox_memory is None:
			rospy.logerr("[Execution server] - Could not get a proxy to ALMemory")
			exit(1)
			
		print "[Execution server] - ALMotion proxy initialization"
		self.proxy_motion = ALProxy("ALMotion")
		if self.proxy_motion is None:
			rospy.logerr("[Execution server] - Could not get a proxy to ALMotion")
			exit(1)
		print "[Execution server] - ALRobotPosture proxy initialization"
		self.proxy_RobotPosture = ALProxy("ALRobotPosture")
		if self.proxy_RobotPosture is None:
			rospy.logerr("[Execution server] - Could not get a proxy to ALRobotPosture")
			exit(1)

		# self.compass = ALProxy("ALVisualCompass")
		# if self.compass is None:
		# 	rospy.logerr("[Execution server] - Could not get a proxy to ALVisualCompass")
		# 	exit(1)
	# Setting variables
	def setVariables(self):
		print "[Execution server] - Setting variables"
		self.MoveIsFailed = False
		self.GP_seq = -1
		self.tl = tf.TransformListener(True, rospy.Duration(5.0))

	def openServices(self):
		try:
			print "[Execution server] - setting services"
			print "[Execution server] - service - [MoveTo]"
			self.service_mt = rospy.Service('moveTo', MoveTo, self.handle_moveTo)
		except Exception, ex_mt:
			print "[Execution server] - Exception %s" % str(ex_mt)
		try:
			print "[Execution server] - service - [moveVel]"
			self.service_mv = rospy.Service('moveVel', MoveVel, self.handle_moveVel)
		except Exception, ex_mv:
			print "[Execution server] - Exception %s" % str(ex_mv)
		try:
			print "[Execution server] - service - [moveJoint]"
			self.service_mh = rospy.Service('moveJoint', MoveJoint, self.handle_moveJoint)
		except Exception, ex_mh:
			print "[Execution server] - Exception %s" % str(ex_mh)
		try:
			print "[Execution server] - service - [triggerStiffness]"
			self.service_mh = rospy.Service('triggerStiffness', TriggerStiffness, self.handle_triggerStiffness)
		except Exception, ex_mh:
			print "[Execution server] - Exception %s" % str(ex_mh)
			
		try:
			print "[Execution server] - service - [takePredefinedPosture]"
			self.service_mh = rospy.Service('takePredefinedPosture', TakePredefinedPosture, self.handle_takePredefinedPosture)
		except Exception, ex_mh:
			print "[Execution server] - Exception %s" % str(ex_mh)

	def getch(self):
		import sys, tty, termios
		fd = sys.stdin.fileno()
		old = termios.tcgetattr(fd)
		try:
			tty.setraw(fd)
			return sys.stdin.read(1)
		finally:
			termios.tcsetattr(fd, termios.TCSADRAIN, old)
	####
	##  SERVECE HANDLERS
	####

	def handle_moveHead(self,req):
		head_yaw = req.yaw 
		head_pitch = req.pitch 
		self.handle_moveJoint(head_yaw,head_pitcg, 0.2)
		#self.StiffnessOff("Head")
		return MoveHeadResponse(yaw_end,pitch_end)
	def handle_moveTo(self,req):
		try:
			#self.SetPose('StandInit')
			#####################
			## Collision detection
			#####################
			self.proxy_motion.setExternalCollisionProtectionEnabled('All', False)

			#####################
			## Enable arms control by move algorithm
			#####################
			self.proxy_motion.setWalkArmsEnabled(False, False)

			#####################
			## FOOT CONTACT PROTECTION
			#####################
			#~ motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION",False]])
			self.proxy_motion.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])

			#####################
			## get robot position before move
			# #####################
			# InitRobotPosition = m.Pose2D(self.proxy_motion.getRobotPosition(False))

			# print "[MoveVel server] - Velocity X = %s Velocity Y = %s  Velocity Theta = %s" %(req.velocity_x, req.velocity_y, req.velocity_theta)

			# print "[MoveVel server] - Nao init position = ", InitRobotPosition
			self.proxy_motion.moveTo(req.x, req.y, req.theta)
			status = 0
		except Exception, ex:
			print "[MoveTo server] - Exception %s" % str(ex)
			status = 1
		return MoveToResponse(status)	

	
	def handle_moveVel(self,req):
		try:
			#self.SetPose('StandInit')
			#####################
			## Collision detection
			#####################
			self.proxy_motion.setExternalCollisionProtectionEnabled('All', False)

			#####################
			## Enable arms control by move algorithm
			#####################
			self.proxy_motion.setWalkArmsEnabled(False, False)

			#####################
			## FOOT CONTACT PROTECTION
			#####################
			#~ motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION",False]])
			self.proxy_motion.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])

			#####################
			## get robot position before move
			# #####################
			# InitRobotPosition = m.Pose2D(self.proxy_motion.getRobotPosition(False))

			# print "[MoveVel server] - Velocity X = %s Velocity Y = %s  Velocity Theta = %s" %(req.velocity_x, req.velocity_y, req.velocity_theta)

			# print "[MoveVel server] - Nao init position = ", InitRobotPosition

			X = req.velocity_x
			Y = req.velocity_y
			Theta = req.velocity_theta
			
			self.proxy_motion.move(X, Y, Theta)

			status = 0
		except Exception, ex:
			status = 1
			print "[Execution server] - Exception %s" % str(ex)
		return MoveVelResponse(status)	

	def handle_moveJoint(self,req):
		try:
			self.StiffnessOn(req.joint_name)

			self.proxy_motion.angleInterpolationWithSpeed(req.joint_name,req.joint_angle,req.speeds)
			status = 0
		except Exception, ex:
			status = 1
			print "[Execution server] - Exception %s" % str(ex)
		return MoveJointResponse(status)

	def handle_triggerStiffness(self,req):
		pNames = req.joint_name
		try:
			if req.trigger:
				self.StiffnessOn(pNames)
			else:
				self.StiffnessOff(pNames)
			status = 0
		except Exception, ex:
			status = 1
			print "[Execution server] - Exception %s" % str(ex)
		return RemoveStiffnessResponse(status)	

	def handle_moveStop(self):
		self.proxy_motion.move(0, 0, 0)
		return MoveStopResponse(True)

	def handle_takePredefinedPosture(self,req):
		try:
			status = 0

			self.StiffnessOn("Body")
		except Exception, ex:
			status = 1
			print "[Execution server] - Exception %s" % str(ex)
		try:	
			status = 0
			self.proxy_RobotPosture.goToPosture(req.pose,req.speed)
		except Exception, e:
			status = 1
			print "[Execution server] - Exception %s" % str(e)	
		print "[Execution server] - Actual Nao pose : %s" % str(req.pose)
		return TakePredefinedPostureResponse(status)	



			# Nao must move 
	####
	##  Nao core drivers
	####
			# set stiffness
	def StiffnessOn(self, joint):
		# We use the "Body" name to signify the collection of all joints
		pNames = joint
		pStiffnessLists = 1.0
		pTimeLists = 1.0
		self.proxy_motion.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)	
			# set Nao pose
	def StiffnessOff(self,joint):
		pNames = joint
		pStiffnessLists = 0.0
		pTimeLists = 1.0
		self.proxy_motion.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)	

	def SetPose(self,pose):

		try:
			self.StiffnessOn("Body")
		except Exception, ex:
			print "[Execution server] - Exception %s" % str(ex)
		try:	
			self.proxy_RobotPosture.goToPosture(pose, 0.3)
		except Exception, e:
				print "[Execution server] - Exception %s" % str(e)	
		print "[Execution server] - Actual Nao pose : %s" % str(pose)
			# save SONAR data as self.sonar_data[] -> [0] - RIGHT, [1] - LEFT

# Testng SIGINT signal handler
def signal_handler(signal, frame):
	print "[Execution server] - signal SIGINT caught"
	print "[Execution server] - system exits"
	sys.exit(0)

def main():
	""" Main entry point
		"""
	try:
		signal.signal(signal.SIGINT, signal_handler)
		print "[Execution server] - Press Ctrl + C to exit system correctly"
		
		myBroker = ALBroker("myBroker", "0.0.0.0", 0, Constants.NAO_IP,Constants.PORT)
		
		global Execution_server
		Execution_server = MoveNaoModule("Execution_server")

		rospy.spin()
	
	except (KeyboardInterrupt, SystemExit):
		print "[Execution server] - SystemExit Exception caught"
		#unsubscribe()
		myBroker.shutdown()
		sys.exit(0)
		
	except Exception, ex:
		print "[Execution server] - Exception caught %s" % str(ex)
		#unsubscribe()
		myBroker.shutdown()
		sys.exit(0)
		
if __name__ == "__main__":
	try:
		main()
	except Exception,e:
		print "__name__ - Error %s" % str(e)