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
from navfn.srv import *

# Importing core system functionality
import signal
import sys, os
import rospy
import almath as m
import numpy
from geometry_msgs.msg import PoseStamped

import tf.transformations
#import transformPose

# Importing core functionality from Naoqi
from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule
import thread 
#import threading
# import msgs 
import geometry_msgs
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
		
		print "[Move server] - Acore Move Server initialization"
		
		# Initialization of ROS node
		rospy.init_node('acore_move')
		self.moduleName = name
		
		# Initialization of Naoqi modules and ROS services
		self.initALModule()
		self.setVariables()
		self.subscribeToEvents()
		self.openServices()


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
		self.MoveIsFailed = False
		self.GP_seq = -1
		self.tl = tf.TransformListener(True, rospy.Duration(10.0))
		
	# NAOqi Event subscribtion
	def subscribeToEvents(self):
		self.prox_memory.subscribeToEvent("ALMotion/Safety/MoveFailed", self.moduleName, "MoveCallback")
		#self.prox_memory.subscribeToEvent("LeftBumperPressed", self.moduleName, "LeftBumperPressed")
		#self.prox_memory.subscribeToEvent("RightBumperPressed", self.moduleName, "RightBumperPressed")
		self.avoide_ID = 0
		#self.proxy_sonar.subscribe("obstacleAvoidance")
		#self.prox_memory.subscribeToEvent("Navigation/AvoidanceNavigator/ObstacleDetected", self.moduleName, "ObstacleCallback")
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
		try:
			print "[Move server] - service - [rapp_moveJoint]"
			self.service_mh = rospy.Service('rapp_moveJoint', MoveJoint, self.handle_rapp_moveJoint)
		except Exception, ex_mh:
			print "[Move server] - Exception %s" % str(ex)	
		try:
			print "[Move server] - service - [rapp_removeStiffness]"
			self.service_mh = rospy.Service('rapp_removeStiffness', RemoveStiffness, self.handle_rapp_removeStiffness)
		except Exception, ex_mh:
			print "[Move server] - Exception %s" % str(ex)
			
		try:
			print "[Move server] - service - [rapp_takePredefinedPose]"
			self.service_mh = rospy.Service('rapp_takePredefinedPose', TakePredefinedPose, self.handle_rapp_takePredefinedPose)
		except Exception, ex_mh:
			print "[Move server] - Exception %s" % str(ex)
	####
	##  SERVECE HANDLERS
	####
	def handle_rapp_MoveTo(self,req):
		# self.GP_seq +=1
		# #self.lock =thread.allocate_lock()

		# self.prox_memory.removeData("ALMotion/Safety/MoveFailed")
		# self.prox_memory.subscribeToEvent("ALMotion/Safety/MoveFailed", self.moduleName, "MoveCallback")

		# #####################
		# ## Set class variables for MoveTo
		# #####################
		# self.destinationX=req.destination_x
		# self.destinationY=req.destination_y
		# self.destinationTheta=req.destination_theta
		# self.MoveIsFailed = False
		# #self.move_is_finished = False
		# GoalLocalPosition = numpy.array([req.destination_x,req.destination_y,0,1])
		# self.setGoalGlobalPose(GoalLocalPosition)
		# GP = self.getGoalGlobalPose()
		# x=GP.pose.position.x#[0]
		# y=GP.pose.position.y#[1]
		# t=tf.transformations.euler_from_quaternion(GP.pose.orientation)[2]
		# #k#=GP.pose.position[0]
		
		# self.publishGoal(GP)


		# isDestinationReached = False
		# self.SetPose('StandInit')
		# #####################
		# ## Collision detection
		# #####################
		# self.proxy_motion.setExternalCollisionProtectionEnabled('Move', True)

		# #####################
		# ## Enable arms control by move algorithm
		# #####################
		# self.proxy_motion.setWalkArmsEnabled(True, True)

		# #####################
		# ## FOOT CONTACT PROTECTION
		# #####################
		# #~ self.proxy_motion.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION",False]])
		# self.proxy_motion.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])

		# #####################
		# ## get robot position before move
		# #####################
		# InitRobotPosition = m.Pose2D(self.proxy_motion.getRobotPosition(False))

		# print "[Move server] - Destination X = %s Destination Y = %s  Destination Theta = %s" %(req.destination_x, req.destination_y, req.destination_theta)

		# print "[Move server] - Nao init position = ", InitRobotPosition

		# # while self.avoide_ID == False:
		# # 	rospy.sleep(1)

		# self.proxy_motion.post.moveTo(self.destinationX, self.destinationY, self.destinationTheta)
		
		# # wait is useful because with post Move is not blocking function
		# self.proxy_motion.waitUntilMoveIsFinished()
		# rospy.sleep(1)
		# if self.MoveIsFailed == True:
		# 	isDestinationReached = False
		# 	return MoveToResponse(isDestinationReached)

		# # isLocked = self.lock.acquire()
		# # print "isLocked %s" %isLocked
		# # print "[Move server] - Nao finished move"
		# # # while self.avoide_ID == False:
		# # #  	rospy.sleep(1)
		# # #if self.MoveIsFailed == True:
		# # #	self.avoideObstacle()
		# # #	isDestinationReached = False
		# # #	self.MoveIsFailed = False

		# # #else :
		# # self.SetPose('Stand')
		# isDestinationReached = True

		# #####################
		# ## get END robot position
		# #####################
		# EndRobotPosition = m.Pose2D(self.proxy_motion.getRobotPosition(False))
		# print "[Move server] - Nao end position = ", EndRobotPosition
	
		# Nao_x = EndRobotPosition.x
		# Nao_y = EndRobotPosition.y
		# return MoveToResponse(isDestinationReached)


# nowe MOVETO
		self.getNaoCurrentPosition()
		destinationX=req.destination_x
		destinationY=req.destination_y
		destinationTheta=req.destination_theta
		GoalGlobalPose = [destinationX,destinationY,destinationTheta]
		# flag for detectObstacle
		self.move_is_finished = False
		self.obstacle_detected = False
		print "[Move server] - waiting for makePlan service"
		rospy.wait_for_service('/global_planner/make_plan')
		print "[Move server] - makePlan service found"
		
		self.SetPose('Stand')
		self.followPath_flag = 'empty'
		while self.followPath_flag != 'path_end':
			
			path = self.plannPath(GoalGlobalPose)
			# # # zanim zaczniesz ruch ustaw watek do sprawdzania sonarow
			thread.start_new_thread(self.detectObstacle,(99,))	
			
			self.followPath_flag = self.followPath(path)
			print "followPath returned: \n", self.followPath_flag

			if self.followPath_flag == 'obstacle':
				#self.BUG()
				print "BUG BUG BUG BUG BUG BUG BUG BUG \n BUG BUG BUG BUG BUG BUG"
		self.move_is_finished = True

		return MoveToResponse(self.move_is_finished)


	def plannPath(self,GoalGlobalPose):
		naoCurrentPosition = self.getNaoCurrentPosition()
		start = PoseStamped()
		goal = PoseStamped()
		start.header.seq = 0
		goal.header.seq = 0
		start.header.stamp = rospy.Time.now()
		goal.header.stamp = rospy.Time.now()
		start.header.frame_id = "/map"
		goal.header.frame_id = "/map"
		start.pose.position.x = naoCurrentPosition[0][0]
		start.pose.position.y = naoCurrentPosition[0][1]
		start.pose.position.z = naoCurrentPosition[0][2]
		start.pose.orientation.x = naoCurrentPosition[1][0]
		start.pose.orientation.y = naoCurrentPosition[1][1]
		start.pose.orientation.z = naoCurrentPosition[1][2]
		start.pose.orientation.w = naoCurrentPosition[1][3]
		goal.pose.position.x = GoalGlobalPose[0]
		goal.pose.position.y = GoalGlobalPose[1]
		goal.pose.position.z = 0
		goal_orientation_quaternion = tf.transformations.quaternion_from_euler(0,0,GoalGlobalPose[2]) 
		goal.pose.orientation.x = goal_orientation_quaternion[0]
		goal.pose.orientation.y = goal_orientation_quaternion[1]
		goal.pose.orientation.z = goal_orientation_quaternion[2]
		goal.pose.orientation.w = goal_orientation_quaternion[3]
		print "tu jest start \n",start
		print "tu jest goal \n",goal
		path = numpy.array(PoseStamped())
		plan_path = rospy.ServiceProxy('/global_planner/make_plan', MakeNavPlan)
		path = plan_path(start,goal)
		return path

	def followPath(self,path):			
		for i in range(len(path.path)-1):
			naoCurrentPosition = self.getNaoCurrentPosition()
			robot_orientation_euler = tf.transformations.euler_from_quaternion(naoCurrentPosition[1])
			nextPose = path.path[i+1]
			rotation = [nextPose.pose.orientation.x,nextPose.pose.orientation.y,nextPose.pose.orientation.z,nextPose.pose.orientation.w]
			nextPoseOrientationZ = tf.transformations.euler_from_quaternion(rotation)[2]#.x,nextPose.pose.orientation.y,nextPose.pose.orientation.z,nextPose.pose.orientation.w)[2]
			print "[Path tracker] - getting to next point:\n ", i+1
			#pr. k
			a_k= numpy.tan(robot_orientation_euler[2]) 
			#pr. AS : y= a_AS*x + b_AS
			a_AS = -1/a_k
			b_AS = naoCurrentPosition[0][1] - a_AS*naoCurrentPosition[0][0]
			#pr. l
			a_l= numpy.tan(nextPoseOrientationZ) 
			#pr. BS
			a_BS = -1/a_l
			b_BS = nextPose.pose.position.y - a_BS*nextPose.pose.position.x
			# circle center
			x_S = (b_BS-b_AS)/(a_BS-a_AS)
			y_S = a_BS*x_S+b_BS
			# circle radius:
			R = numpy.sqrt((nextPose.pose.position.x-x_S)*(nextPose.pose.position.x-x_S)+(nextPose.pose.position.y-y_S)*(nextPose.pose.position.y-y_S))
			# angle of the curve:
			alpha = numpy.arctan(abs((a_BS-a_AS)/(1+a_AS*a_BS)))
			if alpha > 3.14/2:
				alpha = 3.14 - alpha
			#curve path distance:
			d = R * alpha
			moveTime_x = d/0.04
			moveTime_theta = alpha/0.2
			if moveTime_x >= moveTime_theta:
				velocity_x = 0.04
				velocity_theta =  alpha/moveTime_x
			else:
				velocity_x = d/moveTime_theta
				velocity_theta =  0.2

			if self.obstacle_detected==True:
				flag = 'obstacle'
				return flag
				break
			self.proxy_motion.post.move(velocity_x,0,velocity_theta)
			print "predkosci:\n",velocity_x,"  ",velocity_theta
			print "czasy:\n",moveTime_x,"  ",moveTime_theta
			print "dystanse:\n",d,"  ",alpha


		 	rospy.sleep(numpy.maximum(moveTime_theta,moveTime_x))
		 	self.proxy_motion.post.move(0,0,0)
		flag = 'path_end'
		return flag

		#LOOP
#  MOVE TO next point of path(path_points, max speed) ------>
#  co 10 punkt sprawdz sonar, jesli bedzie przeszkoda:
#			velocity == 0
#			BUG  ------> 
#
#
# BUG -> omijajac przeszkode, idziesz w bok az stracisz przeszkode, nastepnie luk az zlapiesz przeszkode itd
#
#
#
######
#### BUG
#
#
#
#
#
#
#
#
#MOVE_to_next_point
#
#    1. wyznacz proste k i l przechodzace przez obecny i kolejny punkt trajektorii
#	 2. wyznacz proste prostopadle do tych prostych i wyznacz srodek okregu
#	 3. wyznacz promien okregu R
#	 4. z tw. cos wyznacz kat luku alfa
#	 5. wyznacz dlugosc luku 
#	 6. idz tak dlugo prosto aby dl_luku/speed = time, skrecaj z taka predkoscia, aby na koniec czasu obrocic sie o wyznaczony kat alfa
#
#
#
#
#
#
#					lub aktualna pozycja z kalmana - n+1_point_theta
#       theta_diff = n_point_theta - n+1_point_theta
#		TURN(theta_diff)
#		dist_diff = sqrt((n_point_X - n+1_point_X)^2 + (n_point_Y - n+1_point_Y)^2)
#		time = dist_diff/speed
#		move(speed,0,0)
#		wait(time)
#
#
#
#
#BUG
#
#
#
#
#
	def getNaoCurrentPosition(self):

		if self.tl.canTransform("map","base_link",rospy.Time()):
					nao_position = self.tl.lookupTransform("map","base_link",rospy.Time())
		print "nao position",(nao_position)
		return nao_position

	def detectObstacle(self,empty):
		print "[detectObstacle] started"
		while (self.move_is_finished != True): 
			# sonar data = [right_dist, left_dist]
			print "[detectObstacle] new scan"
			data = self.getSonarData()
			if (data[0] <0.6 )or (data[1] <0.6):
				self.obstacle_detected = True
				print "[detectObstacle] stopped, data: \n", data

				return self.obstacle_detected

			rospy.sleep(3)
		thread.exit()


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
		self.StiffnessOn("Head")
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
			rospy.sleep(1)
			sensorAngles = self.proxy_motion.getAngles(names, useSensors)
			yaw_end=sensorAngles[0]
			pitch_end=sensorAngles[1]
			if yaw_end_old == yaw_end and pitch_end_old == pitch_end: 
				break
		self.StiffnessOff("Head")
		return MoveHeadResponse(yaw_end,pitch_end)

	def handle_rapp_moveJoint(self,req):
		self.StiffnessOn(req.joint_name)
		req.fractionMaxSpeed = 0.1
		self.proxy_motion.setAngles(req.joint_name,req.joint_angle,req.fractionMaxSpeed)
		useSensors  = True
		angle_now = 0
	
		while True:
			angle_old = angle_now
			rospy.sleep(1)
			sensorAngles = self.proxy_motion.getAngles(req.joint_name, useSensors)
			angle_now=sensorAngles
			if angle_old == angle_now:
				break
		return MoveJointResponse(angle_now)

	def handle_rapp_removeStiffness(self,req):
		pNames = req.joint_name
		pStiffnessLists = 0.0
		pTimeLists = 1.0
		self.proxy_motion.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)
		status = True
		return RemoveStiffnessResponse(status)	

	def handle_rapp_moveStop(self,req):
		self.proxy_motion.stopMove()
		return MoveStopResponse(True)

	def handle_rapp_moveGetCollisionStatus(self,req):
		self.subscribeToEvents()
		return MoveGetCollisionStatusResponse(self.EventKey,self.EventValue)

	def handle_rapp_takePredefinedPose(self,req):
		try:
			self.StiffnessOn("Body")
		except Exception, ex:
			print "[Move server] - Exception %s" % str(ex)
		try:	
			self.proxy_RobotPosture.goToPosture(req.pose, 0.3)
		except Exception, e:
				print "[Move server] - Exception %s" % str(e)	
		print "[Move server] - Actual Nao pose : %s" % str(req.pose)
		return RemoveStiffnessResponse(True)	

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
			print "[Move server] - Exception %s" % str(ex)
		try:	
			self.proxy_RobotPosture.goToPosture(pose, 0.3)
		except Exception, e:
				print "[Move server] - Exception %s" % str(e)	
		print "[Move server] - Actual Nao pose : %s" % str(pose)
			# save SONAR data as self.sonar_data[] -> [0] - RIGHT, [1] - LEFT
	def getSonarData(self):
		self.proxy_sonar.subscribe("myApplication")
		self.sonar_data = [0,0]
		self.sonar_data[0] =self.prox_memory.getData("Device/SubDeviceList/US/Right/Sensor/Value")
		self.sonar_data[1] = self.prox_memory.getData("Device/SubDeviceList/US/Left/Sensor/Value")
		#self.proxy_sonar.subscribe("obstacleAvoidance")
		return self.sonar_data


	####
	##  Goal handlers
	####
	def setGoalGlobalPose(self,GoalLocalPosition): ### WARNING  - Actual Nao position from Odometry, not from POSE ESTIMATOR !!!!!!!!!!!

		self.GoalGlobalPose = geometry_msgs.msg.PoseStamped()
		ActualRobotPosition = m.Pose2D(self.proxy_motion.getRobotPosition(True))
		rospy.logwarn("WARNING  - Actual Nao position from Odometry, not from POSE ESTIMATOR !!!!!!!!!!!")
		#print ActualRobotPosition
		print "[setGoalGlobalPose] - setting Goal global Pose from: \n",GoalLocalPosition

		TransformMatrix= numpy.array([[numpy.cos(ActualRobotPosition.theta),-numpy.sin(ActualRobotPosition.theta),0,ActualRobotPosition.x],
		 								[numpy.sin(ActualRobotPosition.theta),numpy.cos(ActualRobotPosition.theta),0,ActualRobotPosition.y],
		 								[0,												0,												1,0],
		 								[0,												0,												0,1]])
		GoalGlobalPosition = numpy.dot(TransformMatrix,GoalLocalPosition)
		print "[setGoalGlobalPose] - GoalGlobalPosition: \n",GoalGlobalPosition
		print "[setGoalGlobalPose] - GoalGlobalPosition: \n",GoalGlobalPosition
		GlobalOrientation = tf.transformations.quaternion_from_euler(0, 0, self.destinationTheta+ActualRobotPosition.theta)
		print "[setGoalGlobalPose] - GoalGlobalOrientation (EULER): \n",numpy.array([0, 0, self.destinationTheta+ActualRobotPosition.theta])

		self.GoalGlobalPose.pose.position.x = GoalGlobalPosition[0]#[GoalGlobalPosition[0],GoalGlobalPosition[1],GoalGlobalPosition[2]]
		self.GoalGlobalPose.pose.position.y = GoalGlobalPosition[1]
		self.GoalGlobalPose.pose.position.z = GoalGlobalPosition[2]
		self.GoalGlobalPose.pose.orientation = GlobalOrientation 
		print "[setGoalGlobalPose] - Goal global pose is set"
			# publish "GP" on /goal topic | "GP" - geometry_msgs/PoseStamped object
	def publishGoal(self,GP):
		pub_pose = rospy.Publisher("/goal", PoseStamped , queue_size=10)
		print "PUBLISHING!!"
		print GP
		GP.header.seq = self.GP_seq
		GP.header.stamp = rospy.Time.now()
		GP.header.frame_id = "Nao_footprint"
		# GOAL = PoseStamped()
		# GOAL.header.seq = 0
		# GOAL.header.stamp = rospy.Time.now()

		# GOAL.header.frame_id = "Nao_footprint"
		# GOAL.pose.position.x = GP.pose.position.x
		# GOAL.pose.position.y = GP.pose.position.y
		# GOAL.pose.position.z = 0
		# q = GP.pose.orientatioin
		# GOAL.pose.orientation.x = q[0]
		# GOAL.pose.orientation.y = q[1]
		# GOAL.pose.orientation.z = q[2]
		# GOAL.pose.orientation.w = q[3]
		# rospy.set_param("/globalGoal/pose/x", GP.pose.position.x)
		# rospy.set_param("/globalGoal/pose/y", GP.pose.position.y)

		# rospy.set_param("/globalGoal/pose/t",self.destinationTheta)
		print "Publishing on /goal topic :\n",GP
		pub_pose.publish(GP)
	
	def getGoalGlobalPose(self):
		return self.GoalGlobalPose

	def	getGoalNewLocalPose(self,GoalGlobalPose):
		GoalNewLocalPose = geometry_msgs.msg.PoseStamped()
	 	ActualRobotPosition = m.Pose2D(self.proxy_motion.getRobotPosition(True))

	 	TransformMatrix = numpy.linalg.pinv(numpy.array([[numpy.cos(ActualRobotPosition.theta),-numpy.sin(ActualRobotPosition.theta),0,ActualRobotPosition.x],
	 									[numpy.sin(ActualRobotPosition.theta),numpy.cos(ActualRobotPosition.theta),0,ActualRobotPosition.y],
	 									[0,												0,												1,0],
	 									[0,												0,												0,1]]))
	 	print "[GoalNewLocalPose] Matrix for new coordinations for goal are : %s" %TransformMatrix
	 	GoalNewLocalPose.pose.position = numpy.dot(TransformMatrix,[GoalGlobalPose.pose.position[0],GoalGlobalPose.pose.position[1],0,1])
	 	print "[GoalNewLocalPose] move to : %s" %GoalNewLocalPose
	 	GoalNewLocalPose.pose.orientation = tf.transformations.quaternion_from_euler(0,0,tf.transformations.euler_from_quaternion(GoalGlobalPose.pose.orientation)[2] - ActualRobotPosition.theta)
	 	x=GoalNewLocalPose.pose.position[0]
		y=GoalNewLocalPose.pose.position[1]
		t=tf.transformations.euler_from_quaternion(GoalNewLocalPose.pose.orientation)[2]
		k=GoalNewLocalPose.pose.position[0]
		rospy.set_param('/globalNEWGoal/pose/x',float(x))#Decimal(GP.pose.position[0]))
		rospy.set_param('/globalNEWGoal/pose/y',float(y))
		rospy.set_param('/globalNEWGoal/pose/t',float(t))
	 	return GoalNewLocalPose

	####
	##  Callbacks
	####
			# Obstacle in move direction 
	def MoveCallback(self, strVarName, value,message):
		""" Mandatory docstring.
			comment needed to create a bound method
		"""
		#avoide_alive = threading.isAlive()#self.avoide_ID)
		# print "\t \t avoide ALIVE: %s" %(avoide_alive)

		# self.kill_avoide = True
		# avoide_alive = threading.isAlive()#self.avoide_ID)

		# print "\t \t avoide ALIVE: %s" %(avoide_alive)
		self.prox_memory.unsubscribeToEvent("ALMotion/Safety/MoveFailed",self.moduleName)
		self.proxy_motion.setExternalCollisionProtectionEnabled('Move', False)
		self.AvoideObstacle = True
		self.EventKey = str(strVarName)
		self.EventValue = value
		print "\n \n Event:  ALMotion/Safety/MoveFailed callback: "
		print "key %s" % str(strVarName)
		print "value %s" % str(value)
		print ""
		self.MoveIsFailed = True
		# self.event_trigger = threading.Event()
		# self.naoPause_ID = thread.start_new_thread(self.NaoPause, ())

		# self.avoide_ID = thread.start_new_thread(self.avoideObstacle, ())
		#self.avoideObstacle()
		#self.avoideObstacleSimple()

		#self.prox_memory.unsubscribeToEvent("ALMotion/Safety/MoveFailed",
			# Left bumper pressed
	def LeftBumperPressed(self, strVarName, value,message):
		# get actual NaoPause trigger value
		#param_trig = rospy.get_param('/rapp_NaoMove/NaoMovePause/trigger')
		# stop NaoPause
		#rospy.set_param('/rapp_NaoMove/NaoMovePause/trigger', False)
		# pause avoiding
		#print "L BUMPER PRESSED avoiding pause param_trig: %s" %param_trig

		self.event_trigger.clear()
		# move backwords 
		self.proxy_motion.moveToward(0,0,0)		
		self.proxy_motion.moveToward(-0.4,0,0)
		rospy.sleep(4)
		self.proxy_motion.moveToward(0,0,0)		
		#self.prox_memory.subscribeToEvent("ALMotion/Safety/MoveFailed", self.moduleName, "MoveCallback") < - - - - -  commented at home
		# wait avoide is being killed
		rospy.sleep(1)
		# set NaoPause trigger value as it was before bumper has been pressed

		#rospy.set_param('/rapp_NaoMove/NaoMovePause/trigger', param_trig)
		# resume avoideing
		self.event_trigger.set()
		print "L BUMPER PRESSED avoiding resume"
			# Right bumper pressed
	def RightBumperPressed(self, strVarName, value,message):
		# get actual NaoPause trigger value
		#param_trig = rospy.get_param('/rapp_NaoMove/NaoMovePause/trigger')
		# stop NaoPause
		#rospy.set_param('/rapp_NaoMove/NaoMovePause/trigger', False)

		# pause avoiding
		#print "R BUMPER PRESSED avoiding pause param_trig: %s" %param_trig

		self.event_trigger.clear()
		# move backwords 
		self.proxy_motion.moveToward(0,0,0)	
		rospy.sleep(1.5)
	
		self.proxy_motion.moveToward(-0.4,0,0)
		rospy.sleep(4)
		self.proxy_motion.moveToward(0,0,0)		
		#self.prox_memory.subscribeToEvent("ALMotion/Safety/MoveFailed", self.moduleName, "MoveCallback") < - - - - -  commented at home
		# wait avoide is being killed
		rospy.sleep(1)
		# set NaoPause trigger value as it was before bumper has been pressed

		#rospy.set_param('/rapp_NaoMove/NaoMovePause/trigger', param_trig)
		# resume avoideing
		#self.event_trigger.set()
		print "R BUMPER PRESSED avoiding resume"
	
	####
	##  Navigation methods
	####

	def ObstacleCallback(self, strVarName, value,message):

		print "\n \n Event:  Navigation/AvoidanceNavigator/ObstacleDetected callback: "
		print "key %s" % str(strVarName)
		print "value %s" % str(value)
		print ""

	def avoiedObstacleSimple(self):

		sonar_data = getSonarData()

		while sonar_data[1]>sonar_data[0]:
			self.proxy_motion.moveToward(0, 1, 0)

	def avoideObstacle(self):
		#self.last_avoidance_indent = thread.get_ident()
		#print "Thread ID: %s" %self.last_avoidance_indent
		Obs_flag = 0
		#self.destinationX = 1
		#self.destinationY = 1
		#self.destinationTheta = 0.5
		GoalGlobalPose=self.getGoalGlobalPose()
		EnvirementScenario=self.getEnvirementScenario()
		if EnvirementScenario == "Left1":
		 	self.avoid1L()
		if  EnvirementScenario == "Right1":
		 	self.avoid1R()
		if  EnvirementScenario == "Left2":
			self.avoid2L()
		if  EnvirementScenario == "Right2":
		 	self.avoid2R()
	 	print "[avoideObstacle] SCENARIO EXITED: \n %s" %EnvirementScenario
		
		rospy.set_param('/rapp_NaoMove/NaoMovePause/trigger', False)
		rospy.sleep(1)
		GoalNewLocalPose = self.getGoalNewLocalPose(GoalGlobalPose)
		print "GoalGlobalPose.pose.orientation \n \n : %s" %GoalGlobalPose.pose.orientation
		print "Rotating to GOAL orientation \n \n : %s" %tf.transformations.euler_from_quaternion(GoalGlobalPose.pose.orientation)[2]
		#self.proxy_motion.moveTo(0,0,tf.transformations.euler_from_quaternion(GoalGlobalPose.pose.orientation)[2])
		self.proxy_motion.setExternalCollisionProtectionEnabled('Move', True)
		#self.event_trigger.release()
		self.prox_memory.subscribeToEvent("ALMotion/Safety/MoveFailed", self.moduleName, "MoveCallback")
		self.proxy_motion.moveTo(GoalNewLocalPose.pose.position[0],GoalNewLocalPose.pose.position[1],0)# tf.transformations.euler_from_quaternion(GoalNewLocalPose.pose.orientation)[2])
		self.proxy_motion.waitUntilMoveIsFinished()
		rospy.set_param('/rapp_NaoMove/NaoMovePause/trigger', True)

	def setStartCoordinateFrame(self):
		self.StartRobotPosition = m.Pose2D(self.proxy_motion.getRobotPosition(True))

	def getStartCoordinateFrame(self):
		return self.StartRobotPosition

	def getEnvirementScenario(self):
		sonar_data = self.getSonarData()
		print "SOOOOOOOOOOOOOOOOONAAAARRRR: ",sonar_data
		###  obstacle on the left side 
		if (sonar_data[0]> 0.9):
			print "getEnvirementScenario: Left1"
			return "Left1"
		###  obstacle on the right side 
		elif(sonar_data[1]> 0.9 ):
			print "getEnvirementScenario: Right1"
			return "Right1"
		###  obstacle in front && left is closer
		elif (sonar_data[1]>=sonar_data[0] ):
			print "getEnvirementScenario: Left2"
			return "Left2"
		###  obstacle in front && right is closer
		elif (sonar_data[0]>sonar_data[1] ):
			print "getEnvirementScenario: Right2"
			return "Right2"
		else :
			print "getEnvirementScenario: goodToGo"
			return "goodToGo"

	def avoid1L(self):
		i=0
		sonar_data = self.getSonarData()
		while i <7 :

			self.proxy_motion.moveToward(0, -0.7, 0)
			#rospy.sleep(5)
			sonar_data = self.getSonarData()
			print "[Left1] - left sonar data: %s" %sonar_data[1]
			if sonar_data[1]>0.9:
				i=i+1
			if sonar_data[0]<0.7:
				print "[Left1] - right sonar too close"
				self.followRightUntilLeftFree(0.5,0.7)
				

				break
				#-----------------------------------------------------  SPRAWDZIC
		if sonar_data[1] >0.7 and sonar_data[0] >0.7:
			print "[Left1] - right & left sonars are >0.7, moving toward 10 secs"

			self.proxy_motion.moveToward(0.4,0, 0)
			rospy.sleep(10)
		elif (sonar_data[0]<0.7):
			print "[Left1] - left sonar > 0.7, right sonar < 0.7 -> starting LEFT2"

			self.avoid2L()

	def avoid1R(self):
		sonar_data = self.getSonarData()

		print "[Right1] - right sonar data: %s" %sonar_data[0]
		while sonar_data[0]<0.7:
			self.proxy_motion.moveToward(0, 0.1, 0)
			sonar_data = self.getSonarData()
			if sonar_data[1]<0.7:
				self.followLeftUntilRightFree(0.5,0.7)

				break
		if sonar_data[0] >0.7 and sonar_data[1] >0.7:
			self.proxy_motion.moveToward(0.4,0, 0)
			rospy.sleep(5)
		elif (sonar_data[1]<0.7):
			self.avoid2R()

	def avoid2L(self):
		checkGoalTheta = "none"#self.checkGoalTheta("left")
		print "[Left2] - checkGoalTheta :%s" %checkGoalTheta

		while checkGoalTheta != "goToGoal":
			print "[Left2] - rotating and following wall until checkGoalTheta == goToGoal"
			sonar_data = self.getSonarData()
			if sonar_data[1]< 0.9: 
				self.rotate("right")
			endWall = self.followWall("left")
			if endWall == "corner":
				self.turn("left")
				checkGoalTheta = self.checkGoalTheta("left")

	def avoid2R(self):
		checkGoalTheta = "none"#self.checkGoalTheta("left")
		print "[Right2] - checkGoalTheta :%s" %checkGoalTheta

		while checkGoalTheta != "goToGoal":
			print "[Right2] - rotating and following wall until checkGoalTheta == goToGoal"
			sonar_data = self.getSonarData()
			if sonar_data[0]< 0.9: 
				self.rotate("left")
			endWall = self.followWall("right")
			if endWall == "corner":
				self.turn("right")
				#self.rotate("right")
				checkGoalTheta = self.checkGoalTheta("right")


		# checkGoalTheta = self.checkGoalTheta("right")
		# print "[Right2] - checkGoalTheta :%s" %checkGoalTheta
		# while checkGoalTheta != "goToGoal":
		# 	print "[Right2] - rotating and following wall until checkGoalTheta == goToGoal"

		# 	self.rotate("left")
		# 	endWall = self.followWall("right")
		# 	if endWall == "corner":
		# 		self.turn("right")
		# 		checkGoalTheta = self.checkGoalTheta("right")

	def rotate(self,direction):
		print "[Rotate] Direction:  %s" %direction
		if self.AvoideObstacle == False:
			print "[Move server] - Bumper pressed avoideObstacle finished, starting new one."
			thread.exit()

		if direction == "right":
			theta =-0.3
		else:
			theta = 0.3
		sonar_data = self.getSonarData()
		while (sonar_data[0]<0.6 or sonar_data[1]<0.6):
			print "[Rotate] - rotating until right & left > 0.6 \n %s" %sonar_data

			self.proxy_motion.moveToward(0, 0,theta)
			rospy.sleep(1)
			sonar_data = self.getSonarData()

	def followWall(self,direction):
		i=0
		sonar_data = self.getSonarData()

		if direction == "left":
			data = sonar_data[1]
			data_old = data 

		if direction == "right":
			data = sonar_data[0]
			data_old = data 
		checkGoalTheta = "none"
		while checkGoalTheta != "goToGoal" and direction == "left":
			print "[followWall] - following until corner (or) checkGoalTheta == goToGoal direction: %s" %direction
			print "[followWall] - data = %s followint %s wall" %(data,direction)
			if data > data_old + 0.25:
				corner = True
				break
			if data>1.5:
				data=1.5
			if data <0.5:
				data=0.5
			self.proxy_motion.moveToward(0.5, 0,(data-1)*2)

			#rospy.sleep(0.5)
			data_old = data	

			data = self.getSonarData()[1]


			i=i+1
			if i> 100:
				checkGoalTheta = self.checkGoalTheta("left")
				print "[followWall] - checkGoalTheta = %s" %checkGoalTheta
				i=0
		while checkGoalTheta != "goToGoal" and direction == "right":
			print "[followWall] - following until %s > 0.7 or checkGoalTheta == goToGoal" %direction
			print "[followWall] - data = %s followint %s wall" %(data,direction)
			if data_old < data + 0.25:
				corner = True
				break
			if data>1.5:
				data=1.5
			if data <0.5:
				data=0.5
			self.proxy_motion.moveToward(0.5, 0,(1-data)*2)
			#rospy.sleep(0.5)
			data_old = data		
			data = self.getSonarData()[0]


			i=i+1
			if i> 15:
				checkGoalTheta = self.checkGoalTheta("right")
				print "[followWall] - checkGoalTheta = %s" %checkGoalTheta
				i=0
		if checkGoalTheta == "goToGoal":
			print "[followWall] - returning = %s" %checkGoalTheta

			return checkGoalTheta
		else: 
			print "[followWall] - returning =  corner \ndata: %s" %(data)

			return "corner"

	def turn(self,direction):
		print "[turning] --------------- %s" %direction
		sonar_data_old = 0
		if direction == "left":
			#self.proxy_motion.moveToward(0, -0.7,0)
			#rospy.sleep(4)
			self.proxy_motion.moveToward(0.8, 0,0)
			rospy.sleep(5)
			sonar_data = self.getSonarData()

			while ( sonar_data[1]>0.7): # or sonar_data[1]> sonar_data_old ):
				print "[turning] left sonar %s" %sonar_data[1]

				sonar_data_old = sonar_data[1]
				sonar_data = self.getSonarData()
				self.proxy_motion.moveToward(0, 0,0.4)
				rospy.sleep(0.1)
		else:
			self.proxy_motion.moveToward(0.8, 0,0)
			rospy.sleep(2)
			sonar_data = self.getSonarData()

			while ( sonar_data[0]>0.7): # or sonar_data[0]> sonar_data_old ):
				print "[turning] right sonar %s" %sonar_data[0]

				sonar_data_old = sonar_data[0]
				sonar_data = self.getSonarData()
				self.proxy_motion.moveToward(0, 0,-0.4)
				rospy.sleep(0.1)

			self.proxy_motion.moveToward(0.8, 0,0)
			rospy.sleep(2)

	def checkGoalTheta(self,direction):
		# if self.AvoideObstacle == False:
		# 	print "[Move server] - Bumper pressed avoideObstacle finished, starting new one."
		# 	thread.exit()

		NLP=self.getGoalNewLocalPose(self.GoalGlobalPose)
		ActualRobotPosition = m.Pose2D(self.proxy_motion.getRobotPosition(True))
		GoalGlobalPose = self.getGoalGlobalPose()


		# when goal is on X axis (negative), theta sign is changing in NaoQI -> this exception has to be caught
		theta_change = False



		x_goal = GoalGlobalPose.pose.position[0]
		y_goal = GoalGlobalPose.pose.position[1]
		theta_goal = tf.transformations.euler_from_quaternion(GoalGlobalPose.pose.orientation)[2]

		x=ActualRobotPosition.x
		y=ActualRobotPosition.y
		theta=ActualRobotPosition.theta


					# <---------------------------------- DOPRACOWAC gdy x=x_goal i gdy x = 0
		


		if (x-x_goal!=0 and x!=0):
			
			#y=a*x+b
			#b=y-a*x
			b=(y_goal*x-y*x_goal)/(x-x_goal)
			a=(y-b)/x
			print "[checkGoalTheta] direction : %s \n a = %s" %(direction,a)
			##
			#  1. & 2. quadrants of map frame (or) goal is on X axis 
			##
			if (y_goal-y >0):
				##
				#   1. quadrant in relation to map frame 
				##
				if x_goal-x>0:
					print "\t\t1. quadrant"
					alpha =numpy.arctan(a) 

				##
				#   2. quadrant in relation to map frame
				##
				
				elif x_goal-x<0:
					print "\t\t2. quadrant"

					alpha =numpy.pi + numpy.arctan(a)
				#
				#  goal is on Y axis 
				else :
					# Y axis (positive)
					print "\t\tY axis positive"

					alpha = 0
	
			##
			#  3. & 4. quadrants of map frame
			##				

			elif (y_goal-y < 0):
				##
				#   3. quadrant in relation to map frame
				##
				if x_goal-x<0:
					print "\t\t3. quadrant"

					alpha =-numpy.pi - numpy.arctan(a)
				##
				#   4. quadrant in relation to map frame 
				##
				elif x_goal-x>0:
					print "\t\t4. quadrant"

					alpha =numpy.arctan(a)
				
				#  goal is on Y axis (negative)

				else:
					print "\t\tY axis negative"

					alpha = -numpy.pi/2
			##
			#   goal is on X axis
			##
			else: 
				print "\t\tX axis"

				print "y_goal -y: %s" %(y_goal -y)
				# positive part
				if x_goal-x>0:
					print "\t\tX axis positive"

					alpha =0
				# negative part
				else:
					print "\t\tX axis negative"

					alpha =numpy.pi
					theta_change = True # < --- EXCEPTION!!!! SIGN CHANGE!!!

		# goal is on Y axis in relation to map frame 
		elif x - x_goal==0 and x!=0:
			
			# positive part
			if y_goal-y>0:
				alpha = numpy.pi/2
			# negative part
			if y_goal-y<0:
				alpha = -numpy.pi/2
		# Nao is in the goal posision
		elif x==x_goal and y==y_goal:
			print "[checkGoalTheta] - goToGoal - Nao is in the goal posision"
			return "goToGoal"			
		# x == 0 (actual Nao position)

		else :

			print "[checkGoalTheta] - Actual Nao posotion in map frame disables computation of GoalTheta \n x = 0"
			return "Failed"


		if ((direction == "right" and theta <= alpha) or (direction == "left" and theta >= alpha)) and not theta_change:
			beta = alpha-theta
			print "[checkGoalTheta] - goToGoal, rotating toward goal"
			self.proxy_motion.moveTo(0,0,beta)
			self.proxy_motion.waitUntilMoveIsFinished()
			return "goToGoal"
		#
		# theta_change = True there is exception !!!
		#
		elif theta_change :#and (direction == "right" and theta >= alpha) or (direction == "left" and theta <= alpha):
			print "[checkGoalTheta] - THETA CHANGE"
			print "[checkGoalTheta] - THETA CHANGE"

			print "[checkGoalTheta] - THETA CHANGE"

			print "[checkGoalTheta] - THETA CHANGE"

			print "[checkGoalTheta] - THETA CHANGE"

			if direction == "right":
				beta = alpha-theta
				self.proxy_motion.moveTo(0,0,beta)
				self.proxy_motion.waitUntilMoveIsFinished()

			if direction == "left":
				beta = -alpha-theta

				self.proxy_motion.moveTo(0,0,beta)
				self.proxy_motion.waitUntilMoveIsFinished()
			return "goToGoal"
		
		else:
			print "[checkGoalTheta] - GoalNotClear - continue avoiding"
			return "GoalNotClear"

	def followRightUntilLeftFree(self,distR, distL):
		sonar_data = self.getSonarData()
		print "followRightUntilLeftFree : %s" %sonar_data[1]
		
		while sonar_data[1] < distL:
			sonar_data = self.getSonarData()

			if sonar_data[0]>distR: # [0] - right,  [1] - left
				#avoide_direction = "left"
				self.proxy_motion.moveToward(0.2, 0, -sonar_data[0]/6)
			 		
		 		if sonar_data[1] < 0.40:

		 			self.proxy_motion.moveToward(0.2, 0, sonar_data[0]/6)
	 				print "WATCH OUT - LEFT OBSTACLE"
			else:
				self.proxy_motion.moveToward(0.2, 0, 1-sonar_data[0]*sonar_data[0])# 0.9/1.1-sonar_data[0] )

	def followLeftUntilRightFree(self, distL, distR):
		sonar_data = self.getSonarData()

		print "followLeftUntilRightFree : %s" %sonar_data[0]
		while sonar_data[1] < distR:
	 		sonar_data = self.getSonarData()

			if sonar_data[1]>distL: # [0] - right,  [1] - left
			#avoide_direction = "right"
				self.proxy_motion.moveToward(0.2, 0, sonar_data[1]/6)
					
				if sonar_data[0] < 0.40 and sonar_data[0] :

					self.proxy_motion.moveToward(0.2, 0, -sonar_data[1]/6)
					print "WATCH OUT - RIGHT OBSTACLE"

				else:
					#avoide_direction = "right"
			 		self.proxy_motion.moveToward(0.2, 0, -(1-sonar_data[1]*sonar_data[1]) )

		### 
		#duza przeszkoda:
		#  obracaj sie w odpowiednia strone az jeden sonar bedzie wolny, idz do przodu az drugi sonar bedzie wolny,
		#  skrec w ta strone z ktorej uwolnil sie sonar, idz prosto
		# mala:  Kieruj sie non stop na mete, az oba sonary beda wolne.

	####
	##  thread operating
	####
	def NaoPause(self):
		print "NaoPause - starts"
		#param_trig = rospy.get_param('/rapp_NaoMove/NaoMovePause/trigger')
		#param_move = rospy.get_param('/rapp_NaoMove/NaoMovePause/move')
		#param_sleep = rospy.get_param('/rapp_NaoMove/NaoMovePause/sleep')
		while True:
			self.event_trigger.set()
			if param_trig == True: #<---------------------------------------------------- moze byc potrzeba zmiany na stringa

				self.event_trigger.set()#set()
				rospy.sleep(param_move)
				self.event_trigger.clear()#set()
				self.proxy_motion.moveToward(0,0,0)		

				rospy.sleep(param_sleep)

			# if param_trig == True: #<---------------------------------------------------- moze byc potrzeba zmiany na stringa
			# 	self.event_trigger.set()
			# 	rospy.sleep(param_move)
			# 	print "NAoPause - przed clear: move: %s     sleep: %s" %(	param_move,	param_sleep )

			# 	self.event_trigger.clear()
			# 	rospy.sleep(param_sleep)
			#param_trig = rospy.get_param('/rapp_NaoMove/NaoMovePause/trigger')
			#param_move = rospy.get_param('/rapp_NaoMove/NaoMovePause/move')
			#param_sleep = rospy.get_param('/rapp_NaoMove/NaoMovePause/sleep')


	# OLD METHOD
 	def followObstaclesBoundary(self,direction):
 		k=0.1
 		i=0
 		#l=0
 		print min(self.sonar_data)
 		self.proxy_motion.setWalkArmsEnabled(True, True)

 	 	#while (k >0 and k<0.8 and (i < 5 )):#or l <5)):
	 	 			
  		sonar_data = self.getSonarData()
 	 		#      |  <-sonar_left_distance 
 	 		#      |  ___________                 AVOIDE LEFT DIRACTION
  		#      |  \ <- sonar_right_distance  
 	 		#      | / \
 	 		#      |/   \
 	 		#      *
  		if sonar_data[1]>sonar_data[0]: # [0] - right,  [1] - left
  			#avoide_direction = "left"
  			theta = 0.1
  			i=0
  			while i<2:
  				sonar_data = self.getSonarData()
  				self.proxy_motion.moveToward(0, 0, theta)
  				if sonar_data[1]>0.7 and sonar_data[0]>0.7:
  					i=i+1
 	 	else:

  			#avoide_direction = "right"
  			theta = -0.1
  			i=0
  			while i<2:
  				sonar_data = self.getSonarData()
 				self.proxy_motion.moveToward(0, 0, theta)
 				if sonar_data[0]>0.7 and sonar_data[1]>0.7:
 					i=i+1
 			

 	 		#k = min(sonar_data)
 	 		# print sonar_data
 	 		# if k == sonar_data[0]:
 	 		# 	print "\n ROTATE:\n right sonar : %s" %k
 	 		# else :
 	 		# 	print "\n left sonar : %s" %k
 	 		# if (k > 1):# and theta == 0.1):
 	 		# 	i=i+1
 	 		# 	#l=0
 	 		# if (k > 1 and theta == -0.1):
 	 		# 	l=l+1
 	 		# 	i=0
 	 	print "STOP theta: %s" %theta
 	 	Xvel = 0.4
 	 	maxDist = 0.75
 	 	while True:
 	 		sonar_data = self.getSonarData()
 	 		if theta >0:
 	 			print "right sonar : %s" %sonar_data[0]
 	 			print "left sonar : %s" %sonar_data[1]
 		 		if sonar_data[0]>maxDist: # [0] - right,  [1] - left
 		 			#avoide_direction = "left"
 			 		self.proxy_motion.moveToward(0.4, 0, -sonar_data[0]/4)

			 		
 			 		# if sonar_data[1] < 0.30:
			 	
 			 		# 	self.proxy_motion.moveToward(0.4, 0, sonar_data[0]/4)
 			 		# 	print "WATCH OUT - LEFT OBSTACLE"


 		 		else:
 			 		self.proxy_motion.moveToward(0.4, 0, 1-sonar_data[0]*sonar_data[0])# 0.9/1.1-sonar_data[0] )
 					if sonar_data[1] < 0.40:
 			 			self.followLeftUntilRightFree(0.4,0.6)
 	 		else:
 		 		if sonar_data[1]>maxDist: # [0] - right,  [1] - left
 		 			#avoide_direction = "left"
 					print "dociskamy \n : %s" %(sonar_data[1]) 
 					print "kat: %s" %(sonar_data[1]/5)#(maxDist-sonar_data[1])*(maxDist-sonar_data[1]))
 					if sonar_data[1] > 2:
 						sonar_data[1] = 2
 			 		self.proxy_motion.moveToward(0.4, 0, sonar_data[1]/2)#(-maxDist+sonar_data[1])*(-maxDist+sonar_data[1]))

 			 		# if sonar_data[0] < 0.40 and sonar_data[0] :
 			 		# 	self.followRight(0.4)
 			 		# 	trzymaj sie prawej az lewa sie zwolni
 			 		# 	self.proxy_motion.moveToward(0.4, 0, -sonar_data[1]/4)
 						# print "WATCH OUT - RIGHT OBSTACLE"
 		 		else:
 		 			#avoide_direction = "right"
 		 			print "odpychamy %s \n " %(sonar_data[1]) 
 					print "kat: %s" %(-(maxDist-sonar_data[1])*(maxDist-sonar_data[1]))
 			 		self.proxy_motion.moveToward(0.4, 0, -maxDist/sonar_data[1]-(maxDist-2*sonar_data[1]/3)*(maxDist-2*sonar_data[1]/3))
 			 		if sonar_data[0] < maxDist/2:
 			 			print "too close"
 			 			print "too close"
 			 			print "too close"
 			 			print "too close"
 			 			print "too close"
 			 			print "too close"
 			 			print "too close"
 			 			print "too close"
 			 			print "too close"
 			 			print "too close"
		 			
 			 			self.followRightUntilLeftFree(0.5,0.8)

	
 	 	# while (k >0 and k<1):
	 	 			
 	 	# 	sonar_data = self.getSonarData()
 	 	# 	#      |  <-sonar_left_distance 
 	 	# 	#      |  ___________                 AVOIDE LEFT DIRACTION
 	 	# 	#      |  \ <- sonar_right_distance  
 	 	# 	#      | / \
	 	# 	#      |/   \
 	 	# 	#      *
 	 	# 	if sonar_data[1]>sonar_data[0]: # [0] - right,  [1] - left
 	 	# 		#avoide_direction = "left"
 	 	# 		theta = 0.1
 	 	# 	else:
 	 	# 		#avoide_direction = "right"
 	 	# 		theta = -0.1
 			# self.proxy_motion.moveToward(0, 0, theta)
 	 	# 	k = min(sonar_data)
 	 	# 	print k


 	 	self.proxy_motion.moveToward(0, 0, theta)
 	 	self.proxy_motion.stopMove()




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