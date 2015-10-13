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
from geometry_msgs.msg import PolygonStamped, Point32

import tf.transformations
#import transformPose

# Importing core functionality from Naoqi
from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule
import motion
import threading 
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
		# self.compass = ALProxy("ALVisualCompass")
		# if self.compass is None:
		# 	rospy.logerr("[Move server] - Could not get a proxy to ALVisualCompass")
		# 	exit(1)
	# Setting variables
	def setVariables(self):
		print "[Move server] - Setting variables"
		self.MoveIsFailed = False
		self.GP_seq = -1
		self.tl = tf.TransformListener(True, rospy.Duration(5.0))
		
	# NAOqi Event subscribtion
	def subscribeToEvents(self):
		#self.prox_memory.subscribeToEvent("ALMotion/Safety/MoveFailed", self.moduleName, "MoveCallback")
		#self.prox_memory.subscribeToEvent("LeftBumperPressed", self.moduleName, "LeftBumperPressed")
		#self.prox_memory.subscribeToEvent("RightBumperPressed", self.moduleName, "RightBumperPressed")
		#self.avoide_ID = 0
		#self.proxy_sonar.subscribe("obstacleAvoidance")
		#self.prox_memory.subscribeToEvent("Navigation/AvoidanceNavigator/ObstacleDetected", self.moduleName, "ObstacleCallback")
	# Initialization of ROS services
		print "subscribing to events"
	def openServices(self):
		try:
			print "[Move server] - setting services"
			print "[Move server] - service - [rapp_plannPath]"
			self.service_pp = rospy.Service('rapp_plannPath', PlannPath, self.plannPath)
		except Exception, ex_pp:
			print "[Move server] - Exception %s" % str(ex_pp)
		try:
			print "[Move server] - setting services"
			print "[Move server] - service - [rapp_MoveAlongPath]"
			self.service_map = rospy.Service('rapp_moveAlongPath', MoveAlongPath, self.handle_rapp_MoveAlongPath)
		except Exception, ex_map:
			print "[Move server] - Exception %s" % str(ex_map)
		try:
			print "[Move server] - setting services"
			print "[Move server] - service - [rapp_MoveTo]"
			self.service_mt = rospy.Service('rapp_moveTo', MoveTo, self.handle_rapp_moveTo)
		except Exception, ex_mt:
			print "[Move server] - Exception %s" % str(ex_mt)
		try:
			print "[Move server] - service - [rapp_moveVel]"
			self.service_mv = rospy.Service('rapp_moveVel', MoveVel, self.handle_rapp_moveVel)
		except Exception, ex_mv:
			print "[Move server] - Exception %s" % str(ex_mv)
		try:
			print "[Move server] - service - [rapp_moveHead]"
			self.service_mh = rospy.Service('rapp_moveHead', MoveHead, self.handle_rapp_moveHead)
		except Exception, ex_mh:
			print "[Move server] - Exception %s" % str(ex_mh)
		try:
			print "[Move server] - service - [rapp_moveStop]"
			self.service_mh = rospy.Service('rapp_moveStop', MoveStop, self.handle_rapp_moveStop)
		except Exception, ex_mh:
			print "[Move server] - Exception %s" % str(ex_mh)
		try:
			print "[Move server] - service - [rapp_moveGetCollisionStatus]"
			self.service_mh = rospy.Service('rapp_moveGetCollisionStatus', MoveGetCollisionStatus, self.handle_rapp_moveGetCollisionStatus)
		except Exception, ex_mh:
			print "[Move server] - Exception %s" % str(ex_mh)	
		try:
			print "[Move server] - service - [rapp_moveJoint]"
			self.service_mh = rospy.Service('rapp_moveJoint', MoveJoint, self.handle_rapp_moveJoint)
		except Exception, ex_mh:
			print "[Move server] - Exception %s" % str(ex_mh)	
		try:
			print "[Move server] - service - [rapp_removeStiffness]"
			self.service_mh = rospy.Service('rapp_removeStiffness', RemoveStiffness, self.handle_rapp_removeStiffness)
		except Exception, ex_mh:
			print "[Move server] - Exception %s" % str(ex_mh)
			
		try:
			print "[Move server] - service - [rapp_takePredefinedPosture]"
			self.service_mh = rospy.Service('rapp_takePredefinedPosture', TakePredefinedPosture, self.handle_rapp_takePredefinedPosture)
		except Exception, ex_mh:
			print "[Move server] - Exception %s" % str(ex_mh)
		try:
			print "[Move server] - service - [rapp_lookAtPoint]"
			self.service_mh = rospy.Service('rapp_lookAtPoint', LookAtPoint, self.handle_rapp_lookAtPoint)
		except Exception, ex_mh:
			print "[Move server] - Exception %s" % str(ex_mh)
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
	def handle_rapp_moveTo(self,req):
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



	def handle_rapp_MoveAlongPath(self,req):
		self.SetPose('StandInit')
		naoCurrentPosition = self.getNaoCurrentPosition()
		nao_theta = tf.transformations.euler_from_quaternion(naoCurrentPosition[1])[2]
		# destinationX=req.destination_x
		# destinationY=req.destination_y
		# destinationTheta=req.destination_theta
		# GoalGlobalPose = [destinationX,destinationY,destinationTheta]
		# flag for detectObstacle
		self.move_is_finished = False
		self.path_is_finished = False
		self.kill_thread_followPath = False
		self.obstacle_detected = False
		print "[Move server] - waiting for makePlan service"
		rospy.wait_for_service('/global_planner/make_plan')
		print "[Move server] - makePlan service found"
		
		self.followPath_flag = 'empty'
		rate_mainThread = rospy.Rate(1)
		noPathavaliable = False
		plannNewPath = rospy.ServiceProxy('rapp_plannPath', PlannPath)
		path = plannNewPath(naoCurrentPosition[0][0],naoCurrentPosition[0][1],nao_theta,req.x,req.y,req.theta)

		#path = req.path
		# OBSLUGA SCIEZKI

		#while self.path_is_finished != True:
		# path = self.plannPath(GoalGlobalPose)
		# if len(path.path)<3:
		# 	noPathavaliable = True
		# 	break

		self.thread_followPath = threading.Thread(None,self.followPath,None,[path.path])
		thread_detectObstacle = threading.Thread(None,self.detectObstacle,None)	
			# # # zanim zaczniesz ruch ustaw watek do sprawdzania sonarow
		#thread_detectObstacle.start()
		self.thread_followPath.start()
		#self.followPath_flag = self.followPath(path)
		while self.path_is_finished == False and self.obstacle_detected == False:
			rate_mainThread.sleep()		#wait to nao stop move
		if self.path_is_finished == True:
			self.kill_thread_detectObstacle = True
			#thread_detectObstacle.join()
			print "destination reached, and obstacle detection is:\n",thread_detectObstacle.isAlive()
			print "destination reached, and following path is:\n",self.thread_followPath.isAlive()
			status = 0

		else:
			self.kill_thread_followPath = True
			#thread_detectObstacle.join()
			print "OBSTACLE DETECTED, path following is:\n",self.thread_followPath.isAlive()
			print "OBSTACLE DETECTED, obstacle detection is:\n",thread_detectObstacle.isAlive()
			#self.followObstaclesBoundary2()
			print "BUG BUG BUG BUG BUG BUG BUG BUG \n BUG BUG BUG BUG BUG BUG"
			status = 1
		# if noPathavaliable == True:
		# 	status = "no path avaliable"
		# else:
		# 	status = "move finished"

		#self.move_is_finished = True

		return MoveToResponse(status)


	def plannPath(self,req):
		naoCurrentPosition = [req.start_x,req.start_y,req.start_theta]#self.getNaoCurrentPosition()
		start = PoseStamped()
		goal = PoseStamped()
		start.header.seq = 0
		goal.header.seq = 0
		start.header.stamp = rospy.Time.now()
		goal.header.stamp = rospy.Time.now()
		start.header.frame_id = "/map"
		goal.header.frame_id = "/map"
		start.pose.position.x = req.start_x#naoCurrentPosition[0][0]
		start.pose.position.y = req.start_y#naoCurrentPosition[0][1]
		start.pose.position.z = 0#naoCurrentPosition[0][2]
		start_orientation_quaternion = tf.transformations.quaternion_from_euler(0,0,req.start_theta) 

		start.pose.orientation.x = start_orientation_quaternion[0]
		start.pose.orientation.y = start_orientation_quaternion[1]
		start.pose.orientation.z = start_orientation_quaternion[2]
		start.pose.orientation.w = start_orientation_quaternion[3]
		goal.pose.position.x = req.finish_x
		goal.pose.position.y = req.finish_y
		goal.pose.position.z = 0
		goal_orientation_quaternion = tf.transformations.quaternion_from_euler(0,0,req.finish_theta) 
		goal.pose.orientation.x = goal_orientation_quaternion[0]
		goal.pose.orientation.y = goal_orientation_quaternion[1]
		goal.pose.orientation.z = goal_orientation_quaternion[2]
		goal.pose.orientation.w = goal_orientation_quaternion[3]
		print "tu jest start \n",start
		print "tu jest goal \n",goal
		path = numpy.array(PoseStamped())
		print "sdsanaskd"
		plan_path = rospy.ServiceProxy('/global_planner/make_plan', MakeNavPlan)
		print "okmpmmlkmvxc ssdf "
		path = plan_path(start,goal)
		print "response:", path
		return PlannPathResponse(path)

#proporcjonalna odleglosc miedzy punktami
	# def followPath(self,path):			
		
	# 	for i in range(int(numpy.floor(len(path)-1)/((len(path))*0.05))+3):
	# 	#int(numpy.floor(len(path.path)/200))+1):
	# 		print "liczba petli :\n",int(numpy.floor(len(path)-1)/(len(path)*0.05))+3
	# 		print "liczba punktow: \n", len(path)
	# 		rospy.sleep(3)
	# 		naoCurrentPosition = self.getNaoCurrentPosition()
	# 		robot_orientation_euler = tf.transformations.euler_from_quaternion(naoCurrentPosition[1])
	# 		if (len(path)-1-(i+1)*int(numpy.floor(len(path))*0.05))<0:
	# 			point_number = len(path)-1
	# 		else:
	# 			point_number = ((i+1)*int(numpy.floor(len(path))*0.05))
# staly odstep miedzy punkami
	def followPath(self,path):			
		
		for i in range(int(numpy.ceil(len(path)/(20)))+1):
		#int(numpy.floor(len(path.path)/200))+1):
			print "i= ",i
			print "liczba punktow: \n", len(path)
			rospy.sleep(3)
			naoCurrentPosition = self.getNaoCurrentPosition()
			robot_orientation_euler = tf.transformations.euler_from_quaternion(naoCurrentPosition[1])
			if (len(path)-(i+1)*20<0.1):
				point_number = len(path)-1
			else:
				point_number = (i+1)*20


			nextPose = path[point_number]
			nextRotation = [nextPose.pose.orientation.x,nextPose.pose.orientation.y,nextPose.pose.orientation.z,nextPose.pose.orientation.w]
			nextPoseOrientationZ = tf.transformations.euler_from_quaternion(nextRotation)[2]#.x,nextPose.pose.orientation.y,nextPose.pose.orientation.z,nextPose.pose.orientation.w)[2]
			print "[Path tracker] - getting to next point:\n ", point_number," / ", (len(path)-1)
			print "start:\n ",naoCurrentPosition[0][0],naoCurrentPosition[0][1]
			print "finish:\n",nextPose.pose.position.x,nextPose.pose.position.y

			x_A = naoCurrentPosition[0][0]
			y_A = naoCurrentPosition[0][1]
			robot_orientation_euler = tf.transformations.euler_from_quaternion(naoCurrentPosition[1])
			gamma = robot_orientation_euler[2]
			x_B = nextPose.pose.position.x
			y_B = nextPose.pose.position.y
			AB = numpy.sqrt((x_A-x_B)*(x_A-x_B)+(y_A-y_B)*(y_A-y_B))
			nextPoseOrientationZ = tf.transformations.euler_from_quaternion(nextRotation)[2]#.x,nextPose.pose.orientation.y,nextPose.pose.orientation.z,nextPose.pose.orientation.w)[2]

			alpha = numpy.arctan2(y_B-y_A,x_B-x_A)
			#dist_Nao_trajectory = numpy.sqrt(()*()+()*())
			print "gamma|alpha\n", gamma," | ",alpha
			print "gamma|alpha", gamma," | ",nextPoseOrientationZ
			if abs(gamma)> abs(alpha):
			 	theta = -1*(gamma - alpha)
			elif abs(gamma)< abs(alpha):
				theta = (alpha - gamma)
			else:
				theta =0
			if abs(theta) > 3.14:
				print"\n theta > 3.14\n"
				theta = theta-(numpy.sign(theta)*2*numpy.pi)
			print "nawrotka na AB"
			print "theta= ",theta
			#if abs(theta) > 0.15:
			if (abs(theta) > 20*numpy.pi/180 and AB > 0.08) or (point_number == len(path)-1):
				thetaTime = abs(theta)/0.3
				self.rapp_move_vel_interface(0,0,0.3*numpy.sign(theta))
				thetaTime_now = 0
				while (thetaTime-thetaTime_now)>0:
					if self.kill_thread_followPath == True:
						break	
					rospy.sleep(0.1)
					
					thetaTime_now = thetaTime_now + 0.1
				self.handle_rapp_moveStop(0)
					#self.getch() 
			print "pojscie na AB"
			move_X_time = AB/0.03
			self.rapp_move_vel_interface(0.03,0,0)
			move_X_time_now = 0
			while (move_X_time-move_X_time_now)>0:
				if self.kill_thread_followPath == True:
					break	
				rospy.sleep(0.1)
				
				move_X_time_now = move_X_time_now + 0.1
			
			self.handle_rapp_moveStop(0)
			#self.getch() 

			# print "nawrotka na kierunek B"
			# theta2 = nextPoseOrientationZ - alpha
			# theta2_Time = abs(theta2)/0.2
			# self.proxy_motion.post.move(0,0,0.2*numpy.sign(theta2))
			# rospy.sleep(theta2_Time)

			if self.kill_thread_followPath == True:
				break
		if self.kill_thread_followPath == True:
			 self.path_is_finished = False
		else:
			print "nawrotka na kierunek koncowy"
			naoCurrentPosition = self.getNaoCurrentPosition()
			robot_orientation_euler = tf.transformations.euler_from_quaternion(naoCurrentPosition[1])
			print "last point orientation: \n", nextPoseOrientationZ
			theta2 = nextPoseOrientationZ - robot_orientation_euler[2]
			if abs(theta2) > 3.14:
				print"\n theta > 3.14\n"
				theta2 = theta2-(numpy.sign(theta2)*2*numpy.pi)

			theta2_Time = abs(theta2)/0.2
			self.proxy_motion.post.move(0,0,0.2*numpy.sign(theta2))
			rospy.sleep(theta2_Time)
			self.proxy_motion.post.move(0,0,0)

			self.path_is_finished = True
			print "PATH END"

	def handle_rapp_MoveTo2(self,req):
		self.SetPose('StandInit')
		self.getNaoCurrentPosition()
		destinationX=req.destination_x
		destinationY=req.destination_y
		destinationTheta=req.destination_theta
		GoalGlobalPose = [destinationX,destinationY,destinationTheta]
		# flag for detectObstacle
		self.move_is_finished = False
		self.path_is_finished = False
		self.lastTimePlanning = 0
		self.timeTrigger = False
		self.plannedAlready = False

		print "[Move server] - waiting for makePlan service"
		rospy.wait_for_service('/global_planner/make_plan')
		print "[Move server] - makePlan service found"
		
		self.followPath_flag = 'empty'
		rate_mainThread = rospy.Rate(1)
		noPathavaliable = False
		print "[Move server] - walk tajectory planning"

		while self.path_is_finished != True:
			self.path_is_finished = False
			self.obstacle_detected = False
			self.plannNewPath = False
			self.kill_thread_detectObstacle = False
			self.kill_thread_followPath = False
			self.noPathavaliable = False
			old_path = MakeNavPlanResponse()
			path = self.plannPath2(GoalGlobalPose)
			if len(path.path)<3:
				self.noPathavaliable = True
				if len(old_path.path) > 1:
					path = old_path
				else:
					break
			self.thread_followPath = threading.Thread(None,self.followPath2,None,[path.path])
			thread_detectObstacle = threading.Thread(None,self.detectObstacle,None)	

			# # # zanim zaczniesz ruch ustaw watek do sprawdzania sonarow
			#thread_detectObstacle.start()
			self.thread_followPath.start()
			#self.followPath_flag = self.followPath(path)
			while self.path_is_finished == False and self.obstacle_detected == False and self.plannNewPath==False:
				rate_mainThread.sleep()
			
			old_path = path
			
			#wait to nao stop move
			if self.path_is_finished == True:
				self.kill_thread_detectObstacle = True
				#thread_detectObstacle.join()
				print "destination reached, and obstacle detection is:\n",thread_detectObstacle.isAlive()
				print "destination reached, and following path is:\n",self.thread_followPath.isAlive()

			elif self.obstacle_detected == True:
				self.kill_thread_followPath = True
				#thread_detectObstacle.join()
				print "OBSTACLE DETECTED, path following is:\n",self.thread_followPath.isAlive()
				print "OBSTACLE DETECTED, obstacle detection is:\n",thread_detectObstacle.isAlive()
				#self.followObstaclesBoundary2()
				print "BUG BUG BUG BUG BUG BUG BUG BUG \n BUG BUG BUG BUG BUG BUG"
				break
			else:
				self.kill_thread_detectObstacle = True
				print "killing threads"
				print "destination reached, and obstacle detection is:\n",thread_detectObstacle.isAlive()
				print "destination reached, and following path is:\n",self.thread_followPath.isAlive()

		if self.noPathavaliable == True:
			status = "no path avaliable"
		else:
			status = "move finished"
		self.move_is_finished = True
		print "[Move server] - move finished"

		return MoveToResponse(status)


	def plannPath2(self,GoalGlobalPose):
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
		print "sdsanaskd"
		plan_path = rospy.ServiceProxy('/global_planner/make_plan', MakeNavPlan)
		print "okmpmmlkmvxc ssdf "
		path = plan_path(start,goal)
		print "mm,nk jnk jnj dnknsd"
		return path

#proporcjonalna odleglosc miedzy punktami
	# def followPath(self,path):			
		
	# 	for i in range(int(numpy.floor(len(path)-1)/((len(path))*0.05))+3):
	# 	#int(numpy.floor(len(path.path)/200))+1):
	# 		print "liczba petli :\n",int(numpy.floor(len(path)-1)/(len(path)*0.05))+3
	# 		print "liczba punktow: \n", len(path)
	# 		rospy.sleep(3)
	# 		naoCurrentPosition = self.getNaoCurrentPosition()
	# 		robot_orientation_euler = tf.transformations.euler_from_quaternion(naoCurrentPosition[1])
	# 		if (len(path)-1-(i+1)*int(numpy.floor(len(path))*0.05))<0:
	# 			point_number = len(path)-1
	# 		else:
	# 			point_number = ((i+1)*int(numpy.floor(len(path))*0.05))
# staly odstep miedzy punkami
	def canPlannPath2(self):
		print "path = self.plannPath(GoalGlobalPose)"
		print "pathLen + len(path)"

	def followPath2(self,path):			
		
		for i in range(int(numpy.ceil(len(path)/(20)))+1):
		#int(numpy.floor(len(path.path)/200))+1):
			print "i= ",i
			print "liczba punktow: \n", len(path)
			timeDelay =rospy.Time.now().secs -self.lastTimePlanning
			if self.timeTrigger == True and timeDelay<3:
				print "Delay   ",timeDelay
				rospy.sleep(3-timeDelay)
			elif self.timeTrigger == True and timeDelay>=3:
				print "Delay   ",timeDelay
			else:
				rospy.sleep(3)
			self.timeTrigger == False
			naoCurrentPosition = self.getNaoCurrentPosition()
			robot_orientation_euler = tf.transformations.euler_from_quaternion(naoCurrentPosition[1])
			if (len(path)-(i+1)*20<0.1):
				point_number = len(path)-1
			else:
				point_number = (i+1)*20


			nextPose = path[point_number]
			nextRotation = [nextPose.pose.orientation.x,nextPose.pose.orientation.y,nextPose.pose.orientation.z,nextPose.pose.orientation.w]
			nextPoseOrientationZ = tf.transformations.euler_from_quaternion(nextRotation)[2]#.x,nextPose.pose.orientation.y,nextPose.pose.orientation.z,nextPose.pose.orientation.w)[2]
			print "[Path tracker] - getting to next point:\n ", point_number," / ", (len(path)-1)
			print "start:\n ",naoCurrentPosition[0][0],naoCurrentPosition[0][1]
			print "finish:\n",nextPose.pose.position.x,nextPose.pose.position.y

			x_A = naoCurrentPosition[0][0]
			y_A = naoCurrentPosition[0][1]
			robot_orientation_euler = tf.transformations.euler_from_quaternion(naoCurrentPosition[1])
			gamma = robot_orientation_euler[2]
			x_B = nextPose.pose.position.x
			y_B = nextPose.pose.position.y
			AB = numpy.sqrt((x_A-x_B)*(x_A-x_B)+(y_A-y_B)*(y_A-y_B))
			nextPoseOrientationZ = tf.transformations.euler_from_quaternion(nextRotation)[2]#.x,nextPose.pose.orientation.y,nextPose.pose.orientation.z,nextPose.pose.orientation.w)[2]

			alpha = numpy.arctan2(y_B-y_A,x_B-x_A)
			#dist_Nao_trajectory = numpy.sqrt(()*()+()*())
			print "gamma|alpha\n", gamma," | ",alpha
			print "gamma|alpha", gamma," | ",nextPoseOrientationZ
			if abs(gamma)> abs(alpha):
			 	theta = -1*(gamma - alpha)
			elif abs(gamma)< abs(alpha):
				theta = (alpha - gamma)
			else:
				theta =0
			if abs(theta) > 3.14:
				print"\n theta > 3.14\n"
				theta = theta-(numpy.sign(theta)*2*numpy.pi)
			print "nawrotka na AB"
			print "theta= ",theta
			#if abs(theta) > 0.15:
			if ((abs(theta) > 20*numpy.pi/180 and AB > 0.08)) or (point_number == len(path)-1):
				#trigger = self.canPlannPath() # if not in infation space, plann new path
				if self.noPathavaliable == False and (point_number != 20) and len(path)>20:
					self.plannNewPath = True
					self.timeTrigger = True
					self.plannedAlready = True
					print "planning new path\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\nblah blah"
					self.lastTimePlanning = rospy.Time.now().secs
					break
				else:
					self.plannedAlready = False

					thetaTime = abs(theta)/0.4
					self.proxy_motion.post.move(0,0,0.4*numpy.sign(theta))
					thetaTime_now = 0
					while (thetaTime-thetaTime_now)>0:
						if self.kill_thread_followPath == True:
							break	
						rospy.sleep(0.1)
						
						thetaTime_now = thetaTime_now + 0.1
					self.proxy_motion.post.move(0,0,0)
					#self.getch()

			print "pojscie na AB"
			move_X_time = AB/0.06
			self.proxy_motion.post.move(0.06,0,0)
			move_X_time_now = 0
			while (move_X_time-move_X_time_now)>0:
				if self.kill_thread_followPath == True:
					break	
				rospy.sleep(0.1)
				
				move_X_time_now = move_X_time_now + 0.1
			
			self.proxy_motion.post.move(0,0,0)
			#self.getch() 

			# print "nawrotka na kierunek B"
			# theta2 = nextPoseOrientationZ - alpha
			# theta2_Time = abs(theta2)/0.2
			# self.proxy_motion.post.move(0,0,0.2*numpy.sign(theta2))
			# rospy.sleep(theta2_Time)

			if self.kill_thread_followPath == True:
				break
		if self.kill_thread_followPath == True:
			self.path_is_finished = False
		elif self.plannNewPath == True:
			return
		else:
			print "nawrotka na kierunek koncowy"
			naoCurrentPosition = self.getNaoCurrentPosition()
			robot_orientation_euler = tf.transformations.euler_from_quaternion(naoCurrentPosition[1])
			print "last point orientation: \n", nextPoseOrientationZ
			theta2 = nextPoseOrientationZ - robot_orientation_euler[2]
			if abs(theta2) > 3.14:
				print"\n theta > 3.14\n"
				theta2 = theta2-(numpy.sign(theta2)*2*numpy.pi)

			theta2_Time = abs(theta2)/0.2
			self.proxy_motion.post.move(0,0,0.2*numpy.sign(theta2))
			rospy.sleep(theta2_Time)
			self.proxy_motion.post.move(0,0,0)

			self.path_is_finished = True
			print "PATH END"
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
		# while self.tl.canTransform("map","base_link",rospy.Time()) == False:
		# 	rospy.sleep(0.2)
		# 	print "cant transform base_link to map frame now. "

		ekf_position = self.tl.lookupTransform("map","base_link",rospy.Time())
		ekf_euler = tf.transformations.euler_from_quaternion(ekf_position[1])
		torso_position = self.tl.lookupTransform("map","Nao_T_odom",rospy.Time())
		torso_euler = tf.transformations.euler_from_quaternion(torso_position[1])
		quaternion_to_publish = tf.transformations.quaternion_from_euler(torso_euler[0],torso_euler[1],ekf_euler[2])
		nao_position = [[ekf_position[0][0],ekf_position[0][1],torso_position[0][2]],[quaternion_to_publish[0],quaternion_to_publish[1],quaternion_to_publish[2],quaternion_to_publish[3]]] #torso_position[1][0],torso_position[1][1],ekf_position[1][2]]] 
		#print "nao position",(nao_position)
		sadasda = nao_position[1][1]
		return nao_position
	#def publishOdomToEKF (self):
	def getCameraCurrentPosition(self):
		camera_position = self.tl.lookupTransform("map","cameraTop",rospy.Time())

		return camera_position

	def rapp_move_head_interface(self,head_yaw,head_pitch):
		moveNaoHead = rospy.ServiceProxy('rapp_moveHead', MoveHead)
		resp1 = moveNaoHead(head_yaw,head_pitch)
	def rapp_move_vel_interface(self,x,y,theta):
		moveVel = rospy.ServiceProxy('rapp_moveVel', MoveVel)
		resp1 = moveVel(x,y,theta)
		return resp1
	def rapp_move_to_interface(self,x,y,theta):
		moveTo = rospy.ServiceProxy('rapp_moveTo', MoveTo)
		resp1 = moveTo(x,y,theta)

	def rapp_take_predefined_pose_interface(self,pose):
		takePosture = rospy.ServiceProxy('rapp_takePredefinedPosture', TakePredefinedPosture)
		resp1 = takePosture(pose)

	def detectObstacle(self):
		print "[detectObstacle] started"
		k=1
		l=1
		while (self.path_is_finished != True): 
			# sonar data = [right_dist, left_dist]
			print "[detectObstacle] new scan"
			data = self.getSonarData()

			if (data[0] <0.4 )or (data[1] <0.4):
  				k=k+1
  				l=1
  			else:
  				l=l+1
  				k=1
  			print "k/l\n",k/l
			if k/l>4:
				self.kill_thread_followPath = True
				print "[detectObstacle] stopped, data: \n", data
				#wait untill followPath thread will be closed
				self.thread_followPath.join()
				self.proxy_motion.post.move(0, 0,0)
				self.obstacle_detected = True
				break
			rospy.sleep(3)

	def handle_rapp_moveVel(self,req):

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

		isVelocitySetted = True
		return MoveVelResponse(isVelocitySetted)	

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
		#self.StiffnessOff("Head")
		return MoveHeadResponse(yaw_end,pitch_end)

	def handle_rapp_moveJoint(self,req):
		self.StiffnessOn(req.joint_name)
		maxSpeed = 0.2
		self.proxy_motion.angleInterpolationWithSpeed(req.joint_name,req.joint_angle,maxSpeed)
		useSensors  = True
		sensorAngles = self.proxy_motion.getAngles(req.joint_name, useSensors)
		# print "sensorAngles type is: \n", type(sensorAngles)
		# joint_angle = []float(sensorAngles[0])
		return MoveJointResponse(sensorAngles)

	def handle_rapp_removeStiffness(self,req):
		pNames = req.joint_name
		pStiffnessLists = 0.0
		pTimeLists = 1.0
		self.proxy_motion.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)
		status = True
		return RemoveStiffnessResponse(status)	

	def handle_rapp_moveStop(self,req):
		#self.proxy_motion.stopMove()
		self.proxy_motion.move(0, 0, 0)
		return MoveStopResponse(True)

	def handle_rapp_takePredefinedPosture(self,req):
		try:
			self.StiffnessOn("Body")
		except Exception, ex:
			print "[Move server] - Exception %s" % str(ex)
		try:	
			self.proxy_RobotPosture.goToPosture(req.pose, 0.3)
		except Exception, e:
				print "[Move server] - Exception %s" % str(e)	
		print "[Move server] - Actual Nao pose : %s" % str(req.pose)
		status = True
		return TakePredefinedPostureResponse(status)	
	def compute_turn_head_angles(self, point):
		pointX = point[0]
		pointY = point[1]
		pointZ = point[2]

		rospy.sleep(2)
		nao_position = self.getNaoCurrentPosition()
		robot_orientation_euler = tf.transformations.euler_from_quaternion(nao_position[1])
		camera_Nao_position = self.tl.lookupTransform("base_link_nao","cameraTop",rospy.Time())
		camera_Map_position = self.tl.lookupTransform("map","cameraTop",rospy.Time())
		camera_Map_orientation_euler = tf.transformations.euler_from_quaternion(camera_Map_position[1])

		camera_Nao_orientation_euler = tf.transformations.euler_from_quaternion(camera_Nao_position[1])
		cameraY_sit = 0.35#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		dist2D = numpy.sqrt((pointX - camera_Map_position[0][0])*(pointX - camera_Map_position[0][0])+(pointY - camera_Map_position[0][1])*(pointY - camera_Map_position[0][1]))
		#    *  - point
		#    |  }
		#    |  }  h = pointZ - NaoCamera
		#    o    - Nao camera
		h = pointZ - camera_Map_position[0][2]#camera_position[2]-nao_position[0][2]
		h_sitting = 9999#h - cameraY_sit
		minimalDist = h_sitting/numpy.tan(0.45)
		#print "camera pose:\n",camera_Nao_orientation_euler[2]
		#print "= ",robot_orientation_euler[2]
		#print "torso h",nao_position[0][2]
		gamma = camera_Map_orientation_euler[2]
		blabla = camera_Nao_orientation_euler[2]
		alpha = numpy.arctan2(pointY-camera_Map_position[0][1],pointX-camera_Map_position[0][0])
		print "gamma = ",gamma, "alpha = ",alpha, "blabla = ",blabla
		if abs(gamma)> abs(alpha):
		 	theta = -1*(gamma - alpha)
		elif abs(gamma)< abs(alpha):
			theta = (alpha - gamma)
		else:
			theta =0
		if abs(theta) > 3.14:
			print"\n theta > 3.14\n"
			theta = theta-(numpy.sign(theta)*2*numpy.pi)
		print "theta = ", theta
		head_yaw = theta + camera_Nao_orientation_euler[2]#sign*(2*numpy.pi - (abs(theta) + abs(robot_orientation_euler[2])+abs(camera_orientation_euler[2])))

		head_pitch = -numpy.arctan(h/dist2D) - robot_orientation_euler[1]
		turnHeadAngles = [head_yaw,head_pitch]
		return turnHeadAngles

	def handle_rapp_lookAtPoint(self,req):
		pointX = req.pointX
		pointY = req.pointY
		pointZ = req.pointZ
		# line = PolygonStamped()
		# line_Pub = rospy.Publisher("line", PolygonStamped, queue_size=10)

		# point1 = Point32()
		# point2 = Point32()
		# point1.x = camera_Map_position[0][0]
		# point1.y = camera_Map_position[0][1]
		# point1.z = camera_Map_position[0][2]
		# point2.x = pointX
		# point2.y = pointY
		# point2.z = pointZ
		# line.polygon.points= [point1,point2]
		# line.header.frame_id = "map"
		# line.header.seq = 0
		# line.header.stamp = rospy.Time.now()
		# # line.points.y = [4,4]
		# # line.points.z = [4,4]
		# line_Pub.publish(line)
		# thetaTime = abs(theta)/0.4
		# self.proxy_motion.post.move(0,0,0.4*numpy.sign(theta))
		# rospy.sleep(thetaTime)
		# self.proxy_motion.post.move(0,0,0)

		turnHeadAngles = self.compute_turn_head_angles([pointX,pointY,pointZ])	
		head_yaw = turnHeadAngles[0]
		head_pitch = turnHeadAngles[1]
		#define ranges
		range_matrix = [-2.086017,-0.449073,0.330041,
						-1.526988,-0.330041,0.200015,
						-1.089958,-0.430049,0.300022,
						-0.903033,-0.479965,0.330041,
						-0.756077,-0.548033,0.370010,
						-0.486074,-0.641951,0.422021,
						0.000000,-0.671951,0.515047]

		#check if Nao has to turn around to reach the point
		if abs(head_yaw) < abs(range_matrix[0]):
			canLookAtPoint_yaw = True
		else:
			canLookAtPoint_yaw = False

		i=1
		#search the matrix for pitch boundaries for estimated head yaw position 
		while i <= 6 :
			if abs(head_yaw) > abs(range_matrix[i*3]) and canLookAtPoint_yaw:
				head_pitch_max = range_matrix[(i-1)*3+2]
				head_pitch_min = range_matrix[(i-1)*3+1]
				break
			elif not canLookAtPoint_yaw:
				head_pitch_max = range_matrix[20]
				head_pitch_min = range_matrix[19]
				break
			i+=1			
			# Nao can reach head_yaw and head_pitch
		if (head_pitch > head_pitch_min and head_pitch < head_pitch_max and canLookAtPoint_yaw):
			case = 0
			print "CASE 0"
			# Nao can reach head_yaw but cant reach head_pitch
		elif (not (head_pitch > head_pitch_min and head_pitch < head_pitch_max) and canLookAtPoint_yaw):
			case = 1
			print "CASE 1"

			# Nao cant reach head_yaw
		elif not canLookAtPoint_yaw:
			case = 1
			print "CASE 2"

		if case == 0 :
			print "HeadPitch: \n",head_pitch, "HeadYaw: \n",head_yaw	
			self.rapp_move_head_interface(head_yaw,head_pitch)
			status = "point reached"
		elif case == 1:

			head_pitch_max = range_matrix[20]
			head_pitch_min = range_matrix[19]

			if head_pitch > head_pitch_min and head_pitch < head_pitch_max:		
				thetaTime = abs(head_yaw)/0.4
				moveVel_trigger = self.rapp_move_vel_interface(0,0,0.4*numpy.sign(head_yaw))

				rospy.sleep(thetaTime)
				self.proxy_motion.post.move(0,0,0)
			
				turnHeadAngles = self.compute_turn_head_angles([pointX,pointY,pointZ])	
				head_yaw = turnHeadAngles[0]
				head_pitch = turnHeadAngles[1]
				print "HeadPitch: \n",head_pitch, "HeadYaw: \n",head_yaw	

				self.rapp_move_head_interface(head_yaw,head_pitch)
				status = "point reached"	
			else:
				status = "Nao camera will not reach altitude of the point even after robot rotation"

		return LookAtPointResponse(status)	
		




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

	def followObstaclesBoundary2(self):
		print "BUG start"
 		self.proxy_motion.setWalkArmsEnabled(True, True)

 	 	#while (k >0 and k<0.8 and (i < 5 )):#or l <5)):
	 	print "sonar check"			
  		sonar_data = self.getSonarData()
 	 		#      |  <-sonar_left_distance 
 	 		#      |  ___________                 AVOIDE LEFT DIRACTION
  			#      |  \ <- sonar_right_distance  
 	 		#      | / \
 	 		#      |/   \
 	 		#      *
  		if sonar_data[1]>sonar_data[0]: # [0] - right,  [1] - left
  			avoide_direction = "left"
  			print "direction left"
  			velocity_theta = 0.1
  		else:
  			avoide_direction = "right"
			print "direction right"
			velocity_theta = -0.1
  		d=1
  		h=1
  		while d/h<4:
  			sonar_data = self.getSonarData()
  			print sonar_data
  			self.proxy_motion.moveToward(0, 0, velocity_theta)
  			if sonar_data[1]>0.6 and sonar_data[0]>0.6:
  				d=d+1
  				h=1
  			else:
  				h=h+1
  				d=1
 			print "d/h\n",(d/h)
 			print "d | h\n",d," | ",h

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
 	 	#print "STOP theta: %s" %theta
 	 	velocity_x = 0.4
 	 	maxDist = 0.75
 	 	d=1
 	 	h=1
 	 	while True:
 	 		sonar_data = self.getSonarData()
 			if sonar_data[1] > 2:
 				sonar_data[1] = 2
 			if sonar_data[0] > 2:
				sonar_data[0] = 2			

 	 		if avoide_direction == "left":
 	 			print "right sonar : %s" %sonar_data[0]
 	 			print "left sonar : %s" %sonar_data[1]
 		 		if sonar_data[0]>maxDist:
 		 			d=d+1
 		 		else:
 		 			h=h+1
 		 		if d/h > 4:
 		 			print "dociskamy \n : %s" %(sonar_data[0]) 
 		 			velocity_theta = -numpy.arctan((sonar_data[0]-maxDist)*1.2)
 		 			
 		 		if 1/h <0.25:
 		 			print "odpychamy \n : %s" %(sonar_data[0]) 
 		 			velocity_theta = numpy.arctan((sonar_data[0]-maxDist)*1.2)#numpy.arctan((sonar_data[0]-maxDist)*4)

		 		#self.proxy_motion.moveToward(0.4, 0, velocity_theta)
		 		
 		 	# 	if sonar_data[0]>maxDist: # [0] - right,  [1] - left
 		 	# 		print "dociskamy \n : %s" %(sonar_data[0]) 
 		 	# 		velocity_theta = -numpy.arctan((sonar_data[0]-maxDist)*1.2)

 			 # 		self.proxy_motion.moveToward(0.4, 0, velocity_theta)
 				# else:
 		 	# 		print "odpychamy \n : %s" %(sonar_data[0]) 
 		 	# 		velocity_theta = numpy.arctan((sonar_data[0]-maxDist)*1.2)#numpy.arctan((sonar_data[0]-maxDist)*4)

 			 # 		self.proxy_motion.moveToward(0.4, 0, velocity_theta)

 			 		# if sonar_data[1] < 0.40:
 			 		# 	self.followLeftUntilRightFree(0.4,0.6)
			 		
 			 		# if sonar_data[1] < 0.30:
			 	
 			 		# 	self.proxy_motion.moveToward(0.4, 0, sonar_data[0]/4)
 			 		# 	print "WATCH OUT - LEFT OBSTACLE"
 	 		else:
 	 			print "right sonar : %s" %sonar_data[0]
 	 			print "left sonar : %s" %sonar_data[1]
 	 			print "direction:\n",avoide_direction
 
  		 		if sonar_data[1]>maxDist:
 		 			i=i+1
 		 		else:
 		 			l=l+1
 		 		if i/l > 4:
 		 			print "dociskamy \n : %s" %(sonar_data[1]) 
					velocity_theta = numpy.arctan((sonar_data[0]-maxDist)*1.2)
 		 			
 		 		if 1/l <0.25:
 		 			print "odpychamy \n : %s" %(sonar_data[0]) 
 		 			velocity_theta = -numpy.arctan((sonar_data[0]-maxDist)*1.2)

		 	self.proxy_motion.moveToward(0.4, 0, velocity_theta)		 	


 		 	# 	if sonar_data[1]>maxDist: # [0] - right,  [1] - left
 		 	# 		#avoide_direction = "left"
 		 	# 		print "dociskamy \n : %s" %(sonar_data[1]) 
					# velocity_theta = numpy.arctan((sonar_data[0]-maxDist)*1.2)
 			 # 		self.proxy_motion.moveToward(0.4, 0, velocity_theta)
 				# else:
 		 	# 		print "odpychamy \n : %s" %(sonar_data[0]) 
 		 	# 		velocity_theta = -numpy.arctan((sonar_data[0]-maxDist)*1.2)#numpy.arctan((sonar_data[0]-maxDist)*4)

 			 # 		self.proxy_motion.moveToward(0.4, 0, velocity_theta)

 			 		# if sonar_data[1] < 0.40:
 			 		# 	self.followLeftUntilRightFree(0.4,0.6)
			 		
 			 		# if sonar_data[1] < 0.30:
			 	
 			 		# 	self.proxy_motion.moveToward(0.4, 0, sonar_data[0]/4)
 			 		# 	print "WATCH OUT - LEFT OBSTACLE"


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