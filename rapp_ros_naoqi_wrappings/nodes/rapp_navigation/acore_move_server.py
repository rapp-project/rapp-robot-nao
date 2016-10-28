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
#from rapp_api.objects import * 
#from navfn.srv import *

# Importing core system functionality
import signal
import sys, os
import rospy
import almath as m
import numpy 
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PointStamped
from geometry_msgs.msg import PolygonStamped, Point32
import tf2_ros
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
		#self.subscribeToEvents()
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
		self.tf_br = tf.TransformBroadcaster()
		self.sub_obstacle = None
		self.getTransform_interface = rospy.ServiceProxy('rapp_get_transform', GetTransform)
		#globalPosePublisher = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
	def subscribeToObstacle(self):
		self.sub_obstacle = rospy.Subscriber("/obstacleDetectorState", obstacleData , self.detectObstacle)

	def unsubscribeToObstacle(self):
		self.sub_obstacle.unregister()
		self.sub_obstacle = None

	def isSubscribedToObstacle(self):
		if self.sub_obstacle is None:
			return False
		else:
			return True
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
			print "[Move server] - service - [rapp_moveAlongPath]"
			self.service_map = rospy.Service('rapp_moveAlongPath', MoveAlongPath, self.handle_rapp_MoveAlongPath)
		except Exception, ex_map:
			print "[Move server] - Exception %s" % str(ex_map)
		try:
			print "[Move server] - setting services"
			print "[Move server] - service - [rapp_moveTo]"
			self.service_mt = rospy.Service('rapp_moveTo', MoveTo, self.handle_rapp_moveTo)
		except Exception, ex_mt:
			print "[Move server] - Exception %s" % str(ex_mt)
		try:
			print "[Move server] - service - [rapp_moveVel]"
			self.service_mv = rospy.Service('rapp_moveVel', MoveVel, self.handle_rapp_moveVel)
		except Exception, ex_mv:
			print "[Move server] - Exception %s" % str(ex_mv)
		try:
			print "[Move server] - service - [rapp_moveStop]"
			self.service_ms = rospy.Service('rapp_moveStop', MoveStop, self.handle_rapp_moveStop)
		except Exception, ex_ms:
			print "[Move server] - Exception %s" % str(ex_ms)	
		try:
			print "[Move server] - service - [rapp_moveJoint]"
			self.service_mj = rospy.Service('rapp_moveJoint', MoveJoint, self.handle_rapp_moveJoint)
		except Exception, ex_mj:
			print "[Move server] - Exception %s" % str(ex_mj)	
		try:
			print "[Move server] - service - [rapp_rest]"
			self.service_rest = rospy.Service('rapp_rest', Rest, self.handle_rapp_rest)
		except Exception, ex_rest:
			print "[Move server] - Exception %s" % str(ex_rest)
		try:
			print "[Move server] - service - [rapp_takePredefinedPosture]"
			self.service_takePosture = rospy.Service('rapp_takePredefinedPosture', TakePredefinedPosture, self.handle_rapp_takePredefinedPosture)
		except Exception, ex_takePosture:
			print "[Move server] - Exception %s" % str(ex_takePosture)
		try:
			print "[Move server] - service - [rapp_lookAtPoint]"
			self.service_lookAt = rospy.Service('rapp_lookAtPoint', LookAtPoint, self.handle_rapp_lookAtPoint)
		except Exception, ex_lookAt:
			print "[Move server] - Exception %s" % str(ex_lookAt)

		try:
			print "[Move server] - service - [rapp_pointArm]"
			self.service_pointArm = rospy.Service('rapp_pointArm', LookAtPoint, self.handle_pointArm)
		except Exception, ex_pointArm:
			print "[Move server] - Exception %s" % str(ex_pointArm)			
		# try:
		# 	print "[Move server] - service - [rapp_getRobotPose]"
		# 	self.service_getPosition = rospy.Service('rapp_getRobotPose', GetRobotPose, self.handle_rapp_getRobotPose)
		# except Exception, ex_getPosition:
		# 	print "[Move server] - Exception %s" % str(ex_getPosition)
		# try:
		# 	print "[Move server] - service - [rapp_setGlobalPose]"
		# 	self.service_getPosition = rospy.Service('rapp_setGlobalPose', SetGlobalPose, self.handle_rapp_setGlobalPose)
		# except Exception, ex_getPosition:
		# 	print "[Move server] - Exception %s" % str(ex_getPosition)
	def getch(self):
		import sys, tty, termios
		fd = sys.stdin.fileno()
		old = termios.tcgetattr(fd)
		try:
			tty.setraw(fd)
			return sys.stdin.read(1)
		finally:
			termios.tcsetattr(fd, termios.TCSADRAIN, old)
#
#
#   Interfaces to virtual effector
#
#
	def rapp_move_vel_interface(self,x,y,theta):
		moveVel = rospy.ServiceProxy('moveVel', MoveVel)
		resp1 = moveVel(x,y,theta)
		return resp1.status
	def rapp_move_to_interface(self,x,y,theta):
		moveTo = rospy.ServiceProxy('moveTo', MoveTo)
		resp1 = moveTo(x,y,theta)
		return resp1.status

	def rapp_take_predefined_posture_interface(self,pose,speed):
		takePosture = rospy.ServiceProxy('takePredefinedPosture', TakePredefinedPosture)
		resp1 = takePosture(pose,speed)
		return resp1.status

	def rapp_move_joint_interface(self,joints, angles, speed):
		#self.unsubscribeToObstacle()
		moveJoint = rospy.ServiceProxy('moveJoint', MoveJoint)
		resp1 = moveJoint(joints, angles, speed)
		return resp1.status

	def rapp_stiffness_interface(self,joint,trigger):
		if self.isSubscribedToObstacle():
			self.unsubscribeToObstacle()
		triggerStiffness = rospy.ServiceProxy('triggerStiffness', TriggerStiffness)
		resp1 = triggerStiffness(joint, trigger)
		return resp1.status
	def rapp_stop_move_interface(self):
		#self.unsubscribeToObstacle()
		moveStop = rospy.ServiceProxy('moveStop', MoveStop)
		resp1 = moveStop()
		return resp1.status
#
#
#   Interfaces to virtual receptor ---- TO DO
#
#
	####
	##  SERVECE HANDLERS
	####
	# def handle_rapp_setGlobalPose(self,req):
	# 	try:
	# 		data_to_publish = PoseWithCovarianceStamped()
	# 		data_to_publish.pose.pose = req.pose
	# 		data_to_publish.header.seq = 0
	# 		data_to_publish.header.stamp = rospy.Time.now()
	# 		data_to_publish.header.frame_id = "/map"
	# 		globalPosePublisher.publish(data_to_publish)
	# 	except Exception, ex_setPosition:
	# 		print "[Move server] - Exception %s" % str(ex_setPosition)
	# 	return SetGlobalPoseResponse(status)

	# def handle_rapp_getRobotPose(self,req):
	# 	try:
	# 		naoCurrentPosition = self.getNaoCurrentPosition()
	# 		nao_euler = tf.transformations.euler_from_quaternion(naoCurrentPosition[1])
	# 		position = PoseStamped()
	# 		position.header.frame_id = "/map"
	# 		position.header.seq = 0
	# 		position.header.stamp = rospy.Time.now()

	# 		position.pose.position.x = naoCurrentPosition
	# 		position.pose.position.y = naoCurrentPosition[0][1]
	# 		position.pose.position.z = naoCurrentPosition[0][2]
	# 		position.pose.orientation.x = naoCurrentPosition[1][0]
	# 		position.pose.orientation.y = naoCurrentPosition[1][1]
	# 		position.pose.orientation.z = naoCurrentPosition[1][2]
	# 		position.pose.orientation.w = naoCurrentPosition[1][3]
	# 	except Exception, ex_getPosition:
	# 		print "[Move server] - Exception %s" % str(ex_getPosition)
	# 	return GetRobotPositionResponse(position)
	def handle_rapp_moveTo(self,req):
		try:
			self.subscribeToObstacle()

			resp = self.rapp_move_to_interface(req.x,req.y,req.theta)

			self.unsubscribeToObstacle()

			status = resp
		except Exception, ex:
			print "[MoveTo server] - Exception %s" % str(ex)
			status = True
		return MoveToResponse(status)	

	def handle_rapp_MoveAlongPath(self,req):
		
		naoCurrentPosition = self.getNaoCurrentPosition()
		nao_theta = tf.transformations.euler_from_quaternion(naoCurrentPosition[1])[2]

		self.move_is_finished = False
		self.path_is_finished = False
		self.kill_thread_followPath = False
		self.obstacle_detected = False
		# print "[Move server] - waiting for makePlan service"
		# rospy.wait_for_service('/global_planner/make_plan')
		# print "[Move server] - makePlan service found"
		
		self.followPath_flag = 'empty'
		rate_mainThread = rospy.Rate(1)
		# noPathavaliable = False
		#plannNewPath = rospy.ServiceProxy('rapp_plannPath', PlannPath)
		#path = plannNewPath(naoCurrentPosition[0][0],naoCurrentPosition[0][1],nao_theta,req.x,req.y,req.theta)

		#path = req.path
		# OBSLUGA SCIEZKI

		#while self.path_is_finished != True:
		# path = self.plannPath(GoalGlobalPose)
		# if len(path.path)<3:
		# 	noPathavaliable = True
		# 	break

		# self.thread_followPath = threading.Thread(None,self.followPath,None,[req.path.poses])
		

		#thread_detectObstacle = threading.Thread(None,self.detectObstacle,None)	
			# # # zanim zaczniesz ruch ustaw watek do sprawdzania sonarow
		#thread_detectObstacle.start()

		self.rapp_take_predefined_posture_interface('StandInit',0.3)
		
		self.subscribeToObstacle()
		
		pathFollowingStatus = self.followPath(req.poses)
		
		self.unsubscribeToObstacle()

		#self.followPath_flag = self.followPath(path)
		# while self.path_is_finished == False and self.obstacle_detected == False:
		# 	rate_mainThread.sleep()		#wait to nao stop move
		if pathFollowingStatus == "finished":
			#self.kill_thread_detectObstacle = True
			#thread_detectObstacle.join()
			#print "destination reached, and obstacle detection is:\n",thread_detectObstacle.isAlive()
			status = False

		else:
			#print "OBSTACLE DETECTED, obstacle detection is:\n",thread_detectObstacle.isAlive()
			#self.followObstaclesBoundary2()
			print "BUG BUG BUG BUG BUG BUG BUG BUG \n BUG BUG BUG BUG BUG BUG"
			status = True
		# if noPathavaliable == True:
		# 	status = "no path avaliable"
		# else:
		# 	status = "move finished"

		#self.move_is_finished = True

		return MoveAlongPathResponse(status)

	def detectObstacle(self,msg):
		# while (self.path_is_finished != True): 
		# sonar data = [right_dist, left_dist]
		sum_data = 0
		i = 0

		if (msg.RightBumper == 1):
			#self.unsubscribeToObstacle()
			self.obstacle_detected = True
			self.kill_thread_followPath = True
			self.rapp_stop_move_interface()

			print "Obstacle detected by RIGHT BUMPER " 
		elif ( msg.LeftBumper==1):
			#self.unsubscribeToObstacle()
			self.obstacle_detected = True
			self.kill_thread_followPath = True
			self.rapp_stop_move_interface()
			print "Obstacle detected by LEFT BUMPER " 



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
		path_resp = plan_path(start,goal)
		print "response:", path_resp.path
		print type(path_resp)
		return PlannPathResponse(path_resp.plan_found,path_resp.error_message, path_resp.path)

	def followPath(self,path):			
		status = "start"

		for i in range(len(path)):

		#int(numpy.floor(len(path.path)/200))+1):
			print "i= ",i
			print "liczba punktow: \n", len(path)
			rospy.sleep(3)
			naoCurrentPosition = self.getNaoCurrentPosition()
			robot_orientation_euler = tf.transformations.euler_from_quaternion(naoCurrentPosition[1])

			if (i >= len(path) -1):
				point_number = len(path)-1
			else:
				point_number = (i+1)

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
				resp = self.rapp_move_vel_interface(0,0,0.3*numpy.sign(theta))
				#self.proxy_motion.move(0,0,0.3*numpy.sign(theta))
				
				#while bool(resp):
					
				thetaTime_now = 0
				while (thetaTime-thetaTime_now)>0:
					if self.obstacle_detected == True:
						status = "obstacle"
						print "AAAAAAAAAAAA"
						break	
					rospy.sleep(0.1)
					
					thetaTime_now = thetaTime_now + 0.1
			self.rapp_stop_move_interface()
					#self.getch() 
			print "pojscie na AB"
			move_X_time = AB/0.05
			#self.proxy_motion.move(0.03,0,0)
			if self.obstacle_detected == False:
				resp = self.rapp_move_vel_interface(0.05,0,0)
				
				# while not bool(resp):
				#  	rospy.sleep(0.1)
				#  	pass

				print "po ruchu"

				move_X_time_now = 0
				while (move_X_time-move_X_time_now)>0:
					if self.obstacle_detected == True:
						status = "obstacle"
						print "BBBBBB"
						break	
					rospy.sleep(0.1)
					
					move_X_time_now = move_X_time_now + 0.1
				
				self.rapp_stop_move_interface()
			#self.getch() 

			# print "nawrotka na kierunek B"
			# theta2 = nextPoseOrientationZ - alpha
			# theta2_Time = abs(theta2)/0.2
			# self.proxy_motion.post.move(0,0,0.2*numpy.sign(theta2))
			# rospy.sleep(theta2_Time)
			if self.obstacle_detected == True:
				print "WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW"

				return status
		if self.obstacle_detected == True:
			print "WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW"

			return status
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
			
			resp = self.rapp_move_vel_interface(0,0,0.2*numpy.sign(theta2))
			# while not bool(resp):
			# 	rospy.sleep(0.1)
			# 	pass			
			thetaTime_now = 0
			while (theta2_Time-thetaTime_now)>0:
				if self.obstacle_detected == True:
					status = "obstacle"
					break	
				rospy.sleep(0.1)
				
				thetaTime_now = thetaTime_now + 0.1
			self.rapp_stop_move_interface()

		if status == "obstacle":
			return status
		else:
			status = "finished"
			return status
	def followPath2(self,path):			
		status = "start"
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
			nextPose_POSE = PoseStamped()
			cos = PoseStamped()
			nextPose_POSE.header.frame_id = "map"
			nextPose_POSE.header.stamp = self.tl.getLatestCommonTime("map","base_link")
			print "time: ",self.tl.getLatestCommonTime("map","base_link")
			nextPose_POSE = nextPose
			# nextPose_POSE.pose.position.y = naoCurrentPosition[0][1]
			# nextPose_POSE.pose.position.z = naoCurrentPosition[0][2]
			# nextPose_POSE.pose.orientation.x = naoCurrentPosition[1][0]
			# nextPose_POSE.pose.orientation.y = naoCurrentPosition[1][1]
			# nextPose_POSE.pose.orientation.z = naoCurrentPosition[1][2]
			# nextPose_POSE.pose.orientation.w = naoCurrentPosition[1][3]
			# while not self.tl.canTransform("map","base_link",rospy.Time.now()):
			# 	rospy.sleep(1)
			if self.tl.canTransform("map","base_link",self.tl.getLatestCommonTime("map","base_link")):
				#torso_position = self.tl.lookupTransform("map","base_link",rospy.Time())
				#print numpy.dot(torso_position,nextPose_POSE)
				#cos.setData(torso_position*nextPose_POSE)

				#cos = self.tl.transformPose("base_link",nextPose_POSE)
				#print "cos: ", cos  #"ekfpose_type is: ",type(torso_position),": \n",torso_position
			#print "ekf_orientation_type is: ",type(ekf_rotation),": \n",ekf_rotation

				nextPose_matrix = [[np.cos(nextPoseOrientationZ), np.sin(nextPoseOrientationZ),0,nextPose.pose.position.x],
								[-np.sin(nextPoseOrientationZ), np.cos(nextPoseOrientationZ),0,nextPose.pose.position.y],
								[0,0,1,0],
								[0,0,0,1]]
			#np.dot()
			print "ZONG"
			#self.rapp_move_to_interface(x_A-x_B,y_A-y_B,-(nextPoseOrientationZ-robot_orientation_euler[2]))
		# 	alpha = numpy.arctan2(y_B-y_A,x_B-x_A)
		# 	#dist_Nao_trajectory = numpy.sqrt(()*()+()*())
		# 	print "gamma|alpha\n", gamma," | ",alpha
		# 	print "gamma|alpha", gamma," | ",nextPoseOrientationZ
		# 	if abs(gamma)> abs(alpha):
		# 	 	theta = -1*(gamma - alpha)
		# 	elif abs(gamma)< abs(alpha):
		# 		theta = (alpha - gamma)
		# 	else:
		# 		theta =0
		# 	if abs(theta) > 3.14:
		# 		print"\n theta > 3.14\n"
		# 		theta = theta-(numpy.sign(theta)*2*numpy.pi)
		# 	print "nawrotka na AB"
		# 	print "theta= ",theta
		# 	#if abs(theta) > 0.15:
		# 	if (abs(theta) > 20*numpy.pi/180 and AB > 0.08) or (point_number == len(path)-1):
		# 		thetaTime = abs(theta)/0.3
		# 		resp = self.rapp_move_vel_interface(0,0,0.3*numpy.sign(theta))
		# 		#self.proxy_motion.move(0,0,0.3*numpy.sign(theta))
		# 		while not bool(resp):
		# 			rospy.sleep(0.1)
		# 			pass
		# 		thetaTime_now = 0
		# 		while (thetaTime-thetaTime_now)>0:
		# 			if self.obstacle_detected == True:
		# 				status = "obstacle"
		# 				print "AAAAAAAAAAAA"
		# 				break	
		# 			rospy.sleep(0.1)
					
		# 			thetaTime_now = thetaTime_now + 0.1
		# 	self.rapp_stop_move_interface()
		# 			#self.getch() 
		# 	print "pojscie na AB"
		# 	move_X_time = AB/0.05
		# 	#self.proxy_motion.move(0.03,0,0)
		# 	if self.obstacle_detected == False:
		# 		resp = self.rapp_move_vel_interface(0.05,0,0)
		# 		# while not bool(resp):
		# 		# 	rospy.sleep(0.1)
		# 		# 	pass

		# 		print "po ruchu"

		# 		move_X_time_now = 0
		# 		while (move_X_time-move_X_time_now)>0:
		# 			if self.obstacle_detected == True:
		# 				status = "obstacle"
		# 				print "BBBBBB"
		# 				break	
		# 			rospy.sleep(0.1)
					
		# 			move_X_time_now = move_X_time_now + 0.1
				
		# 		self.rapp_stop_move_interface()
		# 	#self.getch() 

		# 	# print "nawrotka na kierunek B"
		# 	# theta2 = nextPoseOrientationZ - alpha
		# 	# theta2_Time = abs(theta2)/0.2
		# 	# self.proxy_motion.post.move(0,0,0.2*numpy.sign(theta2))
		# 	# rospy.sleep(theta2_Time)
		# 	if self.obstacle_detected == True:
		# 		print "WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW"

		# 		return status
		# if self.obstacle_detected == True:
		# 	print "WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW"

		# 	return status
		# else:
		# 	print "nawrotka na kierunek koncowy"
		# 	naoCurrentPosition = self.getNaoCurrentPosition()
		# 	robot_orientation_euler = tf.transformations.euler_from_quaternion(naoCurrentPosition[1])
		# 	print "last point orientation: \n", nextPoseOrientationZ
		# 	theta2 = nextPoseOrientationZ - robot_orientation_euler[2]
		# 	if abs(theta2) > 3.14:
		# 		print"\n theta > 3.14\n"
		# 		theta2 = theta2-(numpy.sign(theta2)*2*numpy.pi)

		# 	theta2_Time = abs(theta2)/0.2
			
		# 	resp = self.rapp_move_vel_interface(0,0,0.2*numpy.sign(theta2))
		# 	while not bool(resp):
		# 		rospy.sleep(0.1)
		# 		pass			
		# 	thetaTime_now = 0
		# 	while (theta2_Time-thetaTime_now)>0:
		# 		if self.obstacle_detected == True:
		# 			status = "obstacle"
		# 			break	
		# 		rospy.sleep(0.1)
				
		# 		thetaTime_now = thetaTime_now + 0.1
		# 	self.rapp_stop_move_interface()

		if status == "obstacle":
			return status
		else:
			status = "finished"
			return status
	def getNaoCurrentPosition(self):
		if self.tl.canTransform("map","Nao_Torso",rospy.Time()):
			torso_position = self.tl.lookupTransform("map","Nao_Torso",rospy.Time())
			torso_euler = tf.transformations.euler_from_quaternion(torso_position[1])
		# ekf_euler - orientation from EKF
		# torso_euler - orientation from Odometry
			quaternion_to_publish = tf.transformations.quaternion_from_euler(torso_euler[0],torso_euler[1],torso_euler[2])
			nao_position = [[torso_position[0][0],torso_position[0][1],torso_position[0][2]],[quaternion_to_publish[0],quaternion_to_publish[1],quaternion_to_publish[2],quaternion_to_publish[3]]] #torso_position[1][0],torso_position[1][1],torso_position[1][2]]] 
		#print "nao position",(nao_position)
			return nao_position
		else:
			print "can't transform base_blink to map frame" 



	def getCameraCurrentPosition(self):
		if self.tl.canTransform("map","cameraTop",rospy.Time()):
			camera_position = self.tl.lookupTransform("map","cameraTop",rospy.Time())
			return camera_position
		else:
			print "can't transform cameraTop to map frame" 

	def handle_rapp_moveVel(self,req):

		X = req.velocity_x
		Y = req.velocity_y
		Theta = req.velocity_theta
		
		self.subscribeToObstacle()
		try:
			self.rapp_move_vel_interface(X, Y, Theta)
			status = False
		except Exception, ex:
			print "[Move server] - Exception in rapp_moveVel service handling: \n %s" % str(ex)
			status = True

		return MoveVelResponse(status)	


	def handle_rapp_moveJoint(self,req):
		try:
			status = self.rapp_move_joint_interface(req.joint_name,req.joint_angle,req.speeds)
		except Exception, ex:
			print "[Move server] - Exception in rapp_moveJoint service handling: \n %s" % str(ex)
			status = True
		return MoveJointResponse(status)

	def handle_rapp_rest(self,req):
		try:
			status = self.rapp_take_predefined_posture_interface(req.posture,0.3)
			print "[rapp_rest service] - take posture status:\n%s" % str(status) 
			status = self.rapp_stiffness_interface("Body", False)
			print "[rapp_rest service] - rapp_stiffness_interface status:\n%s" % str(status) 

		except Exception, ex:
			print "[Move server] - Exception in rapp_rest service handling: \n %s" % str(ex)
			status = True
		return RestResponse(status)	

	def handle_rapp_moveStop(self,req):
		try:
			status = rapp_stop_move_interface()
			self.unsubscribeToObstacle()
		except Exception, ex:
			print "[Move server] - Exception in rapp_moveStop service handling: \n %s" % str(ex)
			status = True
		return MoveStopResponse(status)

	def handle_rapp_takePredefinedPosture(self,req):
		try:
			status = self.rapp_take_predefined_posture_interface(req.pose, req.speed)
		except Exception, ex:
			status = True
			print "[Move server] - Exception %s" % str(ex)
		return TakePredefinedPostureResponse(status)	

	def transformPoint(self,target_frame,ps, time):
		
		r = PointStamped()
		self.tf_buffer = tf2_ros.Buffer()
		self.tf2_listener = tf2_ros.TransformListener(self.tf_buffer)
		transform_stamped = self.tf_buffer.lookup_transform(target_frame,ps.header.frame_id,time, rospy.Duration(10))
		point_translation_upper = transform_stamped.transform.translation
		point_rotation_upper = transform_stamped.transform.rotation
		#self.tl.waitForTransform(target_frame,ps.header.frame_id,time, rospy.Duration(10))
		#point_translation_upper,point_rotation_upper = self.tl.lookupTransform(target_frame,ps.header.frame_id,time)
		transform_matrix = numpy.dot(tf.transformations.translation_matrix([point_translation_upper.x, point_translation_upper.y, point_translation_upper.z]), tf.transformations.quaternion_matrix([point_rotation_upper.x,point_rotation_upper.y,point_rotation_upper.z,point_rotation_upper.w]))
		xyz = tuple(numpy.dot(transform_matrix, numpy.array([ps.point.x, ps.point.y, ps.point.z, 1.0])))[:3] 
		r.header.stamp = ps.header.stamp 
		r.header.frame_id = target_frame 
		r.point = geometry_msgs.msg.Point(*xyz) 	
		return r	

	def handle_pointArm(self,req):


		now = rospy.Time()
		pointX = req.pointX
		pointY = req.pointY
		pointZ = req.pointZ
		self.tf_br.sendTransform([pointX, pointY, pointZ], [0,0,0,1],
                                    rospy.Time.now(), "POINT", "/map")
		# dest_point = PointStamped()
		# dest_point.header.frame_id = "/map"
		# dest_point.header.seq = 0
		# dest_point.header.stamp = now
		# dest_point.point.x = pointX
		# dest_point.point.y = pointY
		# dest_point.point.z = pointZ


		dest_point = PointStamped()
		dest_point.header.frame_id = "map"
		dest_point.header.seq = 0
		dest_point.header.stamp = now
		dest_point.point.x = pointX
		dest_point.point.y = pointY
		dest_point.point.z = pointZ

		point_in_head_yaw = self.transformPoint("RShoulder", dest_point, now)
		print point_in_head_yaw
		point_in_arm = [pointX, pointY+0.097999997437, pointZ+0.10000000149]
		alpha = -numpy.arctan2(point_in_head_yaw.point.z,point_in_head_yaw.point.x)##*180/3.14
		beta = numpy.arctan2(point_in_head_yaw.point.y,numpy.sqrt(point_in_head_yaw.point.x*point_in_head_yaw.point.x+point_in_head_yaw.point.z*point_in_head_yaw.point.z))##*180/3.14
		print alpha, "  ||", beta
		pNames = ["RShoulderPitch","RShoulderRoll","Head"]
		pStiffnessLists = [1.0,1.0,1.0]
		pTimeLists = [1.0, 1.0,1.0]
		self.proxy_motion.setStiffnesses("Head", 1.0)
		self.proxy_motion.setStiffnesses("RShoulderRoll", 1.0)
		self.proxy_motion.setStiffnesses("RShoulderPitch", 1.0)

		# self.proxy_motion.angleInterpolationWithSpeed(pNames,alpha,0.6)
		# self.proxy_motion.angleInterpolationWithSpeed("HeadYaw",1,0.6)
		self.rapp_move_joint_interface(["RShoulderPitch","RShoulderRoll"],[alpha,beta],0.4)
		self.rapp_move_joint_interface(["RElbowRoll","RElbowYaw"],[0,0],0.4)

		# self.rapp_move_joint_interface(["RShoulderRoll"],[beta],0.4)


	def compute_turn_head_angles(self, point):

		now = rospy.Time()
		pointX = point[0]
		pointY = point[1]
		pointZ = point[2]
		dest_point = PointStamped()
		dest_point.header.frame_id = "map"
		dest_point.header.seq = 0
		dest_point.header.stamp = now
		dest_point.point.x = point[0]
		dest_point.point.y = point[1]
		dest_point.point.z = point[2]

		self.tf_br.sendTransform(point, [0,0,0,1],
                                         rospy.Time.now(), "POINT", "/map")

		if(self.tl.canTransform("Neck","cameraTop", now) and self.tl.canTransform("HeadPitch","cameraTop", now) and self.tl.canTransform("Neck", "map", now) and self.tl.canTransform("Neck", "cameraTop", now) ):
			point_in_head_yaw = self.transformPoint("Neck", dest_point,now)

			point_in_head_pitch = self.transformPoint("Neck", dest_point,now)

			###
			#  compute head pitch angle. Based on MMAR publication
			###
			
			# point_in_head_pitch = self.transformerROS.transformPoint("head_upper_fixed_link_1", dest_point)
			# print "point_in_head_pitch = ", point_in_head_pitch
			camera_in_fixed_head_pitch_transform = self.tl.lookupTransform("Neck","cameraTop",now)
			camera_in_revolute_head_pitch_transform = self.tl.lookupTransform("HeadPitch","cameraTop",now)
			x_y_len = numpy.sqrt(point_in_head_pitch.point.x*point_in_head_pitch.point.x+point_in_head_pitch.point.y*point_in_head_pitch.point.y)
			D = numpy.sqrt(camera_in_fixed_head_pitch_transform[0][0]*camera_in_fixed_head_pitch_transform[0][0]+camera_in_fixed_head_pitch_transform[0][2]*camera_in_fixed_head_pitch_transform[0][2]) 
			# print "D = ", D
			C = numpy.sqrt(x_y_len*x_y_len + point_in_head_pitch.point.z*point_in_head_pitch.point.z)
			# print "C = ", C
			gamma = numpy.arctan(camera_in_revolute_head_pitch_transform[0][2]/camera_in_revolute_head_pitch_transform[0][0])
			##print "gamma = ", gamma
			#print "camera_in_revoluteZ = ", camera_in_revolute_head_pitch_transform[0][2]
			#print "camera_in_revoluteX = ", camera_in_revolute_head_pitch_transform[0][0]

			beta = numpy.pi - gamma
			# print "beta = ", beta
			sin_sigma = (D/C) * numpy.sin(beta)
			# print "sin_sigma = ", sin_sigma
			alpha = numpy.arctan2( point_in_head_pitch.point.z,x_y_len) + numpy.arctan2(numpy.sin(beta)*D,numpy.sqrt(1-sin_sigma*sin_sigma)*C)
			#print "alpha = ", alpha
			alpha2 = numpy.arctan2(x_y_len, point_in_head_pitch.point.z) + numpy.arctan2(numpy.sin(beta)*D,numpy.sqrt(1-sin_sigma*sin_sigma)*C)
			#print "alpha2 = ", alpha2
			head_pitch = alpha2 - (90*numpy.pi)/180#+ beta + gamma 
			# print "head_pitch = ", head_pitch
			if (abs(head_pitch) < numpy.pi+0.01 and abs(head_pitch) > numpy.pi-0.01):
				head_pitch = 0

			###
			#  compute head yaw angle. Based on MMAR publication
			###

			head_yaw = numpy.arctan2(point_in_head_yaw.point.y, point_in_head_yaw.point.x)
			if (abs(head_yaw) < numpy.pi+0.01 and abs(head_yaw) > numpy.pi-0.01):
				head_yaw = 0
			if (abs(head_yaw) < 0+0.01 and abs(head_yaw) > 0-0.01):
				head_yaw = 0


			turnHeadAngles = [head_yaw,head_pitch]
			#print "turnHeadAngles = ", turnHeadAngles
			return turnHeadAngles
		else:
			turnHeadAngles = []
			print "[Cannot calculate turn head angles] - cannot transform frames"
			return turnHeadAngles

#		now = rospy.Time()
#		pointX = point[0]
#		pointY = point[1]
#		pointZ = point[2]
#		dest_point = PointStamped()
#		dest_point.header.frame_id = "/map"
#		dest_point.header.seq = 0
#		dest_point.header.stamp = now
#		dest_point.point.x = point[0]
#		dest_point.point.y = point[1]
#		dest_point.point.z = point[2]

#		self.tf_br.sendTransform(point, [0,0,0,1],
 #                                        rospy.Time.now(), "POINT", "/map")
#		head_yaw = 0
#		head_pitch = 0
#		if(self.tl.canTransform("Neck","cameraTop", now)):
#			point_in_neck = self.transformPoint("Neck", dest_point,now)

#			camera_neck_position = self.tl.lookupTransform("Neck","cameraTop",rospy.Time())
#			gamma = - numpy.arctan(camera_neck_position[0][2]/camera_neck_position[0][0])
#			# print "gamma = ", gamma
#			x_y_len = numpy.sqrt(point_in_neck.point.x*point_in_neck.point.x+point_in_neck.point.y*point_in_neck.point.y)
#			D = numpy.sqrt(camera_neck_position[0][0]*camera_neck_position[0][0]+camera_neck_position[0][2]*camera_neck_position[0][2]) 
#			# print "D = ", D
#			C = numpy.sqrt(x_y_len*x_y_len + point_in_neck.point.z*point_in_neck.point.z)
#			beta = numpy.pi - gamma
#			sin_sigma = (D/C)*numpy.sin(beta)

#			alpha = numpy.arctan2(point_in_neck.point.z,point_in_neck.point.x)+numpy.arctan2(sin_sigma, numpy.sqrt(1-sin_sigma*sin_sigma))

#			head_pitch = -alpha
#			print "head_pitch = ", head_pitch
#			head_yaw = numpy.arctan2(point_in_neck.point.y, point_in_neck.point.x)
#			print "head_yaw = ", head_yaw
#		turnHeadAngles = [head_yaw,head_pitch]
#		return turnHeadAngles
	#	rospy.sleep(2)
#		nao_position = self.getNaoCurrentPosition()
#		robot_orientation_euler = tf.transformations.euler_from_quaternion(nao_position[1])
#		camera_Nao_position = self.tl.lookupTransform("base_link","cameraTop",rospy.Time())
#		camera_Map_position = self.tl.lookupTransform("map","cameraTop",rospy.Time())
#		camera_Map_orientation_euler = tf.transformations.euler_from_quaternion(camera_Map_position[1])
#
#		camera_Nao_orientation_euler = tf.transformations.euler_from_quaternion(camera_Nao_position[1])
#
#
#
#		###
#		#   OLD
#		###
#
#		dist2D = numpy.sqrt((pointX - camera_Map_position[0][0])*(pointX - camera_Map_position[0][0])+(pointY - camera_Map_position[0][1])*(pointY - camera_Map_position[0][1]))
#		#    *  - point
#		#    |  }
#		#    |  }  h = pointZ - NaoCamera
#		#    o    - Nao camera
#		h = pointZ - camera_Map_position[0][2]#camera_position[2]-nao_position[0][2]
#
#		#print "= ",robot_orientation_euler[2]
#		#print "torso h",nao_position[0][2]
#
#

#		gamma = camera_Map_orientation_euler[2]
#		blabla = camera_Nao_orientation_euler[2]
#		alpha = numpy.arctan2(pointY-camera_Map_position[0][1],pointX-camera_Map_position[0][0])
#		print "gamma = ",gamma, "alpha = ",alpha, "blabla = ",blabla
#		if abs(gamma)> abs(alpha):
#		 	theta = -1*(gamma - alpha)
#		elif abs(gamma)< abs(alpha):
#			theta = (alpha - gamma)
#		else:
#			theta =0
#		if abs(theta) > 3.14:
#			print"\n theta > 3.14\n"
#			theta = theta-(numpy.sign(theta)*2*numpy.pi)
#		print "theta = ", theta
#		head_yaw = theta + camera_Nao_orientation_euler[2]#sign*(2*numpy.pi - (abs(theta) + abs(robot_orientation_euler[2])+abs(camera_orientation_euler[2])))

#		head_pitch = -numpy.arctan(h/dist2D) - robot_orientation_euler[1]
#		turnHeadAngles = [head_yaw,head_pitch]
#		return turnHeadAngles

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
		#define ranges     YAW    Pitch_min  Pitch_max
		range_matrix = [-2.086017,-0.449073,0.330041,
						-1.526988,-0.330041,0.200015,
						-1.089958,-0.430049,0.300022,
						-0.903033,-0.479965,0.330041,
						-0.756077,-0.548033,0.370010,
						-0.486074,-0.641951,0.422021,
						0.000000,-0.671951,0.515047]
		#check if pitch component can be reached by Nao
		if not(head_pitch > range_matrix[19] and head_pitch < range_matrix[20]):
			print "Nao cant reach the pitch angle"
			status = True
			return LookAtPointResponse(status)	
		#check if Nao has to turn around to reach the point
		if ((abs(head_yaw) < abs(range_matrix[0])) or (abs(head_yaw) <= 0.2) ):
			canLookAtPoint_yaw = True
		else:
			canLookAtPoint_yaw = False

		i=1
		#search the matrix for pitch boundaries for the desired head yaw position 
		while i <= 6 :
			if abs(head_yaw) >= abs(range_matrix[i*3]) and canLookAtPoint_yaw:
				head_pitch_max = range_matrix[(i-1)*3+2]
				head_pitch_min = range_matrix[(i-1)*3+1]
				break
			i+=1			


			# Nao can reach head_yaw and head_pitch
		if (head_pitch > head_pitch_min and head_pitch < head_pitch_max and canLookAtPoint_yaw):

			self.rapp_move_joint_interface(["Head"],[head_yaw,head_pitch],0.2)
			status = False

			# Nao will rotate to be ahead the point in yaw direction, then he will look at it
		else:
			thetaTime = abs(head_yaw)/0.4
			moveVel_trigger = self.rapp_move_vel_interface(0,0,0.4*numpy.sign(head_yaw))

			rospy.sleep(thetaTime)
			self.proxy_motion.post.move(0,0,0)
			
			turnHeadAngles = self.compute_turn_head_angles([pointX,pointY,pointZ])	
			head_yaw = turnHeadAngles[0]
			head_pitch = turnHeadAngles[1]

			self.rapp_move_joint_interface(["Head"],[head_yaw,head_pitch],0.2)
			status = False	

		return LookAtPointResponse(status)	
		

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
