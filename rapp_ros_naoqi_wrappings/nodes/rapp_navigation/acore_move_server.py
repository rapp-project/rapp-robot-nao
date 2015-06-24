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
#from navfn.srv import *

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
import geometry_msgs
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
		fractionMaxSpeed = 0.1
		self.proxy_motion.setAngles(req.joint_name,req.joint_angle,fractionMaxSpeed)
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
