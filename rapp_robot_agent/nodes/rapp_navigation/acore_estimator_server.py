#!/usr/bin/env python
######################
## written by Wojciech Dudek
######################
__author__ = "Wojciech Dudek"

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf import transformations
import tf
import rospy
import sys
import signal
import motion
from rapp_ros_naoqi_wrappings.srv import GetImage
from rapp_ros_naoqi_wrappings.srv import DetectQRcodes
from rapp_ros_naoqi_wrappings.srv import GetTransform
from rapp_ros_naoqi_wrappings.srv import VisOdom,VisOdomResponse
from rapp_ros_naoqi_wrappings.srv import MoveHead
from rapp_ros_naoqi_wrappings.srv import MoveVel,TakePredefinedPosture
import threading 

from sensor_msgs.msg import Image
from geometry_msgs.msg import *
from tf2_msgs.msg import TFMessage
from naoqi import ALModule
from naoqi import ALBroker
from naoqi import ALProxy
import numpy as np
# Constants
class Constants:

	NAO_IP = "nao.local"
	PORT = 9559

class NaoEstimator(ALModule):
	def __init__(self,name):
		ALModule.__init__(self, name)
		rospy.init_node('acore_estimator')
		self.moduleName = name
		self.connectNaoQi()
		self.setVariables()
		self.dataNamesList =   ["DCM/Time",
						"Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value",
						"Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value",
						"Device/SubDeviceList/InertialSensor/AngleZ/Sensor/Value",
						"Device/SubDeviceList/InertialSensor/GyroscopeX/Sensor/Value", 
						"Device/SubDeviceList/InertialSensor/GyroscopeY/Sensor/Value",
						"Device/SubDeviceList/InertialSensor/GyroscopeZ/Sensor/Value",
						"Device/SubDeviceList/InertialSensor/AccelerometerX/Sensor/Value",
						"Device/SubDeviceList/InertialSensor/AccelerometerY/Sensor/Value",
						"Device/SubDeviceList/InertialSensor/AccelerometerZ/Sensor/Value"]
		
		self.FSRdataList = ["Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/Value",
							"Device/SubDeviceList/LFoot/FSR/FrontRight/Sensor/Value",
							"Device/SubDeviceList/LFoot/FSR/RearLeft/Sensor/Value",
							"Device/SubDeviceList/LFoot/FSR/RearRight/Sensor/Value",
							"Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value",
							"Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/Value",
							"Device/SubDeviceList/RFoot/FSR/RearLeft/Sensor/Value",
							"Device/SubDeviceList/RFoot/FSR/RearRight/Sensor/Value"]
		self.MsgsInit()
		self.startSubscribers()
		self.openServices()
		#self.run()

	def connectNaoQi(self):
		self.motionProxy = ALProxy("ALMotion")
		if self.motionProxy is None:
			rospy.logerr("[Estimator server] - Could not get a proxy to ALMemory")
			exit(1)

		self.memProxy = ALProxy("ALMemory")
		if self.motionProxy is None or self.memProxy is None:
			exit(1)
	def setVariables(self):
		self.markers=["Wall","Door","Wardrobe","Stable object","Microwave","Fridge"]
		self.tl = tf.TransformListener(True, rospy.Duration(20.0))

		# stare zmienne - do przejrzenia
		self.constPose = PoseWithCovarianceStamped()
		self.constPose.pose.pose.position.x=0
		self.constPose.pose.pose.position.y=0
		self.constPose.pose.pose.orientation.z=0
		self.constPose.pose.pose.orientation.w=1

		self.odom_transformation = Pose()
		self.odom_transformation.position = [0,0,0]
		self.odom_transformation.orientation = [0,0,0,1]

		self.odom_combined_transformation = Pose()
		self.odom_combined_transformation.position = [0,0,0]
		self.odom_combined_transformation.orientation = [0,0,0,1]

		# self.thread_publishOdom = threading.Thread(None,self.publishOdom,None)
		# self.kill_publishOdom = False
		#setting initial data for odom_EKF
		iterations_between_EKF_publish = 0
		# self.torsoOdom_EKF.pose.pose.position.x = 0
		# self.torsoOdom_EKF.pose.pose.position.y = 0
		# self.torsoOdom_EKF.pose.pose.position.z = 0
		# self.torsoOdom_EKF.pose.pose.orientation.x = 0
		# self.torsoOdom_EKF.pose.pose.orientation.y = 0
		# self.torsoOdom_EKF.pose.pose.orientation.z = 0
		# self.torsoOdom_EKF.pose.pose.orientation.w = 0


	def startSubscribers(self):
		rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.SubCall)
		rospy.Subscriber("/odometry/filtered", Odometry, self.publishEKFframe)

	def SubCall(self,data):

		self.constPose.pose.pose.position = data.pose.pose.position
		self.constPose.pose.pose.orientation = data.pose.pose.orientation
		#
		# handle inirial pose
		#
		self.euler_initial = tf.transformations.euler_from_quaternion((0,0,self.constPose.pose.pose.orientation.z,self.constPose.pose.pose.orientation.w))
		#
		# find fransformation from odom to Nao_T_odom
		#
		if self.tl.canTransform("odom","base_link",rospy.Time()):
			transform_Nao_odom = self.tl.lookupTransform("base_link","odom", rospy.Time())
			euler_transform_Nao_odom =  tf.transformations.euler_from_quaternion(transform_Nao_odom[1])
			#
			# calculate new odom position, so Nao_T_odom will be in pointed position
			#
			matrix_Nao_odom= np.linalg.pinv(np.array([[np.cos(euler_transform_Nao_odom[2]),-np.sin(euler_transform_Nao_odom[2]),0,transform_Nao_odom[0][0]],
												[np.sin(euler_transform_Nao_odom[2]),np.cos(euler_transform_Nao_odom[2]),0,transform_Nao_odom[0][1]],
												[0,0,0,transform_Nao_odom[0][2]],
												[0,0,0,1]]))

			self.odom_transformation.position = [self.constPose.pose.pose.position.x+np.cos(self.euler_initial[2])*transform_Nao_odom[0][0]-np.sin(self.euler_initial[2])*transform_Nao_odom[0][1],
												self.constPose.pose.pose.position.y+np.sin(self.euler_initial[2])*transform_Nao_odom[0][0]+np.cos(self.euler_initial[2])*transform_Nao_odom[0][1],
												0]
			self.odom_transformation.orientation = tf.transformations.quaternion_from_euler(0,0,euler_transform_Nao_odom[2]+self.euler_initial[2])#matrix_Nao_odom[0][1]/matrix_Nao_odom[0][0])#+self.euler_initial[2])

		# #
		# # find fransformation from odom_combined to base_link
		# #
		# if self.tl.canTransform("odom_combined","base_link",rospy.Time()):
		# 	transform_Nao_odom_combined = self.tl.lookupTransform("base_link","odom_combined", rospy.Time())
		# 	euler_transform_Nao_odom_combined =  tf.transformations.euler_from_quaternion(transform_Nao_odom_combined[1])
		# 	#
		# 	# calculate new odom position, so Nao_T_odom will be in pointed position
		# 	#
		# 	matrix_Nao_odom_combined= np.linalg.pinv(np.array([[np.cos(euler_transform_Nao_odom_combined[2]),-np.sin(euler_transform_Nao_odom_combined[2]),0,transform_Nao_odom_combined[0][0]],
		# 										[np.sin(euler_transform_Nao_odom_combined[2]),np.cos(euler_transform_Nao_odom_combined[2]),0,transform_Nao_odom_combined[0][1]],
		# 										[0,0,0,transform_Nao_odom_combined[0][2]],
		# 										[0,0,0,1]]))

		# 	self.odom_combined_transformation.position = [self.constPose.pose.pose.position.x+np.cos(self.euler_initial[2])*transform_Nao_odom_combined[0][0]-np.sin(self.euler_initial[2])*transform_Nao_odom_combined[0][1],
		# 										self.constPose.pose.pose.position.y+np.sin(self.euler_initial[2])*transform_Nao_odom_combined[0][0]+np.cos(self.euler_initial[2])*transform_Nao_odom_combined[0][1],
		# 										0]
		# 	self.odom_combined_transformation.orientation = tf.transformations.quaternion_from_euler(0,0,euler_transform_Nao_odom_combined[2]+self.euler_initial[2])#matrix_Nao_odom[0][1]/matrix_Nao_odom[0][0])#+self.euler_initial[2])
	def publishEKFframe(self,data):
		#set initial data for two rotations taken from odometry
		ekf_orientation_euler = tf.transformations.euler_from_quaternion((data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w))
		ekf_orientation_z_euler = ekf_orientation_euler[2]
		marged_angles = tf.transformations.quaternion_from_euler(self.odomData[3], self.odomData[4],ekf_orientation_z_euler)
		 
		self.tf_br.sendTransform((data.pose.pose.position.x,data.pose.pose.position.y,self.odomData[2]), marged_angles,
                                      rospy.Time.now(), "base_link_nao", "odom")
	def MsgsInit(self):
		# init. messages:
		self.torsoOdom = Odometry()
		self.torsoOdom.header.frame_id = "odom"
		self.torsoOdom.child_frame_id = "Nao_T_odom"
		self.torsoOdomPub = rospy.Publisher("odom", Odometry, queue_size=10)

		self.torsoOdom_EKF = Odometry()
		self.torsoOdom_EKF.header.frame_id = "odom"
		self.torsoOdom_EKF.child_frame_id = "Nao_T_odom"
		self.torsoOdom_EKF_Pub = rospy.Publisher("odom_EKF", Odometry, queue_size=10)

		self.torsoIMU = Imu()
		self.torsoIMU.header.frame_id = "base_link"
		self.torsoIMUPub = rospy.Publisher("imu_data", Imu, queue_size=10)

		self.torsoIMU_EKF = Imu()
		self.torsoIMU_EKF.header.frame_id = "Nao_T_odom"
		self.torsoIMU_EKF_Pub = rospy.Publisher("imu_data_EKF", Imu, queue_size=10)

		self.tf_br = tf.TransformBroadcaster()

		self.ODOM_POSE_COVARIANCE = [1e-3, 0, 0, 0, 0, 0, 
		                        0, 1e-3, 0, 0, 0, 0,
		                        0, 0, 1e6, 0, 0, 0,
		                        0, 0, 0, 1e6, 0, 0,
		                        0, 0, 0, 0, 1e6, 0,
		                        0, 0, 0, 0, 0, 1e3]
		self.ODOM_POSE_COVARIANCE2 = [1e-9, 0, 0, 0, 0, 0, 
		                         0, 1e-3, 1e-9, 0, 0, 0,
		                         0, 0, 1e6, 0, 0, 0,
		                         0, 0, 0, 1e6, 0, 0,
		                         0, 0, 0, 0, 1e6, 0,
		                         0, 0, 0, 0, 0, 1e-9]
		self.ODOM_TWIST_COVARIANCE = [1e-3, 0, 0, 0, 0, 0, 
		                         0, 1e-3, 0, 0, 0, 0,
		                         0, 0, 1e6, 0, 0, 0,
		                         0, 0, 0, 1e6, 0, 0,
		                         0, 0, 0, 0, 1e6, 0,
		                         0, 0, 0, 0, 0, 1e3]
		self.ODOM_TWIST_COVARIANCE2 = [1e-9, 0, 0, 0, 0, 0, 
		                          0, 1e-3, 1e-9, 0, 0, 0,
		                          0, 0, 1e6, 0, 0, 0,
		                          0, 0, 0, 1e6, 0, 0,
		                          0, 0, 0, 0, 1e6, 0,
		                          0, 0, 0, 0, 0, 1e-9]
	def openServices(self):
		try:
			print "[Estimator server] - checking required services"
			print "[Estimator server] - waiting for service 'rapp_moveHead' from acore_move_server.py"
			rospy.wait_for_service('rapp_moveHead', timeout=None)
			print "[Estimator server] - service 'rapp_moveHead' is enabled"
			print "[Estimator server] - waiting for service 'rapp_capture_image' from core_agent_camera_server.launch"
			rospy.wait_for_service('rapp_capture_image', timeout=None)
			print "[Estimator server] - service 'rapp_capture_Image' is enabled"
			print "[Estimator server] - waiting for service 'getTransform' from core_agent_camera_server.launch"
			rospy.wait_for_service('rapp_get_transform', timeout=None)
			print "[Estimator server] - service 'getTransform' is enabled"
			print "[Estimator server] - waiting for service 'rapp_detect_qrcodes' from dyn_agent_qrcode_server.launch"
			rospy.wait_for_service('rapp_detect_qrcodes', timeout=None)
			print "[Estimator server] - service 'rapp_detect_qrcodes' is enabled"
			print "[Estimator server] - setting services"
			print "[Estimator server] - service - [rapp_VisOdom]"
			self.service_get = rospy.Service('rapp_VisOdom', VisOdom, self.handle_rapp_VisOdom)
		except Exception, ex_mt:
			print "[Estimator server] - Exception %s" % str(ex)
	def rapp_take_predefined_pose_interface(self,pose):
		takePosture = rospy.ServiceProxy('rapp_takePredefinedPosture', TakePredefinedPosture)
		resp1 = takePosture(pose)
	def rapp_move_vel_interface(self,v_x,v_y,v_theta):
		move = rospy.ServiceProxy('rapp_moveVel', MoveVel)
		speeds = MoveVelRequest()
		speeds.velocity_x = v_x
		speeds.velocity_y = v_y
		speeds.velocity_theta = v_theta
		resp1 = move(speeds)
	def handle_rapp_VisOdom(self,req):
		#transform = self.locateMarkers()
		self.rapp_take_predefined_pose_interface("Stand")
		decomposed_robot_in_QR = self.estimateNaoPosition()
		self.moveOdom(decomposed_robot_in_QR)
		feedback = True
		return VisOdomResponse(feedback)

	# def locateMarkers(self):
	# 	# Find closest marker to "base_link" frame
	# 	min_dist = 1000
	# 	can_locate = False
	# 	while can_locate == False:
	# 		if self.tl.canTransform("base_link","Wall",rospy.Time()):
	# 			i=0
	# 			can_locate = True
	# 			for i in range(len(self.markers)):
	# 				i+=1

	# 				transform = self.tl.lookupTransform("base_link", self.markers[i-1], rospy.Time())
	# 				if reachable
	# 				distance= np.sqrt(transform[0][0]*transform[0][0]+transform[0][1]*transform[0][1])
	# 				if min_dist > distance:
	# 					min_dist = distance
	# 					marker_id = i-1
	# 					marker_transformation = transform
	# 				elif min_dist == 1000:
	# 					i=0
	# 					print "min_dist == 1000 "
	# 	print "Closest marker : %s" %(self.markers[i-1])
	# 	return transform

	def turnNaoHead(self,head_yaw,head_pitch):
		moveNaoHead = rospy.ServiceProxy('rapp_moveHead', MoveHead)
		resp1 = moveNaoHead(head_yaw,head_pitch)

	def estimateNaoPosition(self):
		# get image
		head_yaw = 0
		head_pitch = 0
		head_yaw_max = 2.08
		gotQRcode = False
		resetHeadPosition = False
		while gotQRcode!=True:
			self.turnNaoHead(head_yaw,head_pitch)
			getImage = rospy.ServiceProxy('rapp_capture_image', GetImage)
			image_response= getImage("top - adaptive auto exposure 2",3)
			frame = image_response.frame
			if (frame.height == 0 or frame.width == 0):    # //frame is empty
				print "Camera frame is empty"
			else:
				print "QRcode detection ..."
				detectQRcodes = rospy.ServiceProxy('rapp_detect_qrcodes',DetectQRcodes)
				QRcodeDetectionStruct = detectQRcodes(frame)
				print "number of QRcodes: \n",QRcodeDetectionStruct.numberOfQRcodes
				# Is Qrcode found
				if QRcodeDetectionStruct.numberOfQRcodes > 0:
					gotQRcode = True
					#get transform camera -> torso
					robot_camera_transform = self.motionProxy.getTransform("CameraTop", 0, True)
					matrix_camera_torso = (np.array([[robot_camera_transform[0],robot_camera_transform[1],robot_camera_transform[2],robot_camera_transform[3]],
											[robot_camera_transform[4],robot_camera_transform[5],robot_camera_transform[6],robot_camera_transform[7]],
											[robot_camera_transform[5],robot_camera_transform[6],robot_camera_transform[7],robot_camera_transform[8]],
											[robot_camera_transform[9],robot_camera_transform[10],robot_camera_transform[11],robot_camera_transform[12]]]))
					print "matrix camera->torso: \n",matrix_camera_torso
					#compute robot position in QRcode frame
					self.VO_frame_ID = QRcodeDetectionStruct.message[0]
					print "Localising via : [",self.VO_frame_ID ,"] position"

					matrix_R_in_C_Rz = np.array([[np.cos((3.14/2)-head_yaw),-np.sin((3.14/2)-head_yaw),0,0],
										 [np.sin((3.14/2)-head_yaw), np.cos((3.14/2)-head_yaw),0,0],
										 [0		  ,0        ,1,0],
										 [0		  ,0        ,0,1]])	
					matrix_R_in_C_Rx = np.array([[1,0,0,0],
											[0,np.cos((3.14/2)+head_pitch),-np.sin((3.14/2)+head_pitch),0],
											[0,np.sin((3.14/2)+head_pitch),np.cos((3.14/2)+head_pitch),0],
											[0,0,0,1]])		
					matrix_R_in_C_translation = np.array([[1,0,0,robot_camera_transform[7]],
											[0,1,0,-robot_camera_transform[3]],
											[0,0,1,-0.2],
											[0,0,0,1]])	
					matrix_camera_in_QR =  np.linalg.pinv(np.array([[QRcodeDetectionStruct.cameraToQRcode.r11[0],QRcodeDetectionStruct.cameraToQRcode.r12[0],QRcodeDetectionStruct.cameraToQRcode.r13[0],QRcodeDetectionStruct.cameraToQRcode.r14[0]],
												[QRcodeDetectionStruct.cameraToQRcode.r21[0],QRcodeDetectionStruct.cameraToQRcode.r22[0],QRcodeDetectionStruct.cameraToQRcode.r23[0],QRcodeDetectionStruct.cameraToQRcode.r24[0]],
												[QRcodeDetectionStruct.cameraToQRcode.r31[0],QRcodeDetectionStruct.cameraToQRcode.r32[0],QRcodeDetectionStruct.cameraToQRcode.r33[0],QRcodeDetectionStruct.cameraToQRcode.r34[0]],
												[QRcodeDetectionStruct.cameraToQRcode.r41[0],QRcodeDetectionStruct.cameraToQRcode.r42[0],QRcodeDetectionStruct.cameraToQRcode.r43[0],QRcodeDetectionStruct.cameraToQRcode.r44[0]]]))
					print "Rotation matrix: camera in QRcode: \" %s \" : \n"%(QRcodeDetectionStruct.message[0]), matrix_camera_in_QR

					matrix_robot_in_QR = (np.dot(matrix_camera_in_QR,matrix_R_in_C_Rx))
					matrix_robot_in_QR = (np.dot(matrix_robot_in_QR,matrix_R_in_C_translation))
					matrix_robot_in_QR = (np.dot(matrix_robot_in_QR,matrix_R_in_C_Rz))
					print "Rotation matrix: Torso in QRcode: \" %s \" : \n"%(QRcodeDetectionStruct.message[0]), matrix_robot_in_QR
					# decompose matrix
					decomposed_robot_in_QR=self.decompose_matrix(matrix_robot_in_QR)
					decomposed_camera_in_QR=self.decompose_matrix(matrix_camera_in_QR)

					# send transform  Camera in QRcode position
					self.tf_br.sendTransform((decomposed_camera_in_QR[0][0],decomposed_camera_in_QR[0][1],decomposed_camera_in_QR[0][2]), 
											tf.transformations.quaternion_from_euler(decomposed_camera_in_QR[1][0],
																					decomposed_camera_in_QR[1][1],
																					decomposed_camera_in_QR[1][2]),
		                                         rospy.Time.now(), "QR_camera", QRcodeDetectionStruct.message[0])

					# send transform Rrbot in QRcode position
					self.tf_br.sendTransform((decomposed_robot_in_QR[0][0],decomposed_robot_in_QR[0][1],decomposed_robot_in_QR[0][2]), 
											tf.transformations.quaternion_from_euler(decomposed_robot_in_QR[1][0],
																					decomposed_robot_in_QR[1][1],
																					decomposed_robot_in_QR[1][2]),
		                                         rospy.Time.now(), "QR_Torso", QRcodeDetectionStruct.message[0])
					self.marker_id = QRcodeDetectionStruct.message[0]

					return decomposed_robot_in_QR
				else:
					gotQRcode = False
					head_yaw += 3.14/5 # turn head + 0.628 rad | + 36 deg
					if head_yaw > head_yaw_max:
						head_yaw_max_set = 0
						if resetHeadPosition == True: # if no QRcode avaliable in robot position, move robot +180 deg
							#moveNao = rospy.ServiceProxy('rapp_moveVel', MoveVel)
			
							self.rapp_move_vel_interface(0,0,0.4)
						#	moveVel_resp = moveNao(0,0,0.4)
							rospy.sleep(3.14/0.4)
						
							self.rapp_move_vel_interface(0,0,0)		
							self.rapp_take_predefined_pose_interface("Stand")
							#print "rapp_moveVel response: \n",moveVel_resp
							head_yaw_max_set = 2.08
						resetHeadPosition = True
						head_yaw = -2.08
						head_yaw_max = head_yaw_max_set

	def moveOdom(self,matrix):
		time = rospy.Time.now()
		self.tf_br.sendTransform((matrix[0][0],matrix[0][1],matrix[0][2]), 
									tf.transformations.quaternion_from_euler(matrix[1][0],
																			matrix[1][1],
																			matrix[1][2]),
                                         time, "QR_Torso", self.marker_id )
		rospy.sleep(1)
		if self.tl.canTransform("QR_Torso",self.marker_id ,time):
			transform_QR_map = self.tl.lookupTransform("map","QR_Torso", time)
			euler_transform_QR_map =  tf.transformations.euler_from_quaternion(transform_QR_map[1])
			if self.tl.canTransform("odom","base_link",rospy.Time()):
				transform_Nao_odom = self.tl.lookupTransform("base_link","odom", rospy.Time())
				euler_transform_Nao_odom =  tf.transformations.euler_from_quaternion(transform_Nao_odom[1])
				#
				# calculate new odom position, so Nao_T_odom will be in pointed position
				#
				matrix_Nao_odom= np.linalg.pinv(np.array([[np.cos(euler_transform_Nao_odom[2]),-np.sin(euler_transform_Nao_odom[2]),0,transform_Nao_odom[0][0]],
													[np.sin(euler_transform_Nao_odom[2]),np.cos(euler_transform_Nao_odom[2]),0,transform_Nao_odom[0][1]],
													[0,0,0,transform_Nao_odom[0][2]],
													[0,0,0,1]]))

				self.odom_transformation.position = [transform_QR_map[0][0]+np.cos(euler_transform_QR_map[2])*transform_Nao_odom[0][0]-np.sin(euler_transform_QR_map[2])*transform_Nao_odom[0][1],
													transform_QR_map[0][1]+np.sin(euler_transform_QR_map[2])*transform_Nao_odom[0][0]+np.cos(euler_transform_QR_map[2])*transform_Nao_odom[0][1],
													0]
				self.odom_transformation.orientation = tf.transformations.quaternion_from_euler(0,0,euler_transform_Nao_odom[2]+euler_transform_QR_map[2])#matrix_Nao_odom[0][1]/matrix_Nao_odom[0][0])#+self.euler_initial[2])

			# if self.tl.canTransform("odom","base_link",rospy.Time()):
			# 	transform_Nao_odom_combined = self.tl.lookupTransform("base_link","odom_combined", rospy.Time())
			# 	euler_transform_Nao_odom_combined =  tf.transformations.euler_from_quaternion(transform_Nao_odom[1])
			# 	#
			# 	# calculate new odom position, so Nao_T_odom will be in pointed position
			# 	#
			# 	matrix_Nao_odom= np.linalg.pinv(np.array([[np.cos(euler_transform_Nao_odom_combined[2]),-np.sin(euler_transform_Nao_odom_combined[2]),0,transform_Nao_odom_combined[0][0]],
			# 										[np.sin(euler_transform_Nao_odom_combined[2]),np.cos(euler_transform_Nao_odom_combined[2]),0,transform_Nao_odom_combined[0][1]],
			# 										[0,0,0,transform_Nao_odom_combined[0][2]],
			# 										[0,0,0,1]]))

			# 	self.odom_transformation.position = [transform_QR_map[0][0]+np.cos(euler_transform_QR_map[2])*transform_Nao_odom_combined[0][0]-np.sin(euler_transform_QR_map[2])*transform_Nao_odom_combined[0][1],
			# 										transform_QR_map[0][1]+np.sin(euler_transform_QR_map[2])*transform_Nao_odom_combined[0][0]+np.cos(euler_transform_QR_map[2])*transform_Nao_odom_combined[0][1],
			# 										0]
			# 	self.odom_transformation.orientation = tf.transformations.quaternion_from_euler(0,0,euler_transform_Nao_odom_combined[2]+euler_transform_QR_map[2])#matrix_Nao_odom[0][1]/matrix_Nao_odom[0][0])#+self.euler_initial[2])

	def decompose_matrix(self,matrix):
		euler_x = np.arctan2(matrix[2][1],matrix[2][2])
		euler_y = np.arctan2(-matrix[2][0],np.sqrt(matrix[2][1]*matrix[2][1]+matrix[2][2]*matrix[2][2]))
		euler_z = np.arctan2(matrix[1][0],matrix[0][0])
		position_x = matrix[0][3]
		position_y = matrix[1][3]
		position_z = matrix[2][3]
		decomposed_matrix = np.array([[position_x,position_y,position_z],[euler_x,euler_y,euler_z]])
		return (decomposed_matrix)
	def run(self):
		
		#print "run",not rospy.is_shutdown()
		
		self.timestamp = rospy.Time.now()
		###
		#   Get data from NaoQi Driver
		###
		try:
			# imu data:
			self.memData = self.memProxy.getListData(self.dataNamesList)
			t = rospy.Time.now().to_sec()

			# odometry data:
			self.odomData = self.motionProxy.getPosition('Torso', motion.SPACE_WORLD, True)
			t1 = rospy.Time.now().to_sec()

			d_t = t1 -t
			if d_t> 0.005:
				print "naoqi",d_t

			# FSR data
			#self.FSRData = self.memProxy.getListData(self.FSRdataList)
			#positionData = self.motionProxy.getAngles('Body', True)
				
		except RuntimeError, e:
			print "Error accesnp.sing ALMemory, exiting...\n"
			print e
			rospy.signal_shutdown("No NaoQI available anymore")

		# print "FSR:"
		# print "left: ", (self.FSRData[0]+self.FSRData[1]+self.FSRData[2]+self.FSRData[3])/4
		# print "right: ", (self.FSRData[4]+self.FSRData[5]+self.FSRData[6]+self.FSRData[7])/4
		#NaoStandStable = bool((self.FSRData[0]+self.FSRData[1]+self.FSRData[2]+self.FSRData[3])/4 > 0.2 and (self.FSRData[4]+self.FSRData[5]+self.FSRData[6]+self.FSRData[7])/4 >0.2)
		# print  NaoStandStable

			
		###
		#  Fill msgs data
		###
		# /odom

		self.torsoOdom.header.stamp = self.timestamp
		self.torsoOdom.pose.pose.position.x = self.odomData[0]
		self.torsoOdom.pose.pose.position.y = self.odomData[1]
		self.torsoOdom.pose.pose.position.z = 0#self.odomData[2]
		self.q_odom = transformations.quaternion_from_euler(self.odomData[3], self.odomData[4], self.odomData[5])
		self.torsoOdom.pose.pose.orientation.x = self.q_odom[0]
		self.torsoOdom.pose.pose.orientation.y = self.q_odom[1]
		self.torsoOdom.pose.pose.orientation.z = self.q_odom[2]
		self.torsoOdom.pose.pose.orientation.w = self.q_odom[3]
		#print "orientation odom: z: \n",self.odomData[5]
		ODOM_POSE_COVARIANCE = [1e-2, 0, 0, 0, 0, 0, 
		                        0, 1e-2, 0, 0, 0, 0,
		                        0, 0, 1e-3, 0, 0, 0,
		                        0, 0, 0, 1e6, 0, 0,
	                        0, 0, 0, 0, 1e6, 0,
		                        0, 0, 0, 0, 0, 1e-2]
		self.torsoOdom.pose.covariance =  ODOM_POSE_COVARIANCE
			
		# /odom_EKF

		self.torsoOdom_EKF.header.stamp = self.timestamp
		self.torsoOdom_EKF.pose.pose.position.x = self.torsoOdom_EKF.pose.pose.position.x + self.odomData[0]
		self.torsoOdom_EKF.pose.pose.position.y = self.torsoOdom_EKF.pose.pose.position.y+ self.odomData[1]
		self.torsoOdom_EKF.pose.pose.position.z = self.torsoOdom_EKF.pose.pose.position.z+ self.odomData[2]
		self.q_odom = transformations.quaternion_from_euler(self.odomData[3], self.odomData[4], self.odomData[5])
		self.torsoOdom_EKF.pose.pose.orientation.x =self.q_odom[0]# +  self.torsoOdom_EKF.pose.pose.orientation.x
		# self.torsoOdom_EKF.pose.pose.orientation.y = self.q_odom[1]# +  self.torsoOdom_EKF.pose.pose.orientation.x
		# self.torsoOdom_EKF.pose.pose.orientation.z = self.q_odom[2]# +  self.torsoOdom_EKF.pose.pose.orientation.x
		# self.torsoOdom_EKF.pose.pose.orientation.w = self.q_odom[3]# +  self.torsoOdom_EKF.pose.pose.orientation.x
		# #print "orientation odom: z: \n",self.odomData[5]
		# ODOM_POSE_COVARIANCE = [1e-4, 0, 0, 0, 0, 0, 
		#                         0, 1e-4, 0, 0, 0, 0,
		#                         0, 0, 1e-3, 0, 0, 0,
		#                         0, 0, 0, 1e6, 0, 0,
		#                         0, 0, 0, 0, 1e6, 0,
		#                         0, 0, 0, 0, 0, 1e-2]
		# self.torsoOdom_EKF.pose.covariance =  ODOM_POSE_COVARIANCE


		# /imu_data
		self.torsoIMU.header.stamp = self.timestamp

		q_IMU =transformations.quaternion_from_euler(self.memData[1], self.memData[2], -self.memData[3])#+self.euler_substract_IMU[2])
		self.torsoIMU.orientation.x = q_IMU[0]
		self.torsoIMU.orientation.y = q_IMU[1]
		self.torsoIMU.orientation.z = q_IMU[2]
		self.torsoIMU.orientation.w = q_IMU[3]
		self.torsoIMU.angular_velocity.x = self.memData[4]
		self.torsoIMU.angular_velocity.y = self.memData[5]
		self.torsoIMU.angular_velocity.z = -self.memData[6] 
		self.torsoIMU.linear_acceleration.x = self.memData[7]
		self.torsoIMU.linear_acceleration.y = self.memData[8]
		self.torsoIMU.linear_acceleration.z = -self.memData[9]
		# cf http://www.ros.org/doc/api/sensor_msgs/html/msg/Imu.html
		self.torsoIMU.orientation_covariance = [1e-5,0,0,
													0,1e-5,0,
													0,0,1e-3]
		self.torsoIMU.angular_velocity_covariance= [1e-3,0,0,
														0,1e-3,0,
														0,0,1e-4]
		self.torsoIMU.linear_acceleration_covariance = [1e-3,0,0,
															0,1e2,0,
															0,0,1e2]

					# /imu_data_EKF
		self.torsoIMU_EKF.header.stamp = self.timestamp
		q_IMU =transformations.quaternion_from_euler(self.memData[1], self.memData[2], -self.memData[3])#+self.euler_substract_IMU[2])
		self.torsoIMU_EKF.orientation.x = q_IMU[0]
		self.torsoIMU_EKF.orientation.y = q_IMU[1]
		self.torsoIMU_EKF.orientation.z = q_IMU[2]
		self.torsoIMU_EKF.orientation.w = q_IMU[3]
		self.torsoIMU_EKF.angular_velocity.x = self.memData[4]
		self.torsoIMU_EKF.angular_velocity.y = self.memData[5]
		self.torsoIMU_EKF.angular_velocity.z = -self.memData[6] 
		self.torsoIMU_EKF.linear_acceleration.x = self.memData[7]
		self.torsoIMU_EKF.linear_acceleration.y = self.memData[8]
		self.torsoIMU_EKF.linear_acceleration.z = -self.memData[9]
		# cf http://www.ros.org/doc/api/sensor_msgs/html/msg/Imu.html
		self.torsoIMU_EKF.orientation_covariance = [1e-5,0,0,
												0,1e-5,0,
												0,0,1e-5]
		self.torsoIMU_EKF.angular_velocity_covariance= [1e-3,0,0,
													0,1e-3,0,
													0,0,1e-4]
		self.torsoIMU_EKF.linear_acceleration_covariance = [1e-3,0,0,
														0,1e2,0,
														0,0,1e2]

		# if self.tl.canTransform("Nao_T_odom","base_link" ,rospy.Time.now()):
		# 	print "can transform"
		# else:
		# 	print "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
			#transform_QR_map = self.tl.lookupTransform("Nao_T_odom","base_link", time)
		###
		#  Set required frames IDs
		###
		# self.tf_br.sendTransform((0,0,0), (0,0,0,1),
  #                                        self.timestamp, "map", "World")	
		# self.tf_br.sendTransform(self.odom_transformation.position, self.odom_transformation.orientation,
  #                                        self.timestamp, "odom", "map")	
			# self.tf_br.sendTransform(self.odom_combined_transformation.position, self.odom_combined_transformation.orientation,
	  #                                        self.timestamp, "odom_combined", "map")
		# self.tf_br.sendTransform((self.torsoOdom.pose.pose.position.x,self.torsoOdom.pose.pose.position.y,self.odomData[2]), self.q_odom,
	                                       # self.timestamp, "Nao_T_odom", "odom")
		self.camera_To_Torso_Position = self.motionProxy.getPosition('CameraTop', 0, True)
		self.quaternion_cameta_to_torso = tf.transformations.quaternion_from_euler(self.camera_To_Torso_Position[3],self.camera_To_Torso_Position[4],self.camera_To_Torso_Position[5])
		self.start_publishingOdom = True	
		t2 = rospy.Time.now().to_sec()

		d_t = t2-t1
		if d_t> 0.005:
			print "camera_nao",d_t
		# self.tf_br.sendTransform((self.camera_To_Torso_Position[0],self.camera_To_Torso_Position[1],self.camera_To_Torso_Position[2]), self.quaternion_cameta_to_torso,
	 #                                       self.timestamp, "cameraTop", "base_link_nao")
		# 	#self.tf_br.sendTransform((self.torsoOdom.pose.pose.position.x,self.torsoOdom.pose.pose.position.y,self.odomData[2]), self.q_odom,
	 #      #                               self.timestamp, "Nao_T_odom", "odom")	

		# 	#
		# 	#  publishing msgs 
		# 	#
		# 	#/odom
		# self.torsoOdomPub.publish(self.torsoOdom)		
		# 	#/imu_data
		# self.torsoIMUPub.publish(self.torsoIMU)
		# # if abs(-self.memData[3])< 3.14:
		# 	print "euler Z_IMU: \n",-self.memData[3]*180/np.pi
		# else:
		# 	print "euler Z_IMU: \n",(3.14+self.memData[3])*180/np.pi			
		#iterations_between_EKF_publish = iterations_between_EKF_publish+1
			#print iterations_between_EKF_publish
		# if NaoStandStable:
		# 	self.torsoOdom_EKF.pose.pose.position.x =self.torsoOdom_EKF.pose.pose.position.x/iterations_between_EKF_publish
		# 	self.torsoOdom_EKF.pose.pose.position.y =self.torsoOdom_EKF.pose.pose.position.y/iterations_between_EKF_publish
		# 	self.torsoOdom_EKF.pose.pose.position.z =self.torsoOdom_EKF.pose.pose.position.z/iterations_between_EKF_publish
		# 	# self.torsoOdom_EKF.pose.pose.orientation.x =self.torsoOdom_EKF.pose.pose.orientation.x/iterations_between_EKF_publish
		# 		# self.torsoOdom_EKF.pose.pose.orientation.y =self.torsoOdom_EKF.pose.pose.orientation.y/iterations_between_EKF_publish
		# 		# self.torsoOdom_EKF.pose.pose.orientation.z =self.torsoOdom_EKF.pose.pose.orientation.z/iterations_between_EKF_publish
		# 		# self.torsoOdom_EKF.pose.pose.orientation.w =self.torsoOdom_EKF.pose.pose.orientation.w/iterations_between_EKF_publish
		# 	self.torsoIMU_EKF_Pub.publish(self.torsoIMU)
		# 	self.torsoOdom_EKF_Pub.publish(self.torsoOdom_EKF)
		# 	iterations_between_EKF_publish = 1
		self.tf_br.sendTransform((0,0,0), (0,0,0,1),
                                        self.timestamp, "map", "World")	
		self.tf_br.sendTransform(self.odom_transformation.position, self.odom_transformation.orientation,
                                         rospy.Time.now(), "odom", "map")
		self.tf_br.sendTransform((self.camera_To_Torso_Position[0],self.camera_To_Torso_Position[1],self.camera_To_Torso_Position[2]), self.quaternion_cameta_to_torso,
                                        self.timestamp, "cameraTop", "base_link_nao")
		self.tf_br.sendTransform((self.torsoOdom.pose.pose.position.x,self.torsoOdom.pose.pose.position.y,self.odomData[2]), self.q_odom,
                                       self.timestamp, "Nao_T_odom", "odom")
		
		#print "start",rospy.Time.now()
		self.torsoOdomPub.publish(self.torsoOdom)		
		#/imu_data
		self.torsoIMUPub.publish(self.torsoIMU)
		#print "stop",rospy.Time.now()
		t3 = rospy.Time.now().to_sec()

		d_t = t3-t2
		if d_t > 0.005:
			print "publish",d_t	

		print "total", t3-t

	

	# def publishOdom(self):
	# 	while not rospy.is_shutdown()  and not self.start_publishingOdom:
	# 		rospy.sleep(1)
	# 		print "sleeping"

	# 	while not rospy.is_shutdown() and not self.kill_publishOdom:
	# 		self.tf_br.sendTransform((0,0,0), (0,0,0,1),
 #                                         self.timestamp, "map", "World")	
	# 		self.tf_br.sendTransform(self.odom_transformation.position, self.odom_transformation.orientation,
	#                                          rospy.Time.now(), "odom", "map")
	# 		self.tf_br.sendTransform((self.camera_To_Torso_Position[0],self.camera_To_Torso_Position[1],self.camera_To_Torso_Position[2]), self.quaternion_cameta_to_torso,
	#                                        self.timestamp, "cameraTop", "base_link_nao")
	# 		self.tf_br.sendTransform((self.torsoOdom.pose.pose.position.x,self.torsoOdom.pose.pose.position.y,self.odomData[2]), self.q_odom,
	#                                        self.timestamp, "Nao_T_odom", "odom")
	# 		self.torsoOdomPub.publish(self.torsoOdom)		
	# 			#/imu_data
	# 		self.torsoIMUPub.publish(self.torsoIMU)
	# 		#print "publishing"
	# 		rospy.sleep(0.1)
	# def killall(self):
	# 	self.kill_publishOdom = True
	# 	rospy.sleep(1)
def handle_markers_tf():
	br = tf.TransformBroadcaster()
	time = rospy.Time.now()
	
	br.sendTransform((0.066,1.7, 0.54),
                     tf.transformations.quaternion_from_euler(3.14/2, 0, 3.14/2),
                     time,
                     "Wall",
                     "map")
	br.sendTransform((0.256,0.534, 0.32),
                     tf.transformations.quaternion_from_euler(3.14/2, 0, 3.14),
                     time,
                     "Wardrobe",
                     "map")
	br.sendTransform((2.5097,0.44, 0.825),
                     tf.transformations.quaternion_from_euler(3.14/2, 0, 3.14),
                     time,
                     "Microwave",
                     "map")
	br.sendTransform((3.306,0.6173, 0.75),
                     tf.transformations.quaternion_from_euler(3.14/2, 0, 3.14),
                     time,
                     "Fridge",
                     "map")

	br.sendTransform((4.04,0.697, 0.505),
                     tf.transformations.quaternion_from_euler(3.14/2, 0, 3.14),
                     time,
                     "Stable object",
                     "map")
	br.sendTransform((0.0727,1.963, 0.555),
                     tf.transformations.quaternion_from_euler(3.14/2, 0, 3.14/2),
                     time,
                     "Door",
                     "map")

def signal_handler(signal, frame):
	print "[Estimator server] - signal SIGINT caught"
	print "[Estimator server] - system exits"
	# estimator.killall()
	sys.exit(0)

if __name__ == '__main__':
	try:
		signal.signal(signal.SIGINT, signal_handler)
		print "[Estimator server] - Press Ctrl + C to exit system correctly"
		
		myBroker = ALBroker("myBroker", "0.0.0.0", 0, Constants.NAO_IP,Constants.PORT)
		
		global estimator
		estimator = NaoEstimator("estimator")
		TfRate = rospy.Rate(20)

		while not rospy.is_shutdown():
			handle_markers_tf()
			
			estimator.run()

			TfRate.sleep()
			
		
	except (KeyboardInterrupt, SystemExit):
		print "[Estimator server] - SystemExit Exception caught"
		#unsubscribe()
		myBroker.shutdown()
	#	estimator.killall()
		sys.exit(0)
		
	except Exception, ex:
		print "[Estimator server] - Exception caught %s" % str(ex)
		#unsubscribe()
		myBroker.shutdown()
		#estimator.killall()

		sys.exit(0)