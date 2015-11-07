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
		rospy.init_node('acore_state_server')
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
			rospy.logerr("[State server] - Could not get a proxy to ALMemory")
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

		self.camera_To_Torso_Position = std::vector<int>

		self.camera_To_Torso_Position_Pub = rospy.Publisher("cameraToTorsoPosition", std::vector<int>, queue_size=10)

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

	def run(self):
		
		#print "run",not rospy.is_shutdown()
		
		self.timestamp = rospy.Time.now()
		###
		#   Get data from NaoQi Driver
		###
		try:
			# imu data:
			self.memData = self.memProxy.getListData(self.dataNamesList)
			# t = rospy.Time.now().to_sec()

			# odometry data:
			self.odomData = self.motionProxy.getPosition('Torso', motion.SPACE_WORLD, True)
			self.camera_To_Torso_Position = self.motionProxy.getPosition('CameraTop', 0 , True)
			# t1 = rospy.Time.now().to_sec()

			# d_t = t1 -t
			# if d_t> 0.005:
			# 	print "naoqi",d_t

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

	
		self.quaternion_cameta_to_torso = tf.transformations.quaternion_from_euler(self.camera_To_Torso_Position[3],self.camera_To_Torso_Position[4],self.camera_To_Torso_Position[5])
		self.start_publishingOdom = True	

		self.torsoOdomPub.publish(self.torsoOdom)		
		#/imu_data
		self.torsoIMUPub.publish(self.torsoIMU)
		self.camera_To_Torso_Position_Pub(self.camera_To_Torso_Position)
def signal_handler(signal, frame):
	print "[State server] - signal SIGINT caught"
	print "[State server] - system exits"
	# estimator.killall()
	sys.exit(0)

if __name__ == '__main__':
	try:
		signal.signal(signal.SIGINT, signal_handler)
		print "[State server] - Press Ctrl + C to exit system correctly"
		
		myBroker = ALBroker("myBroker", "0.0.0.0", 0, Constants.NAO_IP,Constants.PORT)
		
		global state_server
		state_server = NaoEstimator("state_server")
		state_rate = rospy.Rate(20)

		while not rospy.is_shutdown():
			#handle_markers_tf()
			
			state_server.run()

			state_rate.sleep()
			
		
	except (KeyboardInterrupt, SystemExit):
		print "[State server] - SystemExit Exception caught"
		#unsubscribe()
		myBroker.shutdown()
	#	estimator.killall()
		sys.exit(0)
		
	except Exception, ex:
		print "[State server] - Exception caught %s" % str(ex)
		#unsubscribe()
		myBroker.shutdown()
		#estimator.killall()

		sys.exit(0)