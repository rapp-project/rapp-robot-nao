#!/usr/bin/env python
######################
## written by Wojciech Dudek
######################
__author__ = "Wojciech Dudek"

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import tf
import rospy
import sys
import signal
import motion

from naoqi import ALModule
from naoqi import ALBroker
from naoqi import ALProxy

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
		self.startSubscriber()
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

	def connectNaoQi(self):

		self.motionProxy = ALProxy("ALMotion")
		if self.motionProxy is None:
			rospy.logerr("[State server] - Could not get a proxy to ALMemory")
			exit(1)

		self.memProxy = ALProxy("ALMemory")
		if self.motionProxy is None or self.memProxy is None:
			exit(1)

	def startSubscriber(self):
		rospy.Subscriber("/odometry/filtered", Odometry, self.publishEKFframe)

	def MsgsInit(self):
		# init. messages:
		self.torsoOdom = Odometry()
		self.torsoOdom.header.frame_id = "odom"
		self.torsoOdom.child_frame_id = "Nao_T_odom"
		self.torsoOdomPub = rospy.Publisher("odom", Odometry, queue_size=10)
		self.torsoOdom_EKF = Odometry()
		self.torsoOdom_EKF.header.frame_id = "odom"
		self.torsoOdom_EKF.child_frame_id = "Nao_T_odom_EKF"
		self.torsoOdom_EKFPub = rospy.Publisher("odom_ekf", Odometry, queue_size=10)
		self.torsoIMU = Imu()
		self.torsoIMU.header.frame_id = "base_link"
		self.torsoIMUPub = rospy.Publisher("imu_data", Imu, queue_size=10)

		#self.camera_To_Torso_Position = std::vector<int>

		self.camera_To_Torso_Position_Pub = rospy.Publisher("cameraToTorsoPosition", Float32, queue_size=10)

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
		                         0, 0, 0, 1e6, 0, 0
,		                         0, 0, 0, 0, 1e6, 0,
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
	def publishEKFframe(self,data):
		#set initial data for two rotations taken from odometry
		ekf_orientation_euler = tf.transformations.euler_from_quaternion((data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w))
		ekf_orientation_z_euler = ekf_orientation_euler[2]
		
		#### marged_angles needs extra information from virtual efector (state_server) to publish TORSO transform with around X and around Y rotations
		marged_angles = tf.transformations.quaternion_from_euler(self.odomData[3], self.odomData[4],ekf_orientation_z_euler)
		#marged_angles = tf.transformations.quaternion_from_euler(0,0,ekf_orientation_z_euler)
		self.tf_br.sendTransform((data.pose.pose.position.x,data.pose.pose.position.y,self.odomData[2]), marged_angles,
                                      rospy.Time.now(), "Nao_Torso", "odom")
		
		self.camera_To_Torso_Position = self.motionProxy.getPosition('CameraTop', 0, True)
		self.quaternion_cameta_to_torso = tf.transformations.quaternion_from_euler(self.camera_To_Torso_Position[3],self.camera_To_Torso_Position[4],self.camera_To_Torso_Position[5])
		self.HeadPitch_To_Torso_Position = self.motionProxy.getPosition('HeadPitch', 0, True)
		self.quaternion_HeadPitch_to_torso = tf.transformations.quaternion_from_euler(self.HeadPitch_To_Torso_Position[3],self.HeadPitch_To_Torso_Position[4],self.HeadPitch_To_Torso_Position[5])
		self.HeadYaw_To_Torso_Position = self.motionProxy.getPosition('HeadYaw', 0, True)
		self.quaternion_HeadYaw_to_torso = tf.transformations.quaternion_from_euler(self.HeadYaw_To_Torso_Position[3],self.HeadYaw_To_Torso_Position[4],self.HeadYaw_To_Torso_Position[5])

		self.tf_br.sendTransform((self.HeadYaw_To_Torso_Position[0],self.HeadYaw_To_Torso_Position[1],self.HeadYaw_To_Torso_Position[2]), self.quaternion_HeadYaw_to_torso,
                                        self.timestamp, "HeadYaw", "Nao_Torso")
		self.tf_br.sendTransform((self.HeadPitch_To_Torso_Position[0],self.HeadPitch_To_Torso_Position[1],self.HeadPitch_To_Torso_Position[2]), self.quaternion_HeadPitch_to_torso,
                                        self.timestamp, "HeadPitch", "Nao_Torso")
		self.tf_br.sendTransform((self.camera_To_Torso_Position[0],self.camera_To_Torso_Position[1],self.camera_To_Torso_Position[2]), self.quaternion_cameta_to_torso,
                                        self.timestamp, "cameraTop", "Nao_Torso")
		self.tf_br.sendTransform((self.HeadPitch_To_Torso_Position[0],self.HeadPitch_To_Torso_Position[1],self.HeadPitch_To_Torso_Position[2]), (0,0,0,1),
                                        self.timestamp, "Neck", "Nao_Torso")
	def run(self):
		
		self.timestamp = rospy.Time.now()
		###
		#   Get data from NaoQi Driver
		###
		try:
			# imu data:
			self.memData = self.memProxy.getListData(self.dataNamesList)
	
			# odometry data:
			self.odomData = self.motionProxy.getPosition('Torso', motion.SPACE_WORLD, True)
			self.camera_To_Torso_Position = self.motionProxy.getPosition('CameraTop', 0 , True)
				
		except RuntimeError, e:
			print "Error accessing ALMemory, exiting...\n"
			print e
			rospy.signal_shutdown("No NaoQI available anymore")

		###
		#  Fill msgs data
		###
		# /odom

		self.torsoOdom.header.stamp = self.timestamp
		self.torsoOdom.pose.pose.position.x = self.odomData[0]
		self.torsoOdom.pose.pose.position.y = self.odomData[1]
		self.torsoOdom.pose.pose.position.z = 0#self.odomData[2]
		self.q_odom = tf.transformations.quaternion_from_euler(self.odomData[3], self.odomData[4], self.odomData[5])
		self.torsoOdom.pose.pose.orientation.x = self.q_odom[0]
		self.torsoOdom.pose.pose.orientation.y = self.q_odom[1]
		self.torsoOdom.pose.pose.orientation.z = self.q_odom[2]
		self.torsoOdom.pose.pose.orientation.w = self.q_odom[3]
		ODOM_POSE_COVARIANCE = [1e-2, 0, 0, 0, 0, 0, 
		                        0, 1e-2, 0, 0, 0, 0,
		                        0, 0, 1e-3, 0, 0, 0,
		                        0, 0, 0, 1e6, 0, 0,
	                        0, 0, 0, 0, 1e6, 0,
		                        0, 0, 0, 0, 0, 1e-2]
		self.torsoOdom.pose.covariance =  ODOM_POSE_COVARIANCE
			

		# /imu_data
		self.torsoIMU.header.stamp = self.timestamp

		q_IMU =tf.transformations.quaternion_from_euler(self.memData[1], self.memData[2], -self.memData[3])
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
		self.torsoIMU.orientation_covariance = [1e-5,0,0,
													0,1e-5,0,
													0,0,1e-3]
		self.torsoIMU.angular_velocity_covariance= [1e-3,0,0,
														0,1e-3,0,
														0,0,1e-4]
		self.torsoIMU.linear_acceleration_covariance = [1e-3,0,0,
															0,1e2,0,
															0,0,1e2]

		self.torsoOdomPub.publish(self.torsoOdom)		
		self.torsoOdom_EKFPub.publish(self.torsoOdom_EKF)		
	
		#/imu_data
		self.tf_br.sendTransform((0,0,0), (0,0,0,1),
                                        self.timestamp, "map", "World")	
		self.torsoIMUPub.publish(self.torsoIMU)
		self.camera_To_Torso_Position_Pub.publish(self.camera_To_Torso_Position)
def signal_handler(signal, frame):
	print "[State server] - signal SIGINT caught"
	print "[State server] - system exits"
	
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
						
			state_server.run()

			state_rate.sleep()
			
		
	except (KeyboardInterrupt, SystemExit):
		print "[State server] - SystemExit Exception caught"
		
		myBroker.shutdown()
	
		sys.exit(0)
		
	except Exception, ex:
		print "[State server] - Exception caught %s" % str(ex)
	
		myBroker.shutdown()
	
		sys.exit(0)
