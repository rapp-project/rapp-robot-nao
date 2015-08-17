#!/usr/bin/env python

__author__ = "Jan Figat"

########################
# Imports
########################

# Importing services
from rapp_ros_naoqi_wrappings.srv import *

# Importing core system functionality
import signal
import sys, os
import rospy
import time
import smtplib

# Importing core functionality from Naoqi
from naoqi import (ALProxy, ALBroker, ALModule)

# Importing OpenCV library
import cv #cv2.cv as cv #Use OpenCV-2.4.3
import cv2

# Importing ZBar library used for QR-code recognition
import zbar

# Importing others
import numpy as np
import math
import Image
import time #for time measurement
#import almath #Aldebaran's library for matrices operation


# Used for finding a text in a file
import mmap

# Needed for encoding a file
import base64

from std_msgs.msg import String
from sensor_msgs.msg import Image as Image_ros
from cv_bridge import CvBridge, CvBridgeError
#######################################

# Global variables to store the QRcodeDetection module instance and proxy to ALMemory Module
QRcodeDetection = None
prox_memory = None

# Constants
class Constants:
	NAO_IP = "nao.local"
	PORT = 9559
	cameraMatrix=np.asarray([ [182.0992346/0.16,0,658.7582],[0,185.0952141/0.16,484.2186],[0,0,1.0] ]) ## camera matrix from calibration -- for top camera

#######################################

class Matrix4x4Message:
	#row1 = row2 = row3 = row4 = []
	r11=[];r12=[];r13=[];r14=[]
	r21=[];r22=[];r23=[];r24=[]
	r31=[];r32=[];r33=[];r34=[]
	r41=[];r42=[];r43=[];r44=[]
	
	def get_values(self,table,n):
		self.r11=[];self.r12=[];self.r13=[];self.r14=[]
		self.r21=[];self.r22=[];self.r23=[];self.r24=[]
		self.r31=[];self.r32=[];self.r33=[];self.r34=[]
		self.r41=[];self.r42=[];self.r43=[];self.r44=[]
		for k in range(0,n):
			self.r11.append(table[k][0][0]);self.r12.append(table[k][0][1]);self.r13.append(table[k][0][2]);self.r14.append(table[k][0][3])
			self.r21.append(table[k][1][0]);self.r22.append(table[k][1][1]);self.r23.append(table[k][1][2]);self.r24.append(table[k][1][3])
			self.r31.append(table[k][2][0]);self.r32.append(table[k][2][1]);self.r33.append(table[k][2][2]);self.r34.append(table[k][2][3])
			self.r41.append(table[k][3][0]);self.r42.append(table[k][3][1]);self.r43.append(table[k][3][2]);self.r44.append(table[k][3][3])
		return

def rot_matrix_to_euler(R):
    y_rot = math.asin(R[x_rot_2][0]) 
    x_rot = math.acos(R[2][2]/math.cos(y_rot))    
    z_rot = math.acos(R[0][0]/math.cos(y_rot))
    y_rot_angle = y_rot *(180/math.pi)
    x_rot_angle = x_rot *(180/math.pi)
    z_rot_angle = z_rot *(180/math.pi)        
    rangle,y_rot_angle,z_rot_angle

class GetTransformClient:
	#chainName_,space_
	def getTransform(self,chainName_,space_):
		print "[GetTransform client] - Waits for server"
		rospy.wait_for_service('rapp_get_transform')
		try:
			print "[GetTransform client] - getTransform"
			getTransform = rospy.ServiceProxy('rapp_get_transform', GetTransform)
			resp_get_transform = getTransform(chainName_,space_)
			print "[GetTransform client] - Image captured"
			return resp_get_transform
		except rospy.ServiceException, e:
			print "[GetTransform client] - Calling service [/rapp_get_transform] Failed: %s"%e
			exit(1)

class QRdetector_class:
	NAO_IP = "nao.local"#"192.168.18.91"#"nao.local"
	NAO_PORT = 9559
	detected = 0
	currentCamera = "CameraTop"
	landmarkTheoreticalSize=0.16# in meters
	frame_temp = np.float32([[0],[0]])
	QRmessage=[]
	symbol_location=[]
	#symbol_masscenter_location=[]
	#gradient=[]#nachylenie kodu - do orientacji
	#QRcode_height=[]
	#QRcode_width=[]
	#angle_Z=[] #Z inwardly directed
	#distanceFromCameraToLandmark=[]
	## for the information of opened object, such as door
	openedObject=[]
	## for computing translation from image to camera plane
	rvecs = []
	tvecs = []
	eulerAngles = [] #matrix for Euler's angles of QR-code
	scale=[]#for object projection (for real size of object to pixel transform)
	rotMat=[]#rotation matrix -- for object projection
	LandmarkInCameraCoordinate=[]#Transformation matrix from camera to Landmark
	LandmarkInRobotCoordinate=[]#Transformation matrix from camera to robot
	
	#def __init__(self, img_path, img_name):
	#	Frame_image_class.img_path = img_path
	#	Frame_image_class.img_name = img_name	
	def code_detector_procces(self,frame_,set_zbar):
		try:
			QRdetector_class.detected = 0
			#frame=Image.fromstring("RGB", (frame_[0], frame_[1]), frame_[6])
			#width = frame.size[0]
			#height = frame.size[1]
			#frame= np.array(frame)##For NAO #conversion from tuple to numpy array
			frame = frame_ #for frame_ type - numpy array
			width = frame.shape[1] #
			height = frame.shape[0] #
			frame= cv2.cv.fromarray(frame[:,:])##For NAO #conversion from numpy array to CvMat
			
			cm_im = cv.CreateImage((width, height), cv.IPL_DEPTH_8U, 1)
			cv.ConvertImage(frame, cm_im)
			image = zbar.Image(cm_im.width, cm_im.height, 'Y800', cm_im.tostring())    
			set_zbar.scan(image)
			
			QRdetector_class.frame_temp = np.asarray(frame[:,:])
			np.float32(QRdetector_class.frame_temp)
			QRdetector_class.QRmessage = []
			for symbol in image.symbols:
				QRdetector_class.detected+=1 #Number of QR-codes detected
				QRdetector_class.QRmessage.append(symbol.data) # Message from the QR-code
				
		except Exception, e:
			print "Error when processing QR scanner:"
			print str(e)
			exit(1)
		
	def scanner_procces(self,frame_,set_zbar,robotToCameraMatrix_):
		try:
			## empty buffers
			QRdetector_class.QRmessage=[]
			QRdetector_class.symbol_location=[]
			#QRdetector_class.symbol_masscenter_location=[]
			#QRdetector_class.gradient=[]
			#QRdetector_class.QRcode_height=[]
			#QRdetector_class.QRcode_width=[]
			#QRdetector_class.angle_Z=[]
			#QRdetector_class.distanceFromCameraToLandmark=[]
			QRdetector_class.openedObject=[]
			QRdetector_class.rvecs=[]
			QRdetector_class.tvecs=[]
			QRdetector_class.eulerAngles=[]
			QRdetector_class.scale=[]
			QRdetector_class.rotMat=[]
			QRdetector_class.LandmarkInCameraCoordinate=[]
			QRdetector_class.LandmarkInRobotCoordinate=[]

			QRdetector_class.detected = 0
			##width = np.size(frame, 1) ###
			##height = np.size(frame, 0) ###
			#frame=Image.fromstring("RGB", (frame_[0], frame_[1]), frame_[6])
			##frame_=Image.fromstring("RGB", (1280, 960), frame_)
			##width = frame.size[0]
			##height = frame.size[1]
			
			#frame= np.array(frame)##For NAO #conversion from tuple to numpy array
			frame = frame_ ## for frame_ type - numpy array
			#print type(frame)
			#cv2.imshow('image',frame)
			#cv2.waitKey(1)
			#cv2.destroyAllWindows()

			width = frame.shape[1] #
			height = frame.shape[0] #
			frame= cv2.cv.fromarray(frame[:,:])##For NAO #conversion from numpy array to CvMat
			cm_im = cv.CreateImage((width, height), cv.IPL_DEPTH_8U, 1)
			cv.ConvertImage(frame, cm_im)
			
			## ZBar library usage
			image = zbar.Image(cm_im.width, cm_im.height, 'Y800', cm_im.tostring())   
			set_zbar.scan(image) ## Scanning the image in search of recognizable code
			
			QRdetector_class.frame_temp = np.asarray(frame[:,:])
			np.float32(QRdetector_class.frame_temp)
						
			for symbol in image.symbols:
				### Compute point of mass center
				if len(symbol.location)>=3:	
					mc1 = symbol.location[1][1] + int((symbol.location[3][1]-symbol.location[1][1])/2)
					mc2 = symbol.location[1][0] + int((symbol.location[3][0]-symbol.location[1][0])/2)	

				contour = np.asarray([[[symbol.location[0][0], symbol.location[0][1]]], [[ symbol.location[1][0], symbol.location[1][1]]], [[ symbol.location[2][0], symbol.location[2][1]]], [[symbol.location[3][0], symbol.location[3][1]]]])
				
				### solving PnP
				w = QRdetector_class.landmarkTheoreticalSize
				distCoeffs = np.zeros((4,1))# distortion matrix
				
				## Corners of detected QR-code
				corners = np.float32([
					[symbol.location[0][0],symbol.location[0][1]], #left-top
					[symbol.location[1][0],symbol.location[1][1]], #left-bottom
					[symbol.location[2][0],symbol.location[2][1]], #right-bottom
					[symbol.location[3][0],symbol.location[3][1]]  #right-top
				])
								
				## model
				objectPoints = np.asarray([[[-w/2],[w/2],[0]],[[-w/2],[-w/2],[0]],[[w/2],[-w/2],[0]],[[w/2],[w/2],[0]]])#x->,y^
				
				#cameraMatrix=np.asarray([ [182.0992346/0.16,0,658.7582],[0,185.0952141/0.16,484.2186],[0,0,1.0] ]) ## camera matrix from calibration --
				
				## Transition from image plane to camera plane
				found,rvecs_new,tvecs_new = cv2.solvePnP(objectPoints, corners, Constants.cameraMatrix, distCoeffs)
				#QRdetector_class.rvecs.append(rvecs_new);## rotation vector
				#QRdetector_class.tvecs.append(tvecs_new);## transposition vector
				
				rotM_cam = cv2.Rodrigues(rvecs_new)[0]
				#QRdetector_class.rotMat.append(rotM_cam)#for object projection
				#print rotM_cam
				#y_rot = math.asin(rotM_cam[2][0]) 
				#x_rot = math.acos(rotM_cam[2][2]/math.cos(y_rot))    
				#z_rot = math.acos(rotM_cam[0][0]/math.cos(y_rot))
				#y_rot_angle = y_rot *(180/math.pi)
				#x_rot_angle = x_rot *(180/math.pi)
				#z_rot_angle = z_rot *(180/math.pi) 
				#print 'Euler angles:'
				#print x_rot_angle, y_rot_angle, z_rot_angle
				## Euler angles:
				#QRdetector_class.eulerAngles.append(np.float32([x_rot,y_rot,z_rot]))
				
				#cameraToLandmarkTransform
				landmarkToCameraTransform = np.asarray([ 	[rotM_cam[0][0],rotM_cam[0][1],rotM_cam[0][2],tvecs_new[0][0]],
															[rotM_cam[1][0],rotM_cam[1][1],rotM_cam[1][2],tvecs_new[1][0]],
															[rotM_cam[2][0],rotM_cam[2][1],rotM_cam[2][2],tvecs_new[2][0]],
															[0,0,0,1]])

				Rotx_minus90 = np.asarray([ [1.0,0.0,0.0,0.0],[0.0,math.cos(-math.pi/2),-math.sin(-math.pi/2),0],[0.0,math.sin(-math.pi/2),math.cos(-math.pi/2),0],[0.0,0.0,0.0,1.0]])
				Rotz_plus90 = np.asarray([ [math.cos(math.pi/2),-math.sin(math.pi/2),0.0,0.0],[math.sin(math.pi/2),math.cos(math.pi/2),0.0,0.0],[0.0,0.0,1.0,0.0],[0.0,0.0,0.0,1.0] ])
				
				## Transformation from the QR-code coordinate system to the camera coordinate system
				cameraToLandmarkTransformMatrix=np.dot(Rotx_minus90,landmarkToCameraTransform)
				cameraToLandmarkTransformMatrix=np.dot(Rotz_plus90,cameraToLandmarkTransformMatrix)
				cameraToLandmarkTransformMatrix[0][3]*=-1.0;	cameraToLandmarkTransformMatrix[2][3]*=-1.0;
				
				QRdetector_class.LandmarkInCameraCoordinate.append(landmarkToCameraTransform)		
				QRdetector_class.QRmessage.append(symbol.data)#Adds QR-code message to the class variable
				QRdetector_class.detected += 1 #Number of detected QR-codes
				
				## Computing center of QR-code	
				#Moment = cv2.moments(contour)
				#cx = int(Moment['m10']/Moment['m00'])
				#cy = int(Moment['m01']/Moment['m00'])
				#mc_point = np.asarray([cx,cy])
				
				### scale computation for object projection (for right size of object computation)
				#temp_x=(math.sqrt(pow(symbol.location[0][0]-symbol.location[3][0],2)+pow(symbol.location[0][1]-symbol.location[3][1],2))+math.sqrt(pow(symbol.location[1][0]-symbol.location[2][0],2)+pow(symbol.location[1][1]-symbol.location[2][1],2)))/(2*QRdetector_class.landmarkTheoreticalSize)#in meters
				#temp_y=(math.sqrt(pow(symbol.location[0][0]-symbol.location[1][0],2)+pow(symbol.location[0][1]-symbol.location[1][1],2))+math.sqrt(pow(symbol.location[2][0]-symbol.location[3][0],2)+pow(symbol.location[2][1]-symbol.location[3][1],2)))/(2*QRdetector_class.landmarkTheoreticalSize)#in meters
				#temp_mat=np.float32([temp_x,temp_y])
				#QRdetector_class.scale.append(temp_mat)## TEST
				
				
				'''	## marking of code center
				cv.Circle(frame,(mc_point[0],mc_point[1]),3,cv.Scalar(0,0,255),-1,8,0);# center of the QR code is marked
				color=255
				## marking of code corners
				for i in range(0,len(symbol.location)-1):		
					cv.Circle(frame,symbol.location[i],3,cv.Scalar(255-color, color, 0),-1,8,0);#//drawing circles on cornerns of code (QR)
					color = color - 50
				'''
			
				#QRdetector_class.symbol_location.append(corners);


				### -----------------------------------------
				'''### Get current camera position in NAO space.
				motionProxy = ALProxy("ALMotion", QRdetector_class.NAO_IP, QRdetector_class.NAO_PORT)
				transform = motionProxy.getTransform(QRdetector_class.currentCamera, 2, True)
				transformList = almath.vectorFloat(transform)
				robotToCamera = almath.Transform(transformList)
				
				robotToCameraMatrix_= np.asarray([ [robotToCamera.r1_c1,robotToCamera.r1_c2,robotToCamera.r1_c3,robotToCamera.r1_c4],
															[robotToCamera.r2_c1,robotToCamera.r2_c2,robotToCamera.r2_c3,robotToCamera.r2_c4],
															[robotToCamera.r3_c1,robotToCamera.r3_c2,robotToCamera.r3_c3,robotToCamera.r3_c4],
															[0.0,0.0,0.0,1.0]])
				'''
				### -----------------------------------------
				print "robot To Landmark"
				print robotToCameraMatrix_
				print cameraToLandmarkTransformMatrix
				robotToLandmarkMatrix = np.dot(cameraToLandmarkTransformMatrix,robotToCameraMatrix_)
				QRdetector_class.LandmarkInRobotCoordinate.append(robotToLandmarkMatrix)

										
		except Exception, e:
			print "Error when processing QR scanner:"
			print str(e)
			exit(1)

	
	def is_email(self, email):
		pattern = '[\.\w]{1,}[@]\w+[.]\w+'
		if re.match(pattern, email):
			return True
		else:
			return False
	

# QRcodeDetectionModule class definition
class QRcodeDetectionModule(ALModule):
	""" A simple module able to react to
	sound detection events
	"""
	
	# Constructor of QRcodeRcognitionModule
	def __init__(self,name):
		ALModule.__init__(self,name)
		
		print "[QRcode server] - Acore QRcode Server initialization"
		
		self.bridge = CvBridge()

		# Initialization of ROS node
		rospy.init_node('adyn_qrcode_server')
		self.moduleName = name
		
		# Initialization of Naoqi modules and ROS services
		self.initALModule()
		
		#self.setVariables()
		self.openServices()
		
		print "[QRcode server] - Waits for clients ..."
		
		
	
	# Initialization of Naoqi modules
	def initALModule(self):
		print "[QRcode server] - Initialization of Naoqi modules"
		
		print "[QRcode server] - ALMemory proxy initialization"		
		global prox_memory
		prox_memory = ALProxy("ALMemory")
		if prox_memory is None:
			rospy.logerr("[QRcode server] - Could not get a proxy to ALMemory")
			exit(1)

		#print "[QRcode server] - ALVideoDevice proxy initialization"
		#self.prox_camera = ALProxy("ALVideoDevice")
		#if self.prox_camera is None:
		#	rospy.logerr("[QRcode server] - Could not get a proxy to ALVideoDevice")
		#	exit(1)
	
	# Initialization of ROS services
	def openServices(self):
		try:
			print "[QRcode server] - setting services"
			print "[QRcode server] - service - [rapp_detect_qrcodes]"
			self.service_rdqr = rospy.Service('rapp_detect_qrcodes', DetectQRcodes, self.handle_rapp_detect_qrcodes)
		except Exception, ex:
			print "[QRcode server] - Exception %s" % str(ex)
		
	#######################################
	
	# Core functionality methods 
	
	#######################################
	
	
	#########################
	
	# Handling methods - methods that used handling services
	
	#########################
	
	def handle_rapp_detect_qrcodes(self,req):
		print "[QRcode server receives]: \t"#%s\n" % (req.request)
		# Class QRdetector_class initialization
		self.QRdetector = QRdetector_class()
		self.TransformClient = GetTransformClient() # GetTransformClient
		self.isQRcodeFound = False
		self.numberOfQRcodes = 0
		self.cameraToQRcode = []
		self.robotToQRcode = []
		self.message = []
		self.cornersMatrix= []
		self.robotToCameraMatrix= []
		# Communicate with the GetTransform service
		response_robotToCamera = self.TransformClient.getTransform(self.QRdetector.currentCamera, 2) # Transform from currentCamera to the robot coordinate system
		self.robotToCameraMatrix=np.asarray([ 
				[response_robotToCamera.transformMatrix.r11[0],
					response_robotToCamera.transformMatrix.r12[0],
					response_robotToCamera.transformMatrix.r13[0],
					response_robotToCamera.transformMatrix.r14[0]],
				[response_robotToCamera.transformMatrix.r21[0],
					response_robotToCamera.transformMatrix.r22[0],
					response_robotToCamera.transformMatrix.r23[0],
					response_robotToCamera.transformMatrix.r24[0]],
				[response_robotToCamera.transformMatrix.r31[0],
					response_robotToCamera.transformMatrix.r32[0],
					response_robotToCamera.transformMatrix.r33[0],
					response_robotToCamera.transformMatrix.r34[0]],
				[0.0,0.0,0.0,1.0]])
				
		#print self.robotToCameraMatrix

		try:
			## ZBar ImageScanner initialization
			self.set_zbar = zbar.ImageScanner()
			
			self.cv_image = self.bridge.imgmsg_to_cv2(req.request,"rgb8")
			if self.cv_image!=None:#self.naoImage[6]!=None:
				#QRdetector.code_detector_procces(self.cv_image,self.set_zbar) #Detection only
				self.QRdetector.scanner_procces(self.cv_image,self.set_zbar,self.robotToCameraMatrix) #detects and computes the localization matrixix
				
				if self.QRdetector.detected>0: # if QRcode is found in the image
					self.isQRcodeFound = True
					self.numberOfQRcodes = self.QRdetector.detected

					self.message = (self.QRdetector.QRmessage)
					## Copy matrices to class object for the ros communication		
					Matrix4x4Camera = Matrix4x4Message()	#LandmarkInCameraCoordinate
					Matrix4x4Camera.get_values(self.QRdetector.LandmarkInCameraCoordinate,self.numberOfQRcodes)

					Matrix4x4Robot = Matrix4x4Message() #LandmarkInRobotCoordinate
					Matrix4x4Robot.get_values(self.QRdetector.LandmarkInRobotCoordinate,self.numberOfQRcodes)
					
					##WD

					WD_y = np.arctan2(-Matrix4x4Camera.r31[0],np.sqrt(Matrix4x4Camera.r32[0]*Matrix4x4Camera.r32[0]+Matrix4x4Camera.r33[0]*Matrix4x4Camera.r33[0]))		

					cam = np.array([[Matrix4x4Camera.r11[0],Matrix4x4Camera.r12[0],Matrix4x4Camera.r13[0],Matrix4x4Camera.r14[0]],[Matrix4x4Camera.r21[0],Matrix4x4Camera.r22[0],Matrix4x4Camera.r23[0],Matrix4x4Camera.r24[0]],[Matrix4x4Camera.r31[0],Matrix4x4Camera.r32[0],Matrix4x4Camera.r33[0],Matrix4x4Camera.r34[0]],[Matrix4x4Camera.r41[0],Matrix4x4Camera.r42[0],Matrix4x4Camera.r43[0],Matrix4x4Camera.r44[0]]])
					matrix_90_x = np.array([[np.cos(-WD_y),0,np.sin(-WD_y),0],
								[0,1,0,0],
								[-np.sin(-WD_y),0,np.cos(-WD_y),0],
								[0,0,0,1]])	
					
					print "cam: \n",cam	
					WD_matrix = np.dot(cam,matrix_90_x)
					WD_matrix =  np.dot(WD_matrix,matrix_90_x)
					print "WD: \n",WD_matrix
				else:
					self.numberOfQRcodes = 0#self.QRdetector.detected
					self.message = "No QR code have been found"# (self.QRdetector.QRmessage)
					## Copy matrices to class object for the ros communication		
					Matrix4x4Camera = Matrix4x4Message()	#LandmarkInCameraCoordinate
					Matrix4x4Camera.get_values(0,self.numberOfQRcodes)
					Matrix4x4Robot = Matrix4x4Camera
		except AttributeError, ex:
			print "[QRcode server] - Exception AtrributeError = %s" % str(ex)
		except Exception, ex:
			print "[QRcode server] - Unnamed exception = %s" % str(ex)
			

		return DetectQRcodesResponse(self.isQRcodeFound, self.numberOfQRcodes, Matrix4x4Camera, Matrix4x4Robot, self.message)
			
# Testng SIGINT signal handler
def signal_handler(signal, frame):
	print "[QRcode server] - signal SIGINT caught"
	print "[QRcode server] - system exits"
	sys.exit(0)

def main():
	""" Main entry point
	
	"""
	# It is needed to use a broker to be able to construct NAOQI 
	# modules and subscribe to other modules. The broker must stay
	# alive until  the program exists
	try:
		signal.signal(signal.SIGINT, signal_handler)
		print "[QRcode server] - Press Ctrl + C to exit system correctly"	
		myBroker = ALBroker("myBroker", "0.0.0.0", 0, Constants.NAO_IP,Constants.PORT)
		QRcodeDetection = QRcodeDetectionModule("QRcodeDetection")
		rospy.spin()
	
	except AttributeError:
		print "[QRcode server] - QRcodeDetection - AttributeError"
		myBroker.shutdown()
		sys.exit(0)
	
	except (KeyboardInterrupt, SystemExit):
		print "[QRcode server] - SystemExit Exception caught"
		myBroker.shutdown()
		sys.exit(0)
	
	except Exception, ex:
		print "[QRcode server] - Exception caught %s" % str(ex)
		myBroker.shutdown()
		sys.exit(0)
	
if __name__ == "__main__":
	try:
		main()
	except Exception,e:
		print "__name__ - Error %s" % str(e)
