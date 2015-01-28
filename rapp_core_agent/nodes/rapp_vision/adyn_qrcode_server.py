#!/usr/bin/env python

__author__ = "Jan Figat"

########################
# Imports
########################

# Importing services
from rapp_core_agent.srv import *

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
import almath #Aldebaran's library for matrices operation

# Used for finding a text in a file
import mmap

# Needed for encoding a file
import base64

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

	
class QRdetector_class:
	NAO_IP = "nao.local"#"192.168.18.91"#"nao.local"
	NAO_PORT = 9559
	detected = 0
	currentCamera = "CameraTop"
	landmarkTheoreticalSize=0.16# in meters
	frame_temp = np.float32([[0],[0]])
	QRmessage=[]
	symbol_location=[]
	symbol_masscenter_location=[]
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
			#width = np.size(frame, 1) ###
			#height = np.size(frame, 0) ###
			frame=Image.fromstring("RGB", (frame_[0], frame_[1]), frame_[6])
			width = frame.size[0]
			height = frame.size[1]
			
			frame= np.array(frame)##For NAO #conversion from tuple to numpy array
			frame= cv2.cv.fromarray(frame[:,:])##For NAO #conversion from numpy array to CvMat
			
			cm_im = cv.CreateImage((width, height), cv.IPL_DEPTH_8U, 1)
			cv.ConvertImage(frame, cm_im)
			image = zbar.Image(cm_im.width, cm_im.height, 'Y800', cm_im.tostring())    
			set_zbar.scan(image)
			
			QRdetector_class.frame_temp = np.asarray(frame[:,:])
			np.float32(QRdetector_class.frame_temp)
			#frame_temp = np.asarray(frame[:,:])
			#np.float32(frame_temp)
			QRdetector_class.QRmessage = []
			for symbol in image.symbols:
				QRdetector_class.detected+=1 #Number of QR-codes detected
				QRdetector_class.QRmessage.append(symbol.data) # Message from the QR-code
				
		except Exception, e:
			print "Error when processing QR scanner:"
			print str(e)
			exit(1)
		
	def scanner_procces(self,frame_,set_zbar):
		try:
			## empty buffers
			QRdetector_class.QRmessage=[]
			QRdetector_class.symbol_location=[]
			QRdetector_class.symbol_masscenter_location=[]
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
			#width = np.size(frame, 1) ###
			#height = np.size(frame, 0) ###
			frame=Image.fromstring("RGB", (frame_[0], frame_[1]), frame_[6])
			width = frame.size[0]
			height = frame.size[1]
			
			frame= np.array(frame)##For NAO #conversion from tuple to numpy array
			frame= cv2.cv.fromarray(frame[:,:])##For NAO #conversion from numpy array to CvMat
			
			cm_im = cv.CreateImage((width, height), cv.IPL_DEPTH_8U, 1)
			cv.ConvertImage(frame, cm_im)
			## ZBar library usage
			image = zbar.Image(cm_im.width, cm_im.height, 'Y800', cm_im.tostring())    
			set_zbar.scan(image) ## Scanning the image in search of recognizable code
			
			QRdetector_class.frame_temp = np.asarray(frame[:,:])
			np.float32(QRdetector_class.frame_temp)
			#frame_temp = np.asarray(frame[:,:])
			#np.float32(frame_temp)
			
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
				QRdetector_class.rvecs.append(rvecs_new);## rotation vector
				QRdetector_class.tvecs.append(tvecs_new);## transposition vector
							
				### Get current camera position in NAO space.
				motionProxy = ALProxy("ALMotion", QRdetector_class.NAO_IP, QRdetector_class.NAO_PORT)
				transform = motionProxy.getTransform(QRdetector_class.currentCamera, 2, True)
				transformList = almath.vectorFloat(transform)
				robotToCamera = almath.Transform(transformList)
				
				rotM_cam = cv2.Rodrigues(rvecs_new)[0]
				QRdetector_class.rotMat.append(rotM_cam)#for object projection
				#print rotM_cam
				y_rot = math.asin(rotM_cam[2][0]) 
				x_rot = math.acos(rotM_cam[2][2]/math.cos(y_rot))    
				z_rot = math.acos(rotM_cam[0][0]/math.cos(y_rot))
				
				y_rot_angle = y_rot *(180/math.pi)
				x_rot_angle = x_rot *(180/math.pi)
				z_rot_angle = z_rot *(180/math.pi) 
				#print 'Euler angles:'
				#print x_rot_angle, y_rot_angle, z_rot_angle
				## Euler angles:
				QRdetector_class.eulerAngles.append(np.float32([x_rot,y_rot,z_rot]))
				
				
				#cameraToLandmarkTransform
				landmarkToCameraTransform = np.asarray([ 	[rotM_cam[0][0],rotM_cam[0][1],rotM_cam[0][2],tvecs_new[0][0]],
															[rotM_cam[1][0],rotM_cam[1][1],rotM_cam[1][2],tvecs_new[1][0]],
															[rotM_cam[2][0],rotM_cam[2][1],rotM_cam[2][2],tvecs_new[2][0]],
															[0,0,0,1]])
				
				## Computing Transform matrices while using NAOqi functions
				cameraToLandmarkRotationTransform = almath.Transform_from3DRotation(x_rot,y_rot,z_rot)
				cameraToLandmarkTranslationTransform = almath.Transform(tvecs_new[2][0], tvecs_new[0][0], tvecs_new[1][0])
				cameraToLandmarkTransform = cameraToLandmarkRotationTransform * cameraToLandmarkTranslationTransform
				
				cameraToLandmarkTransformMatrix = np.asarray([ [cameraToLandmarkTransform.r1_c1,cameraToLandmarkTransform.r1_c2,cameraToLandmarkTransform.r1_c3,cameraToLandmarkTransform.r1_c4],
												[cameraToLandmarkTransform.r2_c1,cameraToLandmarkTransform.r2_c2,cameraToLandmarkTransform.r2_c3,cameraToLandmarkTransform.r2_c4],
												[cameraToLandmarkTransform.r3_c1,cameraToLandmarkTransform.r3_c2,cameraToLandmarkTransform.r3_c3,cameraToLandmarkTransform.r3_c4],
												[0,0,0,1]])
				QRdetector_class.LandmarkInCameraCoordinate.append(cameraToLandmarkTransformMatrix)
				
				### Combine all transformations to get the landmark position in NAO space.
				#robotToLandmark = np.matrix(robotToCameraFloat) * np.matrix(cameraToLandmarkTransform).I
				## While using NAOqi functions
				robotToLandmark = robotToCamera * cameraToLandmarkTransform
				#print 'NAO to Landmark Transform matrix:\n %s'%robotToLandmark
				robotToLandmarkMatrix=np.asarray([ [robotToLandmark.r1_c1,robotToLandmark.r1_c2,robotToLandmark.r1_c3,robotToLandmark.r1_c4],
												[robotToLandmark.r2_c1,robotToLandmark.r2_c2,robotToLandmark.r2_c3,robotToLandmark.r2_c4],
												[robotToLandmark.r3_c1,robotToLandmark.r3_c2,robotToLandmark.r3_c3,robotToLandmark.r3_c4],
												[0,0,0,1]])
				QRdetector_class.LandmarkInRobotCoordinate.append(robotToLandmarkMatrix)
				
#				print 'QR code location in robot frame \n%s'%robotToLandmark
#				print "x " + str(robotToLandmark[0].item(3)) + " (in meters)"
#				print "y " + str(robotToLandmark[1].item(3)) + " (in meters)"
#				print "z " + str(robotToLandmark[2].item(3)) + " (in meters)"

				QRdetector_class.QRmessage.append(symbol.data)#Adds QR-code message to the class variable
				#print '\033[1;32mMessage: %s\n \033[1;m'%QRdetector_class.QRmessage[len(QRdetector_class.QRmessage)-1]#-1

				## Computing center of QR-code	
				Moment = cv2.moments(contour)
				cx = int(Moment['m10']/Moment['m00'])
				cy = int(Moment['m01']/Moment['m00'])
				mc_point = np.asarray([cx,cy])
				
				### scale computation for object projection (for right size of object computation)
				temp_x=(math.sqrt(pow(symbol.location[0][0]-symbol.location[3][0],2)+pow(symbol.location[0][1]-symbol.location[3][1],2))+math.sqrt(pow(symbol.location[1][0]-symbol.location[2][0],2)+pow(symbol.location[1][1]-symbol.location[2][1],2)))/(2*QRdetector_class.landmarkTheoreticalSize)#in meters
				temp_y=(math.sqrt(pow(symbol.location[0][0]-symbol.location[1][0],2)+pow(symbol.location[0][1]-symbol.location[1][1],2))+math.sqrt(pow(symbol.location[2][0]-symbol.location[3][0],2)+pow(symbol.location[2][1]-symbol.location[3][1],2)))/(2*QRdetector_class.landmarkTheoreticalSize)#in meters
				temp_mat=np.float32([temp_x,temp_y])
				QRdetector_class.scale.append(temp_mat)## TEST
				
				
				'''	## marking of code center
				cv.Circle(frame,(mc_point[0],mc_point[1]),3,cv.Scalar(0,0,255),-1,8,0);# center of the QR code is marked
				color=255
				## marking of code corners
				for i in range(0,len(symbol.location)-1):		
					cv.Circle(frame,symbol.location[i],3,cv.Scalar(255-color, color, 0),-1,8,0);#//drawing circles on cornerns of code (QR)
					color = color - 50
				'''
			
				QRdetector_class.detected += 1
				QRdetector_class.symbol_masscenter_location.append(mc_point)
				QRdetector_class.symbol_location.append(corners);
										
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

		print "[QRcode server] - ALVideoDevice proxy initialization"
		self.prox_camera = ALProxy("ALVideoDevice")
		if self.prox_camera is None:
			rospy.logerr("[QRcode server] - Could not get a proxy to ALVideoDevice")
			exit(1)
		
	
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
		print "[QRcode server receives]: \t%s\n" % (req.request)
		# Class QRdetector_class initialization
		self.QRdetector = QRdetector_class()
		self.isQRcodeFound = False
		self.numberOfQRcodes = 0
		self.cameraToQRcode = []
		self.robotToQRcode = []
		self.message = []
		self.cornersMatrix= []

		
		try:
		#	# Get Frame from Camera
			### Camera initialization
			self.resolution = 3	## k4VGA;
			self.colorSpace = 13	## kBGRColorSpace
			self.fps = 15;
			#global nameId
			self.nameId = self.prox_camera.subscribe("python_camera_client+", self.resolution, self.colorSpace, self.fps);
			## ZBar ImageScanner initialization
			#global set_zbar
			self.set_zbar = zbar.ImageScanner()

			self.naoImage = self.prox_camera.getImageRemote(self.nameId) #frame from the camera
			#imageWidth = naoImage[0]
			#imageHeight = naoImage[1]
			#if (naoImage[6]!=None):
			#	frame_img=Image.fromstring("RGB", (imageWidth, imageHeight), naoImage[6])
			#else :
			#	frame_img=None
			while self.naoImage[6]==None:
				self.naoImage = self.prox_camera.getImageRemote(self.nameId) # for avoidance of the black image (empty frame)
			

			if self.naoImage[6]!=None:#frame_img!=None:
				#QRdetector.code_detector_procces(naoImage,set_zbar) #Detection only
				self.QRdetector.scanner_procces(self.naoImage,self.set_zbar) #detects and computes the localization matrix
				if self.QRdetector.detected>0: # if QRcode is found in the image
					self.isQRcodeFound = True
		except AttributeError, ex:
			print "[QRcode server] - Exception AtrributeError = %s" % str(ex)
		except Exception, ex:
			print "[QRcode server] - Unnamed exception = %s" % str(ex)

		
		self.numberOfQRcodes = self.QRdetector.detected
		self.message = (self.QRdetector.QRmessage)

		## Copy matrices to class object for the ros communication		
		Matrix4x4Camera = Matrix4x4Message()	#LandmarkInCameraCoordinate
		Matrix4x4Camera.get_values(self.QRdetector.LandmarkInCameraCoordinate,self.numberOfQRcodes)
		
		Matrix4x4Robot = Matrix4x4Message() #LandmarkInRobotCoordinate
		Matrix4x4Robot.get_values(self.QRdetector.LandmarkInRobotCoordinate,self.numberOfQRcodes)
			
		#camera unsubscribe			
		self.prox_camera.releaseImage(self.nameId);
		self.prox_camera.unsubscribe(self.nameId);
		
		# Conversion from 'list' to 'array' : np.asarray(list)
		# Conversion from 'list' to 'tuple' : tuple(list)

		
		return DetectQRcodesResponse(self.isQRcodeFound, self.numberOfQRcodes, Matrix4x4Camera, Matrix4x4Robot, self.message)#, self.cornersMatrix)

			
	
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
		global PhotoCapture
		QRcodeDetection = QRcodeDetectionModule("QRcodeDetection")
		rospy.spin()
	
	except AttributeError:
		print "[QRcode server] - QRcodeDetection - AttributeError"
		#QRcodeDetection.unsubscribe()
		myBroker.shutdown()
		sys.exit(0)
	
	except (KeyboardInterrupt, SystemExit):
		print "[QRcode server] - SystemExit Exception caught"
		#QRcodeDetection.unsubscribe()
		myBroker.shutdown()
		sys.exit(0)
	
	except Exception, ex:
		print "[QRcode server] - Exception caught %s" % str(ex)
		#QRcodeDetection.unsubscribe()
		myBroker.shutdown()
		sys.exit(0)
	
if __name__ == "__main__":
	try:
		main()
	except Exception,e:
		print "__name__ - Error %s" % str(e)
