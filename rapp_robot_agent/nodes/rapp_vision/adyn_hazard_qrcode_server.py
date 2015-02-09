#!/usr/bin/env python

__author__ = "Jan Figat"

########################
# Imports
########################

# Importing services
from rapp_robot_agent.srv import *

# Importing core system functionality
import signal
import sys, os
import rospy
import time
import smtplib

# Importing others
import numpy as np
import math
import time #for time measurement

#######################################

class OpenDoorDetection:
	euler1=[];euler2=[];euler3=[]
	# R = [[m00,m01,m02], [m10,m11,m12],[m20,m21,m22]]
	def compute_euler_1(self,m12,m22):
		self.euler1=[]
		for i in range(0,len(m12)):
			self.euler1.append(math.atan2(m12[i], m22[i]) )
		return 

	def compute_euler_2(self,m00,m01,m02):
		self.euler2=[]		
		for i in range(0,len(m00)):	
			temp = math.atan2(-m02[i], math.sqrt( math.pow(m00[i],2) + math.pow(m01[i],2) ) )
			print temp
			self.euler2.append( float(temp) )
		return

	def compute_euler_3(self,m00,m01):
		self.euler3=[]		
		for i in range(0,len(m00)):	
			self.euler3.append(math.atan2(m01[i], m00[i]) )
		return
		

# HazardQRcodeDetectionModule class definition
class HazardQRcodeDetectionModule:
	""" A simple module able to react to
	sound detection events
	"""
	
	# Constructor of QRcodeRcognitionModule
	def __init__(self):
		
		print "[QRcode hazard server] - Adyn QRcode Hazard Server initialization"
	
		# Initialization of ROS node
		rospy.init_node('adyn_qrcode_hazard_server')
	
		self.openServices()
		
		print "[QRcode hazard server] - Waits for clients ..."
		
	# Initialization of ROS services
	def openServices(self):
		try:
			print "[QRcode hazard server] - setting services"
			print "[QRcode hazard server] - service - [rapp_detect_hazard_with_qrcodes]"
			self.service_rdhazardqr = rospy.Service('rapp_detect_hazard_with_qrcodes', DetectHazardByQRcodes, self.handle_rapp_detect_hazard_with_qrcodes)
		except Exception, ex:
			print "[QRcode hazard server] - Exception %s" % str(ex)
		
	#######################################
	
	# Core functionality methods 
	
	#######################################
	
	
	#########################
	
	# Handling methods - methods that used handling services
	
	#########################
	
	def handle_rapp_detect_hazard_with_qrcodes(self,req):
		print "[Hazard QRcode server receives]: \tQRcode localization matrix"#%s\n" % (req.request)
		# Class QRdetector_class initialization
		self.HazardQRcode = OpenDoorDetection()
		self.isHazardFound = False
		self.hazardPosition_x = []
		self.hazardPosition_y = []
		self.message = []
		self.wall_numer = []
		precision = 4 * math.pi/180 # established precision of QR codes detection
		#self.number = []

		try:
			self.HazardQRcode.compute_euler_2(req.robotToQRcode.r11,req.robotToQRcode.r12,req.robotToQRcode.r13)
			for i in range(0,len(req.robotToQRcode.r11)):
				if ( (req.messages[i] == "Wall") or (req.messages[i] == "Stable object") ):
					self.wall_numer.append(i)
			for j in range(0,len(self.wall_numer)):
				for k in [-2,-1,1,2]:
					## comparing the angles (euler2)
					if (len(req.robotToQRcode.r11) > self.wall_numer[j]+k) and (0 <= self.wall_numer[j]+k):
						if not ( ( (self.HazardQRcode.euler2[self.wall_numer[j]] + precision > self.HazardQRcode.euler2[self.wall_numer[j]+k]) and (self.HazardQRcode.euler2[self.wall_numer[j]] - precision < self.HazardQRcode.euler2[self.wall_numer[j]+k]) ) or ( (self.HazardQRcode.euler2[self.wall_numer[j]] - math.pi + precision > self.HazardQRcode.euler2[self.wall_numer[j]+k]) and (self.HazardQRcode.euler2[self.wall_numer[j]] - math.pi - precision < self.HazardQRcode.euler2[self.wall_numer[j]+k]) ) or ( (self.HazardQRcode.euler2[self.wall_numer[j]] + math.pi + precision > self.HazardQRcode.euler2[self.wall_numer[j]+k]) and (self.HazardQRcode.euler2[self.wall_numer[j]] + math.pi - precision < self.HazardQRcode.euler2[self.wall_numer[j]+k]) ) ):#object is open
							self.isHazardFound = True
							self.hazardPosition_x.append(float(req.robotToQRcode.r14[self.wall_numer[j]+k]) )
							self.hazardPosition_y.append(float(req.robotToQRcode.r24[self.wall_numer[j]+k]) )
							self.message.append(req.messages[self.wall_numer[j]+k])
							#print "---!---"
			print self.isHazardFound, self.hazardPosition_x, self.hazardPosition_y, self.message			
			return DetectHazardByQRcodesResponse(self.isHazardFound, self.hazardPosition_x, self.hazardPosition_y, self.message)		
		except TypeError, ex:
			print "[Hazard QRcode server] - Request is not of the valid type\n%s" %str(ex)
		except AttributeError, ex:
			print "[Hazard QRcode server] - Exception AtrributeError = %s" % str(ex)
		except Exception, ex:
			print "[Hazard  QRcode server] - Unnamed exception = %s" % str(ex)

			
# Testng SIGINT signal handler
def signal_handler(signal, frame):
	print "[Hazard QRcode server] - signal SIGINT caught"
	print "[Hazard QRcode server] - system exits"
	sys.exit(0)

def main():
	""" Main entry point
	
	"""

	try:
		signal.signal(signal.SIGINT, signal_handler)
		print "[Hazard QRcode server] - Press Ctrl + C to exit system correctly"	
		HazardQRcodeDetection = HazardQRcodeDetectionModule()
		rospy.spin()
	
	except AttributeError:
		print "[Hazard QRcode server] - QRopenDoorDetectionModule - AttributeError"
		sys.exit(0)
	
	except (KeyboardInterrupt, SystemExit):
		print "[Hazard QRcode server] - SystemExit Exception caught"
		sys.exit(0)
	
	except Exception, ex:
		print "[Hazard QRcode server] - Exception caught %s" % str(ex)
		sys.exit(0)
	
if __name__ == "__main__":
	try:
		main()
	except Exception,e:
		print "__name__ - Error %s" % str(e)
