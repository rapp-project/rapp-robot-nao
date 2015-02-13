#!/usr/bin/env python

__author__ = "Jan Figat"

# Importing core system functionality
import signal
import os, sys
import rospy
import rosparam

import numpy

# Importing services
from rapp_robot_agent.srv import *

# Needed for encoding a file
import base64

class AcoreCameraClient():
	
	def __init__(self):
		print "[Camera client] - Acore camera Client initialization"
	

	# Handling a communication with service "rapp_capture_image"
	def getCameraFrame(self,request):
		print "[Camera client] - Waits for server"
		rospy.wait_for_service('rapp_capture_image')
		try:
			print "[Camera client] - getCameraFrame"
			getImage = rospy.ServiceProxy('rapp_capture_image', GetImage)
			resp_capture_image = getImage(request) ## <--- tu wywala
			print "[Camera client] - Image captured"
			return resp_capture_image
		except rospy.ServiceException, e:
			print "[Camera client] - Calling service [/rapp_capture_image] Failed: %s"%e
			exit(1)

class AdynQRcodeClient():
	
	def __init__(self):
		print "[QRcode client] - Acore QRcode Client initialization"

	def detectQRcodes(self,request):
		print "[DetectQRcodes client] - Waits for server"
		rospy.wait_for_service('rapp_detect_qrcodes')
		try:
			detect_qrcodes = rospy.ServiceProxy('rapp_detect_qrcodes', DetectQRcodes)
			resp_detect_qrcodes = detect_qrcodes(request)

			return resp_detect_qrcodes
			
		except rospy.ServiceException, e:
			print "[DetectQRcodes client] - Calling service [/rapp_detect_qrcodes] Failed: %s"%e
			exit(1)


# Testng SIGINT signal handler
def signal_handler(signal, frame):
	print "[PhotoCapture server] - signal SIGINT caught"
	print "[PhotoCapture server] - system exits"
	sys.exit(0)

# Main entry point
def main():
	signal.signal(signal.SIGINT, signal_handler)
	print "[QRcode server] - Press Ctrl + C to exit system correctly"
	
	# Testing ROS params
	say_param="Dynamic agent"
	rospy.set_param("say_param", say_param)
	xy=rospy.get_param("say_param")	
	
	print "[Camera client] - Requesting %s" % (xy)
	client_camera = AcoreCameraClient()
	
	# Testing [/rapp_capture_image] service
	print "[Camera client] - Testing [/rapp_capture_image] service"
	response_image = client_camera.getCameraFrame("top")
	while response_image.frame==None:
		print "None"
	print "[QRcode client] - Requesting %s" % (xy)
	client_qrcode = AdynQRcodeClient()
	
	
	# Testing [/rapp_detect_qrcodes] service
	print "[QRcode client] - Testing [/rapp_detect_qrcodes] service"
	response_detectQRcodes = client_qrcode.detectQRcodes(response_image.frame)#"localize")
	print "[QRcode client] - Number of detected QRcodes:\t%s\n>"%response_detectQRcodes.numberOfQRcodes
	print"____________________\n"
	for l in range(0,response_detectQRcodes.numberOfQRcodes):
		print "[QRcode client] - QRcode in the CAMERA coordinate system:\n---%d---\n[ %f %f %f %f ]\n[ %f %f %f %f ]\n[ %f %f %f %f ]\n[ %f %f %f %f ]\n-------"%(l+1, response_detectQRcodes.cameraToQRcode.r11[l], response_detectQRcodes.cameraToQRcode.r12[l], response_detectQRcodes.cameraToQRcode.r13[l], response_detectQRcodes.cameraToQRcode.r14[l],
response_detectQRcodes.cameraToQRcode.r21[l],
response_detectQRcodes.cameraToQRcode.r22[l],
response_detectQRcodes.cameraToQRcode.r23[l],
response_detectQRcodes.cameraToQRcode.r24[l],
response_detectQRcodes.cameraToQRcode.r31[l],
response_detectQRcodes.cameraToQRcode.r32[l],
response_detectQRcodes.cameraToQRcode.r33[l],
response_detectQRcodes.cameraToQRcode.r34[l],
response_detectQRcodes.cameraToQRcode.r41[l],
response_detectQRcodes.cameraToQRcode.r42[l],
response_detectQRcodes.cameraToQRcode.r43[l],
response_detectQRcodes.cameraToQRcode.r44[l])
	print"____________________\n"
	for l in range(0,response_detectQRcodes.numberOfQRcodes):
		print "[QRcode client] - QRcode in the ROBOT coordinate system:\n---%d---\n[ %f %f %f %f ]\n[ %f %f %f %f ]\n[ %f %f %f %f ]\n[ %f %f %f %f ]\n-------"%(l+1, response_detectQRcodes.robotToQRcode.r11[l], response_detectQRcodes.robotToQRcode.r12[l], response_detectQRcodes.robotToQRcode.r13[l], response_detectQRcodes.robotToQRcode.r14[l],
response_detectQRcodes.robotToQRcode.r21[l],
response_detectQRcodes.robotToQRcode.r22[l],
response_detectQRcodes.robotToQRcode.r23[l],
response_detectQRcodes.robotToQRcode.r24[l],
response_detectQRcodes.robotToQRcode.r31[l],
response_detectQRcodes.robotToQRcode.r32[l],
response_detectQRcodes.robotToQRcode.r33[l],
response_detectQRcodes.robotToQRcode.r34[l],
response_detectQRcodes.robotToQRcode.r41[l],
response_detectQRcodes.robotToQRcode.r42[l],
response_detectQRcodes.robotToQRcode.r43[l],
response_detectQRcodes.robotToQRcode.r44[l])
	print"____________________\n"	
	print "[QRcode client] - QRcode messages:\t%s "%(response_detectQRcodes.message)
	
	return


if __name__ == "__main__":
	try:
		main()
	except Exception,e:
		print "__name__ - Error %s" % str(e)

