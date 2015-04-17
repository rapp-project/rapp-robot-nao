#!/usr/bin/env python

# Importing core system functionality
import signal
import os, sys
import rospy
import rosparam
import numpy
# Importing services
from rapp_robot_agent.srv import *
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from rapp_robot_agent.msg import MatrixRot 

##############
# Calsses for QR code client
#############
class AcoreCameraClient():
	
	def __init__(self):
		print "[Camera client] - Acore camera Client initialization"
	

	# Handling a communication with service "rapp_get_image"
	def getCameraFrame(self,request):
		print "[Camera client] - Waits for server"
		rospy.wait_for_service('rapp_capture_image')
		try:
			print "[Camera client] - getCameraFrame"
			getImage = rospy.ServiceProxy('rapp_capture_image', GetImage)
			resp_get_image = getImage(request) ## <--- tu wywala
			print "[Camera client] - Image captured"
			return resp_get_image
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

##############
# Calss for follow marker client
#############


class AcorePoseClient():
	
	def __init__(self):
		print "[Follow marker client] - Acore Follow marker client initialization"
	

	# Handling a communication with service "rapp_say"
	def FollowMarker(self,destination_x, destination_y,destination_theta,head_yaw,head_pitch):
		print "[Follow marker client] - Waits for server"
		rospy.wait_for_service('rapp_follow_marker')
		try:
			follow_marker_srv = rospy.ServiceProxy('rapp_follow_marker', FollowMarker)
			resp_follow_marker= follow_marker_srv(destination_x, destination_y,destination_theta,head_yaw,head_pitch)
			return resp_follow_marker
		except rospy.ServiceException, e:
			print "[Follow marker client] - Calling service [/rapp_follow_marker] Failed: %s"%e
			exit(1)



# Testng SIGINT signal handler
def signal_handler(signal, frame):
	print "[Move server] - signal SIGINT caught"
	print "[Move server] - system exits"
	sys.exit(0)
def talker(marker):

	pubA = rospy.Publisher('marker_tf', MatrixRot,queue_size=10)


	rospy.init_node('talker', anonymous=True)
	A1 = [marker.item(0),marker.item(1),marker.item(2),marker.item(3)]
	A2 = [marker.item(4),marker.item(5),marker.item(6),marker.item(7)]#, dtype=numpy.float32)
	A3 = [marker.item(8),marker.item(9),marker.item(10),marker.item(11,)]#, dtype=numpy.float32)
	A4 = [0,0,0,1]#, dtype=numpy.float32)

	pubA.publish(A1,A2,A3,A4)

	print "talker"
# Main entry point
def main():
	signal.signal(signal.SIGINT, signal_handler)
	print "[Move server] - Press Ctrl + C to exit system correctly"
	


	##############
	# Get QR code matrix
	#############

	client_camera = AcoreCameraClient()
	
	# Testing [/rapp_get_image] service
	print "[Camera client] - Start Camera client"
	response_image = client_camera.getCameraFrame("top")
	while response_image.frame==None:
		print "No frame in response"
	print "[Camera client] - Got frame"

	print "[QRcode client] - Start QRcode client"
	client_qrcode = AdynQRcodeClient()
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
		print "[QRcode client] - QRcode in the ROBOT coordinate system:\n---%d---\n[ %f %f %f %f ]\n[ %f %f %f %f ]\n[ %f %f %f %f ]\n[ %f %f %f %f ]\n-------"%(l+1, 
response_detectQRcodes.robotToQRcode.r11[l], 
response_detectQRcodes.robotToQRcode.r12[l], 
response_detectQRcodes.robotToQRcode.r13[l], 
response_detectQRcodes.robotToQRcode.r14[l],
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
	print "[Follow marker client] - Calculating end position of Nao"
	print "[Follow marker client] - %s" %(response_detectQRcodes.robotToQRcode.r14[0])
	##############
	# Get Z rotation of QR code and robot
	#############
	kat=(response_detectQRcodes.cameraToQRcode.r21[0])/(response_detectQRcodes.cameraToQRcode.r11[0])
	gamma = numpy.arctan(numpy.double(response_detectQRcodes.cameraToQRcode.r21[0]/numpy.double(response_detectQRcodes.cameraToQRcode.r11[0])))

	##############
	# Set variables for transformations
	#############
	x = response_detectQRcodes.cameraToQRcode.r14[0]
	y = response_detectQRcodes.cameraToQRcode.r24[0]
	theta_1 = numpy.arctan(y/x)
	fi = 90 - theta_1
	beta = gamma + theta_1 - fi

	theta_2 = -beta-90
	c1 = numpy.cos(theta_1)
	s1 = numpy.sin(theta_1)
	c2 = numpy.cos(theta_2)
	s2 = numpy.sin(theta_2)
	c3 = numpy.cos(90)
	s3 = numpy.sin(90)
	a1 = x/(numpy.cos(theta_1))
	a2 = 1

	##############
	# Get end position of move
	#############
	T_01=numpy.matrix( ((response_detectQRcodes.robotToQRcode.r11[0], 
response_detectQRcodes.robotToQRcode.r12[0], 
response_detectQRcodes.robotToQRcode.r13[0], 
response_detectQRcodes.robotToQRcode.r14[0]),
(response_detectQRcodes.robotToQRcode.r21[0],
response_detectQRcodes.robotToQRcode.r22[0],
response_detectQRcodes.robotToQRcode.r23[0],
response_detectQRcodes.robotToQRcode.r24[0]),
(response_detectQRcodes.robotToQRcode.r31[0],
response_detectQRcodes.robotToQRcode.r32[0],
response_detectQRcodes.robotToQRcode.r33[0],
response_detectQRcodes.robotToQRcode.r34[0]),
(response_detectQRcodes.robotToQRcode.r41[0],
response_detectQRcodes.robotToQRcode.r42[0],
response_detectQRcodes.robotToQRcode.r43[0],
response_detectQRcodes.robotToQRcode.r44[0])) )
	T_12=numpy.matrix( (( c3,s3,0,     -0.5),
						( -s3,c3,0,     0),
						(  0, 0,1,     0),
						(  0, 0,0,     1)) )
	T_03=T_01#numpy.dot(T_01,T_12)#T_12



	try:
		talker(T_01)
	except rospy.ROSInterruptException:
		pass


	print "[Follow marker client] \n %s" %(T_03)
	destination_x =T_01.item(3) - 0.7
	destination_y = -T_01.item(7)
	destination_theta = (numpy.arctan((T_03.item(4))/(T_03.item(0))))# * numpy.pi)/180
    # Y Axis Head Orientation feasible movement = [-75.0, +70.0] degree  -  head_yaw
    # Z Axis Head Orientation feasible movement = [-30.0, +30.0] degree  -  head_pitch
	head_yaw = 0
	head_pitch = -20

	print "[Follow marker client] - Sending variables: Destination_x:\n %s \n Destination_y:\n %s \n Destination_theta:\n %s" %(destination_x,destination_y,destination_theta)

	print "[Follow marker client] - Start follow marker client"
	client = AcorePoseClient()
	response_follow_marker = client.FollowMarker(destination_x, destination_y,destination_theta,head_yaw,head_pitch)
	
	print "[Follow marker client] - Got response: Nao position is: "
	print " X = " , response_follow_marker.Nao_x
	print " Y = ", response_follow_marker.Nao_y

	return


if __name__ == "__main__":
	try:
		main()
	except Exception,e:
		print "__name__ - Error %s" % str(e)


