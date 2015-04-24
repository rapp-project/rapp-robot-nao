#!/usr/bin/env python

# Importing core system functionality
import signal
import os, sys
import rospy
import rosparam
import numpy
# Importing services
from rapp_ros_naoqi_wrappings.srv import *

class AcorePoseClient():
	
	def __init__(self):
		print "[Follow marker client] - Acore Follow marker client initialization"
	

	# Handling a communication with service "rapp_say"
	def FollowMarker(self,marker_x, marker_y,head_yaw,head_pitch):
		print "[Follow marker client] - Waits for server"
		rospy.wait_for_service('rapp_follow_marker')
		try:
			follow_marker_srv = rospy.ServiceProxy('rapp_follow_marker', FollowMarker)
			resp_follow_marker= follow_marker_srv(marker_x, marker_y,head_yaw,head_pitch)
			return resp_follow_marker
		except rospy.ServiceException, e:
			print "[Follow marker client] - Calling service [/rapp_follow_marker] Failed: %s"%e
			exit(1)

# Testng SIGINT signal handler
def signal_handler(signal, frame):
	print "[Move server] - signal SIGINT caught"
	print "[Move server] - system exits"
	sys.exit(0)

# Main entry point
def main():
	signal.signal(signal.SIGINT, signal_handler)
	print "[Move server] - Press Ctrl + C to exit system correctly"

	marker_x = 0.1
	marker_y = 0.1

    # Y Axis Head Orientation feasible movement = [-75.0, +70.0] degree  -  head_yaw
    # Z Axis Head Orientation feasible movement = [-30.0, +30.0] degree  -  head_pitch
	head_yaw = 0
	head_pitch = -20
	client = AcorePoseClient()
	response_follow_marker = client.FollowMarker(marker_x, marker_y,head_yaw,head_pitch)
	
	print "[Follow marker client] - got response: Nao position is: "
	print " X = " , response_follow_marker.Nao_x
	print " Y = ", response_follow_marker.Nao_y

	return


if __name__ == "__main__":
	try:
		main()
	except Exception,e:
		print "__name__ - Error %s" % str(e)


