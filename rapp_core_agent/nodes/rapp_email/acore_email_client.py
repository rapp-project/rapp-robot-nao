#!/usr/bin/env python

import sys
import os
import rospy
import rosparam
from nao_email.srv import *

class AcoreEmailClient():
	
	def __init__(self):
		print "Acore email Client initialization"

	def helloWorld(self,request):
		print "Waits"
		rospy.wait_for_service('hello_world')
		try:
			say_hello = rospy.ServiceProxy('hello_world', HelloWorld)
			resp_hello_world = say_hello(request)
			return resp_hello_world.response
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

if __name__ == "__main__":
	
	say_param="Nao"
	rospy.set_param("say_param", say_param)
	xy=rospy.get_param("say_param")	
	
	print "Requesting %s" % (xy)
	client = AcoreEmailClient()
	print ">request:\t[%s]\n>response:\t[%s]"%(xy,client.helloWorld(xy))

