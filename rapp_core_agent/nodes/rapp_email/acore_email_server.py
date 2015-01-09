#!/usr/bin/env python

from nao_email.srv import *
import rospy

class AcoreEmailServer():
	
	def __init__(self):
		print "[Email server] - Acore email Server initialization"
		rospy.init_node('acore_email_server')
		
		self.setServices()
		
		print "[Email server] - Waits for clients ..."
		rospy.spin()
	
	
	def setServices(self):
		print "[Email server] - setting services"
		s = rospy.Service('hello_world', HelloWorld, self.handle_hello_world)
		
	def handle_hello_world(self,req):
		print "[Email server receives]: \t%s\n[Email server returns]: \t%s" % (req.request, "Hello %s"% req.request)
		return HelloWorldResponse("Hello %s"% req.request)
	

if __name__ == "__main__":
    server = AcoreEmailServer()
    server.acore_email_server()
