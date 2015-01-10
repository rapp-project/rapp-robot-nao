#!/usr/bin/env python

import sys
import os
import rospy
import rosparam
from rapp_core_agent.srv import *

class Constants:
	# Testing on NAO
	#self.dictionaryPath = "/home/nao/naoqi/lib/naoqi/data/"
	# Testing on local computer
	DictionaryPath = "/home/viki/catkin_ws/src/rapp_dynamic_agent/data/email_address.txt"
	# Time to record a message
	RecordingTime = 3
	# Just for testing on computer
	TestEmailAddress = "rapp.nao@gmail.com"
	TestRecordedPath = "/home/viki/catkin_ws/src/rapp_dynamic_agent/data/sample.ogg"

class AcoreEmailClient():
	
	def __init__(self):
		print "[Email client] - Acore email Client initialization"
	

	# Handling a communication with service "rapp_say"
	def say(self,request):
		print "[Email client] - Waits for server"
		rospy.wait_for_service('rapp_say')
		try:
			say = rospy.ServiceProxy('rapp_say', Say)
			resp_say = say(request)
			return resp_say.response
		except rospy.ServiceException, e:
			print "[Email client] - Calling service [/rapp_say] Failed: %s"%e
			exit(1)
	
	# Handling a communication with service "rapp_get_email_address"
	def getEmailAddress(self,request):
		print "[Email client] - Waits for server"
		rospy.wait_for_service('rapp_get_email_address')
		try:
			get_email_address = rospy.ServiceProxy('rapp_get_email_address', GetEmailAddress)
			
			# Setting as a request a path to dictionary downloaded and unpacked by NAO
			resp_get_email_address = get_email_address(request)
			return resp_get_email_address.emailAddress
		except rospy.ServiceException, e:
			print "[Email client] - Calling service [/rapp_get_email_address] Failed: %s"%e
			exit(1)
			
	# Handling a communication with service "rapp_record"
	def record(self,request):
		print "[Email client] - Waits for server"
		rospy.wait_for_service('rapp_record')
		try:
			recordEmail = rospy.ServiceProxy('rapp_record', Record)
			# Setting as a request time to record by NAO microphones
			resp_recordEmail = recordEmail(request)
			return resp_recordEmail.recordedFileDest
		except rospy.ServiceException, e:
			print "[Email client] - Calling service [/rapp_record] Failed: %s"%e
			exit(1)
			
	# Handling a communication with service "rapp_send_email"
	def sendEmail(self,requestEmail, requestPath):
		print "[Email client] - Waits for server"
		rospy.wait_for_service('rapp_send_email')
		try:
			sendEmail = rospy.ServiceProxy('rapp_send_email', SendEmail)
			# Setting as a request time to record by NAO microphones
			resp_sendEmail = sendEmail(requestEmail,requestPath)
			return resp_sendEmail.isSend
		except rospy.ServiceException, e:
			print "[Email client] - Calling service [/rapp_send_email] Failed: %s"%e
			exit(1)

if __name__ == "__main__":
	
	# Testing ROS params
	say_param="Nao"
	rospy.set_param("say_param", say_param)
	xy=rospy.get_param("say_param")	
	
	print "[Email client] - Requesting %s" % (xy)
	client = AcoreEmailClient()
	
	# Testing [/rapp_hello_world] service
	#print "[Email client] - Testing [/rapp_hello_world] service"
	#response_hello = client.say(xy)
	#print "[Email client] - request:\t[%s]\n>[Email client] - response:\t[%s]"%(xy,response_hello)
	
	# Testing [/rapp_get_email_address] service"
	#print "[Email client] - Testing [/rapp_get_email_address] service"
	#response_email = client.getEmailAddress(Constants.DictionaryPath)
	#print "[Email client] - pathToDictionary:\t[%s]\n>[Email client] - emailAddress:\t[ %s ]"%(Constants.DictionaryPath,response_email)
	
	# Testing [/rapp_record] service
	#print "[Email client] - Testing [/rapp_record] service"
	#response_record = client.record(Constants.RecordingTime)
	#print "[Email client] - Recording time:\t[%s]\n>[Email client] - Path to recorded file:\t[ %s ]"%(Constants.RecordingTime,response_record)
	
	# Testing [/rapp_send_email] service
	print "[Email client] - Testing [/rapp_send_email] service"
	response_send_email = client.sendEmail(Constants.TestEmailAddress, Constants.TestRecordedPath)
	print "[Email client] - Send email to:\t[%s] with file:\t [%s]\n>[Email client] - Is an email send:\t[ %d ]"%(Constants.TestEmailAddress,Constants.TestRecordedPath,response_send_email)
	
	

