#!/usr/bin/env python

# Importing core system functionality
import signal
import os, sys
import rospy
import rosparam

# Importing services
#from rapp_core_agent.srv import *
from rapp_ros_naoqi_wrappings.srv import *

# Needed for encoding a file
import base64

# Email sending
import smtplib
from email.mime.text import MIMEText
from email import encoders
from email.message import Message
from email.mime.audio import MIMEAudio
from email.mime.base import MIMEBase
from email.mime.image import MIMEImage
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.Encoders import encode_base64

class Constants:
	dictionary_path = "/home/nao/ws_rapp_application_nao/nao/data/mail/email_address.txt"
	test_data_path = "/home/nao/ws_rapp_application_nao/nao/data/mail/"
	test_recorded_path = "/home/nao/ws_rapp_application_nao/nao/data/mail/sounds/rapp_email.ogg"
	# Testing on local computer
	#dictionary_path = "/home/viki/catkin_ws/src/rapp_dynamic_agent/data/email_address.txt"
	#test_data_path = "/home/viki/catkin_ws/src/rapp_dynamic_agent/data/"
	#test_recorded_path = "/home/viki/catkin_ws/src/rapp_dynamic_agent/data/sample.ogg"

	# Time to record a message
	recording_time = 3
	smtp_service = 'smtp.gmail.com'
	email_address = "rapp.nao@gmail.com"
	email_pswd = 'rapp.nao1'
	test_image_name = "rapp.PNG"

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
			
	'''# Handling a communication with service "rapp_send_email"
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
			exit(1)'''
			
######################################################################
## Functions defined by rapp dynamic agent programmer ##
######################################################################

	# A method that i used to attach files (recorded audio and an Rapp image) 
	# into a message and then sends it to defined email address.
	def sendEmail(self, email_address):
		try:
			print "[Email client] - Entered a method that sends an Rapp email"
			smtp = Constants.smtp_service
			port = int('587')
			
			print "[Email client] - Creating an object smtp.SMTP with smtp =",smtp, "and port=",port
			server= smtplib.SMTP(smtp , port)

			# Account data
			email_user = Constants.email_address
			email_pwd = Constants.email_pswd
			
			if len(email_address)!=0:
				print "[Email client] - An email address is set to %s" %email_address
				email_to=email_address
			else:
				print "[Email client] - An email address is not specified!"
				email_to = Constants.email_address
				print "[Email client] - Sending email to %s" %email_to
				return
			
			subject = "[RAPP message] - Nao sending "			
			text_attach = "Sending an email from NAO"

			audio_nao_attach = Constants.test_recorded_path
			image_nao_attach=Constants.test_data_path+Constants.test_image_name
			attach=audio_nao_attach
			
			print "[Email client] - path to audio attachments:", audio_nao_attach
			print "[Email client] - path to image attachments:", image_nao_attach
			
			print "[Email client] - preparing content of a message"
			msg = MIMEMultipart() 
			msg['From'] = email_user
			msg['To'] = email_to
			msg['Subject'] = subject
				
			print "[Email client] - Attaching files"
			if attach:
				
				# Attaching to a message an Rapp recorded audio
				print "[Email client] - Attaching audio"
				part = MIMEBase('application', 'octet-stream')
				part.set_payload(open(attach, 'rb').read())
				print "[Email client] - Encoding audio"
				encode_base64(part)
				part.add_header('Content-Disposition','attachment; filename=%s' % os.path.basename(attach))
				msg.attach(part)
				
				# Attaching to a message an Rapp image
				print "[Email client] - Attaching image"
				part = MIMEBase('application', 'octet-stream')
				part.set_payload(open(image_nao_attach, 'rb').read())
				print "[Email client] - Encoding image"
				encode_base64(part)
				part.add_header('Content-Disposition','attachment; filename="%s"' % os.path.basename(image_nao_attach))
				msg.attach(part)
				
				# Attaching to a message a body
				part = MIMEText(text_attach, 'plain')
				msg.attach(part)

			if( port != "" ):
				mailServer = smtplib.SMTP(smtp, port)
			else:
				mailServer = smtplib.SMTP(smtp)
				
			print "[Send Email] - logging into a server"
			mailServer.ehlo()
			mailServer.starttls()
			mailServer.ehlo()
			mailServer.login(email_user, email_pwd)
			print "[Send Email] - sending an email"
			mailServer.sendmail(email_user, email_to,msg.as_string())

			mailServer.close()
			return 1
		except Exception, e:
			print "[Send Email] - Exception %s"%str(e)

# Testng SIGINT signal handler
def signal_handler(signal, frame):
	print "[Email server] - signal SIGINT caught"
	print "[Email server] - system exits"
	sys.exit(0)

# Main entry point
def main():
	signal.signal(signal.SIGINT, signal_handler)
	print "[Email server] - Press Ctrl + C to exit system correctly"
	
	# Testing ROS params
	say_param="Dynamic agent"
	rospy.set_param("say_param", say_param)
	xy=rospy.get_param("say_param")	
	
	print "[Email client] - Requesting %s" % (xy)
	client = AcoreEmailClient()
	
	# Testing [/rapp_hello_world] service
	print "[Email client] - Testing [/rapp_hello_world] service"
	response_hello = client.say(xy)
	print "[Email client] - request:\t[%s]\n>[Email client] - response:\t[%s]"%(xy,response_hello)
	
	# Testing [/rapp_get_email_address] service"
	print "[Email client] - Testing [/rapp_get_email_address] service"
	response_email = client.getEmailAddress(Constants.dictionary_path)
	print "[Email client] - path to dictionary:\t[%s]\n>[Email client] - emailAddress:\t[ %s ]"%(Constants.dictionary_path,response_email)
	
	# Testing [/rapp_record] service
	print "[Email client] - Testing [/rapp_record] service"
	response_record = client.record(Constants.recording_time)
	print "[Email client] - Recording time:\t[%s]\n>[Email client] - Path to recorded file:\t[ %s ]"%(Constants.recording_time,response_record)
	
	# Testing send_email funtion that communicates with external service (smtp.gmail.com)
	# Just for testing send functionality - uncomment for testing email with file attached localy
	response_email = "maksym.figat44@gmail.com"
	response_record = Constants.test_recorded_path
	print "[Email client] - Sending an email"
	client.say("Sending an email")
	client.sendEmail(response_email)
	client.say("Email was sent to %s"%response_email)
	
	return


if __name__ == "__main__":
	try:
		main()
	except Exception,e:
		print "__name__ - Error %s" % str(e)

