#!/usr/bin/env python

__author__ = "Maksym Figat"

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

# Importing core functionality from Naoqi
from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule

# Used for finding a text in a file
import mmap

# Needed for encoding a file
import base64

#######################################

# Global variables to store the EmailRecognition module instance and proxy to ALMemory Module
EmailRecognition = None
prox_memory = None

# Constants
class Constants:
	
	SOUND_DETECTED = "onSoundDetected"
	EVENT_SOUND = "SoundDetected"
	NAO_IP = "nao.local"
	PORT = 9559
	# Testing on NAO
	recorded_file_dest="/home/nao/ws_rapp_applications_nao/nao/data/mail/sounds/rapp_email.ogg"
	

#######################################

# EmailRecognitionModule class definition
class EmailRecognitionModule(ALModule):
	""" A simple module able to react to
	sound detection events
	"""
	
	# Constructor of EmailRcognitionModule
	def __init__(self,name):
		ALModule.__init__(self,name)
		
		print "[Core agent server] - Server initialization"
		
		# Initialization of ROS node
		rospy.init_node('acore_email_server')
		self.moduleName = name
		
		# Initialization of Naoqi modules and ROS services
		self.initALModule()
		
		self.setVariables()
		self.openServices()
		
		print "[Core agent server] - Waits for clients ..."
		
		
	
	# Initialization of Naoqi modules
	def initALModule(self):
		print "[Core agent server] - Initialization of Naoqi modules"
		
		print "[Core agent server] - ALMemory proxy initialization"		
		global prox_memory
		prox_memory = ALProxy("ALMemory")
		if prox_memory is None:
			rospy.logerr("[Core agent server] - Could not get a proxy to ALMemory")
			exit(1)
			
		print "[Core agent server] - ALTextToSpeech proxy initialization"
		self.prox_tts = ALProxy("ALTextToSpeech")
		if self.prox_tts is None:
			rospy.logerr("[Core agent server] - Could not get a proxy to ALTextToSpeech")
			exit(1)
			
		print "[Core agent server] - ALSoundDetection proxy initialization"
		self.prox_sd = ALProxy("ALSoundDetection")
		if self.prox_sd is None:
			rospy.logerr("[Core agent server] - Could not get a proxy to ALSoundDetection")
			exit(1)
			
		print "[Core agent server] - ALSpeechRecognition proxy initialization"
		self.prox_sprec = ALProxy("ALSpeechRecognition")
		if self.prox_sprec is None:
			rospy.logerr("[Core agent server] - Could not get a proxy to ALSpeechRecognition")
			exit(1)
		
		print "[Core agent server] - ALAudioRecorder proxy initialization"
		self.prox_ar = ALProxy("ALAudioRecorder")
		if self.prox_ar is None:
			rospy.logerr("[Core agent server]- Could not get a proxy to ALAudioRecorder")
			exit(1)
	
	
	# Setting variables
	def setVariables(self):
		print "[Core agent server] - Setting variables"
		self.isEmailFound = False
		self.stopListening = False
		self.email_address = "rapp.nao@gmail.com"
		self.prox_sd.setParameter("Sensitivity",0.7)
		self.period=500
		self.memValue = "LastWordRecognized"
		# Temporary database of recognized words
		self.database = ["John", "Max", "Rapp", "Nao", "Alarm", "Exit"]
		self.prox_sprec.pause(True)
		self.prox_sprec.setVocabulary(self.database, False)
		self.prox_sprec.pause(False)
		
		
		
		self.recordedExtention="ogg"
		# Sample rate of recorded audio (in Hz)
		self.sampleRate = 16000
		# Creating channel
		self.channels = []
		#Channels setup
		self.channels.append(1)
		self.channels.append(1)
		self.channels.append(1)
		self.channels.append(1)
		
		self.recordingTime = 3.0
		
	
	def setWordDatabase(self, database):
		print "[Core agent server] - Setting word database to recognize a spoken word"
		self.prox_sprec.pause(True)
		self.prox_sprec.setVocabulary(database, False)
		self.prox_sprec.pause(False)
		
	# Initialization of ROS services
	def openServices(self):
		try:
			print "[Core agent server] - setting services"
			print "[Core agent server] - service - [rapp_say]"
			self.service_rs = rospy.Service('rapp_say', Say, self.handle_rapp_say)
			print "[Core agent server] - service - [rapp_get_email_address]"
			self.service_rgea = rospy.Service('rapp_get_email_address', GetEmailAddress, self.handle_rapp_get_email_address)
			print "[Core agent server] - service - [rapp_record]"
			self.service_rr = rospy.Service('rapp_record', Record, self.handle_rapp_record)
			print "[Core agent server] - service - [rapp_get_recognized_word]"
			self.service_rrw = rospy.Service('rapp_get_recognized_word', RecognizeWord, self.handle_rapp_get_recognized_word)
			'''print "[Core agent server] - service - [rapp_send_email]"
			self.service_rse = rospy.Service('rapp_send_email', SendEmail, self.handle_rapp_send_email)'''
		except Exception, ex:
			print "[Core agent server] - Exception %s" % str(ex)
		
		
	#######################################
	
	# Core functionality methods 
	
	#######################################
	
	# Subscribes Nao events
	def subscribe(self):
		print "[Core agent server] - Subscribing SoundDetected event"
		
		try:
			self.prox_sprec.subscribe("Test_SpeechDetected",self.period, 0.0)
			prox_memory.subscribeToEvent(Constants.EVENT_SOUND, self.moduleName, self.functionName)
		except Exception, e:
			print "[Core agent server] - Error in subscribe(): %s", str(e)
			
		
		
	# Unsubscribes Nao events
	def unsubscribe(self):
		
		print "[Core agent server] - Unsubscribing SoundDetected event"
		try:

			self.prox_sprec.unsubscribe("Test_SpeechDetected")
			if (self.stopListening == False):
				prox_memory.unsubscribeToEvent(Constants.EVENT_SOUND, self.moduleName)
			
		except TypeError, e:
			print "[Core agent server] - Error TypeError in unsubscribe(): %s", str(e)
			#self.prox_sprec.unsubscribe("Test_SpeechDetected")
		except Exception, e:
			print "[Core agent server] - Error in unsubscribe(): %s", str(e)
			#self.prox_sprec.unsubscribe("Test_SpeechDetected")
			
			
	# Method that is called when sound is detected
	def onSoundDetected(self, *_args):
		"""This method will be called each time NAO recognised a sound.
		It will try to find out correct email address.
		"""
		try:
			# Unsubscribing to the event of Sound detection to avoid calling a method while executing interior of the method.
			prox_memory.unsubscribeToEvent(Constants.EVENT_SOUND, self.moduleName)
			val = prox_memory.getData(self.memValue)
			
			print "[Core agent server] -       " + val[0]
			print "[Core agent server] - Sleeps"
			time.sleep(1)
			print "[Core agent server] - [onSoundDetected] - Heard name: " +val[0] +" with the probability equals to " + str(val[1])
			
		
			if len(val[0])!=0 and val[1]>0.4:	
				#if(val[0] == "Exit"):
				#	print "[Core agent server] - Exits"
				#	self.stopListening=True
				#	return
				#else:
				#	self.findOutEmail(val[0])
				self.findOutEmail(val[0])
				
				print "[Core agent server] - [onSoundDetected] - Email FOUND: %s" % self.email_address
				return
				
			# Subscribe again to the event
			prox_memory.subscribeToEvent(Constants.EVENT_SOUND, self.moduleName, self.functionName )
			
		except Exception, e:
			print "[Core agent server] - onSoundDetected - Exception %s" %e
	
	# Method that is called when sound is detected, handles word detection from a database
	def onWordRecognized(self, *_args):
		"""This method will be called each time NAO recognised a sound.
		It will try to find out correct email address.
		"""
		try:
			# Unsubscribing to the event of Sound detection to avoid calling a method while executing interior of the method.
			prox_memory.unsubscribeToEvent(Constants.EVENT_SOUND, self.moduleName)
			val = prox_memory.getData(self.memValue)
			
			print "[Core agent server] -       " + val[0]
			print "[Core agent server] - Sleeps"
			time.sleep(1)
			print "[Core agent server] - [onSoundDetected] - Heard name: " +val[0] +" with the probability equals to " + str(val[1])
			
		
			if len(val[0])!=0 and val[1]>0.4:	
				self.stopListening = True
				self.wordRecognized = val[0]
				print "[Core agent server] - [onSoundDetected] - Word recognized: %s" % self.wordRecognized
				val[1]=0
				prox_memory.insertData(self.memValue,val)
				return
				
			# Subscribe again to the event
			prox_memory.subscribeToEvent(Constants.EVENT_SOUND, self.moduleName, self.functionName )
			
		except Exception, e:
			print "[Core agent server] - onSoundDetected - Exception %s" %e
	
	# Method used to find out an email address just using local file with email addesses located 
	# (maybe in the future in "../data/email_address.txt" (it can be done using database in a RAPP cloud)
	def findOutEmail(self,name):
		info = "Find out an email address to %s" % name
		print "[Core agent server] - Find out an email address to %s" % name
		self.prox_tts.say(info)

		try:
			#rapp_f = open("../data/email_address.txt")self.pathToDictionary
			rapp_f = open(self.pathToDictionary)
			rapp_s = mmap.mmap(rapp_f.fileno(), 0, access=mmap.ACCESS_READ)
			rapp_d = rapp_s.find(name,0)
			rapp_s.seek(rapp_d)
			rapp_e= rapp_s.readline()
			rapp_s.seek(rapp_d+len(name) +1)
			rapp_g = len(rapp_e)-(len(name) +1)
			rapp_email = rapp_s.read(rapp_g)
			print "[Core agent server] - An email address is: %s " % rapp_email
			rapp_s.close()
			self.prox_tts.say(rapp_email)
			print "[Core agent server] - Setting an email address"
			self.email_address = rapp_email
			self.isEmailFound  = True
		except ValueError:
			print "[Core agent server] - findOutEmail - Value Error - Probably there is the difference in name - capital letter"
	
	def countDown(self, message):
		print "[Core agent server] - %s in 3 seconds" % message
		self.prox_tts.say(message + " in 3 seconds")
		print "[Core agent server] - %s in 2 seconds" % message
		self.prox_tts.say(message + " in 2 seconds")
		print "[Core agent server] - %s in 1 seconds" % message
		self.prox_tts.say(message + " in 1 second")
		# Sleeps a while (1 second)
		time.sleep(1)
		print "[Core agent server] - GO"
		self.prox_tts.say("GO")
	
	# Record an email message
	def recordSound(self):
		try:
			print "[Core agent server] - Recording an email"
			#self.countDown("Recording an email")
						
			#self.prox_ar.stopMicrophonesRecording()
			
			self.prox_ar.startMicrophonesRecording(Constants.recorded_file_dest, self.recordedExtention, self.sampleRate, self.channels )
			print  "[Core agent server] - Start Microphones Recording"
			print  "[Core agent server] - Sleeps"
			# Waiting recordingTime - it means that the message is being recorded recordingTime (seconds)
			time.sleep(self.recordingTime)
			# Recording stops and the file is being closed after recordingTime
			self.prox_ar.stopMicrophonesRecording()
			print "[Core agent server] - Recording stops"
			#self.prox_tts.say("Recording stops")

		except Exception, e:
			print "[Core agent server] - Error during recording an message"
			print "[Core agent server] - Error: %s" % str(e)
			self.prox_ar.stopMicrophonesRecording()
			
	#########################
	
	# Handling methods - methods that used handling services
	
	#########################
		
	def handle_rapp_say(self,req):
		print "[Core agent server receives]: \t%s\n[Core agent server returns]: \t%s" % (req.request, "Said: %s"% req.request)
		#self.prox_tts.say("Nao says : %s"% req.request)
		self.prox_tts.say(req.request)
		return SayResponse(1)
		
	def handle_rapp_get_email_address(self, req):
		print "[Core agent server] - receives path to dictionary \t%s" % req.pathToDictionary
		
		self.pathToDictionary = req.pathToDictionary
		self.isEmailFound  = False
		self.stopListening = False
		isFound=0
		self.functionName = "onSoundDetected"
		
		try:
			print "[Core agent server] - Subscribing events"
			self.subscribe()
			while self.isEmailFound == False and self.stopListening == False:
				print "[Core agent server] - An email address was not found!"
				print "[Core agent server] - Say a special word from database!"
				time.sleep(4)
				self.stopListening=True
			print "[Core agent server] - Unsubscribing events"
			self.unsubscribe()
			
		except AttributeError, ex:
			print "[Core agent server] - Exception AtrributeError = %s" % str(ex)
		except Exception, ex:
			print "[Core agent server] - Unnamed exception = %s" % str(ex)
		
		print "[Core agent server] - returns email address: \t %s" % self.email_address
		
		if self.isEmailFound == True:
			isFound =1
		return GetEmailAddressResponse(self.email_address, isFound)
	
	def handle_rapp_record(self,req):
		print "[Core agent server]: - Nao records %d [s]" %req.recordingTime
		self.recordingTime = req.recordingTime
		#self.prox_tts.say("Nao records : ")
		self.recordSound()
		reponse = Constants.recorded_file_dest
		return RecordResponse(reponse)
		
	def handle_rapp_get_recognized_word(self,req):
		print "[Core agent server]: - Nao recognizes a word from a list:"
		for i in req.wordsList:
			print "[Core agent server module] - Database = %s" % i
		self.stopListening = False
		self.setWordDatabase(req.wordsList)
		self.functionName = "onWordRecognized"
		self.wordRecognized ="Empty"
		try:
			print "[Core agent server] - Subscribing events"
			self.subscribe()
			print "[Core agent server] - Word was not recognized!"
			print "[Core agent server] - Say a special word from database!"
			time.sleep(4)
			
			print "[Core agent server] - Unsubscribing events"
			self.unsubscribe()
			#self.prox_tts.say("Word recognized %s" % self.wordRecognized)
			
		except AttributeError, ex:
			print "[Core agent server] - Exception AtrributeError = %s" % str(ex)
		except Exception, ex:
			print "[Core agent server] - Unnamed exception = %s" % str(ex)
		
		return RecognizeWordResponse(self.wordRecognized)


# Testng SIGINT signal handler
def signal_handler(signal, frame):
	print "[Core agent server] - signal SIGINT caught"
	print "[Core agent server] - system exits"
	sys.exit(0)

def main():
	""" Main entry point
	
	"""
	# It is needed to use a broker to be able to construct NAOQI 
	# modules and subscribe to other modules. The broker must stay
	# alive until  the program exists
	try:
		signal.signal(signal.SIGINT, signal_handler)
		print "[Core agent server] - Press Ctrl + C to exit system correctly"
		
		myBroker = ALBroker("myBroker", "0.0.0.0", 0, Constants.NAO_IP,Constants.PORT)
		global EmailRecognition
		EmailRecognition = EmailRecognitionModule("EmailRecognition")
		rospy.spin()
	
	except AttributeError:
		print "[Core agent server] - AttributeError"
		EmailRecognition.unsubscribe()
		myBroker.shutdown()
		sys.exit(0)
		
	except (KeyboardInterrupt, SystemExit):
		print "[Core agent server] - SystemExit Exception caught"
		EmailRecognition.unsubscribe()
		myBroker.shutdown()
		sys.exit(0)
		
	except Exception, ex:
		print "[Core agent server] - Exception caught %s" % str(ex)
		EmailRecognition.unsubscribe()
		myBroker.shutdown()
		sys.exit(0)
		
if __name__ == "__main__":
	try:
		main()
	except Exception,e:
		print "__name__ - Error %s" % str(e)
