#!/usr/bin/env python

__author__ = "Maksym Figat & Jan Figat"

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
import numpy

# Importing core functionality from Naoqi
from naoqi import (ALProxy, ALBroker, ALModule)


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
	recorded_file_dest="/home/nao/recordings/microphones/rapp_email.ogg"

	
	

#######################################

# CommunicationModule class definition
class CommunicationModule(ALModule):
	""" A simple module able to react to
	sound detection events
	"""
	
	# Constructor of EmailRcognitionModule
	def __init__(self,name):
		ALModule.__init__(self,name)
		
		print "[Communication server] - Acore Communication server initialization"
		self.isAudDeviceSubscribed = False; # for checking if processRemote is running -- in handle_rapp_microphone_energy and handle_rapp_voice_record

		# Initialization of ROS node
		rospy.init_node('acore_communication_server')
		self.moduleName = name
		
		# Initialization of Naoqi modules and ROS services
		self.initALModule()
		
		self.setVariables()
		self.openServices()
		
		print "[Communication server] - Waits for clients ..."
		
		
	
	# Initialization of Naoqi modules
	def initALModule(self):
		print "[Communication server] - Initialization of Naoqi modules"
		
		print "[Communication server] - ALMemory proxy initialization"		
		global prox_memory
		prox_memory = ALProxy("ALMemory")
		if prox_memory is None:
			rospy.logerr("[Communication server] - Could not get a proxy to ALMemory")
			exit(1)
			
		print "[Communication server] - ALTextToSpeech proxy initialization"
		self.prox_tts = ALProxy("ALTextToSpeech")
		if self.prox_tts is None:
			rospy.logerr("[Communication server] - Could not get a proxy to ALTextToSpeech")
			exit(1)
			
		print "[Communication server] - ALSoundDetection proxy initialization"
		self.prox_sd = ALProxy("ALSoundDetection")
		if self.prox_sd is None:
			rospy.logerr("[Communication server] - Could not get a proxy to ALSoundDetection")
			exit(1)
			
		print "[Communication server] - ALSpeechRecognition proxy initialization"
		self.prox_sprec = ALProxy("ALSpeechRecognition")
		if self.prox_sprec is None:
			rospy.logerr("[Communication server] - Could not get a proxy to ALSpeechRecognition")
			exit(1)
		
		print "[Communication server] - ALAudioRecorder proxy initialization"
		self.prox_ar = ALProxy("ALAudioRecorder")
		if self.prox_ar is None:
			rospy.logerr("[Send Email]- Could not get a proxy to ALAudioRecorder")
			exit(1)

		print "[Communication server] - ALAudioDevice proxy initialization"
		self.prox_audevice = ALProxy("ALAudioDevice")
		if self.prox_audevice is None:
			rospy.logerr("[Record the sound]- Could not get a proxy to ALAudioDevice")
			exit(1)
	
	
	# Setting variables
	def setVariables(self):
		print "[Communication server] - Setting variables"
		self.isEmailFound = False
		self.stopListening = False
		self.email_address = "rapp.nao@gmail.com"
		self.prox_sd.setParameter("Sensitivity",0.7)
		self.period=500
		self.memValue = "LastWordRecognized"
		# Temporary database of recognized words
		self.database = ["Hazard", "Rapp", "Nao", "Alarm", "Exit"]
		self.prox_sprec.pause(True)
		self.prox_sprec.setVocabulary(self.database, False)
		self.prox_sprec.pause(False)
		
		
		
		self.recordedExtention="ogg"
		# Sample rate of recorded audio (in Hz)
		self.sampleRate = 16000
		# Creating channel
		self.channels = []
		#Channels setup
		self.channels.append(0)
		self.channels.append(0)
		self.channels.append(1)
		self.channels.append(0)
		
		self.recordingTime = 3.0

		self.recordedAudioFile = []
	def setWordDatabase(self, database):
		print "[Communication server] - Setting word database to recognize a spoken word"
		self.prox_sprec.pause(True)
		self.prox_sprec.setVocabulary(database, False)
		self.prox_sprec.pause(False)
		
	# Initialization of ROS services
	def openServices(self):
		try:
			print "[Communication server] - setting services"
			print "[Communication server] - service - [rapp_say]"
			self.service_rs = rospy.Service('rapp_say', Say, self.handle_rapp_say)

			print "[Communication server] - service - [rapp_voice_record]"
			self.service_rvr = rospy.Service('rapp_voice_record', VoiceRecord, self.handle_rapp_voice_record)

			print "[Communication server] - service - [rapp_get_email_address]"
			self.service_rgea = rospy.Service('rapp_get_email_address', GetEmailAddress, self.handle_rapp_get_email_address)

			print "[Communication server] - service - [rapp_get_recognized_word]"
			self.service_rgrw = rospy.Service('rapp_get_recognized_word', RecognizeWord, self.handle_rapp_get_recognized_word)

			print "[Communication server] - service - [rapp_get_microphone_energy]"
			self.service_rgme = rospy.Service('rapp_get_microphone_energy', MicrophoneEnergy, self.handle_rapp_microphone_energy)

			print "[Communication server] - service - [rapp_record]"
			self.service_rr = rospy.Service('rapp_record', Record, self.handle_rapp_record)

			print "[Communication server] - service - [rapp_record_with_sound_detection]"
			self.service_rrwsd = rospy.Service('rapp_record_with_sound_detection', RecordWithSoundDetection, self.handle_rapp_record_with_sound_detection)

			'''print "[Communication server] - service - [rapp_send_email]"
			self.service_rse = rospy.Service('rapp_send_email', SendEmail, self.handle_rapp_send_email)'''
		except Exception, ex:
			print "[Communication server] - Exception %s" % str(ex)
		
	'''# Closes ROS services
	def closeServices(self):
		print "[Communication server] - closes services"
		self.service_hw.close()'''
		
	#######################################
	
	# Core functionality methods 
	
	#######################################
	
	# Subscribes Nao events
	def subscribe(self):
		print "[Communication server] - Subscribing SoundDetected event"
		try:
			self.prox_sprec.subscribe("Test_SpeechDetected",self.period, 0.0)
			prox_memory.subscribeToEvent(Constants.EVENT_SOUND, self.moduleName, "onSoundDetected" )
			#prox_memory.subscribeToEvent(Constants.EVENT_SOUND, self.moduleName, "onWordRecognized" )
		except Exception, e:
			print "[Communication server] - Error in subscribe(): %s", str(e)

	def subscribeWordDetection(self):
		print "[Communication server] - Subscribing SoundDetected event"
		try:
			prox_memory.subscribeToEvent("WordRecognized", self.moduleName, "onWordRecognized" )
		except Exception, e:
			print "[Communication server] - Error in subscribe(): %s", str(e)
		
		#######################################
	# Unsubscribes Nao events
	def unsubscribe(self):
		print "[Communication server] - Unsubscribing SoundDetected event"
		try:
			prox_memory.unsubscribeToEvent(Constants.EVENT_SOUND, self.moduleName)
			self.prox_sprec.unsubscribe("Test_SpeechDetected")
		except TypeError, e:
			print "[Communication server] - Error TypeError in unsubscribe(): %s", str(e)
			#self.prox_sprec.unsubscribe("Test_SpeechDetected")
		except Exception, e:
			print "[Communication server] - Error in unsubscribe(): %s", str(e)
			#self.prox_sprec.unsubscribe("Test_SpeechDetected")

	def unsubscribeWordDetection(self):
		print "[Communication server] - Unsubscribing SoundDetected event"
		try:
			prox_memory.unsubscribeToEvent('WordRecognized', self.moduleName)
		except TypeError, e:
			print "[Communication server] - Error TypeError in unsubscribe(): %s", str(e)
		except Exception, e:
			print "[Communication server] - Error in unsubscribe(): %s", str(e)
			
	#######################################
	#######################################
	# Method that is called when sound is detected, handles word detection from a database
	def onWordRecognized(self, *_args):
		"""This method will be called each time NAO recognised a sound.
		It will try to find out correct email address.
		"""
		try:
			# Unsubscribing to the event of Sound detection to avoid calling a method while executing interior of the method.
			prox_memory.unsubscribeToEvent(Constants.EVENT_SOUND, self.moduleName)
			val = prox_memory.getData(self.memValue)
			
			print "[Communication server] - \t" + val[0]
			print "[Communication server] - onWordRecognized - Sleeps"
			time.sleep(1)
			print "[Communication server] - [onWordRecognized] - Recognized word: " +val[0] +" with the probability equals to " + str(val[1])
			
			if len(val[0])!=0 and val[1]>0.45:	# val[0] -- Recognized word; val[1] -- Confidence level;
				self.stopListening = True
				self.wordRecognized = val[0]
				print "[Communication server] - [onWordRecognized] - Recognized word: %s" % self.wordRecognized
				val[1]=0
				prox_memory.insertData(self.memValue,val)
				return
			else :
				# Subscribe again to the event
				prox_memory.subscribeToEvent("WordRecognized", self.moduleName, self.functionName )
				
		except Exception, e:
			print "[Communication server] - onSoundDetected - Exception %s" %e

		#######################################
	# Method that is called when sound is detected
	def onSoundDetected(self, *_args):
		"""This method will be called each time NAO recognised a sound.
		It will try to find out correct email address.
		"""
		try:
			# Unsubscribing to the event of Sound detection to avoid calling a method while executing interior of the method.
			prox_memory.unsubscribeToEvent(Constants.EVENT_SOUND, self.moduleName)
			val = prox_memory.getData(self.memValue)
			
			print "[Communication server] -       " + val[0]
			print "[Communication server] - Sleeps"
			time.sleep(1)
			print "[Communication server] - [onSoundDetected] - Heard name: " +val[0] +" with the probability equals to " + str(val[1])
			
		
			if len(val[0])!=0 and val[1]>0.5:
				## val[1] - probability
				#if(val[0] == "Exit"):
				#	print "[Communication server] - Exits"
				#	self.stopListening=True
				#	return
				#else:
				#	self.findOutEmail(val[0])
				#self.findOutEmail(val[0])
				
				print "[Communication server] - [onSoundDetected] - Word FOUND: %s" % val[0]#self.email_address
				return
			else:
				# Subscribe again to the event
				prox_memory.subscribeToEvent(Constants.EVENT_SOUND, self.moduleName, "onSoundDetected" )
			
		except Exception, e:
			print "[Communication server] - onSoundDetected - Exception %s" %e
	
		#######################################
	
	'''# Method that is called when sound is detected, handles word detection from a database
	def onWordRecognized(self, *_args):
		"""This method will be called each time NAO recognised a sound.
		It will try to find out correct email address.
		"""
		try:
			# Unsubscribing to the event of Sound detection to avoid calling a method while executing interior of the method.
			prox_memory.unsubscribeToEvent(Constants.EVENT_SOUND, self.moduleName)
			val = prox_memory.getData(self.memValue)
			
			print "[Email server] - \t" + val[0]
			print "[Email server] - onWordRecognized - Sleeps"
			time.sleep(1)
			print "[Email server] - [onSoundDetected] - Heard name: " +val[0] +" with the probability equals to " + str(val[1])
			
		
			if len(val[0])!=0 and val[1]>0.45:	
				self.stopListening = True
				self.wordRecognized = val[0]
				print "[Email server] - [onSoundDetected] - Word recognized: %s" % self.wordRecognized
				val[1]=0
				prox_memory.insertData(self.memValue,val)
				return
			else :
				# Subscribe again to the event
				prox_memory.subscribeToEvent(Constants.EVENT_SOUND, self.moduleName, self.functionName ) # testing
				
			# Subscribe again to the event
			#prox_memory.subscribeToEvent(Constants.EVENT_SOUND, self.moduleName, self.functionName )
			
		except Exception, e:
			print "[Email server] - onSoundDetected - Exception %s" %e
	'''
	#######################################
	# Record an audio message
	def recordAudio(self, file_dest, waiting_time, microphone_energy):
		try:
			print "[Audio record] - Recording an audio message"
			#self.countDown("Recording an audio message")
			self.prox_tts.say("Recording starts")

			recording_iterator = 0	# for the waiting time	
			if (file_dest=="" or (file_dest[len(file_dest)-4:len(file_dest)]!=".ogg")):
				file_dest = Constants.recorded_file_dest;
				
			self.prox_ar.startMicrophonesRecording(file_dest, self.recordedExtention, self.sampleRate, self.channels )
			#self.prox_ar.startMicrophonesRecording(file_dest)

			self.prox_audevice.enableEnergyComputation(); #Enables the computation of the energy on each input channel (this computation is off by default)
			print  "[Audio record] - Start Microphones Recording"
			#print recording_iterator
			while (recording_iterator< waiting_time/0.05):
				energy = self.prox_audevice.getFrontMicEnergy()
				if energy <= microphone_energy: # waiting for sound with a sufficient energy level
					#print energy
					if recording_iterator == 0:
						print  "[Audio record] - Waits for the sound for about %.1f seconds"%waiting_time
					recording_iterator += 1
					time.sleep(0.05)
					
				else:
					if recording_iterator>0:
						print  "[Audio record] - Sound detected"
					recording_iterator = 0 # reseting the iterator
			#print recording_iterator
						
			## Recording stops and the file is being closed after waiting_time, if sound with a sufficient energy level was not detected during this time
			self.prox_ar.stopMicrophonesRecording()
			print "[Send Email] - Recording stops"
			self.prox_tts.say("Recording stops")


		except Exception, e:
			print "[Audio record] - Error during recording an message"
			print "[Audio record] - Error: %s" % str(e)
			self.prox_ar.stopMicrophonesRecording()
	#######################################

	# Method used to find out an email address just using local file with email addesses located 
	# (maybe in the future in "../data/email_address.txt" (it can be done using database in a RAPP cloud)
	def findOutEmail(self,name):
		info = "Find out an email address to %s" % name
		print "[Send Email] - Find out an email address to %s" % name
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
			print "[Send Email] - An email address is: %s " % rapp_email
			rapp_s.close()
			self.prox_tts.say(rapp_email)
			print "[Send Email] - Setting an email address"
			self.email_address = rapp_email
			self.isEmailFound  = True
		except ValueError:
			print "[Send Email] - findOutEmail - Value Error - Probably there is the difference in name - capital letter"
	
	def countDown(self, message):
		print "[Send Email] - %s in 3 seconds" % message
		self.prox_tts.say(message + " in 3 seconds")
		print "[Send Email] - %s in 2 seconds" % message
		self.prox_tts.say(message + " in 2 seconds")
		print "[Send Email] - %s in 1 seconds" % message
		self.prox_tts.say(message + " in 1 second")
		# Sleeps a while (1 second)
		time.sleep(1)
		print "[Send Email] - GO"
		self.prox_tts.say("GO")
	
	# Record an email message
	def recordEmail(self):
		try:
			print "[Send Email] - Recording an email"
			self.countDown("Recording an email")
						
			#self.prox_ar.stopMicrophonesRecording()
			
			self.prox_ar.startMicrophonesRecording(Constants.recorded_file_dest, self.recordedExtention, self.sampleRate, self.channels )
			print  "[Send Email] - Start Microphones Recording"
			print  "[Send Email] - Sleeps"
			# Waiting recordingTime - it means that the message is being recorded recordingTime (seconds)
			time.sleep(self.recordingTime)
			# Recording stops and the file is being closed after recordingTime
			self.prox_ar.stopMicrophonesRecording()
			print "[Send Email] - Recording stops"
			self.prox_tts.say("Recording stops")

		except Exception, e:
			print "[Send Email] - Error during recording an message"
			print "[Send Email] - Error: %s" % str(e)
			self.prox_ar.stopMicrophonesRecording()
	
	#######################################

	#------------------ recording whith sound detection ------------------#
	def startAudioRecord(self, file_path, mic_energy, waiting_time):
		## // for saving buffer to file
		
		if file_path=="":
			file_path='testBySoundEnergy.raw'#/home/nao/recordings/microphones/testByTime.raw'
		try:
			self.saveFile= open(file_path, 'wb')#'ab' 'wb'
		except IOError:
			# If not exists, create the file
			print "File was created"
			self.saveFile = open(file_path, 'w+')

		## testing \/
		self.recordedAudioFile = file_path				## testing
		self.packages = 0 								## testing
		self.iterator = 0 								## testing
		self.fileName = ""
		self.audioBufferList = []
		## testing ^^
		
		
		self.prox_audevice.enableEnergyComputation(); #Enables the computation of the energy on each input channel (this computation is off by default)
		detect = True;
		iterator = 0;
		print 'Subscribe AudioDevice will start the processRemote'
		self.prox_audevice.subscribe(self.getName()) # start recording -- start the processRemote
		time.sleep(2)#1
		while (detect == True):
			if self.prox_audevice.getFrontMicEnergy() <=  mic_energy: # Ends the audio recording, when the sound is not detected in some time
				while iterator< waiting_time*20: # zmienic tak by byl odpowiedni czas oczekiwania na kolejne slowo
					iterator +=1
					time.sleep(0.05)
					if self.prox_audevice.getFrontMicEnergy() >  mic_energy:
						print "Energy: %f"%self.prox_audevice.getFrontMicEnergy()
						iterator = 0 # Reseting the iterator if sound of desired energy was detected
					detect = False;
					
		## // for saving buffer to file
		self.saveFile.close() # close the file
		if not (self.saveAudioBuffer.closed): #checks if the file is still open
			self.saveAudioBuffer.close() # close the file
		
		'''
		# for waiting
		if self.prox_audevice.getFrontMicEnergy() <=  mic_energy:
			if iterator< waiting_time*20:
				iterator += 1;
		else:
			iterator = 0;
		'''# ^^

		print 'Unsubscribe AudioDevice will stop the processRemote'
		self.prox_audevice.unsubscribe(self.getName())
		time.sleep(1)

		#print self.audioBufferList ## testing

		
		#return self.audioBufferList ## change the srv file
		return self.buff ## change the srv file

	#------------------ recording for a preset time ------------------#
	def startAudioRecordByTime(self, n, frame_rate, channels):#record for an amount of time
		file_path='testByTime.raw'#/home/nao/recordings/microphones/testByTime.raw'
		try:
			self.saveFile= open(file_path, 'wb')#'ab' 'wb'
		except IOError:
			# If not exists, create the file
			print "File was created"
			self.saveFile = open(file_path, 'w+')


		file_size = os.path.getsize(file_path)
		print "File size: %f"%file_size
		print 'Subscribe AudioDevice will start the processRemote'
		self.prox_audevice.subscribe(self.getName())
		time.sleep(n) ## records for n seconds
		file_size = os.path.getsize(file_path)
		print "File size: %f"%file_size
		self.saveFile.close()
		print 'Unsubscribe AudioDevice will stop the processRemote'
		self.prox_audevice.unsubscribe(self.getName())
		time.sleep(1)
		return file_path

	def remove_the_file(file_name):
		os.remove(file_name);
	#########################
	
	# Handling methods - methods that used handling services
	
	#########################
		
	def handle_rapp_say(self,req):
		print "[Communication server receives]: \t%s\n" % (req.request)
		'''if (req.language=="English" or req.language=="Greek"):
			self.prox_tts.setLanguage(req.language); # sets the language
		else:
			self.prox_tts.setLanguage("English"); # sets the default language
		'''
		try:
			self.prox_tts.setLanguage(req.language); # sets the language
			if self.prox_tts is None:
				raise RuntimeError("Cannot set language: ALTextToSpeech is not available.")
		except:
			error = "Language " + req.language + " cannot be set."
			print error
			print "Setting the default language."
			try:
				self.prox_tts.setLanguage("English"); # sets the default language
			except:
				print "The default language cannot be set."
			
		self.prox_tts.say(req.request)
		return SayResponse(req.request)
	#########################
	def handle_rapp_get_email_address(self, req):
		print "[Communication server] - receives path to dictionary \t%s" % req.pathToDictionary
		
		self.pathToDictionary = req.pathToDictionary
		self.isEmailFound  = False
		self.stopListening = False
		isFound=0
		
		try:
			print "[Communication server] - Subscribing events"
			self.subscribe()
			while self.isEmailFound == False and self.stopListening == False:
				print "[Communication server] - An email address was not found!"
				print "[Communication server] - Say a special word from database!"
				time.sleep(4)
						
			print "[Email Email] - Unsubscribing events"
			self.unsubscribe()
			
		except AttributeError, ex:
			print "[Communication server] - Exception AtrributeError = %s" % str(ex)
		except Exception, ex:
			print "[Communication server] - Unnamed exception = %s" % str(ex)
		
		print "[Communication server] - returns email address: \t %s" % self.email_address
		
		if self.isEmailFound == True:
			isFound =1
		return GetEmailAddressResponse(self.email_address, isFound)

	#########################
	def handle_rapp_record(self,req):
		print "[Communication server]: - Nao records %d [s]" %req.recordingTime
		self.recordingTime = req.recordingTime
		self.prox_tts.say("Nao records : ")
		self.recordEmail()
		reponse = Constants.recorded_file_dest
		return RecordResponse(reponse)

	#########################
	def handle_rapp_record_with_sound_detection(self,req):
		print "[Communication server]: - Nao records [s]"
		file_dest = req.file_dest
		waiting_time = req.waiting_time
		microphone_energy = req.microphone_energy
		#self.prox_tts.say("Nao records : ")
		self.recordAudio(file_dest, waiting_time, microphone_energy)
		if (file_dest=="" or (file_dest[len(file_dest)-4:len(file_dest)]!=".ogg")):
			file_dest = Constants.recorded_file_dest;
		response = file_dest
		return RecordWithSoundDetectionResponse(response)

	#########################
	def handle_rapp_voice_record(self,req):
		#print "[Communication server]: - Nao records - with sound lvl detection"# until no sound, with the microphone energy above %d, will be detected during the waitnig time equal %d [s]" %(req.microphoneEnergy, req.silenceTime)
		nNbrChannelFlag = 3; # ALL_Channels: 0,  AL::LEFTCHANNEL: 1, AL::RIGHTCHANNEL: 2; AL::FRONTCHANNEL: 3  or AL::REARCHANNEL: 4.
		nDeinterleave = 0;
		nSampleRate = 16000;# 16000 or 48000
		voice_detect = False
		
		# response = ""
		if (req.startRecording==True):
			if (self.isAudDeviceSubscribed == False):
				self.buff = []
				self.processIterator = 0 # for limitation of getting buffer from processRemote method
				self.prox_audevice.setClientPreferences( self.getName(),  nSampleRate, nNbrChannelFlag, nDeinterleave ); # setting same as default generate a bug !?!
				print 'Subscribe AudioDevice will start the processRemote \n\t The recording started'
				self.prox_audevice.subscribe(self.getName()) # start recording -- start the processRemote
				self.isAudDeviceSubscribed = True;
				#print "Buffor length: %f"%len(self.buff); #test
				while (len(self.buff) < 8192 and nSampleRate == 48000):#8192 for 48000Hz
					self.isAudDeviceSubscribed = True;
				while (len(self.buff) < 2730 and nSampleRate == 16000):#2730 for 16000Hz
					self.isAudDeviceSubscribed = True;
			'''if (self.processIterator == 1):
				self.buff = []
				return self.buff
			'''
			#print self.processIterator

		elif (req.startRecording==False):
			print 'Unsubscribe AudioDevice will start the processRemote'
			self.prox_audevice.unsubscribe(self.getName()) # stop recording -- stops the processRemote
			self.isAudDeviceSubscribed = False;
			self.processIterator=0;
		
		return VoiceRecordResponse(self.buff)#response)

		#########################
	def processRemote(self, nbOfInputChannels, nbOfInputSamples, timeStamp, inputBuff): #for audio recording
		# print 'Process remote is called every 85 ms'
		t = timeStamp[0]+timeStamp[1]/1e6 # time in s
		#print(("%.3f receive audio buffer composed of " +  str(nbOfInputChannels) +" channels and " + str(nbOfInputSamples) + " samples")%t)
		#self.saveFile.write(inputBuff) # records to the one file
		self.freeze = True
		if (self.processIterator==0):
			self.buff = inputBuff;
			self.processIterator+=1; # for copying the buffer once per 170ms; microphone buffer should be 170ms
			#print self.processIterator
		else:
			
			if (self.buff == inputBuff and self.processIterator > 0):
				self.buff = []
				self.processIterator=0;
			else:
				self.buff = inputBuff;
				self.processIterator=1;
			
		#print len(self.buff) # for testing
		#print type(inputBuff)
		#print type(self.buff)

		self.freeze = False

		

	#########################
	def handle_rapp_microphone_energy(self,req):
		energy = 0
		
		self.prox_audevice.enableEnergyComputation(); #Enables the computation of the energy on each input channel (this computation is off by default)
		if req.microphone == "front":
			energy = int(self.prox_audevice.getFrontMicEnergy())
		elif req.microphone == "left":
			energy = int(self.prox_audevice.getLeftMicEnergy())
		elif req.microphone == "right":
			energy = int(self.prox_audevice.getRightMicEnergy())
		else:
			energy = int(self.prox_audevice.getFrontMicEnergy())
		
		return energy;


	#########################

	#########################
	def handle_rapp_get_recognized_word(self,req):
		print "[Communication server]: - Nao recognizes a word from a list:"
		for i in req.wordsList:
			print "[Communication server module] - Database = %s" % i
		self.stopListening = False
		self.setWordDatabase(req.wordsList)
		#print req.wordsList
		self.functionName = "onWordRecognized"
		self.wordRecognized ="Empty"
		try:
			print "[Communication server] - Subscribing events"
			self.subscribeWordDetection()
			while self.stopListening == False:
				print "[Communication server] - Word was not recognized!"
				#print "[Communication server] - Say a special word from database!"
				time.sleep(4)
			#print "[Communication Email] - Unsubscribing events"
			#self.unsubscribeWordDetection()
			#self.prox_tts.say("Word recognized %s" % self.wordRecognized)
			
		except AttributeError, ex:
			print "[Communication server] - Exception AtrributeError = %s" % str(ex)
		except Exception, ex:
			print "[Communication server] - Unnamed exception = %s" % str(ex)
		
		return RecognizeWordResponse(self.wordRecognized)
	#########################
	'''def handle_rapp_send_email(self,req):
		print "[Communication server]: - Nao sends an email to %s" %req.emailAddress
		to_say = "Nao sends an email to %s" %req.emailAddress
		#self.prox_tts.say(to_say)
		#Constants.recorded_file_dest = req.recordedFileDest
		isEmailSend = self.sendEmail()
		return SendEmailResponse(isEmailSend)'''
	#########################
# Testng SIGINT signal handler
def signal_handler(signal, frame):
	print "[Communication server] - signal SIGINT caught"
	print "[Communication server] - system exits"
	sys.exit(0)

def main():
	""" Main entry point
	
	"""
	
	# It is needed to use a broker to be able to construct NAOQI 
	# modules and subscribe to other modules. The broker must stay
	# alive until  the program exists
	try:
		signal.signal(signal.SIGINT, signal_handler)
		print "[Communication server] - Press Ctrl + C to exit system correctly"
		
		myBroker = ALBroker("myBroker", "0.0.0.0", 0, Constants.NAO_IP, Constants.PORT)
		global Communication
		Communication = CommunicationModule("Communication")
		rospy.spin()
	
	except AttributeError:
		print "[Communication server] - Communication - AttributeError"
		myBroker.shutdown()
		sys.exit(0)
		
	except (KeyboardInterrupt, SystemExit):
		print "[Communication server] - SystemExit Exception caught"
		myBroker.shutdown()
		sys.exit(0)
		
	except Exception, ex:
		print "[Communication server] - Exception caught %s" % str(ex)
		myBroker.shutdown()
		sys.exit(0)
		
if __name__ == "__main__":
	try:
		main()
	except Exception,e:
		print "__name__ - Error %s" % str(e)
