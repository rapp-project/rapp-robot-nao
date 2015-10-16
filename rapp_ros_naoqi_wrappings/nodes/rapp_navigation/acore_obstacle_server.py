#!/usr/bin/env python
######################
## written by Wojciech Dudek
######################
__author__ = "Wojciech Dudek"

from rapp_ros_naoqi_wrappings.msg import obstacleData

import rospy
import sys
import signal

from naoqi import ALModule
from naoqi import ALBroker
from naoqi import ALProxy
# Constants
class Constants:

	NAO_IP = "nao.local"
	PORT = 9559

class NaoObstacles(ALModule):
	def __init__(self,name):
		ALModule.__init__(self, name)
		rospy.init_node('acore_obstacleDetector')
		self.moduleName = name
		self.connectNaoQi()

		self.setVariables()

		self.MsgsInit()

		self.memory.subscribeToEvent("RightBumperPressed",
            self.moduleName,
            "onRightBumperPressed")
		self.memory.subscribeToEvent("LeftBumperPressed",
            self.moduleName,
            "onLeftBumperPressed")
	#def Obstacle_LeftBumper():
	def connectNaoQi(self):

		self.memory = ALProxy("ALMemory")
		if self.memory is None:
			exit(1)
	def setVariables(self):
		self.RightBumperPressed = 0
		self.LeftBumperPressed = 0

	def onRightBumperPressed(self,name,val, id_event):
		self.memory.unsubscribeToEvent("RightBumperPressed",
            "Obstacle_RightBumper")
		print "[onRightBumperPressed]: "
		print "name: ",name, "\nval: ",val,"\nid: ",id_event
		

		self.RightBumperPressed = val

        # Subscribe again to the event
		self.memory.subscribeToEvent("RightBumperPressed",
            "Obstacle_RightBumper",
            "onRightBumperPressed")
	def onLeftBumperPressed(self,name,val, id_event):
		self.memory.unsubscribeToEvent("LeftBumperPressed",
            "Obstacle_LeftBumper")
		print "[onLeftBumperPressed]: "
		print "name: ",name, "\nval: ",val,"\nid: ",id_event


		self.LeftBumperPressed = val

        # Subscribe again to the event
		self.memory.subscribeToEvent("LeftBumperPressed",
            "Obstacle_LeftBumper",
            "onLeftBumperPressed")		
	def MsgsInit(self):

		self.bumpersState_Pub = rospy.Publisher("/obstacleDetectorState", obstacleData , queue_size=10)

	def run(self):
		try:
			msg = obstacleData()
			msg.RightBumper = self.RightBumperPressed
			msg.LeftBumper = self.LeftBumperPressed
			self.bumpersState_Pub.publish(msg)
		except Exception, ex:
			print "[Obstacle server] - RUN Exception caught %s" % str(ex)
			#unsubscribe()

def signal_handler(signal, frame):
	print "[Obstacle server] - signal SIGINT caught"
	print "[Obstacle server] - system exits"
	# Obstacle.killall()
	sys.exit(0)

if __name__ == '__main__':
	try:
		signal.signal(signal.SIGINT, signal_handler)
		print "[Obstacle server] - Press Ctrl + C to exit system correctly"
		
		myBroker = ALBroker("myBroker", "0.0.0.0", 0, Constants.NAO_IP,Constants.PORT)
		
		global obstacleDetector
		obstacleDetector = NaoObstacles("obstacleDetector")
		bumperRate = rospy.Rate(2)

		while not rospy.is_shutdown():
			#handle_markers_tf()
			
			obstacleDetector.run()

			bumperRate.sleep()
			
		
	except (KeyboardInterrupt, SystemExit):
		print "[Obstacle server] - SystemExit Exception caught"
		#unsubscribe()
		myBroker.shutdown()
	#	Obstacle.killall()
		sys.exit(0)
		
	except Exception, ex:
		print "[Obstacle server] - Exception caught %s" % str(ex)
		#unsubscribe()
		myBroker.shutdown()
		#Obstacle.killall()

		sys.exit(0)