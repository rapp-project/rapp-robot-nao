#!/usr/bin/env python
######################
## written by Wojciech Dudek
######################
import rospy
from naoqi import ALModule
from naoqi import ALBroker
from naoqi import ALProxy
import signal
import sys
from geometry_msgs.msg import Twist

class Constants:

    NAO_IP = "nao.local"
    PORT = 9559

class Nao_keyboard(ALModule):
    def __init__(self,name):
        ALModule.__init__(self, name)
        rospy.init_node('acore_keyboard')
        self.moduleName = name
        rospy.Subscriber("/cmd_vel", Twist, self.callback)
        self.connectNaoQi()
        self.standUP()
    def connectNaoQi(self):
        self.motionProxy = ALProxy("ALMotion")
        if self.motionProxy is None:
            rospy.logerr("[Estimator server] - Could not get a proxy to ALMemory")
            exit(1)   
        self.proxy_RobotPosture = ALProxy("ALRobotPosture")
        if self.proxy_RobotPosture is None:
            rospy.logerr("[Move server] - Could not get a proxy to ALRobotPosture")
            exit(1) 
    def StiffnessOn(self):
        # We use the "Body" name to signify the collection of all joints
        pNames = "Body"
        pStiffnessLists = 1.0
        pTimeLists = 1.0
        try:
            self.motionProxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)   
        except Exception, ex:
            print "[Move server] - Exception2 %s" % str(ex)
    def SetPose(self,pose):
        try:
            self.StiffnessOn()
        except Exception, ex:
            print "[Move server] - Exception %s" % str(ex)
        try:    
            self.proxy_RobotPosture.goToPosture(pose, 0.3)
        except Exception, e:
                print "[Move server] - Exception %s" % str(e)   
        print "[Move server] - Actual Nao pose : %s" % str(pose)
    def standUP(self):
        self.SetPose('StandInit')

    def callback(self,data):
        X = data.linear.x
        Y = data.linear.y
        Theta = data.angular.z
        self.motionProxy.move(X, Y, Theta)


def signal_handler(signal, frame):
    print "[Estimator server] - signal SIGINT caught"
    print "[Estimator server] - system exits"
    sys.exit(0)

if __name__ == '__main__':
    try:
        signal.signal(signal.SIGINT, signal_handler)
        print "[Estimator server] - Press Ctrl + C to exit system correctly"
        
        myBroker = ALBroker("myBroker", "0.0.0.0", 0, Constants.NAO_IP,Constants.PORT)
        
        global keyboard
        keyboard = Nao_keyboard("keyboard")
        rospy.spin()
    
    except (KeyboardInterrupt, SystemExit):
        print "[Estimator server] - SystemExit Exception caught"
        #unsubscribe()
        myBroker.shutdown()
        sys.exit(0)
        
    except Exception, ex:
        print "[Estimator server] - Exception caught %s" % str(ex)
        #unsubscribe()
        myBroker.shutdown()
        sys.exit(0)