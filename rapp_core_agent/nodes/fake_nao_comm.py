#!/usr/bin/env python

from rapp_ros_naoqi_wrappings.srv import Say, RecognizeWord
import rospy

def handle_say(req):
    print "NAO says in %s: %s" % (req.language, req.request)
    return 1
    
def handle_recognize_word(req):
    print "NAO is requested to recognize one of those words:"
    for w in req.wordsList:
        print(" - %s" % w)
        
    word = raw_input('Enter your word: ')
    if word in req.wordsList:
        return word
    else:
        return "Empty"

def fake_nao_comm():
    rospy.init_node('fake_nao_comm')
    s1 = rospy.Service('rapp_say', Say, handle_say)
    s2 = rospy.Service('rapp_get_recognized_word', RecognizeWord, handle_recognize_word)
    
    rospy.spin()

if __name__ == "__main__":
    fake_nao_comm()
