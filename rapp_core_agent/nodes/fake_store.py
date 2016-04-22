#!/usr/bin/env python

import os

import rospy
from std_msgs.msg import String

class fake_store:
	def callback(self, data):
		rospy.loginfo(rospy.get_caller_id() + "New request for %s", data.data)
		
		# remove version number and extension from package name
		base_name = data.data.split('-')[0]
		path = os.path.join(self.base_path, base_name)
		
		print path
		
		# check, whether required package is available
		if not os.path.exists(path):
			path = ""
			rospy.loginfo(rospy.get_caller_id() + "Package not available")
		else:
			rospy.loginfo(rospy.get_caller_id() + "Package location: %s", path)
			
		# return path to calling process
		self.pub.publish(str(path))

	def __init__(self):
		rospy.init_node('fake_store', anonymous=True)
		
		self.pub = rospy.Publisher('response', String, queue_size=10)
		rospy.Subscriber('request', String, self.callback)
		
		# read base path from parameter server
		self.base_path = rospy.get_param('weblets_path', '/tmp/rapps')
		
		rospy.spin()

if __name__ == '__main__':
    try:
        fake_store()
    except rospy.ROSInterruptException:
        pass
		
# install with:
# catkin_install_python(PROGRAMS fake_store.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
