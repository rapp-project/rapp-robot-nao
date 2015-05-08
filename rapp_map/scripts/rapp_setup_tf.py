#!/usr/bin/env python  
######################
## written by Wojciech Dudek
######################
import rospy
import tf
import sys
import signal
def handle_markers_tf():
	br = tf.TransformBroadcaster()
	time = rospy.Time.now()
	
	br.sendTransform((0.066,1.7, 0.54),
                     tf.transformations.quaternion_from_euler(3.14/2, 0, 3.14/2),
                     time,
                     "Wall",
                     "map")
	br.sendTransform((0.259,0.603, 0.32),
                     tf.transformations.quaternion_from_euler(3.14/2, 0, 3.14),
                     time,
                     "Wardrobe",
                     "map")
	br.sendTransform((2.5044,0.5097, 0.825),
                     tf.transformations.quaternion_from_euler(3.14/2, 0, 3.14),
                     time,
                     "Microwave",
                     "map")
	br.sendTransform((3.303,0.6697, 0.75),
                     tf.transformations.quaternion_from_euler(3.14/2, 0, 3.14),
                     time,
                     "Fridge",
                     "map")

	br.sendTransform((3.9593,0.751, 0.505),
                     tf.transformations.quaternion_from_euler(3.14/2, 0, 3.14),
                     time,
                     "Stable object",
                     "map")
	br.sendTransform((0.0654,1.993, 0.555),
                     tf.transformations.quaternion_from_euler(3.14/2, 0, 3.14/2),
                     time,
                     "Door",
                     "map")

def signal_handler(signal, frame):
	sys.exit(0)

def main():
	rospy.init_node('rapp_setup_tf')
	signal.signal(signal.SIGINT, signal_handler)
	sensorRate = rospy.Rate(20)
	try:
		while True:
			handle_markers_tf()
			sensorRate.sleep()
	except (KeyboardInterrupt, SystemExit):
		print "SystemExit Exception caught"
		sys.exit(0)

if __name__ == "__main__":
	try:
		main()
	except Exception,e:
		print "__name__ - Error %s" % str(e)