#!/usr/bin/env python
import roslib
import rospy
import time

import tf


if __name__ == '__main__':
	rospy.init_node('tf_broadcaster')
	print 'start'
	br = tf.TransformBroadcaster()
	time.sleep(1)
	print 'przed'
	br.sendTransform((1, 1, 0), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "test", "world")
	print 'po'
	#rospy.spin()
