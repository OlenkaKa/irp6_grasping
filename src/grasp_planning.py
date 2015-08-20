#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import rospy
from irp6_grasping_msgs.srv import *

def get_recognized_objects_list(age, limit):
    rospy.wait_for_service('return_recognized_objects_list')
    try:
        # Get proxy.
        get_ro_list = rospy.ServiceProxy('return_recognized_objects_list', GetRecognizedObjectsList)

        # Query.
        return get_ro_list(age, limit)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e



if __name__ == "__main__":

    while (1):
        # Get list of objects - no limit and objects being perceived in last 10 seconds.
        oids = get_recognized_objects_list(rospy.Time(2), 0)

        # List all objects.
        for oid in oids.object_ids:
            print 'Object {0} with confidence {1:.3f}'.format(oid.id, oid.confidence)

        # Wait
        time.sleep(1)
