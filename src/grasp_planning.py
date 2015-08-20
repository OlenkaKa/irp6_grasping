#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import rospy
from irp6_grasping_msgs.srv import *


# Function call service in order to get list of recignized objects.
# age - limitation to objects being perceived within last 'age' seconds
# limit - limitation of number of objects (0 means that the limit is inactive)
# Returns list of ObjectID objects.
def get_recognized_objects_list(age, limit):
    rospy.wait_for_service('return_recognized_objects_list')
    try:
        # Get proxy.
        get_ro_list = rospy.ServiceProxy('return_recognized_objects_list', GetRecognizedObjectsList)
        # Query.
        return get_ro_list(age, limit)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e



# Function call service in order to get list of recignized objects.
# object_id - id of object pose to be returned
# age - limitation - the object must be perceived within last 'age' seconds
# Returns RecognizedObject object.
def get_recognized_object_pose(object_id, age):
    rospy.wait_for_service('return_recognized_object_pose')
    try:
        # Get proxy.
        get_ro_pose = rospy.ServiceProxy('return_recognized_object_pose', GetRecognizedObjectPose)
        # Query.
        return get_ro_pose(object_id, age)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
    for i in range(1):
        # Get list of objects - objects being perceived in last 5 seconds without limit for their number (0).
        oids = get_recognized_objects_list(rospy.Time(5), 0)
        # List all objects.
        for oid in oids.object_ids:
            print 'Object {0} with confidence {1:.4f}'.format(oid.id, oid.confidence)
        # Wait
        time.sleep(1)
    # Get pose of the first object.
    ret = get_recognized_object_pose('/herbapol_mieta1_object', rospy.Time(5))
    if (ret.status!=GetRecognizedObjectPoseResponse.OBJECT_FOUND):
        print 'Object not found'
        exit(-1)
    # Else - let's grab it!
    print 'Object x= {0:.4f} y={1:.4f} z={2:.4f}'.format(ret.object.pose.pose.pose.position.x,ret.object.pose.pose.pose.position.y,ret.object.pose.pose.pose.position.z)

