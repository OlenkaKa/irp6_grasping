#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
import rospy

import sys
import time
import math

import tf
import tf_conversions.posemath as posemath
import PyKDL

from irp6_grasping_msgs.srv import *

# Initialize predefined grasps in the form of:
# object_id, size(along x,y,z) in m, [grasp_name, translation, rotation (wrt to model reference frame), distance between fingers]
predefined_grasps = [
	['/herbapol_mieta1_object', PyKDL.Vector(0.140, 0.075, 0.064),
		[['grasp_top', PyKDL.Frame(PyKDL.Rotation.RPY(math.pi, 0, 0),PyKDL.Vector(0.07, -0.0375, 0.064)), 0.075],
		['grasp_bottom', PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0),PyKDL.Vector(0.07, -0.0375, 0.0)), 0.075],
		['grasp_left', PyKDL.Frame(PyKDL.Rotation.RPY(math.pi/2, 0, math.pi/2),PyKDL.Vector(0.0, -0.0375, 0.032)), 0.075],
		['grasp_right', PyKDL.Frame(PyKDL.Rotation.RPY(math.pi/2, 0, -math.pi/2),PyKDL.Vector(0.14, -0.0375, 0.032)), 0.075],
		['grasp_front', PyKDL.Frame(PyKDL.Rotation.RPY(math.pi/2, 0, 0),PyKDL.Vector(0.07, 0.0, 0.032)), 0.075],
		['grasp_back', PyKDL.Frame(PyKDL.Rotation.RPY(-math.pi/2, 0, 0),PyKDL.Vector(0.07, -0.075, 0.032)), 0.075]]
	]
]





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


def broadcast_pose(br, pm_pose, child_frame_id, parent_id):
#	print pm_pose.p
#	print pm_pose.M.GetQuaternion()
	br.sendTransform(
		pm_pose.p,
		pm_pose.M.GetQuaternion(),
		rospy.Time.now(),
		child_frame_id, #+"_python",
		parent_id)

def broadcast_corner_poses(br, obj_number, obj_pose, child_frame_id, parent_id):
	obj_dims = obj_number



if __name__ == "__main__":
	# Initialize node
	rospy.init_node('irp6_grasping')
	# Initialize broadcaster
	br = tf.TransformBroadcaster()
	# SLEEP REQUIRED FOR BROADCASTER TO INITIALIZE PROPERLY!!!!
	time.sleep(1)
	#print predefined_grasps
	print '-+  Object 0: ',predefined_grasps[0][0], predefined_grasps[0][1]
	print ' +--  Grasp 0: ',predefined_grasps[0][2][0]
	print ' +--  Grasp 1: ',predefined_grasps[0][2][1]
	#br.sendTransform((1, 1, 0), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "/test2", "world")
	# Get list of objects - objects being perceived in last 5 seconds without limit for their number (0).
	oids = get_recognized_objects_list(rospy.Time(5), 0)
	# List all objects.
	for oid in oids.object_ids:
		print 'Object {0} with confidence {1:.4f}'.format(oid.id, oid.confidence)
	# Wait
	#time.sleep(1)
	while (1):
		# Get pose of the first object.
		ret = get_recognized_object_pose('/herbapol_mieta1_object', rospy.Time(5))
		if (ret.status!=GetRecognizedObjectPoseResponse.OBJECT_FOUND):
			print 'Object not found'
			continue
			#exit(-1)
		# Else - let's grab it!
		#print 'Object x= {0:.4f} y={1:.4f} z={2:.4f}'.format(ret.object.pose.pose.pose.position.x,ret.object.pose.pose.pose.position.y,ret.object.pose.pose.pose.position.z)
		#Recalculate grasp pose
		obj_pose = posemath.fromMsg(ret.object.pose.pose.pose)
		broadcast_pose(br, obj_pose, ret.object.type.key, ret.object.header.frame_id)
		for gr_i in range(6):
			grasp_data = predefined_grasps[0][2][gr_i]
			print grasp_data[0], "with finger distance ",grasp_data[2]
			obj_grasp_pose = grasp_data[1]
			grasp_pose = obj_pose* obj_grasp_pose;
			grasp_name = grasp_data[0]#ret.object.type.key + grasp_data[0]
			broadcast_pose(br, grasp_pose, grasp_name, ret.object.header.frame_id)
		#broadcast_corner_poses(br,
		#break
