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
		['grasp_left', PyKDL.Frame(PyKDL.Rotation.RPY(math.pi/2, 0, math.pi/2),PyKDL.Vector(0.0, -0.0375, 0.032)), 0.064],
		['grasp_right', PyKDL.Frame(PyKDL.Rotation.RPY(math.pi/2, 0, -math.pi/2),PyKDL.Vector(0.14, -0.0375, 0.032)), 0.064],
		['grasp_front', PyKDL.Frame(PyKDL.Rotation.RPY(math.pi/2, 0, 0),PyKDL.Vector(0.07, 0.0, 0.032)), 0.075],
		['grasp_back', PyKDL.Frame(PyKDL.Rotation.RPY(-math.pi/2, 0, 0),PyKDL.Vector(0.07, -0.075, 0.032)), 0.075]]
	]
]
# width, height, depth (along x,y,z)
object_models = [
	['/herbapol_mieta1_object', PyKDL.Vector(0.140, -0.075, 0.064)],
	['/ahmad_object', PyKDL.Vector(0.125, -0.085, 0.085)],
	['/lipton_earl_grey_lemon', PyKDL.Vector(0.149, -0.061, 0.040)]
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


def generate_grasp_poses(obj_id, obj_pose ):
	# Find index for given id
	obj_number = -1;
	for i in range(len(object_models)):
		if object_models[i][0] == obj_id:
			obj_number = i
			break
	if obj_number < 0:
		print 'ERROR: Object model not found on the list!'
		return [];
	#grasp_top_origin = ['grasp_top', PyKDL.Frame(PyKDL.Rotation.RPY(math.pi, 0, 0), PyKDL.Vector(0.07, -0.0375, 0.064)), 0.075]
	grasp_top_origin = ['grasp_top', PyKDL.Frame(PyKDL.Rotation.RPY(math.pi, 0, 0), PyKDL.Vector(object_models[obj_number][1][0]/2, object_models[obj_number][1][1]/2, object_models[obj_number][1][2])), math.fabs(object_models[obj_number][1][1])]
	grasp_bottom_origin = ['grasp_bottom', PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(object_models[obj_number][1][0]/2, object_models[obj_number][1][1]/2, 0.0)), math.fabs(object_models[obj_number][1][1])]
	grasp_left_origin = ['grasp_left', PyKDL.Frame(PyKDL.Rotation.RPY(math.pi/2, 0, math.pi/2), PyKDL.Vector(0.0, object_models[obj_number][1][1]/2, object_models[obj_number][1][2]/2)), math.fabs(object_models[obj_number][1][2])]
	grasp_right_origin = ['grasp_right', PyKDL.Frame(PyKDL.Rotation.RPY(math.pi/2, 0, -math.pi/2), PyKDL.Vector(object_models[obj_number][1][0], object_models[obj_number][1][1]/2, object_models[obj_number][1][2]/2)), math.fabs(object_models[obj_number][1][2])]
	grasp_front_origin = ['grasp_front', PyKDL.Frame(PyKDL.Rotation.RPY(math.pi/2, 0, 0), PyKDL.Vector(object_models[obj_number][1][0]/2, 0.0, object_models[obj_number][1][2]/2)), math.fabs(object_models[obj_number][1][1])]
	grasp_back_origin = ['grasp_back', PyKDL.Frame(PyKDL.Rotation.RPY(-math.pi/2, 0, 0), PyKDL.Vector(object_models[obj_number][1][0]/2, object_models[obj_number][1][1], object_models[obj_number][1][2]/2)), math.fabs(object_models[obj_number][1][1])]
	return [grasp_top_origin,grasp_bottom_origin,grasp_left_origin, grasp_right_origin, grasp_front_origin, grasp_back_origin]


def dummy_highest_grasp_pose_selection(grasp_poses ):
	min_z = 100;
	min_i = -1
	for i in range(len(grasp_poses)):
		print i
		print grasp_poses[i]
		print grasp_poses[i][0]
		if (grasp_poses[i][1][2] < min_z):
			min_z = grasp_poses[i][1][2]
			min_i = i;
	return grasp_poses[i]


if __name__ == "__main__":
	# Initialize node
	rospy.init_node('irp6_grasping')
	# Initialize broadcaster
	br = tf.TransformBroadcaster()
	# SLEEP REQUIRED FOR BROADCASTER TO INITIALIZE PROPERLY!!!!
	time.sleep(1)
	#print predefined_grasps
	#print '-+  Object 0: ',object_models[0], object_models[1]
	#print ' +--  Grasp 0: ',predefined_grasps[0][2][0]
	#print ' +--  Grasp 1: ',predefined_grasps[0][2][1]
	#print ' +--  Grasp 2: ',predefined_grasps[0][2][2]
	#print ' +--  Grasp 3: ',predefined_grasps[0][2][3]
	#print ' +--  Grasp 4: ',predefined_grasps[0][2][4]
	#print ' +--  Grasp 5: ',predefined_grasps[0][2][5]
	#br.sendTransform((1, 1, 0), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "/test2", "world")
	while (1):
		# Get list of objects - objects being perceived in last 1 seconds without limit for their number (0).
		oids = get_recognized_objects_list(rospy.Time(1), 0)
		# Check if there are any objects on the list.
		if len(oids.object_ids) == 0:
			print "ERROR: Cannot recognize any objects!"
			time.sleep(1)
			continue
		# List all objects.
		for oid in oids.object_ids:
			print 'Recognized Object {0} with confidence {1:.4f}'.format(oid.id, oid.confidence)

		# Get pose of the first object.
		id = oids.object_ids[0].id
		print id
		ret = get_recognized_object_pose(id, rospy.Time(5))
		if (ret.status!=GetRecognizedObjectPoseResponse.OBJECT_FOUND):
			print 'ERROR: Pose of object ', id, 'could not be recovered!'
			continue
		# Else - let's grab it!
		#print 'Object x= {0:.4f} y={1:.4f} z={2:.4f}'.format(ret.object.pose.pose.pose.position.x,ret.object.pose.pose.pose.position.y,ret.object.pose.pose.pose.position.z)
		# Recalculate grasp poses
		obj_pose = posemath.fromMsg(ret.object.pose.pose.pose)
		broadcast_pose(br, obj_pose, ret.object.type.key, ret.object.header.frame_id)
		generated_grasps = generate_grasp_poses(id, obj_pose)
		if len(generated_grasps) == 0:
			print "ERROR: Cannot generate grasping points! Skipping this one and trying to recognize and grasp another object!"
			continue
		print "Properly generated grasping points for ", oids.object_ids[0]," object!"
		#print ' +--  Generated Grasp 0: ',generated_grasps[0]
		#print ' +--  Generated Grasp 1: ',generated_grasps[1]
		#print ' +--  Generated Grasp 2: ',generated_grasps[2]
		#print ' +--  Generated Grasp 3: ',generated_grasps[3]
		#print ' +--  Generated Grasp 4: ',generated_grasps[4]
		#print ' +--  Generated Grasp 5: ',generated_grasps[5]
		# Broadcast grasping poses.
		grasp_poses = []
		for gr_i in range(6):
			#print generated_grasps[gr_i][0], "with finger distance ",generated_grasps[gr_i][2]
			grasp_poses.append(obj_pose* generated_grasps[gr_i][1]);
			grasp_name = generated_grasps[gr_i][0]#ret.object.type.key + grasp_data[0]
			broadcast_pose(br, grasp_poses[gr_i], grasp_name, ret.object.header.frame_id)
		# Grasp selection - select "highest" pose.
		selected_grasp_pose = dummy_highest_grasp_pose_selection(grasp_poses)
		broadcast_pose(br, selected_grasp_pose, 'selected_grasp_pose', ret.object.header.frame_id)

		#break
