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
import numpy as np

from irp6_grasping_msgs.srv import *

from irpos import *


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

# Tolerance used for computation of distance between gripper fingers (the distance, computed on the basis of object dimensions, is reduced by this parameter), in meters.
grasp_distance_tolerance = 0.002


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


def generate_grasps_wrt_object(obj_id, obj_pose ):
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
	grasp_top_origin = ['grasp_top', PyKDL.Frame(PyKDL.Rotation.RPY(math.pi, 0, 0), PyKDL.Vector(object_models[obj_number][1][0]/2, object_models[obj_number][1][1]/2, object_models[obj_number][1][2])), math.fabs(object_models[obj_number][1][1])-grasp_distance_tolerance]
	grasp_bottom_origin = ['grasp_bottom', PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(object_models[obj_number][1][0]/2, object_models[obj_number][1][1]/2, 0.0)), math.fabs(object_models[obj_number][1][1])-grasp_distance_tolerance]
	grasp_left_origin = ['grasp_left', PyKDL.Frame(PyKDL.Rotation.RPY(math.pi/2, 0, math.pi/2), PyKDL.Vector(0.0, object_models[obj_number][1][1]/2, object_models[obj_number][1][2]/2)), math.fabs(object_models[obj_number][1][2])-grasp_distance_tolerance]
	grasp_right_origin = ['grasp_right', PyKDL.Frame(PyKDL.Rotation.RPY(math.pi/2, 0, -math.pi/2), PyKDL.Vector(object_models[obj_number][1][0], object_models[obj_number][1][1]/2, object_models[obj_number][1][2]/2)), math.fabs(object_models[obj_number][1][2])-grasp_distance_tolerance]
	grasp_front_origin = ['grasp_front', PyKDL.Frame(PyKDL.Rotation.RPY(math.pi/2, 0, 0), PyKDL.Vector(object_models[obj_number][1][0]/2, 0.0, object_models[obj_number][1][2]/2)), math.fabs(object_models[obj_number][1][1])-grasp_distance_tolerance]
	grasp_back_origin = ['grasp_back', PyKDL.Frame(PyKDL.Rotation.RPY(-math.pi/2, 0, 0), PyKDL.Vector(object_models[obj_number][1][0]/2, object_models[obj_number][1][1], object_models[obj_number][1][2]/2)), math.fabs(object_models[obj_number][1][1])-grasp_distance_tolerance]
	return [grasp_top_origin,grasp_bottom_origin,grasp_left_origin, grasp_right_origin, grasp_front_origin, grasp_back_origin]


def dummy_highest_grasp_pose_selection( grasp_poses ):
	max_z = -100
	max_i = -1
	for i in range(len(grasp_poses)):
		#print grasp_poses[i]
		print grasp_poses[i][1].p
		#print i, grasp_poses[i].p.z(), max_z
		if (grasp_poses[i][1].p.z() > max_z):
			max_z = grasp_poses[i][1].p.z()
			max_i = i;
	return ['selected_grasp', grasp_poses[max_i][1], grasp_poses[max_i][2]]



def generate_pregrasp( selected_grasp ):
	# pregrasp pose: along z by -10 cm.
	pose = selected_grasp[1] * PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0),PyKDL.Vector(0.0, 0.0, -0.20))
	# Set finger distance to 9cm.
	return ['pregrasp', pose, 0.09]


if __name__ == "__main__":
	half_pi = math.pi/2

	# Choose robot
	robot_name = rospy.get_param('robot_name')
	print '%s robot choosed' % robot_name

	if robot_name == 'Irp6ot':
		irpos = IRPOS('irp6ot_grasping', 'Irp6ot', 7, 'irp6ot_manager')
		front_desired_joints = [0, 0, -half_pi, 0, 0, 3*half_pi, -half_pi]
	elif robot_name == 'Irp6p':
		irpos = IRPOS('irp6p_grasping', 'Irp6p', 6, 'irp6p_manager')
		front_desired_joints = [0, -half_pi, 0, 0, 3*half_pi, half_pi]
	else:
		print 'Incorrect robot name'
		sys.exit()

	# Initialize node
	#rospy.init_node('irp6_grasping')
	#Initialize IRPOS for IRp6-ot robot - it also initializes ROS NODE(!)
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
		# Move robot to front position.
		# Get current joint position.
		current_joints = irpos.get_joint_position()
		# Compare joint positions.
		diff = np.array(front_desired_joints[1:6]) - np.array(current_joints[1:6])
		if np.amax(np.absolute(diff)) > 0.01:
			print 'Moving %s to front position' % robot_name
			irpos.move_to_joint_position(front_desired_joints, 20.00)
		else:
			print '%s standing in front position' % robot_name
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
		# Broadcast TF with object pose.
		obj_pose = posemath.fromMsg(ret.object.pose.pose.pose)
		broadcast_pose(br, obj_pose, ret.object.type.key, ret.object.header.frame_id)
		# Generate grasps in object reference frame
		grasps_wrt_object = generate_grasps_wrt_object(id, obj_pose)
		# Check grasps.
		if len(grasps_wrt_object) == 0:
			print "ERROR: Cannot generate grasping points! Skipping this one and trying to recognize and grasp another object!"
			continue
		print "Properly generated grasping points for ", oids.object_ids[0]," object!"
		#print ' +--  Generated Grasp 0: ',generated_grasps[0]
		#print ' +--  Generated Grasp 1: ',generated_grasps[1]
		#print ' +--  Generated Grasp 2: ',generated_grasps[2]
		#print ' +--  Generated Grasp 3: ',generated_grasps[3]
		#print ' +--  Generated Grasp 4: ',generated_grasps[4]
		#print ' +--  Generated Grasp 5: ',generated_grasps[5]
		# Transform grasps to world reference frame
		grasps_wrt_world = []
		for gr_i in range(6):
			#print generated_grasps[gr_i][0], "with finger distance ",generated_grasps[gr_i][2]
			grasps_wrt_world.append([grasps_wrt_object[gr_i][0], obj_pose* grasps_wrt_object[gr_i][1], grasps_wrt_object[gr_i][2]]);
			#grasp_name = generated_grasps[gr_i][0]#ret.object.type.key + grasp_data[0]
			broadcast_pose(br, grasps_wrt_world[gr_i][1], grasps_wrt_world[gr_i][0], ret.object.header.frame_id)
		# Grasp selection - select "highest" pose.
		selected_grasp = dummy_highest_grasp_pose_selection(grasps_wrt_world)
		broadcast_pose(br, selected_grasp[1], selected_grasp[0], ret.object.header.frame_id)
		# Generate pregrasps.
		pregrasp = generate_pregrasp(selected_grasp)
		broadcast_pose(br, pregrasp[1], pregrasp[0], ret.object.header.frame_id)
		# Move to pregrasp pose.
		irpos.move_to_cartesian_pose(20.0,pm.toMsg(pregrasp[1]))
		# Set pregrasp distance between fingers.
		irpos.tfg_to_joint_position(pregrasp[2], 3.0)
		# Move towards the object.
		#pose = pregrasp[1] * PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0),PyKDL.Vector(0.0, 0.0, 0.10))
		irpos.move_to_cartesian_pose(3.0,pm.toMsg(selected_grasp[1]))
		# Set grasp distance between fingers.
		irpos.tfg_to_joint_position(selected_grasp[2], 3.0)
		# Move back to pregrasp pose.
		irpos.move_to_cartesian_pose(3.0,pm.toMsg(pregrasp[1]))
		# Set pregrasp distance between fingers - drop object.
		irpos.tfg_to_joint_position(pregrasp[2], 3.0)

		#end of program;)

