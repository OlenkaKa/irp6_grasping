#!/usr/bin/env python
# -*- coding: utf-8 -*-

import types

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
from irpos import IRPOS
from threading import Lock


def _print_message(message):
    print '\033[1;36m[WatchingIRPOS] %s\033[0m' % message


def _print_error_message(message):
    print '\033[1;36m[WatchingIRPOS] %s\033[0m' % message


class WatchingIRPOS(IRPOS):
    # def stop_object_pose_estimation(self, move_func, stop_observation_func, attr_name):
    #     @wraps
    #     def wrapper(*args, **kwargs):
    #         print "Nazwa:  %s" % attr_name
    #         stop_observation_func(args[0])
    #         return move_func(*args, **kwargs)
    #
    # return wrapper

    def __init__(self, nodeName, robotName, robotJointNumbers, scheme_name):
        self._estimate_object_pose = False
        IRPOS.__init__(self, nodeName, robotName, robotJointNumbers, scheme_name)

    ### OVERRIDE MOVE METHODS

    def _stop_object_pose_estimation_wrapper(self, move_func):
        @wraps
        def wrapper(*args, **kwargs):
            if self._estimate_object_pose is True:
                self.stop_scene_observation()
            return move_func(*args, **kwargs)

        return wrapper

    move_to_synchro_position = _stop_object_pose_estimation_wrapper(IRPOS.move_to_synchro_position.__func__)

    move_to_motor_position = _stop_object_pose_estimation_wrapper(IRPOS.move_to_motor_position.__func__)

    move_rel_to_motor_position = _stop_object_pose_estimation_wrapper(IRPOS.move_rel_to_motor_position.__func__)

    move_along_motor_trajectory = _stop_object_pose_estimation_wrapper(IRPOS.move_along_motor_trajectory.__func__)

    move_to_joint_position = _stop_object_pose_estimation_wrapper(IRPOS.move_to_joint_position.__func__)

    move_rel_to_joint_position = _stop_object_pose_estimation_wrapper(IRPOS.move_rel_to_joint_position.__func__)

    move_along_joint_trajectory = _stop_object_pose_estimation_wrapper(IRPOS.move_along_joint_trajectory.__func__)

    move_to_cartesian_pose = _stop_object_pose_estimation_wrapper(IRPOS.move_to_cartesian_pose.__func__)

    move_rel_to_cartesian_pose = _stop_object_pose_estimation_wrapper(IRPOS.move_rel_to_cartesian_pose.__func__)

    move_rel_to_cartesian_pose_with_contact = _stop_object_pose_estimation_wrapper(
        IRPOS.move_rel_to_cartesian_pose_with_contact.__func__)

    move_along_cartesian_trajectory = _stop_object_pose_estimation_wrapper(
        IRPOS.move_along_cartesian_trajectory.__func__)

    move_along_cartesian_circle = _stop_object_pose_estimation_wrapper(IRPOS.move_along_cartesian_circle.__func__)

    ## OPTOFORCE CONTROLLER

    def tfg_to_joint_position_with_contact(self, position):
        _print_message('Tfg to joint position with contact')

        class OptoforceData:
            def __init__(self, optoforce_initial_value):
                self.no_contact_average_measurement = 0
                self.contact_difference = 17
                self.lock = Lock()
                self.contact = None
                self.check_if_contact(optoforce_initial_value)

            def check_if_contact(self, measurement):
                value = measurement.vector.z
                with self.lock:
                    self.contact = abs(value - self.no_contact_average_measurement) > self.contact_difference
                    if not self.contact:
                        self.no_contact_average_measurement = (self.no_contact_average_measurement + value) / 2

        delta = 0.003
        optoforce1_topic = '/optoforce1/force0'
        optoforce2_topic = '/optoforce2/force0'

        optoforce1_data = OptoforceData(rospy.wait_for_message(optoforce1_topic, Vector3Stamped))
        optoforce2_data = OptoforceData(rospy.wait_for_message(optoforce2_topic, Vector3Stamped))

        optoforce1_sub = rospy.Subscriber(optoforce1_topic, Vector3Stamped, optoforce1_data.check_if_contact)
        optoforce2_sub = rospy.Subscriber(optoforce2_topic, Vector3Stamped, optoforce2_data.check_if_contact)

        r = rospy.Rate(0.9)
        while not ((optoforce1_data.contact is True) and (optoforce2_data.contact is True)):
            tfg_joint_position = self.get_tfg_joint_position()[0]
            tfg_diff = position - tfg_joint_position
            if tfg_diff > delta:
                self.tfg_to_joint_position(tfg_joint_position + delta, 1.0)
            elif tfg_diff < -delta:
                self.tfg_to_joint_position(tfg_joint_position - delta, 1.0)
            else:
                break
            r.sleep()

        optoforce1_sub.unregister()
        optoforce2_sub.unregister()

    ### OBJECT POSE ESTIMATION

    def start_scene_observation(self):
        _print_message('Start scene observation')
        if not self._estimate_object_pose:
            self._set_estimate_pose(True, 1)
            self._estimate_object_pose = True

    def stop_scene_observation(self):
        _print_message('Stop scene observation')
        if self._estimate_object_pose:
            self._set_estimate_pose(False, 1)
            self._estimate_object_pose = False

    def get_scene_objects(self):
        rospy.wait_for_service('return_recognized_objects_list')
        try:
            get_ro_list = rospy.ServiceProxy('return_recognized_objects_list', GetRecognizedObjectsList)
            # Get list of objects - objects being perceived in last 1 seconds without limit for their number (0).
            return get_ro_list(rospy.Time(1), 0)
        except rospy.ServiceException, e:
            _print_error_message('Service call failed: %s' % e)

    def get_scene_object_info(self, object_id):
        rospy.wait_for_service('return_recognized_object_pose')
        try:
            get_ro_pose = rospy.ServiceProxy('return_recognized_object_pose', GetRecognizedObjectPose)
            ret = get_ro_pose(object_id, rospy.Time(5))
            if (ret.status != GetRecognizedObjectPoseResponse.OBJECT_FOUND):
                _print_error_message('Pose of object %s could not be recovered!' % object_id)
                return None
            return ret.object

        except rospy.ServiceException, e:
            _print_error_message('Service call failed: %s' % e)

    @staticmethod
    def _set_estimate_pose(enable_pose_estimation, view_id):
        rospy.wait_for_service('estimate_pose')
        try:
            estimate_pose = rospy.ServiceProxy('estimate_pose', EstimatePose)
            return estimate_pose(enable_pose_estimation, view_id)
        except rospy.ServiceException, e:
            _print_error_message('Service call failed: %s' % e)
