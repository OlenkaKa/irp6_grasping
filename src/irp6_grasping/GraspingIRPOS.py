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


#
# class GraspingIRPOSType(type):
#     def __new__(cls, name, bases, attrs):
#         stop_observation_func = attrs['stop_object_pose_estimation']
#         print attrs
#         for attr_name, attr_value in attrs.iteritems():
#             if isinstance(attr_value, types.FunctionType):# \
#                     # and attr_name.startswith('move') \
#                     # and hasattr(IRPOS, attr_name):
#                 attrs[attr_name] = cls.move_func_decorator(attr_value, stop_observation_func, attr_name)
#         return super(GraspingIRPOSType, cls).__new__(cls, name, bases, attrs)
#
#     @classmethod
#     def move_func_decorator(cls, move_func, stop_observation_func, attr_name):
#         def wrapper(*args, **kwargs):
#             print "Nazwa:  %s" % attr_name
#             stop_observation_func(args[0])
#             return move_func(*args, **kwargs)
#
#         return wrapper


class GraspingIRPOS(IRPOS):
    # __metaclass__ = GraspingIRPOSType

    def __init__(self, nodeName, robotName, robotJointNumbers, scheme_name):
        self.estimate_object_pose = False
        IRPOS.__init__(self, nodeName, robotName, robotJointNumbers, scheme_name)

    ### OPTOFORCE CONTROLLER

    def tfg_to_joint_position_with_contact(self, position):
        self.__print_message('Tfg to joint position with contact')

        class OptoforceData:
            def __init__(self, optoforce_value):
                self.no_contact_average_measurement = 0
                self.contact_difference = 17
                self.lock = Lock()
                self.contact = None
                self.check_if_contact(optoforce_value)

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

    ### OBJECT ESTIMATION

    def start_object_pose_estimation(self):
        self.print_message('Start object pose estimation')
        if not self.estimate_object_pose:
            self.set_estimate_pose(True, 1)
            self.estimate_object_pose = True

    def stop_object_pose_estimation(self):
        self.print_message('Stop object pose estimation')
        if self.estimate_object_pose:
            self.set_estimate_pose(False, 1)
            self.estimate_object_pose = False

    @staticmethod
    def set_estimate_pose(enable_pose_estimation, view_id):
        rospy.wait_for_service('estimate_pose')
        try:
            estimate_pose = rospy.ServiceProxy('estimate_pose', EstimatePose)
            return estimate_pose(enable_pose_estimation, view_id)
        except rospy.ServiceException, e:
            GraspingIRPOS.print_error_message('Service call failed: %s' % e)

    ### GRASPING

    def get_objects_to_grasp(self):
        return []

    def grasp_object(self, object_id):
        pass

    ### OTHER

    @staticmethod
    def print_message(message):
        print '\033[1;36m[GraspingIRPOS] %s\033[0m' % message

    @staticmethod
    def print_error_message(message):
        print '\033[1;36m[GraspingIRPOS] %s\033[0m' % message
