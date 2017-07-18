#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import PyKDL

import tf_conversions.posemath as posemath

from collections import namedtuple
from WatchingIRPOS import WatchingIRPOS


def _print_main_message(message):
    print '\033[1;35m[GraspingIRPOS] %s\033[0m' % message


def _print_message(message):
    print '\033[0;35m[GraspingIRPOS] %s\033[0m' % message


def _print_error_message(message):
    print '\033[1;31m[GraspingIRPOS] %s\033[0m' % message


class GraspingIRPOS(WatchingIRPOS):
    def __init__(self, nodeName, robotName, robotJointNumbers, scheme_name):
        WatchingIRPOS.__init__(self, nodeName, robotName, robotJointNumbers, scheme_name)


    ### GRASPING

    def get_objects_to_grasp(self, min_confidence=0.6):
        return [object for object in WatchingIRPOS.get_scene_objects(self) if object.confidence > min_confidence]


    def grasp_object(self, object_id):
        _print_main_message('Grasp object with id "%s"' % object_id)
        object_info = WatchingIRPOS.get_scene_object_info(self, object_id)
        if object_info is None:
            _print_error_message('Cannot grasp object with id "%s" - object not found' % object_id)
            return

        object_pose = posemath.fromMsg(object_info.pose.pose.pose) * PyKDL.Frame(
            PyKDL.Rotation.RPY(0, -math.pi, 0),
            PyKDL.Vector(0, 0, 0))

        Size = namedtuple('Size', ['x', 'y', 'z'])
        x, y, z = None, None, None
        for vertex in object_info.bounding_mesh.vertices:
            if abs(vertex.x) > x:
                x = abs(vertex.x)
            if abs(vertex.y) > y:
                y = abs(vertex.y)
            if abs(vertex.z) > z:
                z = abs(vertex.z)

        object_size = Size(x, y, z)
        object_side_poses = self._calculate_side_poses(object_pose, object_size)

        # Chwytak ustawiony 2cm poniżej górnej ścianki
        grasp_pose = self._select_highest_pose(object_side_poses) * PyKDL.Frame(
            PyKDL.Rotation.RPY(0, 0, 0),
            PyKDL.Vector(0, 0, 0.02))

        # Chwytak ustawiony 5cm powyżej pozycji chwytu obiektu
        pregrasp_pose = grasp_pose * PyKDL.Frame(
            PyKDL.Rotation.RPY(0, 0, 0),
            PyKDL.Vector(0, 0, -0.05))

        _print_message('Move to pregrasp pose')
        WatchingIRPOS.move_to_cartesian_pose(self, 20.0, posemath.toMsg(pregrasp_pose))

        _print_message('Open gripper')
        WatchingIRPOS.tfg_to_joint_position(self, 0.09, 3.0)

        _print_message('Move to grasp pose')
        WatchingIRPOS.move_to_cartesian_pose(self, 3.0, posemath.toMsg(grasp_pose))

        _print_message('Close gripper')
        WatchingIRPOS.tfg_to_joint_position_with_contact(self, 0.053)

        _print_main_message('Grasp object with id "%s" finished' % object_id)


    @staticmethod
    def _calculate_side_poses(pose, size):
        top = pose * PyKDL.Frame(
            PyKDL.Rotation.RPY(math.pi, 0, 0),
            PyKDL.Vector(size.x / 2, size.y / 2, size.z))

        bottom = pose * PyKDL.Frame(
            PyKDL.Rotation.RPY(0, 0, 0),
            PyKDL.Vector(size.x / 2, size.y / 2, 0))

        left = pose * PyKDL.Frame(
            PyKDL.Rotation.RPY(math.pi / 2, 0, math.pi / 2),
            PyKDL.Vector(0, size.y / 2, size.z / 2))

        right = pose * PyKDL.Frame(
            PyKDL.Rotation.RPY(math.pi / 2, 0, -math.pi / 2),
            PyKDL.Vector(size.x, size.y / 2, size.z / 2))

        front = pose * PyKDL.Frame(
            PyKDL.Rotation.RPY(math.pi / 2, 0, 0),
            PyKDL.Vector(size.x / 2, 0, size.z / 2))

        back = pose * PyKDL.Frame(
            PyKDL.Rotation.RPY(-math.pi / 2, 0, 0),
            PyKDL.Vector(size.x / 2, size.y, size.z / 2))

        return top, bottom, left, right, front, back


    @staticmethod
    def _select_highest_pose(poses):
        max_z = float('-inf')
        result_pose = None
        for pose in poses:
            if pose.p.z() > max_z:
                max_z = pose.p.z()
                result_pose = pose
        return result_pose
