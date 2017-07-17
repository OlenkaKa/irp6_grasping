#!/usr/bin/env python
# -*- coding: utf-8 -*-

from WatchingIRPOS import WatchingIRPOS


def _print_message(message):
    print '\033[1;35m[GraspingIRPOS] %s\033[0m' % message


def _print_error_message(message):
    print '\033[1;31m[GraspingIRPOS] %s\033[0m' % message


class GraspingIRPOS(WatchingIRPOS):
    def __init__(self, nodeName, robotName, robotJointNumbers, scheme_name):
        WatchingIRPOS.__init__(self, nodeName, robotName, robotJointNumbers, scheme_name)

    ### GRASPING

    def get_objects_to_grasp(self, min_confidence=0.6):
        return [object for object in WatchingIRPOS.get_scene_objects(self) if object.confidence > min_confidence]

    def grasp_object(self, object_id):
        _print_message('Grasp object with id "%s"' % object_id)
