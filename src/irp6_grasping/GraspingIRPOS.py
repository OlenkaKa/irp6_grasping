#!/usr/bin/env python
# -*- coding: utf-8 -*-

import types

from irpos import IRPOS


def print_message(message):
    print '\033[0;36m[GraspingIRPOS] %s\033[0m' % message


def print_message_bold(message):
    print '\033[1;36m[GraspingIRPOS] %s\033[0m' % message


class GraspingIRPOSType(type):
    def __new__(cls, name, bases, attrs):
        stop_observation_func = attrs['stop_object_tracking']
        for attr_name, attr_value in attrs.iteritems():
            if isinstance(attr_value, types.FunctionType) \
                    and attr_value.func_name.startswith('move') \
                    and hasattr(IRPOS, attr_value.func_name):
                attrs[attr_name] = cls.move_func_decorator(attr_value, stop_observation_func)
        return super(GraspingIRPOSType, cls).__new__(cls, name, bases, attrs)

    @classmethod
    def move_func_decorator(cls, move_func, stop_observation_func):
        def wrapper(*args, **kwargs):
            stop_observation_func(*args, **kwargs)
            return move_func(*args, **kwargs)

        return wrapper


class GraspingIRPOS(IRPOS):
    __metaclass__ = GraspingIRPOSType

    def __init__(self, nodeName, robotName, robotJointNumbers, scheme_name):
        self.track_object = False
        IRPOS.__init__(self, nodeName, robotName, robotJointNumbers, scheme_name)

    def stop_object_tracking(self):
        self.track_object = False
