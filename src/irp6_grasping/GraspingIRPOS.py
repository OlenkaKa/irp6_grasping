#!/usr/bin/env python

from irpos import IRPOS

class GraspingIRPOS(IRPOS):
    def __init__(self, nodeName, robotName, robotJointNumbers, scheme_name):
        IRPOS.__init__(self, nodeName, robotName, robotJointNumbers, scheme_name)
