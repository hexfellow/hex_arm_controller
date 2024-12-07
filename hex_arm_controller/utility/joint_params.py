#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import json
from xpkg_arm_msgs.msg import XmsgArmJointParamList, XmsgArmJointParam
from enum import Enum
import struct

class HEXARMModeClass(Enum):
    POSITION = "position_mode"
    MIT = "mit_mode"

class HEXARMJointCtlExtraParams:
    def __init__(self, braking_state=None, mit_kp=None, mit_kd=None):
        self.braking_state = braking_state
        self.mit_kp = mit_kp
        self.mit_kd = mit_kd

    def encode(self):
        param_dict = {
            'braking_state': self.braking_state,
            'mit_kp': self.mit_kp,
            'mit_kd': self.mit_kd
        }
        return json.dumps(param_dict)

class HEXARMJointStateMsg:
    def __init__(self, motor_cnt, mode, pose, speed, effort, extra_param=None):
        self.motor_cnt = motor_cnt
        self.mode = mode
        self.pose = pose
        self.speed = speed
        self.effort = effort
        self.extra_param = extra_param

    def get_msg(self) -> XmsgArmJointParamList:
        if len(self.mode) != len(self.pose) or len(self.pose) != len(self.speed) or len(self.speed) != len(self.effort) or len(self.effort) != self.motor_cnt:
            raise InvalidJointStateError(f"Joint parameters not match with motor count: {self.motor_cnt}")

        joint_state_msg = XmsgArmJointParamList()

        for i in range(self.motor_cnt):
            joint_param = XmsgArmJointParam()

            if self.is_match_mode(self.mode[i]):
                joint_param.mode = self.mode[i]
            else:
                raise InvalidJointStateError(f"Invalid motor_mode: '{self.mode[i]}'")

            joint_param.pose = self.pose[i]
            joint_param.speed = self.speed[i]
            joint_param.effort = self.effort[i]
            if self.extra_param is not None:
                joint_param.extra_param = self.extra_param[i].encode()
            joint_state_msg.joints.append(joint_param)

        return joint_state_msg

    def is_match_mode(self, input_string):
        return input_string in [m.value for m in HEXARMModeClass]


class InvalidJointStateError(Exception):
    """Custom exception for invalid joint state"""
    pass