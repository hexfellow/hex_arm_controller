#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import os
from .joint_params import HEXARMJointCtlExtraParams as HEXARMJointCtlExtraParams
from .joint_params import HEXARMJointStateMsg as HEXARMJointStateMsg

ROS_VERSION = os.environ.get('ROS_VERSION')
if ROS_VERSION == '1':
    from .ros1_arm_controller import ArmController as ArmController
elif ROS_VERSION == '2':
    from .ros2_arm_controller import ArmController as ArmController
else:
    raise ValueError("ROS_VERSION is not set")

__all__ = [
    "JointCtlExtraParams",
    "JointStateMsg",
    "ArmController",
]
