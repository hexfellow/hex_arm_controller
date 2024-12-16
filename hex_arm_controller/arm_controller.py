#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import os
import sys

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(script_dir)

from utility import ArmController, HEXARMJointCtlExtraParams

def main():
    arm_controller = ArmController("joint_state_publisher")

    ### init test point
    test_mode = [
        ["mit_mode", "position_mode", "position_mode", "position_mode", "position_mode", "position_mode"],
        ["mit_mode", "position_mode", "position_mode", "position_mode", "position_mode", "position_mode"],
        ["mit_mode", "position_mode", "position_mode", "position_mode", "position_mode", "position_mode"],
        ]

    test_pose = [
        [0.0, 0.523598775598, 2.09439265359, 0.0, -1.0472, 0.0],
        [0.0, 0.523598775598, 2.09439265359, 0.0, -1.0472, 0.0],
        [0.0, 0.523598775598, 2.09439265359, 0.0, -1.0472, 0.0],
        ]

    test_speed = [
        [0.3, 0.3, 0.3, 0.3, 0.3, 0.3],
        [0.3, 0.3, 0.3, 0.3, 0.3, 0.3],
        [0.3, 0.3, 0.3, 0.3, 0.3, 0.3],
        ]

    test_effort = [
        [0.0, 20.0, 20.0, 20.0, 20.0, 20.0],
        [0.0, 20.0, 20.0, 20.0, 20.0, 20.0],
        [0.0, 20.0, 20.0, 20.0, 20.0, 20.0],
        ]

    extra_param_test = []

    for i in range(3):
        extra_param = [
            HEXARMJointCtlExtraParams(braking_state=None, mit_kp=0.5, mit_kd=0.1),
            HEXARMJointCtlExtraParams(braking_state=None, mit_kp=None, mit_kd=None),
            HEXARMJointCtlExtraParams(braking_state=None, mit_kp=None, mit_kd=None),
            HEXARMJointCtlExtraParams(braking_state=None, mit_kp=None, mit_kd=None),
            HEXARMJointCtlExtraParams(braking_state=None, mit_kp=None, mit_kd=None),
            HEXARMJointCtlExtraParams(braking_state=None, mit_kp=None, mit_kd=None),
            ]
        extra_param_test.append(extra_param)

    arm_controller.init_publish_list(6, test_mode, test_pose, test_speed, test_effort, extra_param_test)

    ### start test
    try:
        arm_controller.work(cycle=5.0)
    except KeyboardInterrupt:
        arm_controller.shutdown()


if __name__ == '__main__':
    main()